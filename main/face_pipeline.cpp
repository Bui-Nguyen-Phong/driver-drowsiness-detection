/*
 * Face pipeline: HumanFaceDetect (MSR+MNP) -> PPA crop/resize -> PFLD landmarks.
 * Ported from ~/esp/pfld_bench, driven by a frame counter instead of a benchmark loop.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "linux/videodev2.h"
#include "dl_model_base.hpp"
#include "dl_image_ppa.hpp"
#include "dl_image_preprocessor.hpp"
#include "human_face_detect.hpp"
#include "face_pipeline.h"
#include "audio_alert.h"

extern const uint8_t pfld_espdl_start[] asm("_binary_pfld_espdl_start");
#if CONFIG_APP_EYE_CLASSIFIER_ENABLE
extern const uint8_t eye_espdl_start[] asm("_binary_open_closed_eye_espdl_start");
#endif

static const char *TAG = "face_pipeline";

/* ---------------- Tuning (same values as pfld_bench) ---------------- */

/* Face box is made square and enlarged by this factor before cropping for PFLD */
#define FACE_EXPAND         1.2f

/* PFLD input normalization in [0,255] scale: x' = (x - mean) / std. (0,255) == ToTensor() */
static const std::array<float, 3> PFLD_MEAN = {0.f, 0.f, 0.f};
static const std::array<float, 3> PFLD_STD  = {255.f, 255.f, 255.f};
#define PFLD_RGB_SWAP       false   /* true if the model expects BGR */

/* A new detector box keeps the previous landmarks if it overlaps the old box by this IoU */
#define LANDMARK_KEEP_IOU   0.3f

#define BOX_THICKNESS       3
#define LANDMARK_RADIUS     3       /* filled round dot, diameter = 2r+1 pixels */
#define LANDMARK_OUTLINE    1       /* dark ring around each dot for contrast */
#define STATS_PERIOD_US     (3 * 1000 * 1000)
#define LOG_LANDMARKS       1       /* also print box + landmark coordinates with every stats line */
#define LOG_POINTS_PER_LINE 8

/* ---------------- Drowsiness (eyes closed / yawn) ----------------
 * EAR = eye opening / eye width, MAR = inner-lip opening / mouth width.
 * The absolute values depend on the person, on the face size in the frame and on the landmark
 * model (this PFLD under-estimates mouth opening: wide open is only ~0.3-0.4), so the thresholds
 * are calibrated once per session:
 *  - a session starts when a face appears and ends after SESSION_LOST_US without any face
 *  - during the first CALIB_DURATION_US the person looks at the camera, eyes open, mouth closed;
 *    the medians of EAR and MAR (robust to a few blinks) become the baseline
 *  - eyes: relative to the baseline (EAR scales with the eye shape)
 *    mouth: baseline + delta (closed MAR is ~0, so a ratio would be meaningless)
 *  - the session is re-calibrated when the face size changes a lot (distance), or on request.
 * Close/open thresholds differ (hysteresis) so noise around one value does not toggle the state.
 * The alert fires only after the state lasts long enough: a blink is 0.1-0.4 s, talking opens the
 * mouth only briefly. */
#define CALIB_DURATION_US       (3 * 1000 * 1000)
#define CALIB_MIN_SAMPLES       10      /* keep collecting past CALIB_DURATION_US until this many */
#define CALIB_MAX_SAMPLES       96
#define CALIB_MIN_EAR_BASE      0.12f   /* lower baseline = eyes were closed: calibrate again */
#define SESSION_LOST_US         (3 * 1000 * 1000)
#define EAR_CLOSE_RATIO         0.75f   /* closed below 75 % of the open-eye baseline */
#define EAR_OPEN_RATIO          0.85f   /* open again above 85 % */
#define MAR_YAWN_DELTA          0.15f   /* yawn above baseline + 0.15 */
#define MAR_YAWN_END_DELTA      0.08f   /* over below baseline + 0.08 */
#define EYES_CLOSED_ALERT_US    (800 * 1000)
#define YAWN_ALERT_US           (1500 * 1000)
#define RECALIB_ON_DISTANCE     1

/* Eye classifier (open-closed-eye-0001): output index 0 = closed, 1 = open.
 * Input = (pixel - 127) / 255, BGR; trained on grayscale IR eye crops (MRL), so the crop is
 * converted to gray and replicated to 3 channels. The crop is a square centred on the eye contour,
 * rotated so the eye corners are horizontal, side = EYE_CROP_SCALE x corner distance
 * (MRL crops are loose, eyebrow often visible). */
#define EYE_CROP_SCALE          2.0f
#define EYE_CLOSED_P            0.60f   /* eyes closed when smoothed P(closed) > 0.6 ... */
#define EYE_OPEN_P              0.40f   /* ... open again below 0.4 */
#define EYE_CALIB_MAX_P         0.50f   /* calibration rejected if the median P(closed) is above this */
#define DRAW_EYE_CROPS          1       /* draw the classifier crops (debug: check they match MRL-style crops) */
#define RECALIB_SIZE_RATIO      1.5f    /* face width x1.5 or /1.5 vs. calibration ... */
#define RECALIB_DRIFT_US        (3 * 1000 * 1000)   /* ... for this long -> new calibration */
#define METRIC_SMOOTHING        0.5f    /* EMA weight of the newest PFLD result */
#define SHOW_METRICS            1       /* draw "EAR x.xx MAR x.xx" under each face box */
#define TEXT_SCALE              3       /* alert banner: 5x7 font scaled by this */
#define METRIC_TEXT_SCALE       2

/* ---------------- State ---------------- */

enum {
    JOB_DETECT = 1 << 0,
    JOB_PFLD   = 1 << 1,
};

static uint32_t s_width, s_height, s_v4l2_fmt;
static size_t s_frame_size;
static dl::image::img_t s_img;              /* private copy of the frame used by the worker */

static TaskHandle_t s_task;
static std::atomic<bool> s_busy{false};
static std::atomic<uint32_t> s_job{0};

/* Scheduler (only touched from the capture path) */
static uint32_t s_frame_idx;
static bool s_detect_due;
static bool s_pfld_due;

/* Published results */
static SemaphoreHandle_t s_lock;
static SemaphoreHandle_t s_draw_lock;       /* stream + capture handlers run in different tasks */
static face_result_t s_results[FACE_PIPELINE_MAX_FACES];
static int s_result_count;
static face_calib_t s_calib_pub;            /* published copy of s_cal, under s_lock */

/* Session calibration (only touched by the worker task) */
static struct {
    face_calib_state_t state;
    int64_t start_us, last_face_us, drift_since_us;
    int n;
    float ear[CALIB_MAX_SAMPLES], mar[CALIB_MAX_SAMPLES], eyep[CALIB_MAX_SAMPLES];
    float face_w;                           /* face box width at calibration time */
    face_calib_t th;
} s_cal;
static std::atomic<bool> s_recalib_req{false};

/* ---------------- Helpers ---------------- */

/* Square box enlarged by FACE_EXPAND around the face, clamped to the image. Returns {x1,y1,x2,y2}. */
static std::vector<int> make_crop(const face_result_t &f, int img_w, int img_h)
{
    float cx = (f.x1 + f.x2) * 0.5f;
    float cy = (f.y1 + f.y2) * 0.5f;
    float side = std::max(f.x2 - f.x1, f.y2 - f.y1) * FACE_EXPAND;
    int x1 = std::max(0, (int)lroundf(cx - side * 0.5f));
    int y1 = std::max(0, (int)lroundf(cy - side * 0.5f));
    int x2 = std::min(img_w, (int)lroundf(cx + side * 0.5f));
    int y2 = std::min(img_h, (int)lroundf(cy + side * 0.5f));
    return {x1, y1, x2, y2};
}

static float iou(const face_result_t &a, const face_result_t &b)
{
    int ix1 = std::max(a.x1, b.x1), iy1 = std::max(a.y1, b.y1);
    int ix2 = std::min(a.x2, b.x2), iy2 = std::min(a.y2, b.y2);
    float inter = (float)std::max(0, ix2 - ix1) * std::max(0, iy2 - iy1);
    float ua = (float)(a.x2 - a.x1) * (a.y2 - a.y1) + (float)(b.x2 - b.x1) * (b.y2 - b.y1) - inter;
    return ua > 0 ? inter / ua : 0.f;
}

/* ---------------- Drowsiness ---------------- */

static inline float pt_dist(const float *p, int a, int b)
{
    return hypotf(p[2 * a] - p[2 * b], p[2 * a + 1] - p[2 * b + 1]);
}

/* 6-point contour (68 layout): corner, 2 top, corner, 2 bottom */
static float aspect6(const float *p, int o)
{
    float w = pt_dist(p, o + 0, o + 3);
    return w > 1.f ? (pt_dist(p, o + 1, o + 5) + pt_dist(p, o + 2, o + 4)) / (2.f * w) : -1.f;
}

/* 8-point contour (98-layout eyes, both layouts' inner lips): corner, 3 top, corner, 3 bottom */
static float aspect8(const float *p, int o)
{
    float w = pt_dist(p, o + 0, o + 4);
    return w > 1.f ? (pt_dist(p, o + 1, o + 7) + pt_dist(p, o + 2, o + 6) + pt_dist(p, o + 3, o + 5)) / (3.f * w) : -1.f;
}

/* ---------------- Eye classifier ---------------- */

#if CONFIG_APP_EYE_CLASSIFIER_ENABLE
typedef struct {
    dl::Model *model;
    dl::TensorBase *in, *out;
    int in_w, in_h;
    int8_t lut[256];            /* gray value -> quantized model input */
    float out_scale;
} eye_cls_t;

static bool eye_cls_init(eye_cls_t &e)
{
    e.model = new dl::Model((const char *)eye_espdl_start, fbs::MODEL_LOCATION_IN_FLASH_RODATA);
    e.in = e.model->get_inputs().begin()->second;
    e.out = e.model->get_outputs().begin()->second;
    if (e.in->get_dtype() != dl::DATA_TYPE_INT8 || e.out->get_dtype() != dl::DATA_TYPE_INT8 ||
            e.in->shape.size() != 4 || e.in->shape[3] != 3 || e.out->get_size() != 2) {
        ESP_LOGE(TAG, "eye model: unexpected input/output (need INT8 NHWC x3 -> 2 logits), classifier disabled");
        return false;
    }
    e.in_h = e.in->shape[1];
    e.in_w = e.in->shape[2];
    e.out_scale = ldexpf(1.f, e.out->get_exponent());

    /* Same normalization as common.py: x = (p - 127) / 255, then quantize with the input exponent */
    const float in_scale = ldexpf(1.f, -e.in->get_exponent());
    for (int v = 0; v < 256; v++) {
        long q = lroundf((v - 127.f) / 255.f * in_scale);
        e.lut[v] = (int8_t)std::max(-128L, std::min(127L, q));
    }

    ESP_LOGI(TAG, "eye model: input [%d,%d,%d] exp=%d, output %d logits exp=%d",
             e.in_h, e.in_w, e.in->shape[3], e.in->get_exponent(), e.out->get_size(), e.out->get_exponent());
    if (e.model->test() != ESP_OK) {    /* uses the test vectors embedded by esp-ppq */
        ESP_LOGW(TAG, "eye model self-test failed (or no test data embedded)");
    }
    return true;
}

/* Gray (0..255) of frame pixel (x, y), edges replicated */
static inline int gray_at(const uint8_t *img, int x, int y)
{
    x = std::max(0, std::min(x, (int)s_width - 1));
    y = std::max(0, std::min(y, (int)s_height - 1));
    const uint8_t *p = img + ((size_t)y * s_width + x) * 2;
    uint16_t v = s_v4l2_fmt == V4L2_PIX_FMT_RGB565X ? (p[0] << 8 | p[1]) : (p[1] << 8 | p[0]);
    int r = (v >> 11) & 0x1F, g = (v >> 5) & 0x3F, b = v & 0x1F;
    r = (r << 3) | (r >> 2);
    g = (g << 2) | (g >> 4);
    b = (b << 3) | (b >> 2);
    return (77 * r + 150 * g + 29 * b) >> 8;
}

/* Eye contour of a landmark layout: first point, count, the two corner indices */
static bool eye_contour(int n_pts, int eye, int *first, int *count, int *c0, int *c1)
{
    if (n_pts == 68) {          /* 36-41 / 42-47, corners 0 and 3 */
        *first = eye ? 42 : 36;
        *count = 6;
        *c0 = *first;
        *c1 = *first + 3;
    } else if (n_pts == 98) {   /* 60-67 / 68-75, corners 0 and 4 */
        *first = eye ? 68 : 60;
        *count = 8;
        *c0 = *first;
        *c1 = *first + 4;
    } else {
        return false;
    }
    return true;
}

/* Crop one eye (rotated square, bilinear like cv2.INTER_LINEAR) straight into the model input,
 * run it and return P(closed), or -1. box receives cx, cy, side for drawing. */
static float eye_cls_run(eye_cls_t &e, const float *pts, int n_pts, int eye, int16_t box[3])
{
    int first, count, c0, c1;
    box[0] = box[1] = box[2] = 0;
    if (!eye_contour(n_pts, eye, &first, &count, &c0, &c1)) {
        return -1.f;
    }

    float cx = 0, cy = 0;
    for (int k = first; k < first + count; k++) {
        cx += pts[2 * k];
        cy += pts[2 * k + 1];
    }
    cx /= count;
    cy /= count;
    float dx = pts[2 * c1] - pts[2 * c0], dy = pts[2 * c1 + 1] - pts[2 * c0 + 1];
    float w = hypotf(dx, dy);
    if (w < 4.f) {
        return -1.f;
    }
    if (dx < 0) {               /* keep the crop upright whatever the corner order */
        dx = -dx;
        dy = -dy;
    }
    const float side = w * EYE_CROP_SCALE;
    const float ux = dx / w, uy = dy / w;           /* crop x axis along the eye corners */
    const float vx = -uy, vy = ux;                  /* crop y axis */
    box[0] = (int16_t)lroundf(cx);
    box[1] = (int16_t)lroundf(cy);
    box[2] = (int16_t)lroundf(side);

    const uint8_t *img = (const uint8_t *)s_img.data;
    int8_t *dst = (int8_t *)e.in->data;
    const float step_x = side / e.in_w, step_y = side / e.in_h;
    for (int j = 0; j < e.in_h; j++) {
        float oy = (j + 0.5f) * step_y - side * 0.5f;          /* pixel-centre mapping (cv2) */
        for (int i = 0; i < e.in_w; i++) {
            float ox = (i + 0.5f) * step_x - side * 0.5f;
            float sx = cx + ox * ux + oy * vx - 0.5f;
            float sy = cy + ox * uy + oy * vy - 0.5f;
            int x0 = (int)floorf(sx), y0 = (int)floorf(sy);
            float fx = sx - x0, fy = sy - y0;
            float g = (1 - fy) * ((1 - fx) * gray_at(img, x0, y0) + fx * gray_at(img, x0 + 1, y0)) +
                      fy * ((1 - fx) * gray_at(img, x0, y0 + 1) + fx * gray_at(img, x0 + 1, y0 + 1));
            int8_t q = e.lut[std::max(0, std::min(255, (int)lroundf(g)))];
            dst[0] = dst[1] = dst[2] = q;                       /* gray replicated: B = G = R */
            dst += 3;
        }
    }

    e.model->run();
    const int8_t *o = (const int8_t *)e.out->data;
    float l_closed = o[0] * e.out_scale, l_open = o[1] * e.out_scale;   /* index 0 = closed, 1 = open */
    return 1.f / (1.f + expf(l_open - l_closed));
}
#endif /* CONFIG_APP_EYE_CLASSIFIER_ENABLE */

/* Raw EAR / MAR from fresh (float) landmarks. Returns false for an unsupported layout. */
static bool compute_ratios(const float *pts, int n, float *ear, float *mar)
{
    if (n == 68) {          /* iBUG 300-W: eyes 36-41 / 42-47, inner lips 60-67 */
        *ear = (aspect6(pts, 36) + aspect6(pts, 42)) * 0.5f;
        *mar = aspect8(pts, 60);
    } else if (n == 98) {   /* WFLW: eyes 60-67 / 68-75, inner lips 88-95 */
        *ear = (aspect8(pts, 60) + aspect8(pts, 68)) * 0.5f;
        *mar = aspect8(pts, 88);
    } else {
        return false;
    }
    return *ear >= 0 && *mar >= 0;
}

static float median(float *v, int n)
{
    std::sort(v, v + n);
    return n & 1 ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

static void calib_start(int64_t now, const char *why)
{
    s_cal.state = FACE_CALIB_RUNNING;
    s_cal.start_us = now;
    s_cal.drift_since_us = 0;
    s_cal.n = 0;
    ESP_LOGI(TAG, "calibration started (%s): look at the camera, eyes open, mouth closed", why);
}

static void calib_finish(int64_t now, float face_w)
{
    float ear_base = median(s_cal.ear, s_cal.n);
    float mar_base = median(s_cal.mar, s_cal.n);
#if CONFIG_APP_EYE_CLASSIFIER_ENABLE
    float eyep_base = median(s_cal.eyep, s_cal.n);
    if (eyep_base > EYE_CALIB_MAX_P) {
        ESP_LOGW(TAG, "calibration rejected: eyes classified closed (P=%.2f), retrying", eyep_base);
        calib_start(now, "retry");
        return;
    }
#else
    if (ear_base < CALIB_MIN_EAR_BASE) {
        ESP_LOGW(TAG, "calibration rejected: EAR baseline %.3f too low (eyes closed?), retrying", ear_base);
        calib_start(now, "retry");
        return;
    }
#endif

    face_calib_t &t = s_cal.th;
    t.ear_base = ear_base;
    t.mar_base = mar_base;
    t.ear_close = ear_base * EAR_CLOSE_RATIO;
    t.ear_open = ear_base * EAR_OPEN_RATIO;
    t.mar_yawn = mar_base + MAR_YAWN_DELTA;
    t.mar_end = mar_base + MAR_YAWN_END_DELTA;
    s_cal.face_w = face_w;
    s_cal.state = FACE_CALIB_READY;
#if CONFIG_APP_AUDIO_ALERT_ENABLE && CONFIG_APP_AUDIO_ALERT_CALIB_SOUND
    audio_alert_play(AUDIO_ALERT_CALIB_DONE);
#endif
    ESP_LOGI(TAG, "calibrated from %d samples, face %d px: EAR base %.3f -> closed < %.3f / open > %.3f | "
             "MAR base %.3f -> yawn > %.3f / end < %.3f",
             s_cal.n, (int)face_w, t.ear_base, t.ear_close, t.ear_open, t.mar_base, t.mar_yawn, t.mar_end);
}

/* Session bookkeeping, once per worker iteration. main = largest face (or NULL if none). */
static void calib_update(const face_result_t *main, bool main_sampled, float ear_raw, float mar_raw, float eyep_raw,
                         int64_t now)
{
    if (main) {
        s_cal.last_face_us = now;
    }
    if (s_recalib_req.exchange(false)) {
        if (main) {
            calib_start(now, "requested");
        } else {
            s_cal.state = FACE_CALIB_NONE;
        }
    }
    if (!main && s_cal.state != FACE_CALIB_NONE && now - s_cal.last_face_us > SESSION_LOST_US) {
        s_cal.state = FACE_CALIB_NONE;
        ESP_LOGI(TAG, "session ended (no face for %d s)", SESSION_LOST_US / 1000000);
        return;
    }

    switch (s_cal.state) {
    case FACE_CALIB_NONE:
        if (main_sampled) {
            calib_start(now, "new session");
        }
        break;

    case FACE_CALIB_RUNNING:
        if (main_sampled && s_cal.n < CALIB_MAX_SAMPLES) {
            s_cal.ear[s_cal.n] = ear_raw;
            s_cal.mar[s_cal.n] = mar_raw;
            s_cal.eyep[s_cal.n] = eyep_raw;
            s_cal.n++;
        }
        if (main && now - s_cal.start_us >= CALIB_DURATION_US && s_cal.n >= CALIB_MIN_SAMPLES) {
            calib_finish(now, (float)(main->x2 - main->x1));
        }
        break;

    case FACE_CALIB_READY:
#if RECALIB_ON_DISTANCE
        if (main && !main->eyes_closed_alert && !main->yawn_alert) {
            float ratio = (main->x2 - main->x1) / std::max(s_cal.face_w, 1.f);
            if (ratio > RECALIB_SIZE_RATIO || ratio < 1.f / RECALIB_SIZE_RATIO) {
                if (!s_cal.drift_since_us) {
                    s_cal.drift_since_us = now;
                } else if (now - s_cal.drift_since_us >= RECALIB_DRIFT_US) {
                    calib_start(now, "face distance changed");
                }
            } else {
                s_cal.drift_since_us = 0;
            }
        }
#endif
        break;
    }
}

/* Alert state of one face against the session thresholds */
static void update_alerts(face_result_t &r, int64_t now)
{
    const face_calib_t &t = s_cal.th;
    if (s_cal.state != FACE_CALIB_READY || r.ear < 0 || r.mar < 0) {
        r.eyes_closed_since_us = r.mouth_open_since_us = 0;
        r.eyes_closed_alert = r.yawn_alert = false;
        return;
    }

#if CONFIG_APP_EYE_CLASSIFIER_ENABLE
    const bool closed = r.eye_closed_p > EYE_CLOSED_P;
    const bool open = r.eye_closed_p >= 0 && r.eye_closed_p < EYE_OPEN_P;
#else
    const bool closed = r.ear < t.ear_close;
    const bool open = r.ear > t.ear_open;
#endif
    if (closed) {
        if (!r.eyes_closed_since_us) {
            r.eyes_closed_since_us = now;
        }
    } else if (open) {
        r.eyes_closed_since_us = 0;
    }
    if (r.mar > t.mar_yawn) {
        if (!r.mouth_open_since_us) {
            r.mouth_open_since_us = now;
        }
    } else if (r.mar < t.mar_end) {
        r.mouth_open_since_us = 0;
    }

    r.eyes_closed_alert = r.eyes_closed_since_us && now - r.eyes_closed_since_us >= EYES_CLOSED_ALERT_US;
    r.yawn_alert = r.mouth_open_since_us && now - r.mouth_open_since_us >= YAWN_ALERT_US;
}

/* Speaker: eyes-closed alarm repeats while it lasts and pre-empts everything;
 * a yawn chimes once per yawn. */
static void audio_update(const face_result_t *faces, int n, int64_t now)
{
#if CONFIG_APP_AUDIO_ALERT_ENABLE
    static bool eyes_prev, yawn_prev;
    static int64_t eyes_last_us;
    bool eyes = false, yawn = false;
    for (int i = 0; i < n; i++) {
        eyes |= faces[i].eyes_closed_alert;
        yawn |= faces[i].yawn_alert;
    }
    if (eyes) {
        if (!eyes_prev || now - eyes_last_us >= (int64_t)CONFIG_APP_AUDIO_ALERT_REPEAT_MS * 1000) {
            audio_alert_play(AUDIO_ALERT_EYES_CLOSED);
            eyes_last_us = now;
        }
    } else if (yawn && !yawn_prev) {
        audio_alert_play(AUDIO_ALERT_YAWN);
    }
    eyes_prev = eyes;
    yawn_prev = yawn;
#else
    (void)faces; (void)n; (void)now;
#endif
}

/* ---------------- Worker task ---------------- */

static void log_landmarks(const face_result_t *faces, int n);

static void face_pipeline_task(void *arg)
{
    HumanFaceDetect *detector = new HumanFaceDetect();
    dl::Model *pfld = new dl::Model((const char *)pfld_espdl_start, fbs::MODEL_LOCATION_IN_FLASH_RODATA);

    dl::TensorBase *p_in = pfld->get_inputs().begin()->second;
    dl::TensorBase *p_out = pfld->get_outputs().begin()->second;
    /* ESP-DL layout: NHWC */
    const int in_h = p_in->shape[1], in_w = p_in->shape[2], in_c = p_in->shape[3];
    const int n_pts = std::min(p_out->get_size() / 2, FACE_PIPELINE_MAX_LANDMARKS);
    const bool out_int8 = p_out->get_dtype() == dl::DATA_TYPE_INT8;
    const float out_scale = ldexpf(1.f, p_out->get_exponent());
    ESP_LOGI(TAG, "PFLD input [%d,%d,%d] %s, output %d values (%d points) %s exp=%d",
             in_h, in_w, in_c, p_in->get_dtype_string(), p_out->get_size(), n_pts,
             p_out->get_dtype_string(), p_out->get_exponent());

    /* PPA target: PPA can only produce RGB565 or BGR888 from an RGB565 source.
     * The preprocessor converts BGR888 -> RGB (or BGR if PFLD_RGB_SWAP) itself. */
    dl::image::img_t face_img;
    face_img.data = dl::image::alloc_ppa_outbuf(in_w * in_h * 3);
    face_img.width = (uint16_t)in_w;
    face_img.height = (uint16_t)in_h;
    face_img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_BGR888;
    ppa_client_handle_t ppa = dl::image::register_ppa_srm_client();

    dl::image::ImagePreprocessor pfld_pre(pfld, PFLD_MEAN, PFLD_STD, PFLD_RGB_SWAP);

#if CONFIG_APP_EYE_CLASSIFIER_ENABLE
    static eye_cls_t eye_cls;
    const bool eye_ok = eye_cls_init(eye_cls);
#endif

    if (in_c != 3 || !face_img.data) {
        ESP_LOGE(TAG, "PFLD setup failed (channels=%d), pipeline stopped", in_c);
        vTaskDelete(NULL);
        return;
    }

    static float pts[FACE_PIPELINE_MAX_LANDMARKS * 2];     /* float landmarks of the face being processed */

    /* Worker-private copy of the current faces */
    face_result_t work[FACE_PIPELINE_MAX_FACES];
    int work_n = 0;

    /* Stats */
    int64_t stat_t0 = esp_timer_get_time();
    int64_t det_us = 0, pfld_us = 0;
    int det_runs = 0, pfld_runs = 0, pfld_faces = 0, ppa_fail = 0;
    int64_t eye_us = 0;
    int eye_runs = 0;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint32_t job = s_job.load();

        /* 1) Detector */
        if (job & JOB_DETECT) {
            int64_t t0 = esp_timer_get_time();
            std::list<dl::detect::result_t> &faces = detector->run(s_img);
            det_us += esp_timer_get_time() - t0;
            det_runs++;

            face_result_t fresh[FACE_PIPELINE_MAX_FACES];
            int n = 0;
            for (auto &f : faces) {
                if (n >= FACE_PIPELINE_MAX_FACES) {
                    break;
                }
                f.limit_box(s_width, s_height);
                face_result_t &r = fresh[n];
                r.x1 = f.box[0];
                r.y1 = f.box[1];
                r.x2 = f.box[2];
                r.y2 = f.box[3];
                r.score = f.score;
                r.n_landmarks = 0;
                r.ear = r.mar = r.eye_closed_p = -1.f;
                memset(r.eye_box, 0, sizeof(r.eye_box));
                r.eyes_closed_since_us = r.mouth_open_since_us = 0;
                r.eyes_closed_alert = r.yawn_alert = false;

                /* Keep the landmarks of the matching previous face until PFLD refreshes them */
                float best = LANDMARK_KEEP_IOU;
                for (int k = 0; k < work_n; k++) {
                    float v = iou(r, work[k]);
                    if (v >= best && work[k].n_landmarks) {
                        best = v;
                        r.n_landmarks = work[k].n_landmarks;
                        memcpy(r.landmarks, work[k].landmarks, sizeof(r.landmarks));
                        r.ear = work[k].ear;
                        r.mar = work[k].mar;
                        r.eye_closed_p = work[k].eye_closed_p;
                        memcpy(r.eye_box, work[k].eye_box, sizeof(r.eye_box));
                        r.eyes_closed_since_us = work[k].eyes_closed_since_us;
                        r.mouth_open_since_us = work[k].mouth_open_since_us;
                        r.eyes_closed_alert = work[k].eyes_closed_alert;
                        r.yawn_alert = work[k].yawn_alert;
                    }
                }
                n++;
            }
            memcpy(work, fresh, sizeof(face_result_t) * n);
            work_n = n;
        }

        /* The largest face drives the session calibration */
        int main_idx = -1;
        for (int i = 0, best_w = 0; i < work_n; i++) {
            if (work[i].x2 - work[i].x1 > best_w) {
                best_w = work[i].x2 - work[i].x1;
                main_idx = i;
            }
        }
        bool main_sampled = false;
        float main_ear = 0, main_mar = 0, main_eyep = -1.f;

        /* 2..5) PFLD on every known face, using the current frame */
        if (job & JOB_PFLD) {
            for (int i = 0; i < work_n; i++) {
                face_result_t &r = work[i];
                std::vector<int> crop = make_crop(r, s_width, s_height);
                if (crop[2] - crop[0] < 2 || crop[3] - crop[1] < 2) {
                    continue;
                }

                int64_t t0 = esp_timer_get_time();
                float sx = 0, sy = 0;
                if (dl::image::resize_ppa(s_img, face_img, ppa, crop, &sx, &sy) != ESP_OK || sx <= 0 || sy <= 0) {
                    ppa_fail++;
                    continue;
                }
                pfld_pre.preprocess(face_img);
                pfld->run();

                /* Dequantize + map to frame coordinates (PFLD outputs [0,1] relative to its input) */
                float cw = (float)in_w / sx, ch = (float)in_h / sy;   /* crop area covered by the PPA output */
                for (int k = 0; k < n_pts; k++) {
                    float qx, qy;
                    if (out_int8) {
                        qx = ((const int8_t *)p_out->data)[2 * k];
                        qy = ((const int8_t *)p_out->data)[2 * k + 1];
                    } else {
                        qx = ((const int16_t *)p_out->data)[2 * k];
                        qy = ((const int16_t *)p_out->data)[2 * k + 1];
                    }
                    pts[2 * k]     = crop[0] + qx * out_scale * cw;
                    pts[2 * k + 1] = crop[1] + qy * out_scale * ch;
                    r.landmarks[2 * k]     = (int16_t)lroundf(pts[2 * k]);
                    r.landmarks[2 * k + 1] = (int16_t)lroundf(pts[2 * k + 1]);
                }
                r.n_landmarks = n_pts;
                /* Ratios from the unrounded points: rounding would add noise on a few-pixel eye height */
                float ear, mar;
                if (compute_ratios(pts, n_pts, &ear, &mar)) {
                    const float a = METRIC_SMOOTHING;
                    r.ear = r.ear < 0 ? ear : a * ear + (1.f - a) * r.ear;
                    r.mar = r.mar < 0 ? mar : a * mar + (1.f - a) * r.mar;
                    if (i == main_idx) {
                        main_sampled = true;        /* calibration uses raw values, medians do the smoothing */
                        main_ear = ear;
                        main_mar = mar;
                    }
                } else {
                    r.ear = r.mar = -1.f;
                }

#if CONFIG_APP_EYE_CLASSIFIER_ENABLE
                if (eye_ok) {
                    int64_t te = esp_timer_get_time();
                    float p0 = eye_cls_run(eye_cls, pts, n_pts, 0, r.eye_box[0]);
                    float p1 = eye_cls_run(eye_cls, pts, n_pts, 1, r.eye_box[1]);
                    eye_us += esp_timer_get_time() - te;
                    eye_runs += 2;
                    if (p0 >= 0 && p1 >= 0) {
                        float p = 0.5f * (p0 + p1);     /* both eyes */
                        const float a = METRIC_SMOOTHING;
                        r.eye_closed_p = r.eye_closed_p < 0 ? p : a * p + (1.f - a) * r.eye_closed_p;
                        if (i == main_idx) {
                            main_eyep = p;
                        }
                    } else {
                        r.eye_closed_p = -1.f;
                    }
                }
#endif

                pfld_us += esp_timer_get_time() - t0;
                pfld_faces++;
            }
            pfld_runs++;
        }

        /* Session calibration, then alerts against its thresholds */
        int64_t t_now = esp_timer_get_time();
        calib_update(main_idx >= 0 ? &work[main_idx] : NULL, main_sampled, main_ear, main_mar, main_eyep, t_now);
        for (int i = 0; i < work_n; i++) {
            update_alerts(work[i], t_now);
        }
        audio_update(work, work_n, t_now);

        face_calib_t cal = s_cal.th;
        cal.state = s_cal.state;
        cal.progress = s_cal.state == FACE_CALIB_RUNNING ?
                       std::min(1.f, (float)(t_now - s_cal.start_us) / CALIB_DURATION_US) : 0.f;

        /* Publish */
        xSemaphoreTake(s_lock, portMAX_DELAY);
        memcpy(s_results, work, sizeof(face_result_t) * work_n);
        s_result_count = work_n;
        s_calib_pub = cal;
        xSemaphoreGive(s_lock);

        s_busy.store(false);

        int64_t now = esp_timer_get_time();
        if (now - stat_t0 >= STATS_PERIOD_US) {
            float secs = (now - stat_t0) / 1e6f;
            ESP_LOGI(TAG, "faces=%d | detect %.1f/s avg %.1f ms | pfld %.1f/s avg %.1f ms/face | eye avg %.2f ms/eye%s",
                     work_n, det_runs / secs, det_runs ? det_us / 1000.0 / det_runs : 0.0,
                     pfld_runs / secs, pfld_faces ? pfld_us / 1000.0 / pfld_faces : 0.0,
                     eye_runs ? eye_us / 1000.0 / eye_runs : 0.0,
                     ppa_fail ? " (PPA errors)" : "");
#if LOG_LANDMARKS
            log_landmarks(work, work_n);
#endif
            stat_t0 = now;
            det_us = pfld_us = 0;
            det_runs = pfld_runs = pfld_faces = ppa_fail = 0;
            eye_us = 0;
            eye_runs = 0;
        }
    }
}

/* Print the box and every landmark (x,y) of each face, LOG_POINTS_PER_LINE points per log line */
static void log_landmarks(const face_result_t *faces, int n)
{
    static char line[LOG_POINTS_PER_LINE * 16 + 16];   /* only called from the pipeline task */
    for (int i = 0; i < n; i++) {
        const face_result_t &f = faces[i];
        ESP_LOGI(TAG, "face %d: box=(%d,%d)-(%d,%d) score=%.2f landmarks=%d EAR=%.3f MAR=%.3f P(closed)=%.2f%s%s",
                 i, f.x1, f.y1, f.x2, f.y2, f.score, f.n_landmarks, f.ear, f.mar, f.eye_closed_p,
                 f.eyes_closed_alert ? " [EYES CLOSED]" : "", f.yawn_alert ? " [YAWN]" : "");
        for (int k = 0; k < f.n_landmarks; k += LOG_POINTS_PER_LINE) {
            int len = 0;
            int end = std::min(k + LOG_POINTS_PER_LINE, f.n_landmarks);
            for (int j = k; j < end; j++) {
                len += snprintf(line + len, sizeof(line) - len, " %d:(%d,%d)",
                                j, f.landmarks[2 * j], f.landmarks[2 * j + 1]);
            }
            ESP_LOGI(TAG, "  face %d pts%s", i, line);
        }
    }
}

/* ---------------- Public API ---------------- */

extern "C" esp_err_t face_pipeline_init(uint32_t width, uint32_t height, uint32_t pixel_format)
{
    ESP_RETURN_ON_FALSE(!s_task, ESP_ERR_INVALID_STATE, TAG, "already initialized");

    size_t bpp;
    switch (pixel_format) {
    case V4L2_PIX_FMT_RGB565:
        s_img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;
        bpp = 2;
        break;
    case V4L2_PIX_FMT_RGB565X:
        s_img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565BE;
        bpp = 2;
        break;
    default:
        ESP_LOGE(TAG, "unsupported pixel format 0x%08" PRIx32 " (need RGB565)", pixel_format);
        return ESP_ERR_NOT_SUPPORTED;
    }

    s_width = width;
    s_height = height;
    s_v4l2_fmt = pixel_format;
    s_frame_size = width * height * bpp;

    /* Read by the CPU (detector) and by the PPA DMA */
    void *buf = heap_caps_aligned_calloc(128, 1, s_frame_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    ESP_RETURN_ON_FALSE(buf, ESP_ERR_NO_MEM, TAG, "no memory for frame copy");
    s_img.data = buf;
    s_img.width = (uint16_t)width;
    s_img.height = (uint16_t)height;

    s_lock = xSemaphoreCreateMutex();
    s_draw_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_lock && s_draw_lock, ESP_ERR_NO_MEM, TAG, "no memory for mutex");

#if CONFIG_APP_FACE_PIPELINE_TASK_CORE == CONFIG_APP_HTTP_SERVER_TASK_CORE
#warning "Face pipeline and HTTP server share a core: detection and streaming will slow each other down"
#endif
    /* Dedicated core: HTTP, lwIP, Wi-Fi (esp_hosted), timers run on the other one */
    BaseType_t ok = xTaskCreatePinnedToCore(face_pipeline_task, "face_pipeline", CONFIG_APP_FACE_PIPELINE_TASK_STACK,
                                            NULL, CONFIG_APP_FACE_PIPELINE_TASK_PRIORITY, &s_task,
                                            CONFIG_APP_FACE_PIPELINE_TASK_CORE);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "failed to create task");

    s_detect_due = true;
    s_pfld_due = true;
    ESP_LOGI(TAG, "started: %" PRIu32 "x%" PRIu32 ", detector every %d frames, PFLD every %d frames",
             width, height, CONFIG_APP_FACE_DETECT_EVERY_N_FRAMES, CONFIG_APP_PFLD_EVERY_N_FRAMES);
    return ESP_OK;
}

extern "C" int face_pipeline_get_results(face_result_t *out, int max)
{
    if (!s_lock || !out || max <= 0) {
        return 0;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    int n = std::min(s_result_count, max);
    memcpy(out, s_results, sizeof(face_result_t) * n);
    xSemaphoreGive(s_lock);
    return n;
}

extern "C" void face_pipeline_get_calib(face_calib_t *out)
{
    if (!out) {
        return;
    }
    if (!s_lock) {
        memset(out, 0, sizeof(*out));
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_calib_pub;
    xSemaphoreGive(s_lock);
}

extern "C" void face_pipeline_recalibrate(void)
{
    s_recalib_req.store(true);
}

/* Hand the frame to the worker if a job is due and the worker is idle.
 * A due job that cannot be dispatched (worker busy) stays pending for the next frame. */
static void schedule(const uint8_t *frame, size_t size)
{
    if (!s_task || size < s_frame_size) {
        return;
    }

    s_frame_idx++;
    if (s_frame_idx % CONFIG_APP_FACE_DETECT_EVERY_N_FRAMES == 0) {
        s_detect_due = true;
    }
    if (s_frame_idx % CONFIG_APP_PFLD_EVERY_N_FRAMES == 0) {
        s_pfld_due = true;
    }
    if (!s_detect_due && !s_pfld_due) {
        return;
    }

    bool expected = false;
    if (!s_busy.compare_exchange_strong(expected, true)) {
        return;
    }

    uint32_t job = (s_detect_due ? JOB_DETECT : 0) | (s_pfld_due ? JOB_PFLD : 0);
    s_detect_due = false;
    s_pfld_due = false;

    memcpy(s_img.data, frame, s_frame_size);
    s_job.store(job);
    xTaskNotifyGive(s_task);
}

/* ---------------- Drawing (RGB565 in place) ---------------- */

static inline void put_pixel(uint8_t *frame, int x, int y, uint16_t rgb565)
{
    if (x < 0 || y < 0 || x >= (int)s_width || y >= (int)s_height) {
        return;
    }
    uint8_t *p = frame + ((size_t)y * s_width + x) * 2;
    if (s_v4l2_fmt == V4L2_PIX_FMT_RGB565X) {
        p[0] = rgb565 >> 8;
        p[1] = rgb565 & 0xFF;
    } else {
        p[0] = rgb565 & 0xFF;
        p[1] = rgb565 >> 8;
    }
}

static void fill_rect(uint8_t *frame, int x1, int y1, int x2, int y2, uint16_t color)
{
    x1 = std::max(x1, 0);
    y1 = std::max(y1, 0);
    x2 = std::min(x2, (int)s_width - 1);
    y2 = std::min(y2, (int)s_height - 1);
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            put_pixel(frame, x, y, color);
        }
    }
}

/* Filled disc centred at (cx, cy) */
static void fill_circle(uint8_t *frame, int cx, int cy, int r, uint16_t color)
{
    const int rr = r * r + r;   /* +r rounds the edge a little nicer than r*r */
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            if (dx * dx + dy * dy <= rr) {
                put_pixel(frame, cx + dx, cy + dy, color);
            }
        }
    }
}

/* Minimal 5x7 font: only the characters used by the overlay. Row bits, MSB (bit 4) = left column. */
static const uint8_t *glyph(char c)
{
    static const uint8_t blank[7] = {0};
    static const struct {
        char c;
        uint8_t rows[7];
    } font[] = {
        {'0', {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}}, {'1', {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}},
        {'2', {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}}, {'3', {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E}},
        {'4', {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}}, {'5', {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}},
        {'6', {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}}, {'7', {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}},
        {'8', {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}}, {'9', {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}},
        {'A', {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}}, {'C', {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}},
        {'D', {0x1C, 0x12, 0x11, 0x11, 0x11, 0x12, 0x1C}}, {'E', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}},
        {'L', {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}}, {'M', {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}},
        {'N', {0x11, 0x11, 0x19, 0x15, 0x13, 0x11, 0x11}}, {'O', {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}},
        {'R', {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}}, {'S', {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}},
        {'W', {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A}}, {'Y', {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}},
        {'.', {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C}}, {'!', {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04}},
        {'-', {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}}, {'/', {0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10}},
        {'B', {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}}, {'G', {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F}},
        {'H', {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}}, {'I', {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}},
        {'P', {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}}, {'T', {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
        {'U', {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}},
    };
    for (const auto &g : font) {
        if (g.c == c) {
            return g.rows;
        }
    }
    return blank;
}

static inline int text_width(const char *str, int scale)
{
    return (int)strlen(str) * 6 * scale - scale;
}

/* Text on a filled background box (padding = scale). (x, y) is the top-left of the box. */
static void draw_label(uint8_t *frame, int x, int y, const char *str, int scale, uint16_t fg, uint16_t bg)
{
    const int pad = scale * 2;
    const int w = text_width(str, scale) + 2 * pad, h = 7 * scale + 2 * pad;
    x = std::max(0, std::min(x, (int)s_width - w));
    y = std::max(0, std::min(y, (int)s_height - h));
    fill_rect(frame, x, y, x + w - 1, y + h - 1, bg);

    int cx = x + pad;
    for (const char *c = str; *c; c++, cx += 6 * scale) {
        const uint8_t *rows = glyph(*c);
        for (int row = 0; row < 7; row++) {
            for (int col = 0; col < 5; col++) {
                if (rows[row] & (0x10 >> col)) {
                    int px = cx + col * scale, py = y + pad + row * scale;
                    fill_rect(frame, px, py, px + scale - 1, py + scale - 1, fg);
                }
            }
        }
    }
}

extern "C" void face_pipeline_draw(uint8_t *frame)
{
    if (!s_task) {
        return;
    }

    static face_result_t faces[FACE_PIPELINE_MAX_FACES];   /* large: keep off the httpd stack */
    const uint16_t green = 0x07E0, red = 0xF800, black = 0x0000, white = 0xFFFF, orange = 0xFC00;
    const int t = BOX_THICKNESS, r = LANDMARK_RADIUS;

    xSemaphoreTake(s_draw_lock, portMAX_DELAY);
    int n = face_pipeline_get_results(faces, FACE_PIPELINE_MAX_FACES);
    face_calib_t cal;
    face_pipeline_get_calib(&cal);

    /* Calibration banner + progress bar, top-left of the frame */
    if (cal.state == FACE_CALIB_RUNNING) {
        const uint16_t blue = 0x001F;
        const char *msg = "CALIBRATING - EYES OPEN MOUTH CLOSED";
        const int s = METRIC_TEXT_SCALE, x = 8, y = 8;
        const int w = text_width(msg, s) + 4 * s, h = 7 * s + 4 * s;
        draw_label(frame, x, y, msg, s, white, blue);
        fill_rect(frame, x, y + h, x + w - 1, y + h + 5, black);
        fill_rect(frame, x, y + h, x + (int)(cal.progress * (w - 1)), y + h + 5, green);
    }
    for (int i = 0; i < n; i++) {
        const face_result_t &f = faces[i];
        const uint16_t box = f.eyes_closed_alert ? red : f.yawn_alert ? orange : green;
        fill_rect(frame, f.x1, f.y1, f.x2, f.y1 + t - 1, box);    /* top */
        fill_rect(frame, f.x1, f.y2 - t + 1, f.x2, f.y2, box);    /* bottom */
        fill_rect(frame, f.x1, f.y1, f.x1 + t - 1, f.y2, box);    /* left */
        fill_rect(frame, f.x2 - t + 1, f.y1, f.x2, f.y2, box);    /* right */

        for (int k = 0; k < f.n_landmarks; k++) {
            int x = f.landmarks[2 * k], y = f.landmarks[2 * k + 1];
            fill_circle(frame, x, y, r + LANDMARK_OUTLINE, black);
            fill_circle(frame, x, y, r, red);
        }

#if CONFIG_APP_EYE_CLASSIFIER_ENABLE && DRAW_EYE_CROPS
        for (int e = 0; e < 2; e++) {      /* axis-aligned outline of the (possibly rotated) crop */
            const int16_t *b = f.eye_box[e];
            if (b[2] > 0) {
                const uint16_t cyan = 0x07FF;
                int h = b[2] / 2, x1 = b[0] - h, y1 = b[1] - h, x2 = b[0] + h, y2 = b[1] + h;
                fill_rect(frame, x1, y1, x2, y1 + 1, cyan);
                fill_rect(frame, x1, y2 - 1, x2, y2, cyan);
                fill_rect(frame, x1, y1, x1 + 1, y2, cyan);
                fill_rect(frame, x2 - 1, y1, x2, y2, cyan);
            }
        }
#endif

        /* Alert banners stacked above the box (pushed inside the frame if the face is near the top) */
        const int banner_h = 7 * TEXT_SCALE + 4 * TEXT_SCALE + 2;
        int by = f.y1 - banner_h;
        if (f.eyes_closed_alert) {
            draw_label(frame, f.x1, by, "EYES CLOSED!", TEXT_SCALE, white, red);
            by -= banner_h;
        }
        if (f.yawn_alert) {
            draw_label(frame, f.x1, by, "YAWN!", TEXT_SCALE, black, orange);
        }

#if SHOW_METRICS
        if (f.ear >= 0 && f.mar >= 0) {
            char txt[40];
#if CONFIG_APP_EYE_CLASSIFIER_ENABLE
            /* EYE = P(closed) of the classifier / threshold, MAR current / threshold */
            if (cal.state == FACE_CALIB_READY) {
                snprintf(txt, sizeof(txt), "EYE %.2f/%.2f MAR %.2f/%.2f", f.eye_closed_p, EYE_CLOSED_P, f.mar, cal.mar_yawn);
            } else {
                snprintf(txt, sizeof(txt), "EYE %.2f MAR %.2f", f.eye_closed_p, f.mar);
            }
#else
            if (cal.state == FACE_CALIB_READY) {   /* current / threshold */
                snprintf(txt, sizeof(txt), "EAR %.2f/%.2f MAR %.2f/%.2f", f.ear, cal.ear_close, f.mar, cal.mar_yawn);
            } else {
                snprintf(txt, sizeof(txt), "EAR %.2f MAR %.2f", f.ear, f.mar);
            }
#endif
            draw_label(frame, f.x1, f.y2 + 2, txt, METRIC_TEXT_SCALE, white, black);
        }
#endif
    }
    xSemaphoreGive(s_draw_lock);
}

extern "C" void face_pipeline_submit(const uint8_t *frame, size_t size)
{
    if (s_task) {
        schedule(frame, size);
    }
}

extern "C" void face_pipeline_on_frame(uint8_t *frame, size_t size)
{
    if (!s_task) {
        return;
    }
    /* Schedule first: the worker gets a clean copy without the overlay */
    schedule(frame, size);
    face_pipeline_draw(frame);
}
