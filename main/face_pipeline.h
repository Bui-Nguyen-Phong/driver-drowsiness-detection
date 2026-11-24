/*
 * Face pipeline (runs in a background task, never blocks the stream):
 *
 *   frame ──► HumanFaceDetect (MSR+MNP)            every CONFIG_APP_FACE_DETECT_EVERY_N_FRAMES frames
 *         └─► crop (square, x FACE_EXPAND) + PPA resize ──► normalize/quantize ──► PFLD ──► landmarks
 *                                                    every CONFIG_APP_PFLD_EVERY_N_FRAMES frames
 *
 * Between runs the latest boxes / landmarks are reused, so every streamed frame gets an overlay.
 *
 * Usage, for every frame before JPEG encoding:
 *     face_pipeline_on_frame(frame, size);
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FACE_PIPELINE_MAX_FACES         4
#define FACE_PIPELINE_MAX_LANDMARKS     106

typedef struct {
    int x1, y1, x2, y2;                                 /*!< Face box in frame coordinates */
    float score;                                        /*!< Detector score */
    int n_landmarks;                                    /*!< 0 if PFLD has not run for this face yet */
    int16_t landmarks[FACE_PIPELINE_MAX_LANDMARKS * 2]; /*!< x0, y0, x1, y1, ... in frame coordinates */

    /* Drowsiness (needs a 68- or 98-point landmark layout). Thresholds come from the session calibration. */
    float ear;                                          /*!< Smoothed eye aspect ratio, < 0 if unknown */
    float mar;                                          /*!< Smoothed inner-mouth aspect ratio, < 0 if unknown */
    float eye_closed_p;                                 /*!< Smoothed P(closed) of the eye classifier, < 0 if unknown */
    int16_t eye_box[2][3];                              /*!< Eye crops fed to the classifier: cx, cy, side (0 = none) */
    int64_t eyes_closed_since_us;                       /*!< 0 = eyes open */
    int64_t mouth_open_since_us;                        /*!< 0 = mouth not wide open */
    bool eyes_closed_alert;                             /*!< Eyes closed long enough to warn */
    bool yawn_alert;                                    /*!< Mouth wide open long enough to warn */
} face_result_t;

typedef enum {
    FACE_CALIB_NONE = 0,    /*!< No session (no face) */
    FACE_CALIB_RUNNING,     /*!< Collecting the neutral-face baseline, alerts are off */
    FACE_CALIB_READY,       /*!< Thresholds valid for this session */
} face_calib_state_t;

typedef struct {
    face_calib_state_t state;
    float progress;                 /*!< 0..1 while FACE_CALIB_RUNNING */
    float ear_base, mar_base;       /*!< Neutral face: eyes open, mouth closed (medians) */
    float ear_close, ear_open;      /*!< Eyes closed below ear_close, open again above ear_open */
    float mar_yawn, mar_end;        /*!< Yawn above mar_yawn, over below mar_end */
} face_calib_t;

/**
 * @brief Load the models, allocate buffers and start the pipeline task.
 *
 * @param width        Frame width
 * @param height       Frame height
 * @param pixel_format V4L2 pixel format (RGB565 / RGB565X supported)
 */
esp_err_t face_pipeline_init(uint32_t width, uint32_t height, uint32_t pixel_format);

/**
 * @brief Call once per captured frame: schedules detector/PFLD jobs according to the frame counters
 *        and draws the latest results (boxes + landmarks) into the frame in place.
 */
void face_pipeline_on_frame(uint8_t *frame, size_t size);

/**
 * @brief Schedule only (no drawing): call once per camera frame, the frame is not modified.
 */
void face_pipeline_submit(const uint8_t *frame, size_t size);

/**
 * @brief Only draw the latest results into the frame (no scheduling). For snapshots.
 */
void face_pipeline_draw(uint8_t *frame);

/**
 * @brief Copy the latest results.
 *
 * @return Number of faces written (<= max)
 */
int face_pipeline_get_results(face_result_t *out, int max);

/** @brief Copy of the current session calibration (thresholds + state) */
void face_pipeline_get_calib(face_calib_t *out);

/** @brief Restart the calibration now (the person should look at the camera, eyes open, mouth closed) */
void face_pipeline_recalibrate(void);

#ifdef __cplusplus
}
#endif
