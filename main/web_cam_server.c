/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/errno.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include <stdatomic.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "app_camera.h"
#include "web_cam_server.h"
#if CONFIG_APP_FACE_PIPELINE_ENABLE
#include "face_pipeline.h"
#endif

/*
 * Frame flow (camera and face pipeline no longer depend on a browser being connected):
 *
 *   capture task (owns the V4L2 queue, every camera frame)
 *     ├─► face_pipeline_submit()      clean copy for the worker when a job is due
 *     └─► only if a client waits:     overlay + HW JPEG ─► latest JPEG (seq++) ─► wake clients
 *   /stream, /api/capture_image       wait for the next JPEG, copy it, send it (no camera access)
 *   /api/capture_binary               ask for one raw (overlay-free) frame
 */
#define CAPTURE_TASK_STACK      (6 * 1024)
#define CAPTURE_STATS_PERIOD_US (5 * 1000 * 1000)
#define EVT_NEW_JPEG            BIT0
#define EVT_NEW_RAW             BIT1
#define CLIENT_WAIT_MS          1000

#define EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER  CONFIG_EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER

#define EXAMPLE_JPEG_ENC_QUALITY            CONFIG_EXAMPLE_JPEG_COMPRESSION_QUALITY

#define EXAMPLE_PART_BOUNDARY               CONFIG_EXAMPLE_HTTP_PART_BOUNDARY

static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" EXAMPLE_PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" EXAMPLE_PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_html_gz_end[] asm("_binary_index_html_gz_end");
extern const uint8_t loading_jpg_gz_start[] asm("_binary_loading_jpg_gz_start");
extern const uint8_t loading_jpg_gz_end[] asm("_binary_loading_jpg_gz_end");
extern const uint8_t favicon_ico_gz_start[] asm("_binary_favicon_ico_gz_start");
extern const uint8_t favicon_ico_gz_end[] asm("_binary_favicon_ico_gz_end");
extern const uint8_t assets_index_js_gz_start[] asm("_binary_index_js_gz_start");
extern const uint8_t assets_index_js_gz_end[] asm("_binary_index_js_gz_end");
extern const uint8_t assets_index_css_gz_start[] asm("_binary_index_css_gz_start");
extern const uint8_t assets_index_css_gz_end[] asm("_binary_index_css_gz_end");

/**
 * @brief Web cam control structure
 */
typedef struct web_cam_video {
    int fd;
    uint8_t index;

    app_jpeg_handle_t encoder_handle;
    uint8_t *jpeg_out_buf;
    uint32_t jpeg_out_size;

    uint8_t *buffer[EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER];
    uint32_t buffer_size;

    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint8_t jpeg_quality;

    uint32_t frame_rate;

    SemaphoreHandle_t sem;          /* JPEG encoder + jpeg_out_buf/jpeg_len/jpeg_seq */
    EventGroupHandle_t evt;         /* EVT_NEW_JPEG / EVT_NEW_RAW broadcasts */
    TaskHandle_t capture_task;

    uint32_t jpeg_len;              /* latest encoded frame (overlay included) */
    volatile uint32_t jpeg_seq;
    atomic_int jpeg_waiters;        /* clients waiting: capture encodes only when > 0 */

    SemaphoreHandle_t raw_lock;     /* one raw snapshot at a time */
    uint8_t *raw_buf;
    uint32_t raw_len;
    atomic_bool raw_wanted;

    float cam_fps;                  /* measured by the capture task */
    float jpeg_fps;
} web_cam_video_t;

typedef struct web_cam {
    uint8_t video_count;
    web_cam_video_t video[0];
} web_cam_t;

typedef struct web_cam_video_config {
    const char *dev_name;
    uint32_t buffer_count;
} web_cam_video_config_t;

typedef struct request_desc {
    int index;
} request_desc_t;

static const char *TAG = "web_cam_server";

static bool is_valid_web_cam(web_cam_video_t *video)
{
    return video->fd != -1;
}

static esp_err_t decode_request(web_cam_t *web_cam, httpd_req_t *req, request_desc_t *desc)
{
    esp_err_t ret;
    int index = -1;
    char buffer[32];

    if ((ret = httpd_req_get_url_query_str(req, buffer, sizeof(buffer))) != ESP_OK) {
        return ret;
    }
    ESP_LOGD(TAG, "source: %s", buffer);

    for (int i = 0; i < web_cam->video_count; i++) {
        char source_str[16];

        if (snprintf(source_str, sizeof(source_str), "source=%d", i) <= 0) {
            return ESP_FAIL;
        }

        if (strcmp(buffer, source_str) == 0) {
            index = i;
            break;
        }
    }
    if (index == -1) {
        return ESP_ERR_INVALID_ARG;
    }

    desc->index = index;
    return ESP_OK;
}

/* Wait for a JPEG newer than *seq and copy it into dst (size cap). Returns the length, 0 on timeout. */
static uint32_t wait_next_jpeg(web_cam_video_t *video, uint32_t *seq, uint8_t *dst, uint32_t cap)
{
    uint32_t len = 0;
    atomic_fetch_add(&video->jpeg_waiters, 1);
    const int64_t deadline = esp_timer_get_time() + CLIENT_WAIT_MS * 1000LL;
    while (video->jpeg_seq == *seq && esp_timer_get_time() < deadline) {
        /* a frame encoded between the check and the wait costs at most one camera frame */
        xEventGroupWaitBits(video->evt, EVT_NEW_JPEG, pdFALSE, pdFALSE, pdMS_TO_TICKS(50));
    }
    atomic_fetch_sub(&video->jpeg_waiters, 1);

    xSemaphoreTake(video->sem, portMAX_DELAY);
    if (video->jpeg_seq != *seq && video->jpeg_len <= cap) {
        memcpy(dst, video->jpeg_out_buf, video->jpeg_len);
        len = video->jpeg_len;
        *seq = video->jpeg_seq;
    }
    xSemaphoreGive(video->sem);
    return len;
}

static esp_err_t capture_video_image(httpd_req_t *req, web_cam_video_t *video, bool is_jpeg)
{
    esp_err_t ret = ESP_OK;

    if (!is_jpeg) {
        /* Raw frame (no overlay), handed over by the capture task */
        xSemaphoreTake(video->raw_lock, portMAX_DELAY);
        if (!video->raw_buf) {
            video->raw_buf = heap_caps_malloc(video->buffer_size, MALLOC_CAP_SPIRAM);
        }
        ESP_GOTO_ON_FALSE(video->raw_buf, ESP_ERR_NO_MEM, raw_out, TAG, "no memory for raw frame");
        xEventGroupClearBits(video->evt, EVT_NEW_RAW);
        atomic_store(&video->raw_wanted, true);
        EventBits_t bits = xEventGroupWaitBits(video->evt, EVT_NEW_RAW, pdTRUE, pdFALSE, pdMS_TO_TICKS(CLIENT_WAIT_MS));
        ESP_GOTO_ON_FALSE(bits & EVT_NEW_RAW, ESP_ERR_TIMEOUT, raw_out, TAG, "no raw frame from capture task");
        ESP_GOTO_ON_ERROR(httpd_resp_send(req, (char *)video->raw_buf, video->raw_len), raw_out, TAG, "failed to send binary");
raw_out:
        atomic_store(&video->raw_wanted, false);
        xSemaphoreGive(video->raw_lock);
        return ret;
    }

    uint8_t *jpg = heap_caps_malloc(video->jpeg_out_size, MALLOC_CAP_SPIRAM);
    ESP_RETURN_ON_FALSE(jpg, ESP_ERR_NO_MEM, TAG, "no memory for snapshot");
    uint32_t seq = video->jpeg_seq;
    uint32_t len = wait_next_jpeg(video, &seq, jpg, video->jpeg_out_size);
    if (len) {
        ret = httpd_resp_send(req, (char *)jpg, len);
    } else {
        ESP_LOGW(TAG, "snapshot: no frame from capture task");
        ret = httpd_resp_send_500(req);
    }
    free(jpg);
    return ret;
}

static char *get_cameras_json(web_cam_t *web_cam)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *cameras = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "cameras", cameras);

    for (int i = 0; i < web_cam->video_count; i++) {
        char src_str[32];

        if (!is_valid_web_cam(&web_cam->video[i])) {
            continue;
        }

        cJSON *camera = cJSON_CreateObject();
        cJSON_AddNumberToObject(camera, "index", i);
        assert(snprintf(src_str, sizeof(src_str), ":%d/stream", i + 81) > 0);
        cJSON_AddStringToObject(camera, "src", src_str);
        cJSON_AddNumberToObject(camera, "currentFrameRate", web_cam->video[i].frame_rate);
        cJSON_AddNumberToObject(camera, "currentImageFormat", 0);
        assert(snprintf(src_str, sizeof(src_str), "JPEG %" PRIu32 "x%" PRIu32, web_cam->video[i].width, web_cam->video[i].height) > 0);
        cJSON_AddStringToObject(camera, "currentImageFormatDescription", src_str);

        cJSON_AddNumberToObject(camera, "currentQuality", web_cam->video[i].jpeg_quality);

        cJSON *current_resolution = cJSON_CreateObject();
        cJSON_AddNumberToObject(current_resolution, "width", web_cam->video[i].width);
        cJSON_AddNumberToObject(current_resolution, "height", web_cam->video[i].height);
        cJSON_AddItemToObject(camera, "currentResolution", current_resolution);

        cJSON *image_formats = cJSON_CreateArray();
        cJSON *image_format = cJSON_CreateObject();
        cJSON_AddNumberToObject(image_format, "id", 0);
        assert(snprintf(src_str, sizeof(src_str), "JPEG %" PRIu32 "x%" PRIu32, web_cam->video[i].width, web_cam->video[i].height) > 0);
        cJSON_AddStringToObject(image_format, "description", src_str);

        cJSON *image_format_quality = cJSON_CreateObject();
        cJSON_AddNumberToObject(image_format_quality, "min", 1);
        cJSON_AddNumberToObject(image_format_quality, "max", 100);
        cJSON_AddNumberToObject(image_format_quality, "step", 1);
        cJSON_AddNumberToObject(image_format_quality, "default", EXAMPLE_JPEG_ENC_QUALITY);
        cJSON_AddItemToObject(image_format, "quality", image_format_quality);
        cJSON_AddItemToArray(image_formats, image_format);

        cJSON_AddItemToObject(camera, "imageFormats", image_formats);
        cJSON_AddItemToArray(cameras, camera);
    }

    char *output = cJSON_Print(root);
    cJSON_Delete(root);
    return output;
}

static esp_err_t set_camera_jpeg_quality(web_cam_video_t *video, int quality)
{
    if (quality < 1) {
        quality = 1;
    } else if (quality > 100) {
        quality = 100;
    }

    xSemaphoreTake(video->sem, portMAX_DELAY);
    esp_err_t err = app_jpeg_set_quality(video->encoder_handle, quality);
    xSemaphoreGive(video->sem);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to set jpeg quality");
    video->jpeg_quality = quality;
    ESP_LOGI(TAG, "video%d: set jpeg quality %d success", video->index, quality);
    return ESP_OK;
}

static esp_err_t camera_info_handler(httpd_req_t *req)
{
    esp_err_t ret;
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;
    char *output = get_cameras_json(web_cam);

    httpd_resp_set_type(req, "application/json");
    ret = httpd_resp_sendstr(req, output);
    free(output);

    return ret;
}

#if CONFIG_APP_FACE_PIPELINE_ENABLE
/* GET /api/recalibrate: restart the drowsiness calibration (look at the camera, eyes open, mouth closed) */
static esp_err_t recalibrate_handler(httpd_req_t *req)
{
    face_pipeline_recalibrate();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"calibrating\"}");
}
#endif

static esp_err_t camera_settings_handler(httpd_req_t *req)
{
    esp_err_t ret;
    char *content;
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;

    content = (char *)calloc(1, req->content_len + 1);
    ESP_RETURN_ON_FALSE(content, ESP_ERR_NO_MEM, TAG, "failed to allocate memory");

    ESP_GOTO_ON_FALSE(httpd_req_recv(req, content, req->content_len) > 0, ESP_FAIL, fail0, TAG, "failed to recv content");
    ESP_LOGD(TAG, "content: %s", content);

    cJSON *json_root = cJSON_Parse(content);
    free(content);
    content = NULL;
    ESP_GOTO_ON_FALSE(json_root, ESP_FAIL, fail0, TAG, "failed to parse JSON");

    cJSON *json_index = cJSON_GetObjectItem(json_root, "index");
    ESP_GOTO_ON_FALSE(json_index && cJSON_IsNumber(json_index), ESP_ERR_INVALID_ARG, fail1, TAG, "missing or invalid index field");
    int index = json_index->valueint;
    ESP_GOTO_ON_FALSE(index >= 0 && index < web_cam->video_count && is_valid_web_cam(&web_cam->video[index]), ESP_ERR_INVALID_ARG, fail1, TAG, "invalid index");

    cJSON *json_image_format = cJSON_GetObjectItem(json_root, "image_format");
    ESP_GOTO_ON_FALSE(json_image_format && cJSON_IsNumber(json_image_format), ESP_ERR_INVALID_ARG, fail1, TAG, "missing or invalid image_format field");
    int image_format = json_image_format->valueint;

    cJSON *json_jpeg_quality = cJSON_GetObjectItem(json_root, "jpeg_quality");
    ESP_GOTO_ON_FALSE(json_jpeg_quality && cJSON_IsNumber(json_jpeg_quality), ESP_ERR_INVALID_ARG, fail1, TAG, "missing or invalid jpeg_quality field");
    int jpeg_quality = json_jpeg_quality->valueint;

    ESP_LOGI(TAG, "JSON parse success - index:%d, image_format:%d, jpeg_quality:%d", index, image_format, jpeg_quality);
    cJSON_Delete(json_root);
    json_root = NULL;

    ESP_GOTO_ON_ERROR(set_camera_jpeg_quality(&web_cam->video[index], jpeg_quality), fail1, TAG, "failed to set camera jpeg quality");

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;

fail1:
    if (json_root) {
        cJSON_Delete(json_root);
    }
fail0:
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
        httpd_resp_send_408(req);
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON format");
    }
    if (content) {
        free(content);
    }
    return ret;
}

static esp_err_t static_file_handler(httpd_req_t *req)
{
    const char *uri = req->uri;

    /* Route to appropriate static file based on URI */
    if (strcmp(uri, "/") == 0) {
        httpd_resp_set_type(req, "text/html");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)index_html_gz_start, index_html_gz_end - index_html_gz_start);
    } else if (strcmp(uri, "/loading.jpg") == 0) {
        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)loading_jpg_gz_start, loading_jpg_gz_end - loading_jpg_gz_start);
    } else if (strcmp(uri, "/favicon.ico") == 0) {
        httpd_resp_set_type(req, "image/x-icon");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)favicon_ico_gz_start, favicon_ico_gz_end - favicon_ico_gz_start);
    } else if (strcmp(uri, "/assets/index.js") == 0) {
        httpd_resp_set_type(req, "application/javascript");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)assets_index_js_gz_start, assets_index_js_gz_end - assets_index_js_gz_start);
    } else if (strcmp(uri, "/assets/index.css") == 0) {
        httpd_resp_set_type(req, "text/css");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)assets_index_css_gz_start, assets_index_css_gz_end - assets_index_css_gz_start);
    }

    /* If no static file matches, return 404 */
    ESP_LOGW(TAG, "File not found: %s", uri);
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

static esp_err_t image_stream_handler(httpd_req_t *req)
{
    esp_err_t ret = ESP_OK;
    char http_string[128];
    web_cam_video_t *video = (web_cam_video_t *)req->user_ctx;

    ESP_RETURN_ON_FALSE(snprintf(http_string, sizeof(http_string), "%" PRIu32, video->frame_rate) > 0,
                        ESP_FAIL, TAG, "failed to format framerate buffer");

    ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, STREAM_CONTENT_TYPE), TAG, "failed to set content type");
    ESP_RETURN_ON_ERROR(httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"), TAG, "failed to set access control allow origin");
    ESP_RETURN_ON_ERROR(httpd_resp_set_hdr(req, "X-Framerate", http_string), TAG, "failed to set x framerate");

    /* Private copy so the capture task can encode the next frame while this one is on the wire */
    uint8_t *jpg = heap_caps_malloc(video->jpeg_out_size, MALLOC_CAP_SPIRAM);
    ESP_RETURN_ON_FALSE(jpg, ESP_ERR_NO_MEM, TAG, "no memory for stream buffer");
    uint32_t seq = video->jpeg_seq;
    ESP_LOGI(TAG, "stream client connected");

    while (1) {
        int hlen;
        struct timespec ts;

        uint32_t len = wait_next_jpeg(video, &seq, jpg, video->jpeg_out_size);
        if (!len) {
            continue;   /* camera stalled: keep the connection, try again */
        }

        ESP_GOTO_ON_ERROR(httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY)), out, TAG, "failed to send boundary");
        ESP_GOTO_ON_ERROR(clock_gettime(CLOCK_MONOTONIC, &ts), out, TAG, "failed to get time");
        ESP_GOTO_ON_FALSE((hlen = snprintf(http_string, sizeof(http_string), STREAM_PART, (unsigned)len, (int)ts.tv_sec, (int)(ts.tv_nsec / 1000))) > 0,
                          ESP_FAIL, out, TAG, "failed to format part buffer");
        ESP_GOTO_ON_ERROR(httpd_resp_send_chunk(req, http_string, hlen), out, TAG, "failed to send part header");
        ESP_GOTO_ON_ERROR(httpd_resp_send_chunk(req, (char *)jpg, len), out, TAG, "failed to send jpeg");
    }

out:
    ESP_LOGI(TAG, "stream client disconnected");
    free(jpg);
    return ret;
}

static esp_err_t capture_image_handler(httpd_req_t *req)
{
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;

    request_desc_t desc;
    ESP_RETURN_ON_ERROR(decode_request(web_cam, req, &desc), TAG, "failed to decode request");

    char type_ptr[32];
    ESP_RETURN_ON_FALSE(snprintf(type_ptr, sizeof(type_ptr), "image/jpeg;name=image%d.jpg", desc.index) > 0, ESP_FAIL, TAG, "failed to format buffer");
    ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, type_ptr), TAG, "failed to set content type");

    return capture_video_image(req, &web_cam->video[desc.index], true);
}

static esp_err_t capture_binary_handler(httpd_req_t *req)
{
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;

    request_desc_t desc;
    ESP_RETURN_ON_ERROR(decode_request(web_cam, req, &desc), TAG, "failed to decode request");

    char type_ptr[56];
    ESP_RETURN_ON_FALSE(snprintf(type_ptr, sizeof(type_ptr), "application/octet-stream;name=image_binary%d.bin", desc.index) > 0, ESP_FAIL, TAG, "failed to format buffer");
    ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, type_ptr), TAG, "failed to set content type");

    return capture_video_image(req, &web_cam->video[desc.index], false);
}

/* Owns the camera queue: runs at camera rate whether or not anybody is watching */
static void capture_task(void *arg)
{
    web_cam_video_t *video = (web_cam_video_t *)arg;
    struct v4l2_buffer buf;
    int64_t t0 = esp_timer_get_time(), enc_us = 0;
    int frames = 0, encoded = 0;

    while (1) {
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(video->fd, VIDIOC_DQBUF, &buf) != 0) {
            ESP_LOGE(TAG, "capture: DQBUF failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (buf.flags & V4L2_BUF_FLAG_DONE) {
            uint8_t *frame = video->buffer[buf.index];
            frames++;

            /* 1) raw snapshot request: clean frame, before anything is drawn */
            if (atomic_load(&video->raw_wanted) && video->raw_buf) {
                memcpy(video->raw_buf, frame, buf.bytesused);
                video->raw_len = buf.bytesused;
                atomic_store(&video->raw_wanted, false);
                xEventGroupSetBits(video->evt, EVT_NEW_RAW);
            }

#if CONFIG_APP_FACE_PIPELINE_ENABLE
            /* 2) face pipeline: copies the clean frame when a job is due and the worker is idle */
            face_pipeline_submit(frame, buf.bytesused);
#endif

            /* 3) overlay + JPEG only when a stream/snapshot client is waiting */
            if (atomic_load(&video->jpeg_waiters) > 0) {
#if CONFIG_APP_FACE_PIPELINE_ENABLE
                face_pipeline_draw(frame);
#endif
                int64_t te = esp_timer_get_time();
                uint32_t len = 0;
                xSemaphoreTake(video->sem, portMAX_DELAY);
                esp_err_t err = app_jpeg_process(video->encoder_handle, frame, video->buffer_size,
                                                 video->jpeg_out_buf, video->jpeg_out_size, &len);
                if (err == ESP_OK) {
                    video->jpeg_len = len;
                    video->jpeg_seq++;
                }
                xSemaphoreGive(video->sem);
                enc_us += esp_timer_get_time() - te;
                if (err == ESP_OK) {
                    encoded++;
                    /* broadcast to every waiting client */
                    xEventGroupSetBits(video->evt, EVT_NEW_JPEG);
                    xEventGroupClearBits(video->evt, EVT_NEW_JPEG);
                } else {
                    ESP_LOGW(TAG, "capture: JPEG encode failed: %s", esp_err_to_name(err));
                }
            }
        }
        ioctl(video->fd, VIDIOC_QBUF, &buf);

        int64_t now = esp_timer_get_time();
        if (now - t0 >= CAPTURE_STATS_PERIOD_US) {
            float secs = (now - t0) / 1e6f;
            video->cam_fps = frames / secs;
            video->jpeg_fps = encoded / secs;
            ESP_LOGI(TAG, "capture: camera %.1f fps | stream jpeg %.1f fps (encode avg %.1f ms) | clients %d",
                     video->cam_fps, video->jpeg_fps, encoded ? enc_us / 1000.0 / encoded : 0.0,
                     atomic_load(&video->jpeg_waiters));
            t0 = now;
            frames = encoded = 0;
            enc_us = 0;
        }
    }
}

static esp_err_t init_web_cam_video(web_cam_video_t *video, const web_cam_video_config_t *config, int index)
{
    int fd;
    int ret;
    struct v4l2_format format;
    struct v4l2_streamparm sparm;
    struct v4l2_requestbuffers req;
    struct v4l2_captureparm *cparam = &sparm.parm.capture;
    struct v4l2_fract *timeperframe = &cparam->timeperframe;

    fd = open(config->dev_name, O_RDWR);
    ESP_RETURN_ON_FALSE(fd >= 0, ESP_ERR_NOT_FOUND, TAG, "Open video device %s failed", config->dev_name);

    memset(&format, 0, sizeof(struct v4l2_format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_G_FMT, &format), fail0, TAG, "Failed get fmt from %s", config->dev_name);

    if (format.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB565X) {
        /* The hardware JPEG encoder only accepts RGB565 little endian */
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_S_FMT, &format), fail0, TAG, "failed to set fmt to %s", config->dev_name);
    }

    memset(&sparm, 0, sizeof(sparm));
    sparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_G_PARM, &sparm), fail0, TAG, "failed to get frame rate from %s", config->dev_name);
    video->frame_rate = timeperframe->denominator / timeperframe->numerator;

    memset(&req, 0, sizeof(req));
    req.count  = EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_REQBUFS, &req), fail0, TAG, "failed to req buffers from %s", config->dev_name);

    for (int i = 0; i < EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER; i++) {
        struct v4l2_buffer buf;

        memset(&buf, 0, sizeof(buf));
        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;
        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_QUERYBUF, &buf), fail0, TAG, "failed to query vbuf from %s", config->dev_name);

        video->buffer[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        ESP_GOTO_ON_FALSE(video->buffer[i] != MAP_FAILED, ESP_ERR_NO_MEM, fail0, TAG, "failed to mmap buffer");
        video->buffer_size = buf.length;

        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_QBUF, &buf), fail0, TAG, "failed to queue frame vbuf from %s", config->dev_name);
    }

    video->fd = fd;
    video->width = format.fmt.pix.width;
    video->height = format.fmt.pix.height;
    video->pixel_format = format.fmt.pix.pixelformat;
    video->jpeg_quality = EXAMPLE_JPEG_ENC_QUALITY;

    ESP_GOTO_ON_ERROR(app_jpeg_init(video->width, video->height, video->pixel_format, EXAMPLE_JPEG_ENC_QUALITY,
                                    &video->encoder_handle), fail0, TAG, "failed to init encoder");
    ESP_GOTO_ON_ERROR(app_jpeg_alloc_output_buffer(video->encoder_handle, &video->jpeg_out_buf, &video->jpeg_out_size),
                      fail1, TAG, "failed to alloc jpeg output buf");

#if CONFIG_APP_FACE_PIPELINE_ENABLE
    if (face_pipeline_init(video->width, video->height, video->pixel_format) != ESP_OK) {
        ESP_LOGW(TAG, "face pipeline disabled for %s", config->dev_name);
    }
#endif

    video->sem = xSemaphoreCreateMutex();
    video->raw_lock = xSemaphoreCreateMutex();
    video->evt = xEventGroupCreate();
    ESP_GOTO_ON_FALSE(video->sem && video->raw_lock && video->evt, ESP_ERR_NO_MEM, fail2, TAG, "failed to create sync objects");
    atomic_init(&video->jpeg_waiters, 0);
    atomic_init(&video->raw_wanted, false);

    return ESP_OK;

fail2:
    if (video->sem) {
        vSemaphoreDelete(video->sem);
        video->sem = NULL;
    }
    if (video->raw_lock) {
        vSemaphoreDelete(video->raw_lock);
        video->raw_lock = NULL;
    }
    if (video->evt) {
        vEventGroupDelete(video->evt);
        video->evt = NULL;
    }
    app_jpeg_deinit(video->encoder_handle, video->jpeg_out_buf);
    video->jpeg_out_buf = NULL;
    video->encoder_handle = NULL;
    goto fail0;
fail1:
    app_jpeg_deinit(video->encoder_handle, NULL);
    video->encoder_handle = NULL;
fail0:
    close(fd);
    video->fd = -1;
    return ret;
}

static esp_err_t deinit_web_cam_video(web_cam_video_t *video)
{
    if (video->sem) {
        vSemaphoreDelete(video->sem);
        video->sem = NULL;
    }

    app_jpeg_deinit(video->encoder_handle, video->jpeg_out_buf);
    video->jpeg_out_buf = NULL;
    video->encoder_handle = NULL;

    close(video->fd);
    return ESP_OK;
}

static esp_err_t new_web_cam(const web_cam_video_config_t *config, int config_count, web_cam_t **ret_wc)
{
    int i;
    web_cam_t *wc;
    esp_err_t ret = ESP_FAIL;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    wc = calloc(1, sizeof(web_cam_t) + config_count * sizeof(web_cam_video_t));
    ESP_RETURN_ON_FALSE(wc, ESP_ERR_NO_MEM, TAG, "failed to alloc web cam");
    wc->video_count = config_count;

    for (i = 0; i < config_count; i++) {
        wc->video[i].index = i;
        wc->video[i].fd = -1;

        ret = init_web_cam_video(&wc->video[i], &config[i], i);
        if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "failed to find web_cam %d", i);
            continue;
        } else if (ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to initialize web_cam %d", i);
            goto fail0;
        }

        ESP_LOGI(TAG, "video%d: width=%" PRIu32 " height=%" PRIu32 " format=" V4L2_FMT_STR, i, wc->video[i].width,
                 wc->video[i].height, V4L2_FMT_STR_ARG(wc->video[i].pixel_format));
    }

    for (i = 0; i < config_count; i++) {
        if (is_valid_web_cam(&wc->video[i])) {
            ESP_GOTO_ON_ERROR(ioctl(wc->video[i].fd, VIDIOC_STREAMON, &type), fail1, TAG, "failed to start stream");
        }
    }

    for (i = 0; i < config_count; i++) {
        if (!is_valid_web_cam(&wc->video[i])) {
            continue;
        }
        BaseType_t ok = xTaskCreatePinnedToCore(capture_task, "cam_capture", CAPTURE_TASK_STACK, &wc->video[i],
                                                CONFIG_APP_CAPTURE_TASK_PRIORITY, &wc->video[i].capture_task,
                                                CONFIG_APP_HTTP_SERVER_TASK_CORE);
        ESP_GOTO_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, fail1, TAG, "failed to create capture task");
    }
    i = config_count;

    *ret_wc = wc;

    return ESP_OK;

fail1:
    for (int j = i - 1; j >= 0; j--) {
        if (is_valid_web_cam(&wc->video[j])) {
            ioctl(wc->video[j].fd, VIDIOC_STREAMOFF, &type);
        }
    }
    i = config_count; // deinit all web_cam
fail0:
    for (int j = i - 1; j >= 0; j--) {
        if (is_valid_web_cam(&wc->video[j])) {
            deinit_web_cam_video(&wc->video[j]);
        }
    }
    free(wc);
    return ret;
}

static esp_err_t http_server_init(web_cam_t *web_cam)
{
    httpd_handle_t stream_httpd;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    /* Keep all HTTP work (frame dequeue, copy for the pipeline, overlay, JPEG, send) off the pipeline core */
    config.core_id = CONFIG_APP_HTTP_SERVER_TASK_CORE;
    config.task_priority = CONFIG_APP_HTTP_SERVER_TASK_PRIORITY;

    /* Unified static file handler for all static resources */
    httpd_uri_t static_file_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = static_file_handler,
        .user_ctx = (void *)web_cam
    };

    /* API handlers */
    httpd_uri_t capture_image_uri = {
        .uri = "/api/capture_image",
        .method = HTTP_GET,
        .handler = capture_image_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t capture_binary_uri = {
        .uri = "/api/capture_binary",
        .method = HTTP_GET,
        .handler = capture_binary_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t camera_info_uri = {
        .uri = "/api/get_camera_info",
        .method = HTTP_GET,
        .handler = camera_info_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t camera_settings_uri = {
        .uri = "/api/set_camera_config",
        .method = HTTP_POST,
        .handler = camera_settings_handler,
        .user_ctx = (void *)web_cam
    };

#if CONFIG_APP_FACE_PIPELINE_ENABLE
    httpd_uri_t recalibrate_uri = {
        .uri = "/api/recalibrate",
        .method = HTTP_GET,
        .handler = recalibrate_handler,
        .user_ctx = NULL
    };
#endif

    config.stack_size = 1024 * 6;
    ESP_LOGI(TAG, "Starting stream server on port: '%d'", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        /* Register API handlers (more specific URIs) */
        httpd_register_uri_handler(stream_httpd, &capture_image_uri);
        httpd_register_uri_handler(stream_httpd, &capture_binary_uri);
        httpd_register_uri_handler(stream_httpd, &camera_info_uri);
        httpd_register_uri_handler(stream_httpd, &camera_settings_uri);
#if CONFIG_APP_FACE_PIPELINE_ENABLE
        httpd_register_uri_handler(stream_httpd, &recalibrate_uri);
#endif

        /* Register wildcard static file handler to catch all other requests */
        httpd_register_uri_handler(stream_httpd, &static_file_uri);
    }

    for (int i = 0; i < web_cam->video_count; i++) {
        if (!is_valid_web_cam(&web_cam->video[i])) {
            continue;
        }

        httpd_uri_t stream_0_uri = {
            .uri = "/stream",
            .method = HTTP_GET,
            .handler = image_stream_handler,
            .user_ctx = (void *) &web_cam->video[i]
        };

        config.stack_size = 1024 * 6;
        config.server_port += 1;
        config.ctrl_port += 1;
        if (httpd_start(&stream_httpd, &config) == ESP_OK) {
            httpd_register_uri_handler(stream_httpd, &stream_0_uri);
        }
    }

    return ESP_OK;
}

static web_cam_t *s_web_cam;

esp_err_t web_cam_capture_start(const char *dev_name)
{
    if (s_web_cam) {
        return ESP_OK;
    }
    const web_cam_video_config_t config[] = {
        {
            .dev_name = dev_name,
        },
    };

    ESP_RETURN_ON_FALSE(dev_name, ESP_ERR_INVALID_ARG, TAG, "invalid device name");
    ESP_RETURN_ON_ERROR(new_web_cam(config, sizeof(config) / sizeof(config[0]), &s_web_cam), TAG, "Failed to new web cam");
    ESP_LOGI(TAG, "camera capture running (face pipeline active without a browser)");
    return ESP_OK;
}

esp_err_t web_cam_server_start(const char *dev_name)
{
    ESP_RETURN_ON_ERROR(web_cam_capture_start(dev_name), TAG, "Failed to start capture");
    ESP_RETURN_ON_ERROR(http_server_init(s_web_cam), TAG, "Failed to init http server");
    ESP_LOGI(TAG, "Camera web server starts");
    return ESP_OK;
}
