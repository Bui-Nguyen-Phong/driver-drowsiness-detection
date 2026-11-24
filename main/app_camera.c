/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 *
 * OV5647 (NoIR) on MIPI-CSI + ESP32-P4 hardware JPEG encoder.
 */

#include <stdlib.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/jpeg_encode.h"
#include "driver/i2c_master.h"
#include "app_camera.h"

static const char *TAG = "app_camera";

/* ------------------------------------------------------------------------- */
/* Camera                                                                    */
/* ------------------------------------------------------------------------- */

/* The I2C bus is created here (not by esp_video) because the ES8311 audio
 * codec sits on the same SDA/SCL pins; app_camera_get_i2c_bus() shares it. */
static i2c_master_bus_handle_t s_i2c_bus;
static esp_video_init_csi_config_t s_csi_config;
static esp_video_init_config_t s_cam_config;
static bool s_cam_inited;

i2c_master_bus_handle_t app_camera_get_i2c_bus(void)
{
    return s_i2c_bus;
}

esp_err_t app_camera_init(void)
{
    if (s_cam_inited) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "OV5647 SCCB: I2C port=%d, scl=%d, sda=%d, freq=%d",
             CONFIG_APP_OV5647_SCCB_I2C_PORT, CONFIG_APP_OV5647_SCCB_I2C_SCL_PIN,
             CONFIG_APP_OV5647_SCCB_I2C_SDA_PIN, CONFIG_APP_OV5647_SCCB_I2C_FREQ);

    if (!s_i2c_bus) {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = CONFIG_APP_OV5647_SCCB_I2C_PORT,
            .sda_io_num = CONFIG_APP_OV5647_SCCB_I2C_SDA_PIN,
            .scl_io_num = CONFIG_APP_OV5647_SCCB_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_i2c_bus), TAG, "failed to create I2C bus");
    }

    s_csi_config.sccb_config.init_sccb = false;
    s_csi_config.sccb_config.i2c_handle = s_i2c_bus;
    s_csi_config.sccb_config.freq = CONFIG_APP_OV5647_SCCB_I2C_FREQ;
    s_csi_config.reset_pin = CONFIG_APP_OV5647_RESET_PIN;
    s_csi_config.pwdn_pin = CONFIG_APP_OV5647_PWDN_PIN;
    s_cam_config.csi = &s_csi_config;

    ESP_RETURN_ON_ERROR(esp_video_init(&s_cam_config), TAG, "failed to initialize video");
    s_cam_inited = true;
    return ESP_OK;
}

esp_err_t app_camera_deinit(void)
{
    if (!s_cam_inited) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(esp_video_deinit(), TAG, "failed to deinitialize video");
    s_cam_inited = false;
    return ESP_OK;
}

/* ------------------------------------------------------------------------- */
/* Hardware JPEG encoder                                                     */
/* ------------------------------------------------------------------------- */

typedef struct {
    jpeg_encoder_handle_t engine;
    jpeg_encode_cfg_t cfg;
    uint32_t out_buf_size;
} app_jpeg_t;

esp_err_t app_jpeg_init(uint32_t width, uint32_t height, uint32_t pixel_format, uint8_t quality,
                        app_jpeg_handle_t *ret_handle)
{
    ESP_RETURN_ON_FALSE(ret_handle, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    jpeg_encode_cfg_t cfg = {
        .width = width,
        .height = height,
        .image_quality = quality,
    };
    uint32_t src_size;

    switch (pixel_format) {
    case V4L2_PIX_FMT_SBGGR8: // RAW8 without ISP: treated as grayscale
    case V4L2_PIX_FMT_GREY:
        cfg.src_type = JPEG_ENCODE_IN_FORMAT_GRAY;
        cfg.sub_sample = JPEG_DOWN_SAMPLING_GRAY;
        src_size = width * height;
        break;
    case V4L2_PIX_FMT_RGB565:
        cfg.src_type = JPEG_ENCODE_IN_FORMAT_RGB565;
        cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV422;
        src_size = width * height * 2;
        break;
    case V4L2_PIX_FMT_RGB24:
        cfg.src_type = JPEG_ENCODE_IN_FORMAT_RGB888;
        cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV444;
        src_size = width * height * 3;
        break;
    case V4L2_PIX_FMT_UYVY:
        cfg.src_type = JPEG_ENCODE_IN_FORMAT_YUV422;
        cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV422;
        src_size = width * height * 2;
        break;
#if CONFIG_ESP32P4_REV_MIN_FULL >= 300
    case V4L2_PIX_FMT_YUV420:
        cfg.src_type = JPEG_ENCODE_IN_FORMAT_YUV420;
        cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV420;
        src_size = width * height * 3 / 2;
        break;
#endif
    default:
        ESP_LOGE(TAG, "unsupported pixel format " V4L2_FMT_STR, V4L2_FMT_STR_ARG(pixel_format));
        return ESP_ERR_NOT_SUPPORTED;
    }

    app_jpeg_t *enc = calloc(1, sizeof(app_jpeg_t));
    ESP_RETURN_ON_FALSE(enc, ESP_ERR_NO_MEM, TAG, "no memory");

    jpeg_encode_engine_cfg_t engine_cfg = {
        .timeout_ms = 5000,
    };
    esp_err_t ret = jpeg_new_encoder_engine(&engine_cfg, &enc->engine);
    if (ret != ESP_OK) {
        free(enc);
        ESP_LOGE(TAG, "failed to create jpeg encoder engine");
        return ret;
    }

    enc->cfg = cfg;
    /* Higher quality -> larger output; 3/4 of the raw size is plenty for quality <= 100 */
    enc->out_buf_size = src_size * 3 / 4;
    *ret_handle = enc;
    return ESP_OK;
}

esp_err_t app_jpeg_alloc_output_buffer(app_jpeg_handle_t handle, uint8_t **buf, uint32_t *size)
{
    app_jpeg_t *enc = (app_jpeg_t *)handle;
    ESP_RETURN_ON_FALSE(enc && buf && size, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    size_t allocated;
    jpeg_encode_memory_alloc_cfg_t mem_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };
    uint8_t *p = (uint8_t *)jpeg_alloc_encoder_mem(enc->out_buf_size, &mem_cfg, &allocated);
    ESP_RETURN_ON_FALSE(p, ESP_ERR_NO_MEM, TAG, "failed to alloc jpeg output buf");

    *buf = p;
    *size = allocated;
    return ESP_OK;
}

esp_err_t app_jpeg_process(app_jpeg_handle_t handle, uint8_t *src_buf, uint32_t src_size,
                           uint8_t *dst_buf, uint32_t dst_size, uint32_t *dst_size_out)
{
    app_jpeg_t *enc = (app_jpeg_t *)handle;
    if (!enc || !src_buf || !src_size || !dst_buf || !dst_size || !dst_size_out) {
        return ESP_ERR_INVALID_ARG;
    }
    return jpeg_encoder_process(enc->engine, &enc->cfg, src_buf, src_size, dst_buf, dst_size, dst_size_out);
}

esp_err_t app_jpeg_set_quality(app_jpeg_handle_t handle, uint8_t quality)
{
    app_jpeg_t *enc = (app_jpeg_t *)handle;
    ESP_RETURN_ON_FALSE(enc, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    enc->cfg.image_quality = quality;
    return ESP_OK;
}

esp_err_t app_jpeg_deinit(app_jpeg_handle_t handle, uint8_t *buf)
{
    app_jpeg_t *enc = (app_jpeg_t *)handle;
    if (buf) {
        free(buf);
    }
    if (enc) {
        jpeg_del_encoder_engine(enc->engine);
        free(enc);
    }
    return ESP_OK;
}
