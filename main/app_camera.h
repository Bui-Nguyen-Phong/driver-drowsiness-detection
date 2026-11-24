/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "linux/videodev2.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "esp_video_ioctl.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Video device path of the OV5647 (MIPI-CSI + ISP)
 */
#define APP_CAMERA_DEV_PATH     ESP_VIDEO_MIPI_CSI_DEVICE_NAME

typedef void *app_jpeg_handle_t;

/**
 * @brief Initialize SCCB/I2C, MIPI-CSI and the OV5647 sensor
 */
esp_err_t app_camera_init(void);

/**
 * @brief I2C bus shared by the camera SCCB and the ES8311 codec (valid after app_camera_init)
 */
i2c_master_bus_handle_t app_camera_get_i2c_bus(void);

/**
 * @brief Deinitialize the video system
 */
esp_err_t app_camera_deinit(void);

/**
 * @brief Create a hardware JPEG encoder for frames of the given size/format
 */
esp_err_t app_jpeg_init(uint32_t width, uint32_t height, uint32_t pixel_format, uint8_t quality,
                        app_jpeg_handle_t *ret_handle);

/**
 * @brief Allocate an output buffer suitable for the hardware JPEG encoder
 */
esp_err_t app_jpeg_alloc_output_buffer(app_jpeg_handle_t handle, uint8_t **buf, uint32_t *size);

/**
 * @brief Encode one frame
 */
esp_err_t app_jpeg_process(app_jpeg_handle_t handle, uint8_t *src_buf, uint32_t src_size,
                           uint8_t *dst_buf, uint32_t dst_size, uint32_t *dst_size_out);

/**
 * @brief Change the JPEG quality (1-100)
 */
esp_err_t app_jpeg_set_quality(app_jpeg_handle_t handle, uint8_t quality);

/**
 * @brief Release the encoder and its output buffer (buf may be NULL)
 */
esp_err_t app_jpeg_deinit(app_jpeg_handle_t handle, uint8_t *buf);

#ifdef __cplusplus
}
#endif
