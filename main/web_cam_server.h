/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Open the camera video device and start the HTTP servers.
 *
 * - Port 80: web UI (static files) + REST API (/api/get_camera_info, /api/set_camera_config,
 *            /api/capture_image, /api/capture_binary)
 * - Port 81: MJPEG stream (/stream)
 *
 * app_camera_init() and the network connection must be done before calling this.
 *
 * @param dev_name Video device path, e.g. APP_CAMERA_DEV_PATH
 *
 * @return ESP_OK on success or other value on failure
 */
esp_err_t web_cam_server_start(const char *dev_name);

/**
 * @brief Open the camera and start the capture task (feeds the face pipeline continuously).
 *        Needs no network; web_cam_server_start() calls it if it has not been called yet.
 */
esp_err_t web_cam_capture_start(const char *dev_name);

#ifdef __cplusplus
}
#endif
