/*
 * ESP32-P4 + OV5647 NoIR: MJPEG web camera server
 */

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "mdns.h"
#include "lwip/apps/netbiosns.h"
#include "protocol_examples_common.h"
#include "app_camera.h"
#include "audio_alert.h"
#include "web_cam_server.h"

static const char *TAG = "main";

static void nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
}

static void mdns_service_init(void)
{
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(CONFIG_EXAMPLE_MDNS_HOST_NAME));
    ESP_ERROR_CHECK(mdns_instance_name_set(CONFIG_EXAMPLE_MDNS_INSTANCE));

    mdns_txt_item_t txt[] = {
        {"board", CONFIG_IDF_TARGET},
        {"path", "/"},
    };
    ESP_ERROR_CHECK(mdns_service_add("ESP32-WebServer", "_http", "_tcp", 80, txt, sizeof(txt) / sizeof(txt[0])));

    netbiosns_init();
    netbiosns_set_name(CONFIG_EXAMPLE_MDNS_HOST_NAME);
}

static void network_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    mdns_service_init();

    /* Wi-Fi or Ethernet, as selected in menuconfig (Example Connection Configuration) */
    ESP_ERROR_CHECK(example_connect());
}

void app_main(void)
{
    nvs_init();

    ESP_ERROR_CHECK(app_camera_init());

#if CONFIG_APP_AUDIO_ALERT_ENABLE
    /* Speaker alerts: core 0 with the other side tasks, pipeline stays on its own core */
    audio_alert_config_t audio_cfg = AUDIO_ALERT_DEFAULT_CONFIG();
    audio_cfg.i2c_bus = app_camera_get_i2c_bus();
    audio_cfg.i2c_port = CONFIG_APP_OV5647_SCCB_I2C_PORT;
    audio_cfg.volume = CONFIG_APP_AUDIO_ALERT_VOLUME;
    audio_cfg.task_core = CONFIG_APP_HTTP_SERVER_TASK_CORE;
    esp_err_t audio_err = audio_alert_init(&audio_cfg);
    if (audio_err != ESP_OK) {
        ESP_LOGW(TAG, "speaker alerts disabled: %s", esp_err_to_name(audio_err));
    } else {
        audio_alert_play(AUDIO_ALERT_BEEP);   /* boot beep = speaker OK */
    }
#endif

    /* Camera + face pipeline + speaker alerts run from here on, even if Wi-Fi never comes up */
    ESP_ERROR_CHECK(web_cam_capture_start(APP_CAMERA_DEV_PATH));

    network_init();

    ESP_ERROR_CHECK(web_cam_server_start(APP_CAMERA_DEV_PATH));

    ESP_LOGI(TAG, "Open http://%s.local/ in a browser", CONFIG_EXAMPLE_MDNS_HOST_NAME);

    /* TODO: add your own tasks / features here */
}
