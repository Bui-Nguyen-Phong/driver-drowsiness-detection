#include "audio_alert.h"

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"

static const char *TAG = "audio_alert";

#define ES8311_I2C_ADDR_7BIT  0x18
#define CHUNK_FRAMES          256      /* 16 ms @ 16 kHz */
#define FADE_MS               4        /* fade in/out per tone, kills clicks */
#define TONE_AMPLITUDE        0.45f    /* of full scale; volume does the rest */

typedef struct {
    uint16_t freq_hz;   /* 0 = silence */
    uint16_t ms;
} tone_t;

/* ---- built-in sounds -------------------------------------------------- */
static const tone_t s_beep[] = { {1000, 150} };
static const tone_t s_eyes[] = { {2800, 120}, {0, 60}, {2800, 120}, {0, 60}, {2800, 120}, {0, 60},
                                 {2800, 120}, {0, 60}, {2800, 120}, {0, 60}, {2800, 120} };
static const tone_t s_yawn[] = { {1318, 220}, {0, 30}, {988, 380} };
static const tone_t s_calib[] = { {784, 120}, {0, 20}, {1175, 200} };
static const tone_t s_error[] = { {200, 400} };

static const struct { const tone_t *t; size_t n; const char *name; } s_sounds[AUDIO_ALERT_COUNT] = {
    [AUDIO_ALERT_BEEP]        = { s_beep,  sizeof(s_beep) / sizeof(tone_t),  "beep" },
    [AUDIO_ALERT_EYES_CLOSED] = { s_eyes,  sizeof(s_eyes) / sizeof(tone_t),  "eyes_closed" },
    [AUDIO_ALERT_YAWN]        = { s_yawn,  sizeof(s_yawn) / sizeof(tone_t),  "yawn" },
    [AUDIO_ALERT_CALIB_DONE]  = { s_calib, sizeof(s_calib) / sizeof(tone_t), "calib_done" },
    [AUDIO_ALERT_ERROR]       = { s_error, sizeof(s_error) / sizeof(tone_t), "error" },
};

/* ---- state ------------------------------------------------------------- */
typedef struct {
    int16_t sound;      /* >= 0: built-in sound, -1: custom tone, -2: stop */
    tone_t tone;
} req_t;

static esp_codec_dev_handle_t s_dev;
static QueueHandle_t s_queue;
static uint32_t s_rate;
static volatile bool s_busy;
static int16_t s_buf[CHUNK_FRAMES * 2];   /* stereo, interleaved */

static inline bool interrupted(void)
{
    return uxQueueMessagesWaiting(s_queue) > 0;
}

/* Play one tone; returns false when interrupted by a new request. */
static bool play_tone(const tone_t *t)
{
    const uint32_t total = (uint32_t)((uint64_t)s_rate * t->ms / 1000);
    const uint32_t fade = s_rate * FADE_MS / 1000;
    const float step = 2.0f * (float)M_PI * t->freq_hz / (float)s_rate;
    float phase = 0.f;
    uint32_t done = 0;

    while (done < total) {
        if (interrupted()) {
            return false;
        }
        uint32_t n = total - done;
        if (n > CHUNK_FRAMES) {
            n = CHUNK_FRAMES;
        }
        for (uint32_t i = 0; i < n; i++) {
            int16_t v = 0;
            if (t->freq_hz) {
                const uint32_t k = done + i;
                float env = 1.f;
                if (k < fade) {
                    env = (float)k / fade;
                } else if (total - k < fade) {
                    env = (float)(total - k) / fade;
                }
                v = (int16_t)(sinf(phase) * env * TONE_AMPLITUDE * 32767.f);
                phase += step;
                if (phase > 2.0f * (float)M_PI) {
                    phase -= 2.0f * (float)M_PI;
                }
            }
            s_buf[2 * i] = v;
            s_buf[2 * i + 1] = v;
        }
        if (esp_codec_dev_write(s_dev, s_buf, n * 2 * sizeof(int16_t)) != ESP_CODEC_DEV_OK) {
            ESP_LOGW(TAG, "codec write failed");
            return false;
        }
        done += n;
    }
    return true;
}

static void flush_silence(void)
{
    /* push one chunk of zeros so the DMA does not keep the last samples */
    memset(s_buf, 0, sizeof(s_buf));
    esp_codec_dev_write(s_dev, s_buf, sizeof(s_buf));
}

static void audio_task(void *arg)
{
    req_t r;
    for (;;) {
        if (!interrupted()) {
            s_busy = false;
        }
        if (xQueueReceive(s_queue, &r, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        s_busy = true;
        if (r.sound == -2) {
            flush_silence();
            continue;
        }
        if (r.sound == -1) {
            play_tone(&r.tone);
        } else {
            const tone_t *t = s_sounds[r.sound].t;
            for (size_t i = 0; i < s_sounds[r.sound].n; i++) {
                if (!play_tone(&t[i])) {
                    break;
                }
            }
        }
        if (!interrupted()) {
            flush_silence();
        }
    }
}

/* ---- public API -------------------------------------------------------- */
esp_err_t audio_alert_init(const audio_alert_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg");
    ESP_RETURN_ON_FALSE(!s_dev, ESP_ERR_INVALID_STATE, TAG, "already initialised");
    s_rate = cfg->sample_rate;

    /* I2C: share the caller's bus (camera SCCB uses the same pins) or make one */
    i2c_master_bus_handle_t bus = cfg->i2c_bus;
    if (!bus) {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = cfg->i2c_port,
            .sda_io_num = cfg->i2c_sda,
            .scl_io_num = cfg->i2c_scl,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus), TAG, "i2c bus");
    }
    esp_err_t pr = i2c_master_probe(bus, ES8311_I2C_ADDR_7BIT, 100);
    ESP_LOGI(TAG, "ES8311 probe @0x%02x: %s", ES8311_I2C_ADDR_7BIT, esp_err_to_name(pr));
    ESP_RETURN_ON_ERROR(pr, TAG, "ES8311 not found on I2C (SDA=%d SCL=%d)", cfg->i2c_sda, cfg->i2c_scl);

    /* I2S TX only; esp_codec_dev reconfigures slot/clock on open */
    i2s_chan_handle_t tx = NULL;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(cfg->i2s_port, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &tx, NULL), TAG, "i2s channel");
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(cfg->sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = cfg->i2s_mclk,
            .bclk = cfg->i2s_bclk,
            .ws = cfg->i2s_ws,
            .dout = cfg->i2s_dout,
            .din = I2S_GPIO_UNUSED,
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx, &std_cfg), TAG, "i2s std");

    audio_codec_i2s_cfg_t i2s_if_cfg = { .port = (uint8_t)cfg->i2s_port, .rx_handle = NULL, .tx_handle = tx };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_if_cfg);
    audio_codec_i2c_cfg_t i2c_if_cfg = { .port = (uint8_t)cfg->i2c_port, .addr = ES8311_CODEC_DEFAULT_ADDR, .bus_handle = bus };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_if_cfg);
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(data_if && ctrl_if && gpio_if, ESP_FAIL, TAG, "codec interfaces");

    es8311_codec_cfg_t es_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin = (int16_t)cfg->pa_pin,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = true,
        .hw_gain = { .pa_voltage = 5.0f, .codec_dac_voltage = 3.3f },
    };
    const audio_codec_if_t *codec_if = es8311_codec_new(&es_cfg);
    ESP_RETURN_ON_FALSE(codec_if, ESP_FAIL, TAG, "es8311_codec_new");

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = codec_if,
        .data_if = data_if,
    };
    s_dev = esp_codec_dev_new(&dev_cfg);
    ESP_RETURN_ON_FALSE(s_dev, ESP_FAIL, TAG, "esp_codec_dev_new");

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = 2,
        .sample_rate = cfg->sample_rate,
    };
    ESP_RETURN_ON_FALSE(esp_codec_dev_open(s_dev, &fs) == ESP_CODEC_DEV_OK, ESP_FAIL, TAG, "codec open");
    ESP_RETURN_ON_ERROR(audio_alert_set_volume(cfg->volume), TAG, "volume");

    s_queue = xQueueCreate(1, sizeof(req_t));
    ESP_RETURN_ON_FALSE(s_queue, ESP_ERR_NO_MEM, TAG, "queue");
    BaseType_t ok = xTaskCreatePinnedToCore(audio_task, "audio_alert", 4096, NULL,
                                            cfg->task_priority, NULL, cfg->task_core);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "task");

    ESP_LOGI(TAG, "ready: %lu Hz, vol %d, PA gpio %d, I2S mclk=%d bclk=%d ws=%d dout=%d",
             (unsigned long)cfg->sample_rate, cfg->volume, cfg->pa_pin,
             cfg->i2s_mclk, cfg->i2s_bclk, cfg->i2s_ws, cfg->i2s_dout);
    return ESP_OK;
}

static esp_err_t post(const req_t *r)
{
    if (!s_queue) {
        return ESP_ERR_INVALID_STATE;   /* audio disabled or init failed: stay silent */
    }
    xQueueOverwrite(s_queue, r);   /* newest request wins */
    s_busy = true;                 /* so wait_idle() right after play() waits */
    return ESP_OK;
}

esp_err_t audio_alert_play(audio_alert_sound_t sound)
{
    ESP_RETURN_ON_FALSE(sound >= 0 && sound < AUDIO_ALERT_COUNT, ESP_ERR_INVALID_ARG, TAG, "sound");
    ESP_LOGD(TAG, "play %s", s_sounds[sound].name);
    req_t r = { .sound = (int16_t)sound };
    return post(&r);
}

esp_err_t audio_alert_tone(uint16_t freq_hz, uint16_t duration_ms)
{
    req_t r = { .sound = -1, .tone = { freq_hz, duration_ms } };
    return post(&r);
}

void audio_alert_stop(void)
{
    req_t r = { .sound = -2 };
    post(&r);
}

esp_err_t audio_alert_set_volume(int volume)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "not initialised");
    if (volume < 0) {
        volume = 0;
    } else if (volume > 100) {
        volume = 100;
    }
    return esp_codec_dev_set_out_vol(s_dev, volume) == ESP_CODEC_DEV_OK ? ESP_OK : ESP_FAIL;
}

bool audio_alert_is_busy(void)
{
    return s_busy;
}

bool audio_alert_wait_idle(uint32_t timeout_ms)
{
    const TickType_t start = xTaskGetTickCount();
    while (s_busy) {
        if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeout_ms)) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return true;
}
