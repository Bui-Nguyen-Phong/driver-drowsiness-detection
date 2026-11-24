/*
 * audio_alert - small tone/alert player for the ES8311 + speaker on the
 * Waveshare ESP32-P4-Module-DEV-KIT (SPK header).
 *
 * Hardware (from the board schematic / Waveshare examples):
 *   ES8311 codec, I2C addr 0x18 on I2C0 SDA=GPIO7 SCL=GPIO8
 *   (the same bus as the MIPI-CSI camera SCCB -> share one bus handle)
 *   I2S MCLK=13 BCLK=12 WS=10 DOUT=9 (DIN=11 is the mic, unused here)
 *   Speaker amplifier enable (PA_CTRL) = GPIO53, active high
 *
 * Sounds are synthesised on the fly (sine + fade), so no audio files are
 * embedded. Playback runs in its own task; audio_alert_play() never blocks
 * and a new request interrupts the sound currently playing.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AUDIO_ALERT_BEEP = 0,       /* short neutral beep                  */
    AUDIO_ALERT_EYES_CLOSED,    /* urgent: fast high triple beep        */
    AUDIO_ALERT_YAWN,           /* warning: descending two-tone chime   */
    AUDIO_ALERT_CALIB_DONE,     /* info: rising two-note "ok"           */
    AUDIO_ALERT_ERROR,          /* low buzz                             */
    AUDIO_ALERT_COUNT
} audio_alert_sound_t;

typedef struct {
    i2c_master_bus_handle_t i2c_bus; /* existing bus to share, or NULL to create one */
    int i2c_port;
    int i2c_sda;
    int i2c_scl;
    int i2s_port;
    int i2s_mclk;
    int i2s_bclk;
    int i2s_ws;
    int i2s_dout;
    int pa_pin;
    uint32_t sample_rate;
    int volume;                      /* 0..100 */
    int task_core;                   /* core for the playback task */
    int task_priority;
} audio_alert_config_t;

#define AUDIO_ALERT_DEFAULT_CONFIG() {  \
    .i2c_bus = NULL,                    \
    .i2c_port = 0,                      \
    .i2c_sda = 7,                       \
    .i2c_scl = 8,                       \
    .i2s_port = 0,                      \
    .i2s_mclk = 13,                     \
    .i2s_bclk = 12,                     \
    .i2s_ws = 10,                       \
    .i2s_dout = 9,                      \
    .pa_pin = 53,                       \
    .sample_rate = 16000,               \
    .volume = 70,                       \
    .task_core = 0,                     \
    .task_priority = 4,                 \
}

esp_err_t audio_alert_init(const audio_alert_config_t *cfg);

/* Queue a built-in sound (non-blocking, interrupts the current one). */
esp_err_t audio_alert_play(audio_alert_sound_t sound);

/* Queue a single tone (non-blocking). freq_hz = 0 plays silence. */
esp_err_t audio_alert_tone(uint16_t freq_hz, uint16_t duration_ms);

/* Stop whatever is playing. */
void audio_alert_stop(void);

esp_err_t audio_alert_set_volume(int volume);

bool audio_alert_is_busy(void);

/* Block until playback is idle (or timeout). Returns true when idle. */
bool audio_alert_wait_idle(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
