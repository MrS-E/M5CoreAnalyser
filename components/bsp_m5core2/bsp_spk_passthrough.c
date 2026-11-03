/**
 * @file bsp_spk_passthrough.c
 * @brief Software-only Analog-to-Speaker audio passthrough over I2S1 (NS4168 amplifier).
 *
 * Implements:
 * - 1st-order high-pass DC-blocking filter: y[n] = x[n] - x[n-1] + 0.995 * y[n-1]
 * - Digital gain scaling (0%..200%) and mute control
 * - Hard-clipping saturation to [INT16_MIN, INT16_MAX]
 * - AXP192 SPK_EN power gating and DMA silence flushing on mute/disable
 */

#include "bsp_m5core2.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"

static const char *TAG = "bsp_spk_pass";

#define BSP_SPK_DC_POLE_COEFF       (0.995f)
#define BSP_SPK_CHUNK_SAMPLES       (256)
#define BSP_SPK_MIN_RATE_HZ         (4000U)
#define BSP_SPK_MAX_RATE_HZ         (96000U)
#define BSP_SPK_I2S_WRITE_TIMEOUT   (20U)

static portMUX_TYPE s_spk_init_spinlock = portMUX_INITIALIZER_UNLOCKED;
static StaticSemaphore_t s_spk_mutex_buf;
static SemaphoreHandle_t s_spk_mutex = NULL;

static bsp_spk_dsp_state_t s_dsp_state = {
    .prev_x = 0.0f,
    .prev_y = 0.0f,
    .gain_percent = BSP_SPK_DEFAULT_GAIN_PCT,
    .muted = false,
    .initialized = false,
};

static volatile bool s_passthrough_enabled = false;
static uint32_t s_sample_rate_hz = BSP_SPK_DEFAULT_RATE_HZ;
static uint32_t s_hw_i2s_rate_hz = 0;
static i2s_chan_handle_t s_i2s_tx_chan = NULL;
static bool s_i2s_chan_enabled = false;

static const int16_t s_silence_pcm[BSP_SPK_CHUNK_SAMPLES] = {0};
static int16_t s_last_pcm_buf[BSP_SPK_CHUNK_SAMPLES];
static size_t s_last_pcm_count = 0;

static void spk_lock(void)
{
    if (s_spk_mutex == NULL) {
        portENTER_CRITICAL(&s_spk_init_spinlock);
        if (s_spk_mutex == NULL) {
            s_spk_mutex = xSemaphoreCreateMutexStatic(&s_spk_mutex_buf);
        }
        portEXIT_CRITICAL(&s_spk_init_spinlock);
    }
    (void)xSemaphoreTake(s_spk_mutex, portMAX_DELAY);
}

static void spk_unlock(void)
{
    if (s_spk_mutex != NULL) {
        (void)xSemaphoreGive(s_spk_mutex);
    }
}

static inline int16_t clamp_float_to_pcm16(float sample)
{
    if (sample >= (float)INT16_MAX) {
        return INT16_MAX;
    }
    if (sample <= (float)INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)lrintf(sample);
}

void bsp_spk_dsp_init(bsp_spk_dsp_state_t *state, uint8_t gain_percent)
{
    if (state == NULL) {
        return;
    }
    if (gain_percent > BSP_SPK_MAX_GAIN_PCT) {
        gain_percent = BSP_SPK_MAX_GAIN_PCT;
    }
    state->prev_x = 0.0f;
    state->prev_y = 0.0f;
    state->gain_percent = gain_percent;
    state->muted = false;
    state->initialized = false;
}

void bsp_spk_dsp_reset(bsp_spk_dsp_state_t *state)
{
    if (state == NULL) {
        return;
    }
    state->prev_x = 0.0f;
    state->prev_y = 0.0f;
    state->initialized = false;
}

static void bsp_spk_dsp_step(bsp_spk_dsp_state_t *state, float x, int16_t *out_sample)
{
    if (!state->initialized) {
        state->prev_x = x;
        state->prev_y = 0.0f;
        state->initialized = true;
    }

    /* 1st-order DC-blocking high-pass filter: y[n] = x[n] - x[n-1] + 0.995 * y[n-1] */
    float y = (x - state->prev_x) + (BSP_SPK_DC_POLE_COEFF * state->prev_y);
    state->prev_x = x;
    state->prev_y = y;

    if (state->muted || state->gain_percent == 0U) {
        *out_sample = 0;
        return;
    }

    float gain = (float)state->gain_percent / 100.0f;
    *out_sample = clamp_float_to_pcm16(y * gain);
}

void bsp_spk_dsp_process_adc12(bsp_spk_dsp_state_t *state,
                               const uint16_t *adc_samples,
                               int16_t *out_pcm16,
                               size_t count)
{
    if (state == NULL || adc_samples == NULL || out_pcm16 == NULL || count == 0) {
        return;
    }

    for (size_t i = 0; i < count; ++i) {
        uint16_t raw = adc_samples[i];
        if (raw > 4095U) {
            raw = 4095U;
        }
        /* Map 12-bit unsigned (0..4095, midpoint 2048) to signed 16-bit full scale (-32768..+32752) */
        float x = (float)((int32_t)raw - 2048) * 16.0f;
        bsp_spk_dsp_step(state, x, &out_pcm16[i]);
    }
}

void bsp_spk_dsp_process_u8(bsp_spk_dsp_state_t *state,
                            const uint8_t *dac_samples,
                            int16_t *out_pcm16,
                            size_t count)
{
    if (state == NULL || dac_samples == NULL || out_pcm16 == NULL || count == 0) {
        return;
    }

    for (size_t i = 0; i < count; ++i) {
        /* Map 8-bit unsigned (0..255, midpoint 128) to signed 16-bit full scale (-32768..+32512) */
        float x = (float)((int32_t)dac_samples[i] - 128) * 256.0f;
        bsp_spk_dsp_step(state, x, &out_pcm16[i]);
    }
}

static esp_err_t hw_i2s1_ensure_init(uint32_t sample_rate_hz)
{
    esp_err_t err = ESP_OK;

    if (s_i2s_tx_chan == NULL) {
        i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
        chan_cfg.auto_clear = true;
        err = i2s_new_channel(&chan_cfg, &s_i2s_tx_chan, NULL);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2S1 channel allocation failed: %s", esp_err_to_name(err));
            s_i2s_tx_chan = NULL;
            return err;
        }

        i2s_std_config_t std_cfg = {
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = BSP_SPK_PIN_BCK,
                .ws = BSP_SPK_PIN_WS,
                .dout = BSP_SPK_PIN_DOUT,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv = false,
                },
            },
        };

        err = i2s_channel_init_std_mode(s_i2s_tx_chan, &std_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2S1 std mode init failed: %s", esp_err_to_name(err));
            (void)i2s_del_channel(s_i2s_tx_chan);
            s_i2s_tx_chan = NULL;
            return err;
        }
        s_hw_i2s_rate_hz = sample_rate_hz;
    } else if (sample_rate_hz != s_hw_i2s_rate_hz) {
        if (s_i2s_chan_enabled) {
            err = i2s_channel_disable(s_i2s_tx_chan);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "I2S1 channel disable for clock reconfig failed: %s", esp_err_to_name(err));
                return err;
            }
            s_i2s_chan_enabled = false;
        }
        i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz);
        err = i2s_channel_reconfig_std_clock(s_i2s_tx_chan, &clk_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2S1 clock reconfig to %lu Hz failed: %s",
                     (unsigned long)sample_rate_hz,
                     esp_err_to_name(err));
            return err;
        }
        s_hw_i2s_rate_hz = sample_rate_hz;
    }

    if (!s_i2s_chan_enabled) {
        err = i2s_channel_enable(s_i2s_tx_chan);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2S1 channel enable failed: %s", esp_err_to_name(err));
            return err;
        }
        s_i2s_chan_enabled = true;
    }
    return ESP_OK;
}

static esp_err_t hw_i2s1_write_pcm(const int16_t *pcm,
                                   size_t byte_len,
                                   size_t *out_bytes_written,
                                   uint32_t timeout_ms)
{
    if (s_i2s_tx_chan == NULL || !s_i2s_chan_enabled) {
        if (out_bytes_written != NULL) {
            *out_bytes_written = 0;
        }
        return ESP_ERR_INVALID_STATE;
    }
    return i2s_channel_write(s_i2s_tx_chan, pcm, byte_len, out_bytes_written, timeout_ms);
}

static esp_err_t hw_i2s1_disable(void)
{
    if (s_i2s_tx_chan != NULL && s_i2s_chan_enabled) {
        esp_err_t err = i2s_channel_disable(s_i2s_tx_chan);
        s_i2s_chan_enabled = false;
        return err;
    }
    return ESP_OK;
}

static const bsp_spk_i2s_ops_t s_hw_i2s1_ops = {
    .ensure_init = hw_i2s1_ensure_init,
    .write_pcm = hw_i2s1_write_pcm,
    .disable = hw_i2s1_disable,
};

static const bsp_spk_i2s_ops_t *s_i2s_ops = &s_hw_i2s1_ops;

void bsp_spk_passthrough_set_i2s_ops(const bsp_spk_i2s_ops_t *ops)
{
    spk_lock();
    s_i2s_ops = (ops != NULL) ? ops : &s_hw_i2s1_ops;
    spk_unlock();
}

static void spk_i2s1_flush_silence_locked(void)
{
    memset(s_last_pcm_buf, 0, sizeof(s_last_pcm_buf));
    s_last_pcm_count = BSP_SPK_CHUNK_SAMPLES;

    if (s_i2s_ops != NULL && s_i2s_ops->write_pcm != NULL) {
        size_t bytes_written = 0;
        (void)s_i2s_ops->write_pcm(
            s_silence_pcm,
            sizeof(s_silence_pcm),
            &bytes_written,
            BSP_SPK_I2S_WRITE_TIMEOUT);
    }
}

static void spk_sync_amp_power_gate_locked(void)
{
    bool amp_on = s_passthrough_enabled && (!s_dsp_state.muted) && (s_dsp_state.gain_percent > 0U);
    (void)bsp_axp192_set_speaker_enable(amp_on);
}

esp_err_t bsp_spk_passthrough_enable(bool enable, uint32_t sample_rate_hz)
{
    if (sample_rate_hz == 0U) {
        sample_rate_hz = BSP_SPK_DEFAULT_RATE_HZ;
    }

    if (enable && (sample_rate_hz < BSP_SPK_MIN_RATE_HZ || sample_rate_hz > BSP_SPK_MAX_RATE_HZ)) {
        return ESP_ERR_INVALID_ARG;
    }

    spk_lock();

    if (!enable) {
        s_passthrough_enabled = false;
        bsp_spk_dsp_reset(&s_dsp_state);
        /* Gate off AXP192 SPK_EN first, then flush silence to DMA and disable I2S1 */
        (void)bsp_axp192_set_speaker_enable(false);
        spk_i2s1_flush_silence_locked();
        if (s_i2s_ops != NULL && s_i2s_ops->disable != NULL) {
            (void)s_i2s_ops->disable();
        }
        spk_unlock();
        return ESP_OK;
    }

    /* Keep speaker amplifier gated off during I2S1 init / clock reconfiguration */
    (void)bsp_axp192_set_speaker_enable(false);
    bsp_spk_dsp_reset(&s_dsp_state);
    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (s_i2s_ops != NULL && s_i2s_ops->ensure_init != NULL) {
        err = s_i2s_ops->ensure_init(sample_rate_hz);
    }
    if (err != ESP_OK) {
        /* Never power the speaker amplifier if I2S1 init or clock reconfig failed */
        s_passthrough_enabled = false;
        if (s_i2s_ops != NULL && s_i2s_ops->disable != NULL) {
            (void)s_i2s_ops->disable();
        }
        (void)bsp_axp192_set_speaker_enable(false);
        spk_unlock();
        return err;
    }

    /* Preload I2S1 DMA with silence before opening the NS4168 power gate */
    spk_i2s1_flush_silence_locked();
    s_sample_rate_hz = sample_rate_hz;
    s_passthrough_enabled = true;
    spk_sync_amp_power_gate_locked();
    uint8_t gain = s_dsp_state.gain_percent;
    bool muted = s_dsp_state.muted;
    spk_unlock();

    ESP_LOGI(TAG, "Speaker passthrough enabled @ %lu Hz (gain=%u%%, muted=%d)",
             (unsigned long)sample_rate_hz,
             (unsigned)gain,
             (int)muted);
    return ESP_OK;
}

bool bsp_spk_passthrough_is_enabled(void)
{
    spk_lock();
    bool enabled = s_passthrough_enabled;
    spk_unlock();
    return enabled;
}

uint32_t bsp_spk_passthrough_get_sample_rate(void)
{
    spk_lock();
    uint32_t rate = s_sample_rate_hz;
    spk_unlock();
    return rate;
}

esp_err_t bsp_spk_passthrough_set_gain(uint8_t gain_percent)
{
    if (gain_percent > BSP_SPK_MAX_GAIN_PCT) {
        gain_percent = BSP_SPK_MAX_GAIN_PCT;
    }
    spk_lock();
    s_dsp_state.gain_percent = gain_percent;
    if (gain_percent == 0U) {
        spk_sync_amp_power_gate_locked();
        spk_i2s1_flush_silence_locked();
    } else {
        spk_sync_amp_power_gate_locked();
    }
    spk_unlock();
    return ESP_OK;
}

uint8_t bsp_spk_passthrough_get_gain(void)
{
    spk_lock();
    uint8_t gain = s_dsp_state.gain_percent;
    spk_unlock();
    return gain;
}

esp_err_t bsp_spk_passthrough_set_mute(bool mute)
{
    spk_lock();
    s_dsp_state.muted = mute;
    if (mute) {
        spk_sync_amp_power_gate_locked();
        spk_i2s1_flush_silence_locked();
    } else {
        spk_sync_amp_power_gate_locked();
    }
    spk_unlock();
    return ESP_OK;
}

bool bsp_spk_passthrough_is_muted(void)
{
    spk_lock();
    bool muted = s_dsp_state.muted;
    spk_unlock();
    return muted;
}

esp_err_t bsp_spk_passthrough_push_adc12(const uint16_t *adc_samples, size_t count)
{
    if (count == 0) {
        return ESP_OK;
    }
    if (adc_samples == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_passthrough_enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    int16_t pcm_chunk[BSP_SPK_CHUNK_SAMPLES];
    size_t offset = 0;
    while (offset < count) {
        size_t chunk = count - offset;
        if (chunk > BSP_SPK_CHUNK_SAMPLES) {
            chunk = BSP_SPK_CHUNK_SAMPLES;
        }

        spk_lock();
        if (!s_passthrough_enabled) {
            spk_unlock();
            return ESP_ERR_INVALID_STATE;
        }

        bsp_spk_dsp_process_adc12(&s_dsp_state, &adc_samples[offset], pcm_chunk, chunk);
        memcpy(s_last_pcm_buf, pcm_chunk, chunk * sizeof(int16_t));
        s_last_pcm_count = chunk;

        if (s_i2s_ops != NULL && s_i2s_ops->write_pcm != NULL) {
            size_t bytes_written = 0;
            (void)s_i2s_ops->write_pcm(
                pcm_chunk,
                chunk * sizeof(int16_t),
                &bytes_written,
                BSP_SPK_I2S_WRITE_TIMEOUT);
        }
        spk_unlock();

        offset += chunk;
    }

    return ESP_OK;
}

esp_err_t bsp_spk_passthrough_push_u8(const uint8_t *dac_samples, size_t count)
{
    if (count == 0) {
        return ESP_OK;
    }
    if (dac_samples == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_passthrough_enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    int16_t pcm_chunk[BSP_SPK_CHUNK_SAMPLES];
    size_t offset = 0;
    while (offset < count) {
        size_t chunk = count - offset;
        if (chunk > BSP_SPK_CHUNK_SAMPLES) {
            chunk = BSP_SPK_CHUNK_SAMPLES;
        }

        spk_lock();
        if (!s_passthrough_enabled) {
            spk_unlock();
            return ESP_ERR_INVALID_STATE;
        }

        bsp_spk_dsp_process_u8(&s_dsp_state, &dac_samples[offset], pcm_chunk, chunk);
        memcpy(s_last_pcm_buf, pcm_chunk, chunk * sizeof(int16_t));
        s_last_pcm_count = chunk;

        if (s_i2s_ops != NULL && s_i2s_ops->write_pcm != NULL) {
            size_t bytes_written = 0;
            (void)s_i2s_ops->write_pcm(
                pcm_chunk,
                chunk * sizeof(int16_t),
                &bytes_written,
                BSP_SPK_I2S_WRITE_TIMEOUT);
        }
        spk_unlock();

        offset += chunk;
    }

    return ESP_OK;
}

const int16_t *bsp_spk_passthrough_get_last_pcm(size_t *out_samples_count)
{
    spk_lock();
    if (out_samples_count != NULL) {
        *out_samples_count = s_last_pcm_count;
    }
    const int16_t *ptr = s_last_pcm_buf;
    spk_unlock();
    return ptr;
}
