/**
 * @file analoggen_lut.c
 * @brief Pure 256-point 8-bit DAC lookup table synthesis, amplitude/offset rail clamping,
 *        phase accumulation, and speaker audio resampling for `tool_analoggen`.
 */

#include "tool_analoggen.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

uint32_t analoggen_clamp_freq_hz(uint32_t requested_hz, bool *out_clamped)
{
    bool clamped = false;
    uint32_t eff = requested_hz;
    if (eff < ANALOGGEN_MIN_FREQ_HZ) {
        eff = ANALOGGEN_MIN_FREQ_HZ;
        clamped = true;
    } else if (eff > ANALOGGEN_MAX_FREQ_HZ) {
        eff = ANALOGGEN_MAX_FREQ_HZ;
        clamped = true;
    }
    if (out_clamped != NULL) {
        *out_clamped = clamped;
    }
    return eff;
}

uint8_t analoggen_mv_to_dac_code(int32_t mv, bool *out_clipped)
{
    bool clipped = false;
    int32_t clamped_mv = mv;
    if (clamped_mv < 0) {
        clamped_mv = 0;
        clipped = true;
    } else if (clamped_mv > (int32_t)ANALOGGEN_MAX_MV) {
        clamped_mv = (int32_t)ANALOGGEN_MAX_MV;
        clipped = true;
    }

    if (out_clipped != NULL) {
        *out_clipped = clipped;
    }

    int32_t code = (clamped_mv * 255 + ((int32_t)ANALOGGEN_MAX_MV / 2)) / (int32_t)ANALOGGEN_MAX_MV;
    if (code < 0) {
        code = 0;
    } else if (code > 255) {
        code = 255;
    }
    return (uint8_t)code;
}

uint16_t analoggen_dac_code_to_mv(uint8_t code)
{
    return (uint16_t)(((uint32_t)code * ANALOGGEN_MAX_MV + 127U) / 255U);
}

esp_err_t analoggen_lut_build(const analoggen_config_t *cfg,
                              uint8_t *out_lut,
                              size_t lut_len,
                              analoggen_lut_stats_t *out_stats)
{
    if (cfg == NULL ||
        out_lut == NULL ||
        lut_len < 8U ||
        cfg->waveform < 0 ||
        cfg->waveform >= ANALOGGEN_WAVE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t steps = cfg->stair_steps;
    if (steps < ANALOGGEN_MIN_STAIR_STEPS) {
        steps = ANALOGGEN_MIN_STAIR_STEPS;
    } else if (steps > ANALOGGEN_MAX_STAIR_STEPS) {
        steps = ANALOGGEN_MAX_STAIR_STEPS;
    }

    float offset_mv = (float)cfg->offset_mv;
    float half_amp_mv = 0.5f * (float)cfg->amplitude_mv;

    uint8_t min_code = 255U;
    uint8_t max_code = 0U;
    uint32_t sum_mv = 0U;
    bool any_clipped = false;

    /* Check theoretical peak excursions as well as per-sample excursions */
    if (cfg->waveform != ANALOGGEN_WAVE_DC) {
        if ((offset_mv - half_amp_mv) < -0.5f ||
            (offset_mv + half_amp_mv) > ((float)ANALOGGEN_MAX_MV + 0.5f)) {
            any_clipped = true;
        }
    } else if (cfg->offset_mv > ANALOGGEN_MAX_MV) {
        any_clipped = true;
    }

    size_t half_len = lut_len / 2U;

    for (size_t i = 0; i < lut_len; ++i) {
        float norm = 0.0f; /* Normalized bipolar value in [-1.0, +1.0] */
        switch (cfg->waveform) {
        case ANALOGGEN_WAVE_SINE: {
            double angle = (2.0 * M_PI * (double)i) / (double)lut_len;
            norm = (float)sin(angle);
            break;
        }
        case ANALOGGEN_WAVE_TRIANGLE: {
            if (i <= half_len) {
                norm = -1.0f + (2.0f * (float)i) / (float)half_len;
            } else {
                norm = 1.0f - (2.0f * (float)(i - half_len)) / (float)(lut_len - half_len);
            }
            break;
        }
        case ANALOGGEN_WAVE_SAWTOOTH: {
            norm = -1.0f + (2.0f * (float)i) / (float)(lut_len - 1U);
            break;
        }
        case ANALOGGEN_WAVE_STAIRCASE: {
            size_t step_idx = (i * (size_t)steps) / lut_len;
            if (step_idx >= (size_t)steps) {
                step_idx = (size_t)steps - 1U;
            }
            norm = -1.0f + (2.0f * (float)step_idx) / (float)(steps - 1U);
            break;
        }
        case ANALOGGEN_WAVE_DC:
        default:
            norm = 0.0f;
            break;
        }

        float sample_mv_f = offset_mv + (half_amp_mv * norm);
        int32_t sample_mv = (int32_t)lroundf(sample_mv_f);
        bool sample_clipped = false;
        uint8_t code = analoggen_mv_to_dac_code(sample_mv, &sample_clipped);
        if (sample_clipped) {
            any_clipped = true;
        }

        out_lut[i] = code;
        if (code < min_code) {
            min_code = code;
        }
        if (code > max_code) {
            max_code = code;
        }
        sum_mv += (uint32_t)analoggen_dac_code_to_mv(code);
    }

    if (out_stats != NULL) {
        uint16_t min_mv = analoggen_dac_code_to_mv(min_code);
        uint16_t max_mv = analoggen_dac_code_to_mv(max_code);
        out_stats->min_code = min_code;
        out_stats->max_code = max_code;
        out_stats->min_mv = min_mv;
        out_stats->max_mv = max_mv;
        out_stats->vpp_mv = (max_mv >= min_mv) ? (uint16_t)(max_mv - min_mv) : 0U;
        out_stats->mean_mv = (uint16_t)((sum_mv + (uint32_t)(lut_len / 2U)) / (uint32_t)lut_len);
        out_stats->clipped = any_clipped;
    }

    return ESP_OK;
}

size_t analoggen_calc_phase_index(uint64_t time_us, uint32_t freq_hz, size_t lut_len)
{
    if (lut_len == 0U || freq_hz == 0U) {
        return 0U;
    }
    uint64_t phase_num = (time_us * (uint64_t)freq_hz) % 1000000ULL;
    size_t idx = (size_t)((phase_num * (uint64_t)lut_len) / 1000000ULL);
    return idx % lut_len;
}

esp_err_t analoggen_resample_lut_for_speaker(const uint8_t *lut,
                                             size_t lut_len,
                                             uint32_t wave_freq_hz,
                                             uint32_t spk_sample_rate_hz,
                                             uint32_t *inout_phase_q16,
                                             uint8_t *out_audio_u8,
                                             size_t audio_count)
{
    if (lut == NULL ||
        lut_len < 2U ||
        spk_sample_rate_hz == 0U ||
        inout_phase_q16 == NULL ||
        out_audio_u8 == NULL ||
        audio_count == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t phase_q16 = *inout_phase_q16;
    uint32_t wrap_q16 = (uint32_t)lut_len << 16;
    uint64_t step_num = (uint64_t)wave_freq_hz * (uint64_t)lut_len * 65536ULL;
    uint32_t step_q16 = (uint32_t)((step_num + (spk_sample_rate_hz / 2U)) / spk_sample_rate_hz);

    for (size_t i = 0; i < audio_count; ++i) {
        size_t idx = (size_t)(phase_q16 >> 16) % lut_len;
        out_audio_u8[i] = lut[idx];
        phase_q16 = (phase_q16 + step_q16) % wrap_q16;
    }

    *inout_phase_q16 = phase_q16;
    return ESP_OK;
}

const char *analoggen_wave_to_str(analoggen_wave_t wave)
{
    switch (wave) {
    case ANALOGGEN_WAVE_SINE:
        return "SINE";
    case ANALOGGEN_WAVE_TRIANGLE:
        return "TRIANGLE";
    case ANALOGGEN_WAVE_SAWTOOTH:
        return "SAWTOOTH";
    case ANALOGGEN_WAVE_STAIRCASE:
        return "STAIR";
    case ANALOGGEN_WAVE_DC:
        return "DC";
    default:
        return "UNKNOWN";
    }
}
