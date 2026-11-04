/**
 * @file wavegen_calc.c
 * @brief Pure timing, resolution, duty-cycle, servo pulse, RMT burst symbol, and waveform
 *        preview calculations for `tool_wavegen`.
 */

#include "tool_wavegen.h"

#include <math.h>
#include <string.h>

#define WAVEGEN_LEDC_MAX_BITS      (18U)
#define WAVEGEN_LEDC_MAX_DIV_INT   (1023U)
#define WAVEGEN_MCPWM_MAX_PERIOD   (65535U)

static float clamp_float(float val, float min_v, float max_v)
{
    if (!isfinite(val) || val < min_v) {
        return min_v;
    }
    if (val > max_v) {
        return max_v;
    }
    return val;
}

esp_err_t wavegen_calc_ledc_params(uint32_t apb_clk_hz,
                                   uint32_t target_freq_hz,
                                   float duty_pct,
                                   wavegen_ledc_params_t *out_params)
{
    if (out_params == NULL ||
        apb_clk_hz == 0U ||
        target_freq_hz < WAVEGEN_LEDC_MIN_FREQ_HZ ||
        target_freq_hz > WAVEGEN_LEDC_MAX_FREQ_HZ) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_params, 0, sizeof(*out_params));
    float clamped_duty = clamp_float(duty_pct, 0.0f, 100.0f);

    uint8_t best_bits = 0U;
    for (int bits = (int)WAVEGEN_LEDC_MAX_BITS; bits >= 1; --bits) {
        uint64_t denom = (uint64_t)target_freq_hz * (1ULL << bits);
        if (denom == 0ULL || denom > (uint64_t)apb_clk_hz) {
            continue;
        }
        /* ESP32 LEDC uses an 18-bit fractional divider (10-bit integer + 8-bit fraction) */
        uint64_t div_q8 = (((uint64_t)apb_clk_hz << 8) + (denom / 2ULL)) / denom;
        if (div_q8 >= 256ULL && div_q8 <= ((uint64_t)WAVEGEN_LEDC_MAX_DIV_INT << 8)) {
            best_bits = (uint8_t)bits;
            break;
        }
    }

    if (best_bits == 0U) {
        /* Fallback for very low frequencies (e.g. 1 Hz where APB/2^20 = 76.29 <= 1023) */
        best_bits = 14U;
    }

    uint32_t max_duty = 1U << best_bits;
    uint64_t denom = (uint64_t)target_freq_hz * (uint64_t)max_duty;
    uint64_t div_q8 = (((uint64_t)apb_clk_hz << 8) + (denom / 2ULL)) / denom;
    if (div_q8 < 256ULL) {
        div_q8 = 256ULL;
    }

    uint64_t actual_freq = (((uint64_t)apb_clk_hz << 8) + ((div_q8 * max_duty) / 2ULL)) /
                           (div_q8 * (uint64_t)max_duty);
    if (actual_freq == 0ULL) {
        actual_freq = 1ULL;
    }

    double raw_ticks = ((double)clamped_duty / 100.0) * (double)max_duty;
    uint32_t duty_ticks = (uint32_t)llround(raw_ticks);
    if (duty_ticks > max_duty) {
        duty_ticks = max_duty;
    }

    out_params->target_freq_hz = target_freq_hz;
    out_params->actual_freq_hz = (uint32_t)actual_freq;
    out_params->duty_resolution_bits = best_bits;
    out_params->max_duty_ticks = max_duty;
    out_params->duty_ticks = duty_ticks;
    out_params->target_duty_pct = clamped_duty;
    out_params->actual_duty_pct = ((float)duty_ticks * 100.0f) / (float)max_duty;
    return ESP_OK;
}

esp_err_t wavegen_calc_mcpwm_params(uint32_t target_freq_hz,
                                    float duty_pct,
                                    wavegen_mcpwm_params_t *out_params)
{
    if (out_params == NULL ||
        target_freq_hz < WAVEGEN_MCPWM_MIN_FREQ_HZ ||
        target_freq_hz > WAVEGEN_MCPWM_MAX_FREQ_HZ) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_params, 0, sizeof(*out_params));
    float clamped_duty = clamp_float(duty_pct, 0.0f, 100.0f);

    /* Choose the highest MCPWM timer resolution where period_ticks <= 65535 */
    static const uint32_t k_resolutions[] = {
        10000000U, /* 10 MHz (0.1 us tick) for >= 153 Hz */
        1000000U,  /* 1 MHz (1 us tick) for >= 16 Hz */
        100000U,   /* 100 kHz (10 us tick) for >= 10 Hz */
    };

    uint32_t chosen_res = k_resolutions[2];
    for (size_t i = 0; i < sizeof(k_resolutions) / sizeof(k_resolutions[0]); ++i) {
        uint32_t ticks = (k_resolutions[i] + (target_freq_hz / 2U)) / target_freq_hz;
        if (ticks >= 2U && ticks <= WAVEGEN_MCPWM_MAX_PERIOD) {
            chosen_res = k_resolutions[i];
            break;
        }
    }

    uint32_t period_ticks = (chosen_res + (target_freq_hz / 2U)) / target_freq_hz;
    if (period_ticks < 2U) {
        period_ticks = 2U;
    } else if (period_ticks > WAVEGEN_MCPWM_MAX_PERIOD) {
        period_ticks = WAVEGEN_MCPWM_MAX_PERIOD;
    }

    uint32_t cmp_ticks = (uint32_t)lroundf((clamped_duty / 100.0f) * (float)period_ticks);
    if (cmp_ticks > period_ticks) {
        cmp_ticks = period_ticks;
    }

    uint32_t actual_freq = (chosen_res + (period_ticks / 2U)) / period_ticks;
    float actual_duty = ((float)cmp_ticks * 100.0f) / (float)period_ticks;
    float period_us = (1000000.0f * (float)period_ticks) / (float)chosen_res;

    out_params->resolution_hz = chosen_res;
    out_params->period_ticks = period_ticks;
    out_params->compare_ticks = cmp_ticks;
    out_params->target_freq_hz = target_freq_hz;
    out_params->actual_freq_hz = actual_freq;
    out_params->target_duty_pct = clamped_duty;
    out_params->actual_duty_pct = actual_duty;
    out_params->pulse_width_us = period_us * (actual_duty / 100.0f);
    return ESP_OK;
}

uint16_t wavegen_calc_servo_angle_to_us(float angle_deg)
{
    float clamped = clamp_float(angle_deg, 0.0f, 180.0f);
    float span_us = (float)(WAVEGEN_SERVO_MAX_US - WAVEGEN_SERVO_MIN_US);
    uint32_t us = (uint32_t)lroundf((float)WAVEGEN_SERVO_MIN_US + (clamped / 180.0f) * span_us);
    if (us < WAVEGEN_SERVO_MIN_US) {
        us = WAVEGEN_SERVO_MIN_US;
    } else if (us > WAVEGEN_SERVO_MAX_US) {
        us = WAVEGEN_SERVO_MAX_US;
    }
    return (uint16_t)us;
}

float wavegen_calc_servo_us_to_angle(uint16_t pulse_us)
{
    uint16_t clamped = pulse_us;
    if (clamped < WAVEGEN_SERVO_MIN_US) {
        clamped = WAVEGEN_SERVO_MIN_US;
    } else if (clamped > WAVEGEN_SERVO_MAX_US) {
        clamped = WAVEGEN_SERVO_MAX_US;
    }
    return ((float)(clamped - WAVEGEN_SERVO_MIN_US) * 180.0f) /
           (float)(WAVEGEN_SERVO_MAX_US - WAVEGEN_SERVO_MIN_US);
}

esp_err_t wavegen_calc_servo_rmt_symbol(uint16_t pulse_us,
                                        uint32_t period_us,
                                        rmt_symbol_word_t *out_symbols,
                                        size_t max_symbols,
                                        size_t *out_symbol_count)
{
    if (out_symbols == NULL || out_symbol_count == NULL || max_symbols == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_symbol_count = 0U;
    uint32_t eff_period_us = (period_us == 0U) ? WAVEGEN_SERVO_PERIOD_US : period_us;
    if (eff_period_us < 3000U || eff_period_us > 65000U) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t clamped_pulse = pulse_us;
    if (clamped_pulse < WAVEGEN_SERVO_MIN_US) {
        clamped_pulse = WAVEGEN_SERVO_MIN_US;
    } else if (clamped_pulse > WAVEGEN_SERVO_MAX_US) {
        clamped_pulse = WAVEGEN_SERVO_MAX_US;
    }

    uint32_t low_us = eff_period_us - (uint32_t)clamped_pulse;
    if (low_us <= WAVEGEN_RMT_MAX_DURATION_US) {
        out_symbols[0] = (rmt_symbol_word_t){
            .duration0 = clamped_pulse,
            .level0 = 1U,
            .duration1 = (uint16_t)low_us,
            .level1 = 0U,
        };
        *out_symbol_count = 1U;
        return ESP_OK;
    }

    if (max_symbols < 2U) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t low_part0 = WAVEGEN_RMT_MAX_DURATION_US;
    uint32_t rem = low_us - (uint32_t)low_part0;
    uint16_t low_part1 = (uint16_t)(rem / 2U);
    uint16_t low_part2 = (uint16_t)(rem - (uint32_t)low_part1);
    if (low_part1 == 0U) {
        low_part0--;
        low_part1 = 1U;
    }

    out_symbols[0] = (rmt_symbol_word_t){
        .duration0 = clamped_pulse,
        .level0 = 1U,
        .duration1 = low_part0,
        .level1 = 0U,
    };
    out_symbols[1] = (rmt_symbol_word_t){
        .duration0 = low_part1,
        .level0 = 0U,
        .duration1 = low_part2,
        .level1 = 0U,
    };
    *out_symbol_count = 2U;
    return ESP_OK;
}

esp_err_t wavegen_calc_burst_rmt_symbols(uint16_t high_us,
                                         uint16_t low_us,
                                         uint16_t pulse_count,
                                         rmt_symbol_word_t *out_symbols,
                                         size_t max_symbols,
                                         size_t *out_symbol_count)
{
    if (out_symbols == NULL ||
        out_symbol_count == NULL ||
        high_us == 0U || high_us > WAVEGEN_RMT_MAX_DURATION_US ||
        low_us == 0U || low_us > WAVEGEN_RMT_MAX_DURATION_US ||
        pulse_count == 0U || pulse_count > WAVEGEN_MAX_BURST_PULSES ||
        max_symbols < (size_t)pulse_count) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint16_t i = 0; i < pulse_count; ++i) {
        out_symbols[i] = (rmt_symbol_word_t){
            .duration0 = high_us,
            .level0 = 1U,
            .duration1 = low_us,
            .level1 = 0U,
        };
    }

    *out_symbol_count = (size_t)pulse_count;
    return ESP_OK;
}

esp_err_t wavegen_build_preview_points(const wavegen_config_t *cfg,
                                       int32_t *out_y,
                                       size_t point_count)
{
    if (cfg == NULL || out_y == NULL || point_count < 8U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!cfg->output_enabled) {
        for (size_t i = 0; i < point_count; ++i) {
            out_y[i] = 0;
        }
        return ESP_OK;
    }

    if (cfg->mode == WAVEGEN_MODE_RMT_BURST) {
        uint16_t pulses = (cfg->burst_pulse_count > 0U) ? cfg->burst_pulse_count : 1U;
        if (pulses > 12U) {
            pulses = 12U;
        }
        size_t active_pts = (point_count * 3U) / 4U;
        size_t pts_per_pulse = active_pts / (size_t)pulses;
        if (pts_per_pulse < 2U) {
            pts_per_pulse = 2U;
        }
        uint32_t total_us = (uint32_t)cfg->burst_high_us + (uint32_t)cfg->burst_low_us;
        if (total_us == 0U) {
            total_us = 2U;
        }
        for (size_t i = 0; i < point_count; ++i) {
            size_t p_idx = i / pts_per_pulse;
            if (p_idx < (size_t)pulses) {
                size_t in_pulse = i % pts_per_pulse;
                size_t high_pts = (pts_per_pulse * (size_t)cfg->burst_high_us + (total_us / 2U)) / total_us;
                if (high_pts == 0U) {
                    high_pts = 1U;
                } else if (high_pts >= pts_per_pulse) {
                    high_pts = pts_per_pulse - 1U;
                }
                out_y[i] = (in_pulse < high_pts) ? 3300 : 0;
            } else {
                out_y[i] = 0;
            }
        }
        return ESP_OK;
    }

    float duty = 50.0f;
    if (cfg->mode == WAVEGEN_MODE_PWM_LEDC || cfg->mode == WAVEGEN_MODE_PWM_MCPWM) {
        duty = clamp_float(cfg->duty_pct, 0.0f, 100.0f);
    } else if (cfg->mode == WAVEGEN_MODE_SERVO) {
        /* Exaggerate servo pulse visibility slightly so 500..2500 us / 20000 us is clearly legible */
        uint16_t pus = cfg->servo_pulse_us;
        if (pus < WAVEGEN_SERVO_MIN_US) {
            pus = WAVEGEN_SERVO_MIN_US;
        } else if (pus > WAVEGEN_SERVO_MAX_US) {
            pus = WAVEGEN_SERVO_MAX_US;
        }
        duty = ((float)pus / (float)WAVEGEN_SERVO_PERIOD_US) * 200.0f;
    }

    size_t cycle_len = point_count / 2U;
    if (cycle_len < 4U) {
        cycle_len = point_count;
    }
    size_t high_pts = (size_t)lroundf((duty / 100.0f) * (float)cycle_len);
    if (duty > 0.0f && high_pts == 0U) {
        high_pts = 1U;
    } else if (duty < 100.0f && high_pts >= cycle_len) {
        high_pts = cycle_len - 1U;
    }

    for (size_t i = 0; i < point_count; ++i) {
        size_t pos = i % cycle_len;
        out_y[i] = (pos < high_pts) ? 3300 : 0;
    }
    return ESP_OK;
}

const char *wavegen_mode_to_str(wavegen_mode_t mode)
{
    switch (mode) {
    case WAVEGEN_MODE_SQUARE:
        return "SQUARE";
    case WAVEGEN_MODE_PWM_LEDC:
        return "LEDC PWM";
    case WAVEGEN_MODE_PWM_MCPWM:
        return "MCPWM";
    case WAVEGEN_MODE_SERVO:
        return "SERVO";
    case WAVEGEN_MODE_RMT_BURST:
        return "BURST";
    default:
        return "UNKNOWN";
    }
}

const char *wavegen_periph_to_str(wavegen_mode_t mode)
{
    switch (mode) {
    case WAVEGEN_MODE_SQUARE:
    case WAVEGEN_MODE_PWM_LEDC:
        return "LEDC";
    case WAVEGEN_MODE_PWM_MCPWM:
        return "MCPWM";
    case WAVEGEN_MODE_SERVO:
    case WAVEGEN_MODE_RMT_BURST:
        return "RMT TX";
    default:
        return "NONE";
    }
}
