/**
 * @file scope_dsp.c
 * @brief Pure DSP trigger detection with hysteresis, timebase window extraction, and
 *        waveform metric computation (`Vpp`, `Vmin`, `Vmax`, `Vavg`, `Vrms`, `Freq`, `Duty`)
 *        for `tool_scope`.
 */

#include "tool_scope.h"

#include <math.h>
#include <string.h>

static const uint32_t s_tb_us_per_div[SCOPE_TB_COUNT] = {
    100U,    /* SCOPE_TB_100US_DIV */
    200U,    /* SCOPE_TB_200US_DIV */
    500U,    /* SCOPE_TB_500US_DIV */
    1000U,   /* SCOPE_TB_1MS_DIV   */
    2000U,   /* SCOPE_TB_2MS_DIV   */
    5000U,   /* SCOPE_TB_5MS_DIV   */
    10000U,  /* SCOPE_TB_10MS_DIV  */
    20000U,  /* SCOPE_TB_20MS_DIV  */
    50000U,  /* SCOPE_TB_50MS_DIV  */
    100000U, /* SCOPE_TB_100MS_DIV */
};

static const char *const s_tb_labels[SCOPE_TB_COUNT] = {
    "100us/d",
    "200us/d",
    "500us/d",
    "1ms/d",
    "2ms/d",
    "5ms/d",
    "10ms/d",
    "20ms/d",
    "50ms/d",
    "100ms/d",
};

static const uint16_t s_vscale_mv_per_div[SCOPE_VSCALE_COUNT] = {
    100U,  /* SCOPE_VSCALE_100MV_DIV  */
    200U,  /* SCOPE_VSCALE_200MV_DIV  */
    500U,  /* SCOPE_VSCALE_500MV_DIV  */
    1000U, /* SCOPE_VSCALE_1000MV_DIV */
};

static const char *const s_vscale_labels[SCOPE_VSCALE_COUNT] = {
    "100mV/d",
    "200mV/d",
    "500mV/d",
    "1.0V/d",
};

uint32_t scope_dsp_timebase_us_per_div(scope_timebase_t tb)
{
    if (tb < 0 || tb >= SCOPE_TB_COUNT) {
        return 1000U;
    }
    return s_tb_us_per_div[tb];
}

uint32_t scope_dsp_target_sample_interval_ns(scope_timebase_t tb)
{
    if (tb < 0 || tb >= SCOPE_TB_COUNT) {
        return 0U;
    }

    uint64_t viewport_ns = (uint64_t)s_tb_us_per_div[tb] * SCOPE_HORIZ_DIVS * 1000ULL;
    uint64_t interval_ns = (viewport_ns + (SCOPE_CHART_POINTS - 1U) / 2U) /
                           (SCOPE_CHART_POINTS - 1U);
    return (interval_ns > UINT32_MAX) ? UINT32_MAX : (uint32_t)interval_ns;
}

uint32_t scope_dsp_viewport_us(uint32_t sample_rate_hz,
                               uint32_t decimation_step,
                               size_t window_points)
{
    if (sample_rate_hz == 0U || decimation_step == 0U || window_points < 2U) {
        return 0U;
    }

    uint64_t duration_numerator = (uint64_t)(window_points - 1U) *
                                  decimation_step * 1000000ULL;
    uint64_t duration_us = (duration_numerator + sample_rate_hz / 2U) / sample_rate_hz;
    return (duration_us > UINT32_MAX) ? UINT32_MAX : (uint32_t)duration_us;
}

const char *scope_dsp_timebase_to_str(scope_timebase_t tb)
{
    if (tb < 0 || tb >= SCOPE_TB_COUNT) {
        return "1ms/d";
    }
    return s_tb_labels[tb];
}

uint16_t scope_dsp_vscale_mv_per_div(scope_vscale_t vs)
{
    if (vs < 0 || vs >= SCOPE_VSCALE_COUNT) {
        return 1000U;
    }
    return s_vscale_mv_per_div[vs];
}

const char *scope_dsp_vscale_to_str(scope_vscale_t vs)
{
    if (vs < 0 || vs >= SCOPE_VSCALE_COUNT) {
        return "1.0V/d";
    }
    return s_vscale_labels[vs];
}

const char *scope_dsp_trig_mode_to_str(scope_trig_mode_t mode)
{
    switch (mode) {
    case SCOPE_TRIG_AUTO:
        return "AUTO";
    case SCOPE_TRIG_RISING:
        return "RISE";
    case SCOPE_TRIG_FALLING:
        return "FALL";
    default:
        return "AUTO";
    }
}

esp_err_t scope_dsp_find_trigger(const uint16_t *samples_mv,
                                 size_t count,
                                 scope_trig_mode_t mode,
                                 uint16_t threshold_mv,
                                 uint16_t hysteresis_mv,
                                 size_t pre_trigger_samples,
                                 scope_trig_result_t *out_res)
{
    if (samples_mv == NULL || out_res == NULL || count < 2U ||
        mode < 0 || mode >= SCOPE_TRIG_MODE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t start_idx = 1U;
    size_t end_idx = count;
    if (pre_trigger_samples > 0U && count > (2U * pre_trigger_samples + 2U)) {
        start_idx = pre_trigger_samples;
        end_idx = count - pre_trigger_samples;
    }

    bool search_falling = (mode == SCOPE_TRIG_FALLING);
    bool armed = false;

    if (!search_falling) {
        uint16_t arm_low = (threshold_mv > hysteresis_mv)
                               ? (uint16_t)(threshold_mv - hysteresis_mv)
                               : 0U;
        for (size_t i = 0; i < end_idx; ++i) {
            uint16_t v = samples_mv[i];
            if (!armed) {
                if (v <= arm_low) {
                    armed = true;
                }
            } else {
                if (i >= start_idx && samples_mv[i - 1U] < threshold_mv && v >= threshold_mv) {
                    out_res->triggered = true;
                    out_res->auto_fallback = false;
                    out_res->trigger_index = i;
                    return ESP_OK;
                }
                if (v > threshold_mv && i < start_idx) {
                    /* Crossed before valid window; require re-arming below arm_low */
                    armed = false;
                }
            }
        }
    } else {
        uint32_t arm_high_u32 = (uint32_t)threshold_mv + (uint32_t)hysteresis_mv;
        uint16_t arm_high = (arm_high_u32 > 65535U) ? 65535U : (uint16_t)arm_high_u32;
        for (size_t i = 0; i < end_idx; ++i) {
            uint16_t v = samples_mv[i];
            if (!armed) {
                if (v >= arm_high) {
                    armed = true;
                }
            } else {
                if (i >= start_idx && samples_mv[i - 1U] > threshold_mv && v <= threshold_mv) {
                    out_res->triggered = true;
                    out_res->auto_fallback = false;
                    out_res->trigger_index = i;
                    return ESP_OK;
                }
                if (v < threshold_mv && i < start_idx) {
                    armed = false;
                }
            }
        }
    }

    out_res->triggered = false;
    out_res->auto_fallback = (mode == SCOPE_TRIG_AUTO);
    out_res->trigger_index = count / 2U;
    return ESP_OK;
}

esp_err_t scope_dsp_extract_window(const uint16_t *samples_mv,
                                   size_t count,
                                   size_t trigger_index,
                                   uint32_t decimation_step,
                                   uint16_t *out_window_mv,
                                   size_t window_points)
{
    if (samples_mv == NULL || out_window_mv == NULL ||
        count == 0U || window_points == 0U || decimation_step == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t half_win = (int64_t)(window_points / 2U);
    int64_t step = (int64_t)decimation_step;
    int64_t trig = (int64_t)((trigger_index < count) ? trigger_index : (count / 2U));
    int64_t max_idx = (int64_t)count - 1;

    for (size_t k = 0; k < window_points; ++k) {
        int64_t src_idx = trig + ((int64_t)k - half_win) * step;
        if (src_idx < 0) {
            src_idx = 0;
        } else if (src_idx > max_idx) {
            src_idx = max_idx;
        }
        out_window_mv[k] = samples_mv[(size_t)src_idx];
    }

    return ESP_OK;
}

esp_err_t scope_dsp_compute_metrics(const uint16_t *samples_mv,
                                    size_t count,
                                    uint32_t sample_rate_hz,
                                    scope_metrics_t *out_metrics)
{
    if (samples_mv == NULL || out_metrics == NULL || count == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t min_v = samples_mv[0];
    uint16_t max_v = samples_mv[0];
    double sum_v = 0.0;
    double sum_sq_v = 0.0;

    for (size_t i = 0; i < count; ++i) {
        uint16_t v = samples_mv[i];
        if (v < min_v) {
            min_v = v;
        }
        if (v > max_v) {
            max_v = v;
        }
        double dv = (double)v;
        sum_v += dv;
        sum_sq_v += dv * dv;
    }

    double mean_v = sum_v / (double)count;
    double sum_ac_sq = 0.0;
    for (size_t i = 0; i < count; ++i) {
        double diff = (double)samples_mv[i] - mean_v;
        sum_ac_sq += diff * diff;
    }

    float vmin = (float)min_v;
    float vmax = (float)max_v;
    float vpp = vmax - vmin;

    out_metrics->vmin_mv = vmin;
    out_metrics->vmax_mv = vmax;
    out_metrics->vpp_mv = vpp;
    out_metrics->vavg_mv = (float)mean_v;
    out_metrics->vrms_mv = (float)sqrt(sum_sq_v / (double)count);
    out_metrics->vrms_ac_mv = (float)sqrt(sum_ac_sq / (double)count);
    out_metrics->freq_hz = 0.0f;
    out_metrics->duty_pct = 0.0f;

    if (vpp < SCOPE_MIN_FREQ_VPP_MV || sample_rate_hz == 0U || count < 4U) {
        return ESP_OK;
    }

    float mid = 0.5f * (vmin + vmax);
    float hyst = fmaxf(20.0f, 0.15f * vpp);
    float arm_low = mid - hyst;
    float arm_high = mid + hyst;

    int state = 0; /* 0 = uninitialized, -1 = armed low, +1 = armed high */
    bool has_pending_cross = false;
    double pending_cross_pos = 0.0;
    size_t pending_cross_idx = 0U;

    double first_rise_pos = 0.0;
    double last_rise_pos = 0.0;
    size_t first_rise_idx = 0U;
    size_t last_rise_idx = 0U;
    size_t rise_count = 0U;

    for (size_t i = 0; i < count; ++i) {
        float v = (float)samples_mv[i];
        if (state == 0) {
            if (v <= arm_low) {
                state = -1;
            } else if (v >= arm_high) {
                state = +1;
            }
        } else if (state == -1) {
            if (!has_pending_cross && i > 0U && (float)samples_mv[i - 1U] < mid && v >= mid) {
                float v_prev = (float)samples_mv[i - 1U];
                float denom = v - v_prev;
                double frac = (denom > 0.001f) ? (double)((mid - v_prev) / denom) : 0.0;
                pending_cross_pos = (double)(i - 1U) + frac;
                pending_cross_idx = i;
                has_pending_cross = true;
            }
            if (v <= arm_low) {
                has_pending_cross = false;
            } else if (v >= arm_high) {
                if (!has_pending_cross) {
                    pending_cross_pos = (double)i;
                    pending_cross_idx = i;
                }
                if (rise_count == 0U) {
                    first_rise_pos = pending_cross_pos;
                    first_rise_idx = pending_cross_idx;
                }
                last_rise_pos = pending_cross_pos;
                last_rise_idx = pending_cross_idx;
                rise_count++;
                has_pending_cross = false;
                state = +1;
            }
        } else {
            /* state == +1 */
            if (v <= arm_low) {
                state = -1;
                has_pending_cross = false;
            }
        }
    }

    if (rise_count >= 2U && (last_rise_pos - first_rise_pos) > 1.0 && last_rise_idx > first_rise_idx) {
        double avg_period_samples = (last_rise_pos - first_rise_pos) / (double)(rise_count - 1U);
        out_metrics->freq_hz = (float)((double)sample_rate_hz / avg_period_samples);

        size_t total_span = last_rise_idx - first_rise_idx;
        double high_weight = 0.0;
        for (size_t j = first_rise_idx; j < last_rise_idx; ++j) {
            float v = (float)samples_mv[j];
            if (v > mid) {
                high_weight += 1.0;
            } else if (fabsf(v - mid) <= 0.5f) {
                high_weight += 0.5;
            }
        }
        out_metrics->duty_pct = (float)((100.0 * high_weight) / (double)total_span);
    }

    return ESP_OK;
}

size_t scope_dsp_resample_for_speaker(const uint16_t *in_raw,
                                      size_t in_count,
                                      uint32_t in_rate_hz,
                                      uint32_t target_rate_hz,
                                      uint16_t *out_raw,
                                      size_t max_out)
{
    if (in_raw == NULL || out_raw == NULL || in_count < 2U || max_out == 0U ||
        in_rate_hz < SCOPE_SPK_MIN_CAPTURE_RATE_HZ || target_rate_hz == 0U) {
        return 0U;
    }

    uint64_t span_out = ((uint64_t)(in_count - 1U) * (uint64_t)target_rate_hz) /
                        (uint64_t)in_rate_hz + 1ULL;
    size_t out_count = (span_out > (uint64_t)max_out) ? max_out : (size_t)span_out;
    if (out_count == 0U) {
        return 0U;
    }

    if (in_rate_hz == target_rate_hz) {
        size_t copy_n = (out_count < in_count) ? out_count : in_count;
        for (size_t k = 0; k < copy_n; ++k) {
            out_raw[k] = (in_raw[k] > 4095U) ? 4095U : in_raw[k];
        }
        return copy_n;
    }

    for (size_t k = 0; k < out_count; ++k) {
        uint64_t pos_fp = (((uint64_t)k * (uint64_t)in_rate_hz) << 16) /
                          (uint64_t)target_rate_hz;
        size_t idx = (size_t)(pos_fp >> 16);
        uint32_t frac = (uint32_t)(pos_fp & 0xFFFFU);
        if (idx + 1U < in_count) {
            uint32_t s0 = (in_raw[idx] > 4095U) ? 4095U : in_raw[idx];
            uint32_t s1 = (in_raw[idx + 1U] > 4095U) ? 4095U : in_raw[idx + 1U];
            uint32_t interp = (s0 * (65536U - frac) + s1 * frac + 32768U) >> 16;
            out_raw[k] = (uint16_t)((interp > 4095U) ? 4095U : interp);
        } else {
            uint16_t last = in_raw[in_count - 1U];
            out_raw[k] = (last > 4095U) ? 4095U : last;
        }
    }

    return out_count;
}
