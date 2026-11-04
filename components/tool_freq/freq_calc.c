/**
 * @file freq_calc.c
 * @brief Pure calculation routines for `tool_freq`: RMT RX period/duty extraction (`f = 1/T`),
 *        PCNT gate-time pulse counting (`f = N/t_gate`), 5 kHz hysteresis crossover,
 *        PSRAM session moving-average/min/max tracking, and unit formatting.
 */

#include "tool_freq.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "psram_buffer.h"

#define FREQ_MAX_COALESCED_PULSES (256U)

typedef struct {
    uint8_t level;
    double duration_us;
} pulse_seg_t;

uint32_t freq_calc_gate_window_us(freq_gate_window_t gate)
{
    switch (gate) {
        case FREQ_GATE_20MS:
            return 20000U;
        case FREQ_GATE_100MS:
            return 100000U;
        case FREQ_GATE_1000MS:
            return 1000000U;
        default:
            return 0U;
    }
}

const char *freq_calc_gate_window_to_str(freq_gate_window_t gate)
{
    switch (gate) {
        case FREQ_GATE_20MS:
            return "20ms";
        case FREQ_GATE_100MS:
            return "100ms";
        case FREQ_GATE_1000MS:
            return "1000ms";
        default:
            return "?";
    }
}

const char *freq_calc_mode_to_str(freq_mode_t mode)
{
    switch (mode) {
        case FREQ_MODE_AUTO:
            return "AUTO";
        case FREQ_MODE_PERIOD_RMT:
            return "1/T";
        case FREQ_MODE_GATE_PCNT:
            return "GATE";
        default:
            return "?";
    }
}

static void push_or_coalesce_pulse(pulse_seg_t *segs,
                                   size_t *inout_count,
                                   size_t max_segs,
                                   uint8_t level,
                                   double dur_us,
                                   double min_glitch_us)
{
    if (dur_us <= 0.0) {
        return;
    }
    if (dur_us < min_glitch_us) {
        /* Reject sub-threshold glitch pulse */
        return;
    }

    size_t n = *inout_count;
    if (n > 0U && segs[n - 1U].level == level) {
        segs[n - 1U].duration_us += dur_us;
        return;
    }
    if (n < max_segs) {
        segs[n].level = level;
        segs[n].duration_us = dur_us;
        *inout_count = n + 1U;
    }
}

esp_err_t freq_calc_from_rmt_symbols(const rmt_symbol_word_t *symbols,
                                     size_t num_symbols,
                                     uint32_t resolution_hz,
                                     uint32_t min_glitch_ns,
                                     freq_period_result_t *out_result)
{
    if (out_result == NULL || resolution_hz == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out_result, 0, sizeof(*out_result));

    if (symbols == NULL || num_symbols == 0U) {
        return ESP_OK;
    }

    const double tick_to_us = 1000000.0 / (double)resolution_hz;
    const double min_glitch_us = (double)min_glitch_ns / 1000.0;

    pulse_seg_t segs[FREQ_MAX_COALESCED_PULSES];
    size_t seg_count = 0U;

    for (size_t i = 0; i < num_symbols; ++i) {
        uint16_t d0 = symbols[i].duration0;
        uint16_t l0 = symbols[i].level0;
        uint16_t d1 = symbols[i].duration1;
        uint16_t l1 = symbols[i].level1;

        if (d0 == 0U) {
            break;
        }
        push_or_coalesce_pulse(
            segs, &seg_count, FREQ_MAX_COALESCED_PULSES, (uint8_t)(l0 & 1U), (double)d0 * tick_to_us, min_glitch_us);

        if (d1 == 0U) {
            break;
        }
        push_or_coalesce_pulse(
            segs, &seg_count, FREQ_MAX_COALESCED_PULSES, (uint8_t)(l1 & 1U), (double)d1 * tick_to_us, min_glitch_us);
    }

    size_t num_cycles = seg_count / 2U;
    if (num_cycles == 0U) {
        return ESP_OK;
    }

    double sum_high_us = 0.0;
    double sum_low_us = 0.0;
    uint32_t valid_cycles = 0U;

    for (size_t c = 0; c < num_cycles; ++c) {
        const pulse_seg_t *a = &segs[2U * c];
        const pulse_seg_t *b = &segs[2U * c + 1U];
        if (a->level == b->level) {
            continue;
        }
        double high_us = (a->level != 0U) ? a->duration_us : b->duration_us;
        double low_us = (a->level != 0U) ? b->duration_us : a->duration_us;
        if (high_us > 0.0 && low_us > 0.0) {
            sum_high_us += high_us;
            sum_low_us += low_us;
            valid_cycles++;
        }
    }

    if (valid_cycles == 0U) {
        return ESP_OK;
    }

    double avg_high_us = sum_high_us / (double)valid_cycles;
    double avg_low_us = sum_low_us / (double)valid_cycles;
    double avg_period_us = avg_high_us + avg_low_us;
    if (avg_period_us <= 0.0) {
        return ESP_OK;
    }

    double freq_hz = 1000000.0 / avg_period_us;
    double duty_pct = (avg_high_us * 100.0) / avg_period_us;
    if (duty_pct < 0.0) {
        duty_pct = 0.0;
    } else if (duty_pct > 100.0) {
        duty_pct = 100.0;
    }

    out_result->valid = true;
    out_result->freq_hz = (float)freq_hz;
    out_result->period_us = (float)avg_period_us;
    out_result->pulse_width_us = (float)avg_high_us;
    out_result->low_width_us = (float)avg_low_us;
    out_result->duty_pct = (float)duty_pct;
    out_result->rpm = (float)(freq_hz * 60.0);
    out_result->cycles_measured = valid_cycles;
    return ESP_OK;
}

esp_err_t freq_calc_from_gate_counts(uint64_t pulse_count,
                                     uint64_t gate_duration_us,
                                     freq_pcnt_edge_mode_t edge_mode,
                                     freq_gate_result_t *out_result)
{
    if (out_result == NULL || gate_duration_us == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out_result, 0, sizeof(*out_result));
    out_result->pulse_count = pulse_count;
    out_result->gate_duration_us = gate_duration_us;

    if (pulse_count == 0U) {
        return ESP_OK;
    }

    double cycles = (edge_mode == FREQ_PCNT_EDGES_BOTH)
                        ? ((double)pulse_count / 2.0)
                        : (double)pulse_count;
    if (cycles <= 0.0) {
        return ESP_OK;
    }

    double freq_hz = (cycles * 1000000.0) / (double)gate_duration_us;
    if (freq_hz <= 0.0) {
        return ESP_OK;
    }

    out_result->valid = true;
    out_result->freq_hz = (float)freq_hz;
    out_result->period_us = (float)(1000000.0 / freq_hz);
    out_result->rpm = (float)(freq_hz * 60.0);
    return ESP_OK;
}

freq_method_t freq_calc_select_method(freq_mode_t mode,
                                      freq_method_t prev_method,
                                      const freq_period_result_t *period_res,
                                      const freq_gate_result_t *gate_res)
{
    bool p_valid = (period_res != NULL && period_res->valid && period_res->freq_hz > 0.0f);
    bool g_valid = (gate_res != NULL && gate_res->valid && gate_res->freq_hz > 0.0f);

    if (mode == FREQ_MODE_PERIOD_RMT) {
        return p_valid ? FREQ_METHOD_PERIOD : FREQ_METHOD_NONE;
    }
    if (mode == FREQ_MODE_GATE_PCNT) {
        return g_valid ? FREQ_METHOD_GATE : FREQ_METHOD_NONE;
    }

    /* FREQ_MODE_AUTO with [4500 Hz, 5500 Hz] hysteresis around 5 kHz */
    if (!p_valid && !g_valid) {
        return FREQ_METHOD_NONE;
    }
    if (p_valid && !g_valid) {
        return FREQ_METHOD_PERIOD;
    }
    if (!p_valid && g_valid) {
        return FREQ_METHOD_GATE;
    }

    /* Both period_res and gate_res are valid */
    float ref_freq = (gate_res->freq_hz > period_res->freq_hz) ? gate_res->freq_hz : period_res->freq_hz;
    if (prev_method == FREQ_METHOD_GATE) {
        if (ref_freq < FREQ_CROSSOVER_LOW_HZ && period_res->freq_hz < FREQ_CROSSOVER_LOW_HZ) {
            return FREQ_METHOD_PERIOD;
        }
        return FREQ_METHOD_GATE;
    }

    if (period_res->freq_hz > FREQ_CROSSOVER_HIGH_HZ || gate_res->freq_hz > FREQ_CROSSOVER_HIGH_HZ) {
        return FREQ_METHOD_GATE;
    }
    return FREQ_METHOD_PERIOD;
}

esp_err_t freq_calc_tracker_init(freq_session_tracker_t *tracker, size_t window_len)
{
    if (tracker == NULL || window_len == 0U || window_len > 64U) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(tracker, 0, sizeof(*tracker));
    tracker->window_hz = (float *)psram_buf_calloc(window_len, sizeof(float));
    if (tracker->window_hz == NULL) {
        return ESP_ERR_NO_MEM;
    }
    tracker->capacity = window_len;
    return ESP_OK;
}

void freq_calc_tracker_free(freq_session_tracker_t *tracker)
{
    if (tracker == NULL) {
        return;
    }
    if (tracker->window_hz != NULL) {
        psram_buf_free(tracker->window_hz);
        tracker->window_hz = NULL;
    }
    memset(tracker, 0, sizeof(*tracker));
}

void freq_calc_tracker_reset(freq_session_tracker_t *tracker)
{
    if (tracker == NULL) {
        return;
    }
    if (tracker->window_hz != NULL && tracker->capacity > 0U) {
        memset(tracker->window_hz, 0, tracker->capacity * sizeof(float));
    }
    tracker->count = 0U;
    tracker->head = 0U;
    tracker->running_sum = 0.0;
    tracker->min_hz = 0.0f;
    tracker->max_hz = 0.0f;
    tracker->total_windows = 0U;
}

static void tracker_push_freq(freq_session_tracker_t *tracker, float freq_hz)
{
    if (tracker == NULL || tracker->window_hz == NULL || tracker->capacity == 0U || freq_hz <= 0.0f) {
        return;
    }
    if (tracker->count == tracker->capacity) {
        tracker->running_sum -= (double)tracker->window_hz[tracker->head];
    } else {
        tracker->count++;
    }
    tracker->window_hz[tracker->head] = freq_hz;
    tracker->running_sum += (double)freq_hz;
    tracker->head = (tracker->head + 1U) % tracker->capacity;

    if (tracker->total_windows == 0U || freq_hz < tracker->min_hz) {
        tracker->min_hz = freq_hz;
    }
    if (tracker->total_windows == 0U || freq_hz > tracker->max_hz) {
        tracker->max_hz = freq_hz;
    }
    tracker->total_windows++;
}

esp_err_t freq_calc_combine_reading(freq_mode_t mode,
                                    freq_method_t prev_method,
                                    const freq_period_result_t *period_res,
                                    const freq_gate_result_t *gate_res,
                                    freq_session_tracker_t *tracker,
                                    freq_reading_t *out_reading)
{
    if (out_reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out_reading, 0, sizeof(*out_reading));

    freq_method_t method = freq_calc_select_method(mode, prev_method, period_res, gate_res);
    out_reading->active_method = method;

    if (method == FREQ_METHOD_PERIOD && period_res != NULL && period_res->valid) {
        out_reading->signal_detected = true;
        out_reading->freq_hz = period_res->freq_hz;
        out_reading->period_us = period_res->period_us;
        out_reading->pulse_width_us = period_res->pulse_width_us;
        out_reading->low_width_us = period_res->low_width_us;
        out_reading->duty_pct = period_res->duty_pct;
        out_reading->rpm = period_res->rpm;
        out_reading->edge_count = (gate_res != NULL && gate_res->pulse_count > 0U)
                                      ? gate_res->pulse_count
                                      : (uint64_t)period_res->cycles_measured * 2ULL;
        out_reading->gate_duration_us = (gate_res != NULL && gate_res->gate_duration_us > 0U)
                                            ? gate_res->gate_duration_us
                                            : (uint64_t)lround(period_res->period_us * (double)period_res->cycles_measured);
    } else if (method == FREQ_METHOD_GATE && gate_res != NULL && gate_res->valid) {
        out_reading->signal_detected = true;
        out_reading->freq_hz = gate_res->freq_hz;
        out_reading->period_us = gate_res->period_us;
        out_reading->rpm = gate_res->rpm;
        out_reading->edge_count = gate_res->pulse_count;
        out_reading->gate_duration_us = gate_res->gate_duration_us;

        if (period_res != NULL && period_res->valid) {
            out_reading->duty_pct = period_res->duty_pct;
            out_reading->pulse_width_us = out_reading->period_us * (period_res->duty_pct / 100.0f);
            out_reading->low_width_us = out_reading->period_us - out_reading->pulse_width_us;
        } else {
            out_reading->duty_pct = 50.0f;
            out_reading->pulse_width_us = out_reading->period_us * 0.5f;
            out_reading->low_width_us = out_reading->period_us * 0.5f;
        }
    } else {
        out_reading->signal_detected = false;
        out_reading->active_method = FREQ_METHOD_NONE;
        if (gate_res != NULL) {
            out_reading->gate_duration_us = gate_res->gate_duration_us;
        }
    }

    if (out_reading->signal_detected && tracker != NULL) {
        tracker_push_freq(tracker, out_reading->freq_hz);
    }

    if (tracker != NULL && tracker->count > 0U) {
        out_reading->fmin_hz = tracker->min_hz;
        out_reading->fmax_hz = tracker->max_hz;
        out_reading->favg_hz = (float)(tracker->running_sum / (double)tracker->count);
        out_reading->sample_Windows = tracker->total_windows;
    }

    return ESP_OK;
}

void freq_calc_format_freq(float freq_hz, char *out_buf, size_t buf_len)
{
    if (out_buf == NULL || buf_len == 0U) {
        return;
    }
    if (!isfinite(freq_hz) || freq_hz <= 0.0f) {
        (void)snprintf(out_buf, buf_len, "0.00 Hz");
        return;
    }

    if (freq_hz >= 1000000.0f) {
        double mhz = (double)freq_hz / 1000000.0;
        long mhz_milli = lround(mhz * 1000.0);
        (void)snprintf(out_buf,
                       buf_len,
                       "%ld.%03ld MHz",
                       mhz_milli / 1000L,
                       labs(mhz_milli % 1000L));
    } else if (freq_hz >= 1000.0f) {
        double khz = (double)freq_hz / 1000.0;
        long khz_milli = lround(khz * 1000.0);
        (void)snprintf(out_buf,
                       buf_len,
                       "%ld.%03ld kHz",
                       khz_milli / 1000L,
                       labs(khz_milli % 1000L));
    } else {
        long hz_centi = lround((double)freq_hz * 100.0);
        (void)snprintf(out_buf,
                       buf_len,
                       "%ld.%02ld Hz",
                       hz_centi / 100L,
                       labs(hz_centi % 100L));
    }
}
