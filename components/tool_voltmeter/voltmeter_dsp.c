/**
 * @file voltmeter_dsp.c
 * @brief Pure DSP filtering, outlier-rejecting median/trimmed-mean, PSRAM moving average,
 *        two-point linear calibration, and DC/AC-RMS/True-RMS computation for `tool_voltmeter`.
 */

#include "tool_voltmeter.h"

#include <math.h>
#include <string.h>

#include "psram_buffer.h"

#define VOLTMETER_CAL_MIN_SLOPE        (0.5f)
#define VOLTMETER_CAL_MAX_SLOPE        (2.0f)
#define VOLTMETER_CAL_MAX_ABS_OFFSET   (1000.0f)
#define VOLTMETER_CAL_MIN_DELTA_MV     (10.0f)

void voltmeter_dsp_cal_init_default(voltmeter_cal_t *cal)
{
    if (cal == NULL) {
        return;
    }
    cal->slope = 1.0f;
    cal->offset_mv = 0.0f;
}

esp_err_t voltmeter_dsp_calc_two_point_cal(float raw_low_mv,
                                           float ref_low_mv,
                                           float raw_high_mv,
                                           float ref_high_mv,
                                           voltmeter_cal_t *out_cal)
{
    if (out_cal == NULL ||
        !isfinite(raw_low_mv) || !isfinite(ref_low_mv) ||
        !isfinite(raw_high_mv) || !isfinite(ref_high_mv)) {
        return ESP_ERR_INVALID_ARG;
    }

    float delta_raw = raw_high_mv - raw_low_mv;
    if (fabsf(delta_raw) < VOLTMETER_CAL_MIN_DELTA_MV) {
        return ESP_ERR_INVALID_ARG;
    }

    float slope = (ref_high_mv - ref_low_mv) / delta_raw;
    float offset_mv = ref_low_mv - (slope * raw_low_mv);

    if (!isfinite(slope) || !isfinite(offset_mv) ||
        slope < VOLTMETER_CAL_MIN_SLOPE || slope > VOLTMETER_CAL_MAX_SLOPE ||
        fabsf(offset_mv) > VOLTMETER_CAL_MAX_ABS_OFFSET) {
        return ESP_ERR_INVALID_ARG;
    }

    out_cal->slope = slope;
    out_cal->offset_mv = offset_mv;
    return ESP_OK;
}

float voltmeter_dsp_apply_cal(float raw_mv, const voltmeter_cal_t *cal)
{
    if (!isfinite(raw_mv)) {
        return 0.0f;
    }

    float slope = 1.0f;
    float offset_mv = 0.0f;
    if (cal != NULL && isfinite(cal->slope) && isfinite(cal->offset_mv)) {
        slope = cal->slope;
        offset_mv = cal->offset_mv;
    }

    float calibrated = (slope * raw_mv) + offset_mv;
    if (calibrated < VOLTMETER_RANGE_MIN_MV) {
        calibrated = VOLTMETER_RANGE_MIN_MV;
    } else if (calibrated > VOLTMETER_CLAMP_MAX_MV) {
        calibrated = VOLTMETER_CLAMP_MAX_MV;
    }
    return calibrated;
}

esp_err_t voltmeter_dsp_avg_init(voltmeter_avg_filter_t *filter, size_t window_len)
{
    if (filter == NULL || window_len == 0 || window_len > VOLTMETER_MAX_MOVING_AVG_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(filter, 0, sizeof(*filter));
    filter->window_mv = (float *)psram_buf_calloc(window_len, sizeof(float));
    if (filter->window_mv == NULL) {
        return ESP_ERR_NO_MEM;
    }

    filter->capacity = window_len;
    voltmeter_dsp_avg_reset(filter);
    return ESP_OK;
}

void voltmeter_dsp_avg_free(voltmeter_avg_filter_t *filter)
{
    if (filter == NULL) {
        return;
    }
    if (filter->window_mv != NULL) {
        psram_buf_free(filter->window_mv);
        filter->window_mv = NULL;
    }
    memset(filter, 0, sizeof(*filter));
}

void voltmeter_dsp_avg_reset(voltmeter_avg_filter_t *filter)
{
    if (filter == NULL) {
        return;
    }
    if (filter->window_mv != NULL && filter->capacity > 0) {
        memset(filter->window_mv, 0, filter->capacity * sizeof(float));
    }
    filter->count = 0;
    filter->head = 0;
    filter->running_sum = 0.0;
    filter->min_mv = 0.0f;
    filter->max_mv = 0.0f;
    filter->total_batches = 0;
}

float voltmeter_dsp_avg_push(voltmeter_avg_filter_t *filter, float sample_mv)
{
    if (filter == NULL || filter->window_mv == NULL || filter->capacity == 0) {
        return sample_mv;
    }
    if (!isfinite(sample_mv)) {
        sample_mv = 0.0f;
    }

    filter->window_mv[filter->head] = sample_mv;
    filter->head = (filter->head + 1U) % filter->capacity;
    if (filter->count < filter->capacity) {
        filter->count++;
    }

    /* Recompute exact sum across active window entries to prevent floating-point drift */
    double sum = 0.0;
    for (size_t i = 0; i < filter->count; ++i) {
        sum += (double)filter->window_mv[i];
    }
    filter->running_sum = sum;

    if (filter->total_batches == 0) {
        filter->min_mv = sample_mv;
        filter->max_mv = sample_mv;
    } else {
        if (sample_mv < filter->min_mv) {
            filter->min_mv = sample_mv;
        }
        if (sample_mv > filter->max_mv) {
            filter->max_mv = sample_mv;
        }
    }
    filter->total_batches++;

    return (float)(sum / (double)filter->count);
}

esp_err_t voltmeter_dsp_median_filter_mv(const float *samples_mv,
                                         size_t count,
                                         float *out_median_mv,
                                         float *out_trimmed_mean_mv)
{
    if (samples_mv == NULL || count == 0 || count > VOLTMETER_MAX_BATCH_SAMPLES) {
        return ESP_ERR_INVALID_ARG;
    }

    float sorted[VOLTMETER_MAX_BATCH_SAMPLES];
    for (size_t i = 0; i < count; ++i) {
        float val = isfinite(samples_mv[i]) ? samples_mv[i] : 0.0f;
        size_t j = i;
        while (j > 0 && sorted[j - 1] > val) {
            sorted[j] = sorted[j - 1];
            j--;
        }
        sorted[j] = val;
    }

    float median = 0.0f;
    if ((count & 1U) != 0U) {
        median = sorted[count / 2U];
    } else {
        median = 0.5f * (sorted[(count / 2U) - 1U] + sorted[count / 2U]);
    }

    float trimmed_mean = median;
    if (count >= 4U) {
        size_t trim = count / 4U;
        size_t start = trim;
        size_t end = count - trim;
        double sum = 0.0;
        for (size_t i = start; i < end; ++i) {
            sum += (double)sorted[i];
        }
        trimmed_mean = (float)(sum / (double)(end - start));
    } else {
        double sum = 0.0;
        for (size_t i = 0; i < count; ++i) {
            sum += (double)sorted[i];
        }
        trimmed_mean = (float)(sum / (double)count);
    }

    if (out_median_mv != NULL) {
        *out_median_mv = median;
    }
    if (out_trimmed_mean_mv != NULL) {
        *out_trimmed_mean_mv = trimmed_mean;
    }
    return ESP_OK;
}

esp_err_t voltmeter_dsp_process_batch(const uint16_t *raw_adc,
                                      const float *uncal_mv,
                                      size_t count,
                                      const voltmeter_cal_t *cal,
                                      voltmeter_avg_filter_t *avg_filter,
                                      voltmeter_metrics_t *out_metrics)
{
    if ((raw_adc == NULL && uncal_mv == NULL) ||
        out_metrics == NULL ||
        count == 0 ||
        count > VOLTMETER_MAX_BATCH_SAMPLES) {
        return ESP_ERR_INVALID_ARG;
    }

    float cal_samples[VOLTMETER_MAX_BATCH_SAMPLES];
    double sum_mv = 0.0;
    double sum_sq_mv = 0.0;
    float batch_min = 0.0f;
    float batch_max = 0.0f;

    for (size_t i = 0; i < count; ++i) {
        float raw_mv = 0.0f;
        if (uncal_mv != NULL) {
            raw_mv = uncal_mv[i];
        } else {
            uint16_t code = (raw_adc[i] > 4095U) ? 4095U : raw_adc[i];
            raw_mv = ((float)code * VOLTMETER_RANGE_MAX_MV) / 4095.0f;
        }

        float v_cal = voltmeter_dsp_apply_cal(raw_mv, cal);
        cal_samples[i] = v_cal;
        sum_mv += (double)v_cal;
        sum_sq_mv += (double)v_cal * (double)v_cal;

        if (i == 0) {
            batch_min = v_cal;
            batch_max = v_cal;
        } else {
            if (v_cal < batch_min) {
                batch_min = v_cal;
            }
            if (v_cal > batch_max) {
                batch_max = v_cal;
            }
        }
    }

    float median_mv = 0.0f;
    float trimmed_dc_mv = 0.0f;
    (void)voltmeter_dsp_median_filter_mv(cal_samples, count, &median_mv, &trimmed_dc_mv);

    double mean_mv = sum_mv / (double)count;
    double sum_ac_sq = 0.0;
    for (size_t i = 0; i < count; ++i) {
        double diff = (double)cal_samples[i] - mean_mv;
        sum_ac_sq += diff * diff;
    }

    float vrms_ac = (float)sqrt(sum_ac_sq / (double)count);
    float vrms_true = (float)sqrt(sum_sq_mv / (double)count);
    float vpp = batch_max - batch_min;

    out_metrics->vdc_mv = trimmed_dc_mv;
    out_metrics->median_mv = median_mv;
    out_metrics->vrms_ac_mv = vrms_ac;
    out_metrics->vrms_true_mv = vrms_true;
    out_metrics->vpp_mv = vpp;

    if (avg_filter != NULL && avg_filter->window_mv != NULL) {
        out_metrics->vavg_mv = voltmeter_dsp_avg_push(avg_filter, trimmed_dc_mv);
        out_metrics->vmin_mv = avg_filter->min_mv;
        out_metrics->vmax_mv = avg_filter->max_mv;
        out_metrics->sample_count = avg_filter->total_batches;
    } else {
        out_metrics->vavg_mv = trimmed_dc_mv;
        out_metrics->vmin_mv = batch_min;
        out_metrics->vmax_mv = batch_max;
        out_metrics->sample_count = 1U;
    }

    return ESP_OK;
}
