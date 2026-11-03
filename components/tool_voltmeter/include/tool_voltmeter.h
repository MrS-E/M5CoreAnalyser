/**
 * @file tool_voltmeter.h
 * @brief 0–3.3 V DC and AC-RMS Voltmeter instrument component (`tool_voltmeter`) for M5CoreAnalyser.
 *
 * Provides calibrated ADC1 sampling on `GPIO36` (Port B / M5-Bus) or `GPIO35` (M5-Bus),
 * PSRAM-backed median/trimmed-mean outlier rejection and moving-average filtering,
 * two-point linear slope/offset calibration persisted in NVS, min/max/avg tracking,
 * real-time speaker audio passthrough, `recorder_sink` frame export, and an LVGL 9 UI.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "tool_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name Voltmeter Constants & Defaults
 *  @{
 */
#define VOLTMETER_TOOL_ID                  "voltmeter"
#define VOLTMETER_DEFAULT_BATCH_SAMPLES    (64)
#define VOLTMETER_MAX_BATCH_SAMPLES        (256)
#define VOLTMETER_DEFAULT_MOVING_AVG_LEN   (16)
#define VOLTMETER_MAX_MOVING_AVG_LEN       (128)
#define VOLTMETER_DEFAULT_SAMPLE_RATE_HZ   (16000) /**< Nominal speaker-monitor output rate, not ADC/recorder rate */
#define VOLTMETER_RANGE_MIN_MV             (0.0f)
#define VOLTMETER_RANGE_MAX_MV             (3300.0f)
#define VOLTMETER_CLAMP_MAX_MV             (3600.0f)
/** @} */

/**
 * @brief Linear two-point calibration parameters: \f$ V_{\text{cal}} = \text{slope} \cdot V_{\text{raw}} + \text{offset\_mv} \f$.
 */
typedef struct {
    float slope;     /**< Multiplicative slope factor (unity = 1.0f, valid range 0.5f..2.0f) */
    float offset_mv; /**< Additive offset in millivolts (unity = 0.0f, valid range -1000..+1000 mV) */
} voltmeter_cal_t;

/**
 * @brief Computed DC, AC-RMS, True-RMS, and session tracking metrics for the Voltmeter.
 */
typedef struct {
    float vdc_mv;          /**< Outlier-rejected & calibrated DC voltage of the current batch (mV) */
    float vrms_ac_mv;      /**< AC-coupled RMS voltage: sqrt(mean((v[i] - mean)^2)) (mV) */
    float vrms_true_mv;    /**< True (DC + AC) RMS voltage: sqrt(mean(v[i]^2)) (mV) */
    float vpp_mv;          /**< Peak-to-peak voltage within the current batch (mV) */
    float median_mv;       /**< Exact median sample voltage of the current batch (mV) */
    float vmin_mv;         /**< Minimum calibrated batch DC voltage observed since reset (mV) */
    float vmax_mv;         /**< Maximum calibrated batch DC voltage observed since reset (mV) */
    float vavg_mv;         /**< PSRAM moving-average filtered DC voltage (mV) */
    uint32_t sample_count; /**< Total batches accumulated since last reset */
} voltmeter_metrics_t;

/**
 * @brief PSRAM-backed moving-average and min/max session tracker state.
 */
typedef struct {
    float *window_mv;       /**< Circular window buffer allocated via `psram_buf_calloc()` */
    size_t capacity;        /**< Window length in samples (1..VOLTMETER_MAX_MOVING_AVG_LEN) */
    size_t count;           /**< Current number of valid entries in `window_mv` */
    size_t head;            /**< Next write index in `window_mv` */
    double running_sum;     /**< Running sum of active window entries */
    float min_mv;           /**< Tracked session minimum voltage in mV */
    float max_mv;           /**< Tracked session maximum voltage in mV */
    uint32_t total_batches; /**< Total batches pushed since last reset */
} voltmeter_avg_filter_t;

/* --------------------------------------------------------------------------
 * Pure DSP, Filtering & Calibration API (voltmeter_dsp.c)
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialize a `voltmeter_cal_t` structure to unity calibration (`slope = 1.0f`, `offset_mv = 0.0f`).
 *
 * @param[out] cal Pointer to calibration structure.
 */
void voltmeter_dsp_cal_init_default(voltmeter_cal_t *cal);

/**
 * @brief Compute linear two-point calibration (`slope` and `offset_mv`) from two reference points.
 *
 * Solves:
 * - \f$ \text{ref\_low\_mv} = \text{slope} \cdot \text{raw\_low\_mv} + \text{offset\_mv} \f$
 * - \f$ \text{ref\_high\_mv} = \text{slope} \cdot \text{raw\_high\_mv} + \text{offset\_mv} \f$
 *
 * Rejects degenerate points (`|raw_high_mv - raw_low_mv| < 10.0f`), NaN/Inf values, or
 * out-of-bounds slopes (`< 0.5f` or `> 2.0f`) and offsets (`< -1000.0f` or `> 1000.0f`)
 * with `ESP_ERR_INVALID_ARG`.
 *
 * @param[in]  raw_low_mv  Measured uncalibrated low voltage in mV.
 * @param[in]  ref_low_mv  True reference low voltage in mV.
 * @param[in]  raw_high_mv Measured uncalibrated high voltage in mV.
 * @param[in]  ref_high_mv True reference high voltage in mV.
 * @param[out] out_cal     Destination calibration structure.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid/degenerate inputs.
 */
esp_err_t voltmeter_dsp_calc_two_point_cal(float raw_low_mv,
                                           float ref_low_mv,
                                           float raw_high_mv,
                                           float ref_high_mv,
                                           voltmeter_cal_t *out_cal);

/**
 * @brief Apply linear slope/offset calibration to a raw mV reading and clamp to `[0, VOLTMETER_CLAMP_MAX_MV]`.
 *
 * @param[in] raw_mv Uncalibrated millivolt value.
 * @param[in] cal    Calibration parameters (if `NULL`, unity calibration is applied).
 * @return Calibrated and clamped voltage in mV.
 */
float voltmeter_dsp_apply_cal(float raw_mv, const voltmeter_cal_t *cal);

/**
 * @brief Allocate and initialize a PSRAM-backed moving-average and min/max tracker.
 *
 * @param[out] filter     Pointer to filter structure to initialize.
 * @param[in]  window_len Moving-average window length (`1`..`VOLTMETER_MAX_MOVING_AVG_LEN`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` or `ESP_ERR_NO_MEM` on failure.
 */
esp_err_t voltmeter_dsp_avg_init(voltmeter_avg_filter_t *filter, size_t window_len);

/**
 * @brief Free the PSRAM buffer owned by a `voltmeter_avg_filter_t`.
 *
 * @param[in,out] filter Pointer to filter structure (safe to pass `NULL`).
 */
void voltmeter_dsp_avg_free(voltmeter_avg_filter_t *filter);

/**
 * @brief Reset the moving-average window and min/max session counters without freeing memory.
 *
 * @param[in,out] filter Pointer to initialized filter structure.
 */
void voltmeter_dsp_avg_reset(voltmeter_avg_filter_t *filter);

/**
 * @brief Push a new batch DC voltage into the moving-average window and update min/max/count.
 *
 * @param[in,out] filter    Pointer to initialized filter structure.
 * @param[in]     sample_mv Batch DC voltage in mV.
 * @return Updated moving-average voltage in mV.
 */
float voltmeter_dsp_avg_push(voltmeter_avg_filter_t *filter, float sample_mv);

/**
 * @brief Compute the exact median and 25%-trimmed interquartile mean of a millivolt sample batch.
 *
 * Sorts a copy of `samples_mv` (`1`..`VOLTMETER_MAX_BATCH_SAMPLES`) and rejects the lowest 25%
 * and highest 25% of samples when `count >= 4` so impulse spikes cannot corrupt the DC reading.
 *
 * @param[in]  samples_mv          Input array of millivolt samples.
 * @param[in]  count               Number of samples (`1`..`VOLTMETER_MAX_BATCH_SAMPLES`).
 * @param[out] out_median_mv       Optional pointer to receive exact median in mV.
 * @param[out] out_trimmed_mean_mv Optional pointer to receive outlier-rejected mean in mV.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t voltmeter_dsp_median_filter_mv(const float *samples_mv,
                                         size_t count,
                                         float *out_median_mv,
                                         float *out_trimmed_mean_mv);

/**
 * @brief Process a batch of ADC samples into calibrated DC, AC-RMS, True-RMS, Vpp, and moving-average metrics.
 *
 * If `uncal_mv` is non-NULL, uses `uncal_mv[i]` as the uncalibrated millivolt input for sample `i`.
 * Otherwise, converts `raw_adc[i]` (`0..4095`) using the linear `3300 mV / 4095` scale.
 * Applies `cal`, computes outlier-rejected DC (`vdc_mv`), `median_mv`, `vrms_ac_mv`, `vrms_true_mv`,
 * and `vpp_mv`, and updates `avg_filter` (if non-NULL) to populate `vmin_mv`, `vmax_mv`, `vavg_mv`,
 * and `sample_count`.
 *
 * @param[in]     raw_adc     Optional array of raw 12-bit ADC counts (`0..4095`).
 * @param[in]     uncal_mv    Optional array of uncalibrated mV values (at least one of `raw_adc` or `uncal_mv` must be non-NULL).
 * @param[in]     count       Number of samples in the batch (`1`..`VOLTMETER_MAX_BATCH_SAMPLES`).
 * @param[in]     cal         Optional pointer to calibration parameters (`NULL` = unity).
 * @param[in,out] avg_filter  Optional pointer to initialized moving-average/min-max tracker.
 * @param[out]    out_metrics Destination metrics structure.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t voltmeter_dsp_process_batch(const uint16_t *raw_adc,
                                      const float *uncal_mv,
                                      size_t count,
                                      const voltmeter_cal_t *cal,
                                      voltmeter_avg_filter_t *avg_filter,
                                      voltmeter_metrics_t *out_metrics);

/* --------------------------------------------------------------------------
 * NVS Calibration Persistence & Runtime Control API (tool_voltmeter.c)
 * -------------------------------------------------------------------------- */

/**
 * @brief Save two-point calibration parameters for `pin` (`GPIO36` or `GPIO35`) to NVS.
 *
 * Also updates the active in-memory calibration if `pin` matches the selected pin.
 *
 * @param[in] pin Target analog input pin (`GPIO_NUM_36` or `GPIO_NUM_35`).
 * @param[in] cal Calibration parameters to persist.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid pin or calibration bounds.
 */
esp_err_t tool_voltmeter_save_cal(gpio_num_t pin, const voltmeter_cal_t *cal);

/**
 * @brief Load two-point calibration parameters for `pin` from NVS.
 *
 * If no stored calibration exists in NVS for `pin`, initializes `*out_cal` to unity
 * (`slope = 1.0f`, `offset_mv = 0.0f`) and returns `ESP_OK`.
 *
 * @param[in]  pin     Target analog input pin (`GPIO_NUM_36` or `GPIO_NUM_35`).
 * @param[out] out_cal Destination calibration structure.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t tool_voltmeter_load_cal(gpio_num_t pin, voltmeter_cal_t *out_cal);

/**
 * @brief Reset calibration for `pin` to unity (`slope = 1.0f`, `offset_mv = 0.0f`) in NVS and RAM.
 *
 * @param[in] pin Target analog input pin (`GPIO_NUM_36` or `GPIO_NUM_35`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid pin.
 */
esp_err_t tool_voltmeter_reset_cal(gpio_num_t pin);

/**
 * @brief Select the active analog input pin (`GPIO_NUM_36` or `GPIO_NUM_35`).
 *
 * If the Voltmeter is currently active in the foreground, atomically claims the new pin via
 * `bsp_pin_claim()` and releases the old pin. If the new pin is owned by another tool, rolls
 * back and returns `ESP_ERR_INVALID_STATE`.
 *
 * @param[in] pin `GPIO_NUM_36` or `GPIO_NUM_35`.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` for unsupported pins, or `ESP_ERR_INVALID_STATE` on conflict.
 */
esp_err_t tool_voltmeter_set_pin(gpio_num_t pin);

/**
 * @brief Get the currently configured analog input pin (`GPIO_NUM_36` or `GPIO_NUM_35`).
 *
 * @return Configured `gpio_num_t`.
 */
gpio_num_t tool_voltmeter_get_pin(void);

/**
 * @brief Freeze or unfreeze live display updates (`HOLD` mode).
 *
 * @param[in] hold True to freeze display metrics, false to resume live updates.
 * @return `ESP_OK` on success.
 */
esp_err_t tool_voltmeter_set_hold(bool hold);

/**
 * @brief Return true if the Voltmeter is currently in `HOLD` mode.
 *
 * @return True if held, false if live.
 */
bool tool_voltmeter_is_hold(void);

/**
 * @brief Reset the running `Vmin`, `Vmax`, moving-average `Vavg`, and sample counter.
 *
 * @return `ESP_OK` on success.
 */
esp_err_t tool_voltmeter_reset_stats(void);

/**
 * @brief Copy out the latest computed `voltmeter_metrics_t` snapshot.
 *
 * @param[out] out_metrics Destination structure.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `out_metrics` is `NULL`.
 */
esp_err_t tool_voltmeter_get_latest_metrics(voltmeter_metrics_t *out_metrics);

/**
 * @brief Return the static `analyser_tool_t` descriptor for the Voltmeter instrument.
 *
 * @return Pointer to `analyser_tool_t` descriptor.
 */
const analyser_tool_t *tool_voltmeter_get_descriptor(void);

/**
 * @brief Register the Voltmeter tool (`id = "voltmeter"`) and its self-test with `tool_core`.
 *
 * @return `ESP_OK` on success, or error code from `tool_registry_register()`.
 */
esp_err_t tool_voltmeter_register(void);

#ifdef __cplusplus
}
#endif
