/**
 * @file tool_scope.h
 * @brief 1-Channel Digital Oscilloscope instrument component (`tool_scope`) for M5CoreAnalyser.
 *
 * Provides high-rate ADC1 capture on `GPIO36` or `GPIO35` into a PSRAM capture buffer,
 * rising/falling/auto trigger detection with hysteresis, timebase decimation and 300-point
 * window extraction, live waveform metrics (`Vpp`, `Vmin`, `Vmax`, `Vavg`, `Vrms`, `Freq`, `Duty`),
 * real-time ADC-to-Speaker audio passthrough, `recorder_sink` frame export, and an LVGL 9 UI.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "tool_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name Oscilloscope Constants & Geometry
 *  @{
 */
#define SCOPE_TOOL_ID                  "scope"
#define SCOPE_CHART_POINTS             (300)
#define SCOPE_HORIZ_DIVS               (10)
#define SCOPE_VERT_DIVS                (6)
#define SCOPE_POINTS_PER_DIV           (SCOPE_CHART_POINTS / SCOPE_HORIZ_DIVS) /**< 30 pts/div */
#define SCOPE_DEFAULT_TRIG_MV          (1650U)
#define SCOPE_DEFAULT_HYST_MV          (40U)
#define SCOPE_MIN_FREQ_VPP_MV          (60.0f)
#define SCOPE_SPK_MIN_CAPTURE_RATE_HZ  (4000U)  /**< Minimum capture rate for speaker passthrough; slower sweeps mute */

#if CONFIG_SPIRAM
#define SCOPE_CAPTURE_BUF_SAMPLES      (16384U) /**< 16K-sample PSRAM capture buffer */
#else
#define SCOPE_CAPTURE_BUF_SAMPLES      (1024U)  /**< Compact capture buffer for PSRAM-less QEMU */
#endif
/** @} */

/**
 * @brief Oscilloscope trigger modes.
 */
typedef enum {
    SCOPE_TRIG_AUTO = 0, /**< Rising-edge trigger with automatic fallback to untriggered trace */
    SCOPE_TRIG_RISING,   /**< Normal rising-edge trigger with hysteresis (holds last trace if untriggered) */
    SCOPE_TRIG_FALLING,  /**< Normal falling-edge trigger with hysteresis (holds last trace if untriggered) */
    SCOPE_TRIG_MODE_COUNT
} scope_trig_mode_t;

/**
 * @brief Oscilloscope horizontal timebase settings (`100 us/div` to `100 ms/div`).
 */
typedef enum {
    SCOPE_TB_100US_DIV = 0,
    SCOPE_TB_200US_DIV,
    SCOPE_TB_500US_DIV,
    SCOPE_TB_1MS_DIV,
    SCOPE_TB_2MS_DIV,
    SCOPE_TB_5MS_DIV,
    SCOPE_TB_10MS_DIV,
    SCOPE_TB_20MS_DIV,
    SCOPE_TB_50MS_DIV,
    SCOPE_TB_100MS_DIV,
    SCOPE_TB_COUNT
} scope_timebase_t;

/**
 * @brief Oscilloscope vertical voltage scale settings (`100 mV/div` to `1000 mV/div`).
 */
typedef enum {
    SCOPE_VSCALE_100MV_DIV = 0,
    SCOPE_VSCALE_200MV_DIV,
    SCOPE_VSCALE_500MV_DIV,
    SCOPE_VSCALE_1000MV_DIV,
    SCOPE_VSCALE_COUNT
} scope_vscale_t;

/**
 * @brief Live oscilloscope waveform measurements computed from a capture buffer.
 */
typedef struct {
    float vpp_mv;     /**< Peak-to-peak voltage (Vmax - Vmin) in mV */
    float vmin_mv;    /**< Minimum sample voltage in mV */
    float vmax_mv;    /**< Maximum sample voltage in mV */
    float vavg_mv;    /**< DC average (mean) voltage in mV */
    float vrms_mv;    /**< True (DC + AC) RMS voltage in mV */
    float vrms_ac_mv; /**< AC-coupled RMS voltage in mV */
    float freq_hz;    /**< Estimated fundamental frequency in Hz (0.0f if flat DC / no edges) */
    float duty_pct;   /**< Estimated positive duty cycle in percent (0.0f..100.0f) */
} scope_metrics_t;

/**
 * @brief Result of a trigger search across a capture buffer.
 */
typedef struct {
    bool triggered;       /**< True if a matching rising/falling edge crossing was found */
    bool auto_fallback;   /**< True if `SCOPE_TRIG_AUTO` fell back to centered untriggered window */
    size_t trigger_index; /**< Sample index of the trigger crossing (or fallback center) */
} scope_trig_result_t;

/**
 * @brief Complete 300-point waveform and metrics frame passed from Core 1 to Core 0 via `psram_double_buf_t`.
 */
typedef struct {
    uint16_t wave_mv[SCOPE_CHART_POINTS]; /**< 300-point display window in mV */
    scope_metrics_t metrics;              /**< Computed waveform metrics */
    scope_trig_result_t trig;             /**< Trigger status for this frame */
    scope_timebase_t timebase;            /**< Active timebase enum */
    scope_vscale_t vscale;                /**< Active vertical scale enum */
    scope_trig_mode_t trig_mode;          /**< Active trigger mode enum */
    uint16_t trig_level_mv;               /**< Active trigger threshold in mV */
    gpio_num_t pin;                       /**< Active input pin (`GPIO36` or `GPIO35`) */
    uint32_t sample_rate_hz;              /**< Effective capture sample rate in Hz */
    uint32_t viewport_us;                  /**< Actual displayed 300-point horizontal span in microseconds */
} scope_frame_t;

/* --------------------------------------------------------------------------
 * Pure DSP, Trigger, Window Extraction & Metric Calculation API (scope_dsp.c)
 * -------------------------------------------------------------------------- */

/**
 * @brief Return the horizontal duration per division in microseconds for `tb`.
 *
 * @param[in] tb Timebase enum value.
 * @return Microseconds per division (`100`..`100000`).
 */
uint32_t scope_dsp_timebase_us_per_div(scope_timebase_t tb);

/**
 * @brief Return the requested sample interval that spans 10 divisions across the chart endpoints.
 *
 * @param[in] tb Target timebase enum value.
 * @return Target interval in nanoseconds, or zero for an invalid timebase.
 */
uint32_t scope_dsp_target_sample_interval_ns(scope_timebase_t tb);

/**
 * @brief Compute the measured horizontal viewport for a uniformly sampled display window.
 *
 * @param[in] sample_rate_hz Effective raw capture sample rate in Hz.
 * @param[in] decimation_step Raw samples between display points (>= 1).
 * @param[in] window_points Number of displayed points (>= 2).
 * @return Displayed endpoint-to-endpoint duration in microseconds, or zero for invalid inputs.
 */
uint32_t scope_dsp_viewport_us(uint32_t sample_rate_hz,
                               uint32_t decimation_step,
                               size_t window_points);

/**
 * @brief Return a compact display string for `tb` (e.g. `"1ms/d"`, `"100us/d"`).
 *
 * @param[in] tb Timebase enum value.
 * @return Static null-terminated string.
 */
const char *scope_dsp_timebase_to_str(scope_timebase_t tb);

/**
 * @brief Return the vertical scale in millivolts per division for `vs`.
 *
 * @param[in] vs Vertical scale enum value.
 * @return Millivolts per division (`100`, `200`, `500`, `1000`).
 */
uint16_t scope_dsp_vscale_mv_per_div(scope_vscale_t vs);

/**
 * @brief Return a compact display string for `vs` (e.g. `"1.0V/d"`, `"500mV/d"`).
 *
 * @param[in] vs Vertical scale enum value.
 * @return Static null-terminated string.
 */
const char *scope_dsp_vscale_to_str(scope_vscale_t vs);

/**
 * @brief Return a compact display string for `mode` (`"AUTO"`, `"RISE"`, `"FALL"`).
 *
 * @param[in] mode Trigger mode enum value.
 * @return Static null-terminated string.
 */
const char *scope_dsp_trig_mode_to_str(scope_trig_mode_t mode);

/**
 * @brief Search a millivolt sample buffer for a rising or falling edge trigger crossing with hysteresis.
 *
 * Hysteresis arming prevents false triggers on noisy signals near `threshold_mv`:
 * - **Rising edge (`SCOPE_TRIG_RISING` / `SCOPE_TRIG_AUTO`)**: The signal must first drop to or
 *   below `max(0, threshold_mv - hysteresis_mv)` to arm the trigger, and subsequently cross from
 *   `< threshold_mv` at `i - 1` to `>= threshold_mv` at `i`.
 * - **Falling edge (`SCOPE_TRIG_FALLING`)**: The signal must first rise to or above
 *   `threshold_mv + hysteresis_mv` to arm the trigger, and subsequently cross from
 *   `> threshold_mv` at `i - 1` to `<= threshold_mv` at `i`.
 * - **Auto fallback (`SCOPE_TRIG_AUTO`)**: If no rising edge crossing is found within the search
 *   range, sets `triggered = false`, `auto_fallback = true`, and `trigger_index = count / 2`.
 *
 * @param[in]  samples_mv          Input array of calibrated sample voltages in mV.
 * @param[in]  count               Total number of samples in `samples_mv` (`>= 2`).
 * @param[in]  mode                Trigger mode (`AUTO`, `RISING`, `FALLING`).
 * @param[in]  threshold_mv        Trigger threshold level in mV (`0..3300`).
 * @param[in]  hysteresis_mv       Hysteresis band in mV (e.g. `40`).
 * @param[in]  pre_trigger_samples Minimum samples to preserve before the trigger index if `count > 2 * pre_trigger_samples`.
 * @param[out] out_res             Destination trigger result structure.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t scope_dsp_find_trigger(const uint16_t *samples_mv,
                                 size_t count,
                                 scope_trig_mode_t mode,
                                 uint16_t threshold_mv,
                                 uint16_t hysteresis_mv,
                                 size_t pre_trigger_samples,
                                 scope_trig_result_t *out_res);

/**
 * @brief Extract a decimated display window centered around `trigger_index`.
 *
 * Places `trigger_index` at the horizontal midpoint (`window_points / 2`) of `out_window_mv`
 * with sample stride `decimation_step >= 1`, clamping the start/end indices cleanly within
 * `[0, count - 1]`.
 *
 * @param[in]  samples_mv      Input array of sample voltages in mV.
 * @param[in]  count           Total samples in `samples_mv` (`>= 1`).
 * @param[in]  trigger_index   Center sample index from `scope_dsp_find_trigger()`.
 * @param[in]  decimation_step Stride in samples between consecutive display points (`>= 1`).
 * @param[out] out_window_mv   Output array of length `window_points`.
 * @param[in]  window_points   Number of points to extract (e.g. `SCOPE_CHART_POINTS = 300`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t scope_dsp_extract_window(const uint16_t *samples_mv,
                                   size_t count,
                                   size_t trigger_index,
                                   uint32_t decimation_step,
                                   uint16_t *out_window_mv,
                                   size_t window_points);

/**
 * @brief Compute `Vpp`, `Vmin`, `Vmax`, `Vavg`, `Vrms`, `Vrms_ac`, `Freq`, and `Duty` from a sample buffer.
 *
 * Uses Schmitt-trigger midpoint hysteresis with sub-sample linear interpolation to estimate
 * frequency and positive duty cycle across complete waveform periods. Safely reports
 * `freq_hz = 0.0f` and `duty_pct = 0.0f` on flat DC lines (`Vpp < SCOPE_MIN_FREQ_VPP_MV`)
 * or 0 V / 3.3 V rail inputs without divide-by-zero.
 *
 * @param[in]  samples_mv     Input array of calibrated sample voltages in mV.
 * @param[in]  count          Number of samples in `samples_mv` (`>= 1`).
 * @param[in]  sample_rate_hz Sampling rate in Hz (`> 0` for frequency estimation).
 * @param[out] out_metrics    Destination metrics structure.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t scope_dsp_compute_metrics(const uint16_t *samples_mv,
                                    size_t count,
                                    uint32_t sample_rate_hz,
                                    scope_metrics_t *out_metrics);

/**
 * @brief Resample a captured 12-bit ADC buffer from `in_rate_hz` to `target_rate_hz` for speaker monitoring.
 *
 * When `in_rate_hz < SCOPE_SPK_MIN_CAPTURE_RATE_HZ` (slow sweep timebases such as `10..100 ms/div`),
 * returns `0` samples so slow acquisitions are muted rather than played back as pitch-shifted bursts.
 * When `in_rate_hz >= SCOPE_SPK_MIN_CAPTURE_RATE_HZ`, linearly interpolates `in_raw` to `target_rate_hz`.
 *
 * @param[in]  in_raw         Input 12-bit ADC sample array.
 * @param[in]  in_count       Number of input samples (`>= 2`).
 * @param[in]  in_rate_hz     Measured capture sample rate in Hz.
 * @param[in]  target_rate_hz Speaker output sample rate in Hz (e.g. `BSP_SPK_DEFAULT_RATE_HZ`).
 * @param[out] out_raw        Output resampled 12-bit ADC buffer.
 * @param[in]  max_out        Maximum capacity of `out_raw`.
 * @return Number of resampled output samples written to `out_raw` (`0` if muted or invalid).
 */
size_t scope_dsp_resample_for_speaker(const uint16_t *in_raw,
                                      size_t in_count,
                                      uint32_t in_rate_hz,
                                      uint32_t target_rate_hz,
                                      uint16_t *out_raw,
                                      size_t max_out);

/* --------------------------------------------------------------------------
 * Oscilloscope Runtime Configuration & Control API (tool_scope.c)
 * -------------------------------------------------------------------------- */

/**
 * @brief Select the active oscilloscope input pin (`GPIO_NUM_36` or `GPIO_NUM_35`).
 *
 * Atomically claims the new pin and releases the previous pin when the oscilloscope is active.
 *
 * @param[in] pin `GPIO_NUM_36` or `GPIO_NUM_35`.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` or `ESP_ERR_INVALID_STATE` on failure.
 */
esp_err_t tool_scope_set_pin(gpio_num_t pin);

/**
 * @brief Get the currently configured oscilloscope input pin.
 *
 * @return `GPIO_NUM_36` or `GPIO_NUM_35`.
 */
gpio_num_t tool_scope_get_pin(void);

/**
 * @brief Set the oscilloscope horizontal timebase.
 *
 * @param[in] tb Target `scope_timebase_t` value.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if out of range.
 */
esp_err_t tool_scope_set_timebase(scope_timebase_t tb);

/**
 * @brief Get the active horizontal timebase.
 *
 * @return Current `scope_timebase_t`.
 */
scope_timebase_t tool_scope_get_timebase(void);

/**
 * @brief Set the oscilloscope vertical voltage scale.
 *
 * @param[in] vs Target `scope_vscale_t` value.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if out of range.
 */
esp_err_t tool_scope_set_vscale(scope_vscale_t vs);

/**
 * @brief Get the active vertical voltage scale.
 *
 * @return Current `scope_vscale_t`.
 */
scope_vscale_t tool_scope_get_vscale(void);

/**
 * @brief Set the oscilloscope trigger mode (`AUTO`, `RISING`, `FALLING`).
 *
 * @param[in] mode Target `scope_trig_mode_t` value.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if out of range.
 */
esp_err_t tool_scope_set_trig_mode(scope_trig_mode_t mode);

/**
 * @brief Get the active trigger mode.
 *
 * @return Current `scope_trig_mode_t`.
 */
scope_trig_mode_t tool_scope_get_trig_mode(void);

/**
 * @brief Set the oscilloscope trigger threshold in millivolts (`0..3300` mV).
 *
 * @param[in] level_mv Trigger threshold in mV (clamped to `0..3300`).
 * @return `ESP_OK` on success.
 */
esp_err_t tool_scope_set_trig_level_mv(uint16_t level_mv);

/**
 * @brief Get the active trigger threshold in millivolts.
 *
 * @return Trigger threshold in mV.
 */
uint16_t tool_scope_get_trig_level_mv(void);

/**
 * @brief Pause or resume oscilloscope waveform acquisition (`STOP`/`RUN`).
 *
 * @param[in] running True to run live capture, false to freeze the trace.
 * @return `ESP_OK` on success.
 */
esp_err_t tool_scope_set_running(bool running);

/**
 * @brief Return true if oscilloscope capture is currently running (`RUN`), false if paused (`STOP`).
 *
 * @return True if running, false if paused.
 */
bool tool_scope_is_running(void);

/**
 * @brief Copy out the most recently published `scope_frame_t` (300-point window + metrics).
 *
 * @param[out] out_frame Destination frame structure.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `out_frame` is `NULL`.
 */
esp_err_t tool_scope_get_latest_frame(scope_frame_t *out_frame);

/**
 * @brief Return the static `analyser_tool_t` descriptor for the Oscilloscope instrument.
 *
 * @return Pointer to `analyser_tool_t` descriptor.
 */
const analyser_tool_t *tool_scope_get_descriptor(void);

/**
 * @brief Register the Oscilloscope tool (`id = "scope"`) and its self-test with `tool_core`.
 *
 * @return `ESP_OK` on success, or error code from `tool_registry_register()`.
 */
esp_err_t tool_scope_register(void);

#ifdef __cplusplus
}
#endif
