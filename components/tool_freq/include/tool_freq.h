/**
 * @file tool_freq.h
 * @brief Digital Frequency Meter instrument component (`tool_freq`) for M5CoreAnalyser.
 *
 * Combines low-frequency edge-period measurement (\f$ f = 1/T \f$ via ESP32 RMT RX + GPTimer)
 * and high-frequency gate-time pulse counting (\f$ f = N / t_{\text{gate}} \f$ via ESP32 PCNT + GPTimer)
 * with automatic hysteresis crossover around `5 kHz`. Reports frequency (`Hz`/`kHz`/`MHz`),
 * period, high/low pulse widths, duty cycle, RPM, and session min/max/average statistics.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/rmt_types.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "tool_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name Frequency Meter Constants & Thresholds
 *  @{
 */
#define FREQ_TOOL_ID                    "freq"
#define FREQ_RMT_RESOLUTION_HZ          (1000000U)  /**< 1 MHz (1 us per tick) RMT RX resolution */
#define FREQ_GPTIMER_RESOLUTION_HZ      (1000000U)  /**< 1 MHz (1 us per tick) GPTimer gate resolution */
#define FREQ_DEFAULT_GLITCH_NS          (1000U)     /**< 1 us default glitch filter threshold */
#define FREQ_PCNT_HIGH_LIMIT            (30000)     /**< PCNT 16-bit watch point for overflow accumulation */
#define FREQ_PCNT_LOW_LIMIT             (-30000)
#define FREQ_CROSSOVER_NOMINAL_HZ       (5000.0f)   /**< Nominal 5 kHz period-to-gate crossover frequency */
#define FREQ_CROSSOVER_LOW_HZ           (4500.0f)   /**< Switch from GATE back to PERIOD below 4.5 kHz */
#define FREQ_CROSSOVER_HIGH_HZ          (5500.0f)   /**< Switch from PERIOD to GATE above 5.5 kHz */
#define FREQ_SESSION_AVG_WINDOW         (16U)       /**< PSRAM session moving-average window length */
#define FREQ_RMT_BOUNCE_SYMBOLS         (64U)       /**< Internal SRAM bounce buffer symbol count for RMT RX ISR */

#if CONFIG_SPIRAM
#define FREQ_PSRAM_SYMBOL_BUF_CAP       (1024U)     /**< PSRAM staging capacity for captured RMT symbols */
#else
#define FREQ_PSRAM_SYMBOL_BUF_CAP       (128U)      /**< Compact staging capacity for PSRAM-less QEMU */
#endif
/** @} */

/**
 * @brief User-selectable Frequency Meter operating mode.
 */
typedef enum {
    FREQ_MODE_AUTO = 0,       /**< Automatic hysteresis crossover between Period (<5kHz) and Gate (>=5kHz) */
    FREQ_MODE_PERIOD_RMT,     /**< Forced low-frequency period measurement (f = 1/T via RMT RX) */
    FREQ_MODE_GATE_PCNT,      /**< Forced high-frequency gate-time pulse counting (f = N/t_gate via PCNT) */
    FREQ_MODE_COUNT
} freq_mode_t;

/**
 * @brief Active underlying frequency measurement method used for a reading.
 */
typedef enum {
    FREQ_METHOD_NONE = 0,     /**< No edges detected within the measurement/timeout window */
    FREQ_METHOD_PERIOD,       /**< Measured via edge-to-edge period (f = 1/T) */
    FREQ_METHOD_GATE,         /**< Measured via gate-time pulse counting (f = N / t_gate) */
} freq_method_t;

/**
 * @brief Selectable gate-time measurement windows.
 */
typedef enum {
    FREQ_GATE_20MS = 0,       /**< Fast 20 ms gate window (50 Hz update rate) */
    FREQ_GATE_100MS,          /**< Standard 100 ms gate window (10 Hz update rate, 10 Hz resolution) */
    FREQ_GATE_1000MS,         /**< High-resolution 1000 ms gate window (1 Hz resolution) */
    FREQ_GATE_COUNT
} freq_gate_window_t;

/**
 * @brief Edge counting mode for PCNT gate-time measurement.
 */
typedef enum {
    FREQ_PCNT_EDGES_RISING_ONLY = 0, /**< 1 count per full signal period (rising edges only) */
    FREQ_PCNT_EDGES_BOTH,            /**< 2 counts per full signal period (rising + falling edges) */
} freq_pcnt_edge_mode_t;

/**
 * @brief Result of RMT RX period, pulse-width, and duty-cycle analysis (`f = 1/T`).
 */
typedef struct {
    bool valid;               /**< True if at least one complete high+low period was measured */
    float freq_hz;            /**< Frequency in Hz (1e6 / period_us) */
    float period_us;          /**< Average period (T_high + T_low) in microseconds */
    float pulse_width_us;     /**< Average high pulse width (T_high) in microseconds */
    float low_width_us;       /**< Average low pulse width (T_low) in microseconds */
    float duty_pct;           /**< Positive duty cycle in percent (0.0..100.0) */
    float rpm;                /**< Revolutions per minute (60.0 * freq_hz) */
    uint32_t cycles_measured; /**< Number of complete cycles averaged */
} freq_period_result_t;

/**
 * @brief Result of PCNT + GPTimer gate-time pulse counting (`f = N / t_gate`).
 */
typedef struct {
    bool valid;               /**< True if gate window is valid and pulse_count > 0 */
    float freq_hz;            /**< Frequency in Hz */
    float period_us;          /**< Derived period in microseconds (1e6 / freq_hz) */
    float rpm;                /**< Revolutions per minute (60.0 * freq_hz) */
    uint64_t pulse_count;     /**< Total edges counted during gate_duration_us */
    uint64_t gate_duration_us;/**< Actual elapsed gate time in microseconds */
} freq_gate_result_t;

/**
 * @brief Combined frequency meter reading and session statistics.
 */
typedef struct {
    bool signal_detected;     /**< True if a periodic signal was detected in the current window */
    freq_method_t active_method; /**< Active measurement method (`PERIOD`, `GATE`, or `NONE`) */
    float freq_hz;            /**< Primary measured frequency in Hz (0.0f on timeout) */
    float period_us;          /**< Measured signal period in microseconds (0.0f on timeout) */
    float pulse_width_us;     /**< High pulse width in microseconds (from RMT RX or duty estimate) */
    float low_width_us;       /**< Low pulse width in microseconds */
    float duty_pct;           /**< Positive duty cycle in percent (0.0f..100.0f) */
    float rpm;                /**< Revolutions per minute (60.0f * freq_hz) */
    uint64_t edge_count;      /**< Pulse/edge count observed in the active window */
    uint64_t gate_duration_us;/**< Elapsed measurement gate duration in microseconds */
    float fmin_hz;            /**< Session minimum non-zero frequency in Hz */
    float fmax_hz;            /**< Session maximum frequency in Hz */
    float favg_hz;            /**< PSRAM moving-average frequency in Hz */
    uint32_t sample_Windows;  /**< Number of valid measurement windows accumulated since reset */
} freq_reading_t;

/**
 * @brief Complete frame transferred from Core 1 to Core 0 via `psram_double_buf_t`.
 */
typedef struct {
    freq_reading_t reading;
    gpio_num_t pin;
    freq_mode_t mode;
    freq_gate_window_t gate_window;
    bool running;
} freq_frame_t;

/**
 * @brief PSRAM-backed session moving-average and min/max frequency tracker.
 */
typedef struct {
    float *window_hz;         /**< Circular buffer allocated in PSRAM via `psram_buf_calloc()` */
    size_t capacity;          /**< Window capacity in samples */
    size_t count;             /**< Current valid entries in `window_hz` */
    size_t head;              /**< Next write index */
    double running_sum;       /**< Sum of entries in `window_hz` */
    float min_hz;             /**< Session minimum frequency in Hz */
    float max_hz;             /**< Session maximum frequency in Hz */
    uint32_t total_windows;   /**< Total valid windows accumulated */
} freq_session_tracker_t;

/* --------------------------------------------------------------------------
 * Pure Frequency Calculation & Crossover API (freq_calc.c)
 * -------------------------------------------------------------------------- */

/**
 * @brief Return the nominal duration in microseconds for a `freq_gate_window_t` setting.
 *
 * @param[in] gate Gate window enum value.
 * @return Gate duration in microseconds (`20000`, `100000`, or `1000000`), or `0` if invalid.
 */
uint32_t freq_calc_gate_window_us(freq_gate_window_t gate);

/**
 * @brief Return a compact display string for `gate` (`"20ms"`, `"100ms"`, `"1000ms"`).
 *
 * @param[in] gate Gate window enum value.
 * @return Static null-terminated string.
 */
const char *freq_calc_gate_window_to_str(freq_gate_window_t gate);

/**
 * @brief Return a compact display string for `mode` (`"AUTO"`, `"1/T"`, `"GATE"`).
 *
 * @param[in] mode Operating mode enum value.
 * @return Static null-terminated string.
 */
const char *freq_calc_mode_to_str(freq_mode_t mode);

/**
 * @brief Compute period, frequency (`f = 1/T`), high/low pulse widths, duty cycle, and RPM
 *        from an array of ESP32 RMT RX symbols.
 *
 * Each `rmt_symbol_word_t` contains two level/duration pairs (`duration0`/`level0` and
 * `duration1`/`level1`). Symbols with duration `0` mark the RMT end-of-frame idle timeout,
 * and pulses narrower than `min_glitch_ns` are rejected. Complete high+low cycles are
 * averaged for sub-microsecond accuracy.
 *
 * @param[in]  symbols        Input array of `rmt_symbol_word_t` captured by RMT RX.
 * @param[in]  num_symbols    Number of symbols in `symbols`.
 * @param[in]  resolution_hz  RMT channel tick resolution in Hz (e.g. `1000000` for 1 us ticks).
 * @param[in]  min_glitch_ns  Minimum valid pulse duration in nanoseconds (e.g. `1000` ns; `0` = no filter).
 * @param[out] out_result     Destination period measurement result.
 * @return `ESP_OK` on success (check `out_result->valid` for edge presence), `ESP_ERR_INVALID_ARG` on NULL/0 resolution.
 */
esp_err_t freq_calc_from_rmt_symbols(const rmt_symbol_word_t *symbols,
                                     size_t num_symbols,
                                     uint32_t resolution_hz,
                                     uint32_t min_glitch_ns,
                                     freq_period_result_t *out_result);

/**
 * @brief Compute frequency (`f = N / t_gate`), period, and RPM from PCNT pulse count and GPTimer gate duration.
 *
 * When `edge_mode == FREQ_PCNT_EDGES_BOTH` (rising + falling edges counted), divides `pulse_count`
 * by `2.0` to obtain full signal cycles. When `pulse_count == 0`, sets `valid = false` and zeroes
 * all output metrics cleanly without divide-by-zero.
 *
 * @param[in]  pulse_count      Total edges counted by PCNT during the gate window.
 * @param[in]  gate_duration_us Actual elapsed gate duration in microseconds (`> 0`).
 * @param[in]  edge_mode        Whether PCNT counted rising-only or both edges per period.
 * @param[out] out_result       Destination gate measurement result.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `out_result == NULL` or `gate_duration_us == 0`.
 */
esp_err_t freq_calc_from_gate_counts(uint64_t pulse_count,
                                     uint64_t gate_duration_us,
                                     freq_pcnt_edge_mode_t edge_mode,
                                     freq_gate_result_t *out_result);

/**
 * @brief Select the active measurement method (`PERIOD`, `GATE`, or `NONE`) using `5 kHz` hysteresis.
 *
 * In `FREQ_MODE_AUTO`:
 * - If previously in `FREQ_METHOD_GATE`, remains in `GATE` unless the frequency drops below
 *   `FREQ_CROSSOVER_LOW_HZ` (`4500 Hz`) and `period_res->valid` is true.
 * - Otherwise, uses `PERIOD` up to `FREQ_CROSSOVER_HIGH_HZ` (`5500 Hz`), switching to `GATE`
 *   when frequency exceeds `5500 Hz` (or when only `gate_res->valid` is available).
 * - If neither `period_res->valid` nor `gate_res->valid` is true (zero-edge timeout), returns `FREQ_METHOD_NONE`.
 *
 * @param[in] mode        Configured `freq_mode_t` (`AUTO`, `PERIOD_RMT`, `GATE_PCNT`).
 * @param[in] prev_method Previously active `freq_method_t` for hysteresis state.
 * @param[in] period_res  Optional pointer to current RMT RX period measurement.
 * @param[in] gate_res    Optional pointer to current PCNT gate measurement.
 * @return Selected `freq_method_t`.
 */
freq_method_t freq_calc_select_method(freq_mode_t mode,
                                      freq_method_t prev_method,
                                      const freq_period_result_t *period_res,
                                      const freq_gate_result_t *gate_res);

/**
 * @brief Allocate and initialize a PSRAM-backed session frequency tracker.
 *
 * @param[out] tracker    Pointer to tracker structure.
 * @param[in]  window_len Moving-average window length (`1..64`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` or `ESP_ERR_NO_MEM` on failure.
 */
esp_err_t freq_calc_tracker_init(freq_session_tracker_t *tracker, size_t window_len);

/**
 * @brief Free the PSRAM buffer owned by a `freq_session_tracker_t`.
 *
 * @param[in,out] tracker Pointer to tracker structure (safe to pass `NULL`).
 */
void freq_calc_tracker_free(freq_session_tracker_t *tracker);

/**
 * @brief Reset the moving-average window and min/max counters of `tracker` without freeing memory.
 *
 * @param[in,out] tracker Pointer to initialized tracker structure.
 */
void freq_calc_tracker_reset(freq_session_tracker_t *tracker);

/**
 * @brief Combine RMT RX period and PCNT gate results into a unified `freq_reading_t` and update `tracker`.
 *
 * On zero-edge timeout (`selected_method == FREQ_METHOD_NONE`), zeroes instantaneous `freq_hz`,
 * `period_us`, `pulse_width_us`, `duty_pct`, and `rpm` while preserving session `fmin_hz`,
 * `fmax_hz`, and `favg_hz` in `tracker`. When `selected_method == FREQ_METHOD_GATE` and
 * `period_res->valid` is also true, retains the measured duty cycle from `period_res` and derives
 * `pulse_width_us` from the high-accuracy gate period.
 *
 * @param[in]     mode        Configured `freq_mode_t`.
 * @param[in]     prev_method Previous active `freq_method_t` for hysteresis.
 * @param[in]     period_res  Optional pointer to RMT period result.
 * @param[in]     gate_res    Optional pointer to PCNT gate result.
 * @param[in,out] tracker     Optional pointer to initialized session tracker.
 * @param[out]    out_reading Destination unified frequency reading structure.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `out_reading` is `NULL`.
 */
esp_err_t freq_calc_combine_reading(freq_mode_t mode,
                                    freq_method_t prev_method,
                                    const freq_period_result_t *period_res,
                                    const freq_gate_result_t *gate_res,
                                    freq_session_tracker_t *tracker,
                                    freq_reading_t *out_reading);

/**
 * @brief Format a frequency value in Hz into an auto-ranging human-readable string (`Hz`, `kHz`, `MHz`).
 *
 * Examples: `"0.00 Hz"`, `"440.00 Hz"`, `"12.345 kHz"`, `"4.000 MHz"`.
 *
 * @param[in]  freq_hz Frequency in Hz.
 * @param[out] out_buf Destination character buffer.
 * @param[in]  buf_len Size of `out_buf` in bytes.
 */
void freq_calc_format_freq(float freq_hz, char *out_buf, size_t buf_len);

/* --------------------------------------------------------------------------
 * Frequency Meter Runtime Configuration & Control API (tool_freq.c)
 * -------------------------------------------------------------------------- */

/**
 * @brief Check whether `pin` is a valid safe external GPIO for the Frequency Meter.
 *
 * Supported pins: `GPIO36`, `GPIO35`, `GPIO26`, `GPIO13`, `GPIO14`, `GPIO32`, `GPIO33`, `GPIO27`, `GPIO19`.
 *
 * @param[in] pin GPIO number to inspect.
 * @return True if supported, false otherwise.
 */
bool tool_freq_is_pin_supported(gpio_num_t pin);

/**
 * @brief Select the active digital input pin for the Frequency Meter.
 *
 * Atomically claims the new pin via `bsp_pin_claim()` and releases the old pin when the tool
 * is active, rolling back on conflict.
 *
 * @param[in] pin Supported GPIO number.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` or `ESP_ERR_INVALID_STATE` on failure.
 */
esp_err_t tool_freq_set_pin(gpio_num_t pin);

/**
 * @brief Get the currently configured input pin.
 *
 * @return Active `gpio_num_t`.
 */
gpio_num_t tool_freq_get_pin(void);

/**
 * @brief Set the Frequency Meter operating mode (`AUTO`, `PERIOD_RMT`, `GATE_PCNT`).
 *
 * @param[in] mode Target `freq_mode_t`.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if out of range.
 */
esp_err_t tool_freq_set_mode(freq_mode_t mode);

/**
 * @brief Get the active Frequency Meter operating mode.
 *
 * @return Current `freq_mode_t`.
 */
freq_mode_t tool_freq_get_mode(void);

/**
 * @brief Set the gate measurement window (`20ms`, `100ms`, `1000ms`).
 *
 * @param[in] gate Target `freq_gate_window_t`.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if out of range.
 */
esp_err_t tool_freq_set_gate_window(freq_gate_window_t gate);

/**
 * @brief Get the active gate measurement window.
 *
 * @return Current `freq_gate_window_t`.
 */
freq_gate_window_t tool_freq_get_gate_window(void);

/**
 * @brief Pause or resume frequency measurement updates (`HOLD` / `RUN`).
 *
 * @param[in] running True for live measurements, false to hold display.
 * @return `ESP_OK` on success.
 */
esp_err_t tool_freq_set_running(bool running);

/**
 * @brief Return true if frequency measurement is running (`RUN`), false if held (`HOLD`).
 *
 * @return True if running, false if held.
 */
bool tool_freq_is_running(void);

/**
 * @brief Reset the session minimum, maximum, moving-average frequency, and window count.
 *
 * @return `ESP_OK` on success.
 */
esp_err_t tool_freq_reset_stats(void);

/**
 * @brief Copy out the latest computed `freq_reading_t` snapshot under spinlock.
 *
 * @param[out] out_reading Destination structure.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `out_reading` is `NULL`.
 */
esp_err_t tool_freq_get_latest_reading(freq_reading_t *out_reading);

/**
 * @brief Return the static `analyser_tool_t` descriptor for the Frequency Meter instrument.
 *
 * @return Pointer to `analyser_tool_t` descriptor.
 */
const analyser_tool_t *tool_freq_get_descriptor(void);

/**
 * @brief Register the Frequency Meter tool (`id = "freq"`) and its self-test with `tool_core`.
 *
 * @return `ESP_OK` on success, or error code from `tool_registry_register()`.
 */
esp_err_t tool_freq_register(void);

#ifdef __cplusplus
}
#endif
