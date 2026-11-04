/**
 * @file tool_wavegen.h
 * @brief Digital Wave Generator instrument (`tool_wavegen`) for M5CoreAnalyser.
 *
 * Generates digital waveforms across selectable external output GPIOs (`GPIO14`, `GPIO13`,
 * `GPIO26`, `GPIO32`, `GPIO33`, `GPIO27`, `GPIO19`, `GPIO25`) using ESP32 hardware peripherals:
 * - **LEDC (`WAVEGEN_MODE_SQUARE`, `WAVEGEN_MODE_PWM_LEDC`)**: `1 Hz` to `1 MHz` square/clock
 *   and variable-duty PWM with automatic bit-depth resolution solver (`1..20` bits).
 * - **MCPWM (`WAVEGEN_MODE_PWM_MCPWM`)**: `10 Hz` to `250 kHz` high-precision variable-duty PWM
 *   using dedicated hardware timer, operator, comparator, and generator units.
 * - **RMT TX (`WAVEGEN_MODE_SERVO`, `WAVEGEN_MODE_RMT_BURST`)**: `50 Hz` (`20 ms`) RC servo
 *   pulses (`500..2500 us`, `0..180 deg`) and programmable N-pulse bursts (`1..64` pulses).
 *
 * Supports background execution (`supports_background = true`) coordinated with `bsp_pin_claim_ex`
 * so the user can keep digital stimuli running while opening foreground measurement tools.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp_m5core2.h"
#include "driver/gpio.h"
#include "driver/rmt_types.h"
#include "esp_err.h"
#include "tool_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name Digital Wave Generator Constants & Limits
 *  @{
 */
#define WAVEGEN_TOOL_ID                "wavegen"
#define WAVEGEN_DEFAULT_PIN            (BSP_PIN_PORT_C_TXD2) /**< Default output pin: GPIO14 (Port C TXD2) */
#define WAVEGEN_APB_CLK_HZ             (80000000U)           /**< ESP32 80 MHz APB reference clock */
#define WAVEGEN_LEDC_MIN_FREQ_HZ       (1U)                  /**< Minimum LEDC output frequency (1 Hz) */
#define WAVEGEN_LEDC_MAX_FREQ_HZ       (1000000U)            /**< Maximum LEDC output frequency (1 MHz) */
#define WAVEGEN_MCPWM_MIN_FREQ_HZ      (10U)                 /**< Minimum MCPWM output frequency (10 Hz) */
#define WAVEGEN_MCPWM_MAX_FREQ_HZ      (250000U)             /**< Maximum MCPWM output frequency (250 kHz) */
#define WAVEGEN_DEFAULT_FREQ_HZ        (1000U)               /**< Default output frequency (1 kHz) */
#define WAVEGEN_DEFAULT_DUTY_PCT       (50.0f)               /**< Default PWM duty cycle (50%) */

#define WAVEGEN_SERVO_FREQ_HZ          (50U)                 /**< Standard 50 Hz RC servo frame rate */
#define WAVEGEN_SERVO_PERIOD_US        (20000U)              /**< 20,000 us (20 ms) servo period */
#define WAVEGEN_SERVO_MIN_US           (500U)                /**< Minimum servo pulse width (0 deg) */
#define WAVEGEN_SERVO_DEFAULT_US       (1500U)               /**< Neutral servo pulse width (90 deg) */
#define WAVEGEN_SERVO_MAX_US           (2500U)               /**< Maximum servo pulse width (180 deg) */

#define WAVEGEN_RMT_RESOLUTION_HZ      (1000000U)            /**< 1 MHz (1 us / tick) RMT TX resolution */
#define WAVEGEN_RMT_MAX_DURATION_US    (32767U)              /**< 15-bit max duration per RMT half-symbol */
#define WAVEGEN_MAX_BURST_PULSES       (64U)                 /**< Maximum pulses per RMT burst transaction */
#define WAVEGEN_DEFAULT_BURST_HIGH_US  (100U)                /**< Default burst high pulse width (100 us) */
#define WAVEGEN_DEFAULT_BURST_LOW_US   (100U)                /**< Default burst low pulse width (100 us) */
#define WAVEGEN_DEFAULT_BURST_PULSES   (8U)                  /**< Default pulses per burst */
#define WAVEGEN_DEFAULT_BURST_REP_MS   (50U)                 /**< Default burst repetition period (50 ms) */

#define WAVEGEN_CHART_POINTS           (120U)                /**< Waveform preview chart point count */
/** @} */

/**
 * @brief Operating mode and peripheral selection for `tool_wavegen`.
 */
typedef enum {
    WAVEGEN_MODE_SQUARE = 0, /**< 50% duty square / clock wave via ESP32 LEDC (1 Hz .. 1 MHz) */
    WAVEGEN_MODE_PWM_LEDC,   /**< Variable-duty PWM via ESP32 LEDC (1 Hz .. 1 MHz) */
    WAVEGEN_MODE_PWM_MCPWM,  /**< High-resolution variable-duty PWM via ESP32 MCPWM (10 Hz .. 250 kHz) */
    WAVEGEN_MODE_SERVO,      /**< 50 Hz RC servo pulse (500 .. 2500 us) via RMT TX */
    WAVEGEN_MODE_RMT_BURST,  /**< Programmable N-pulse digital burst via RMT TX */
    WAVEGEN_MODE_COUNT
} wavegen_mode_t;

/**
 * @brief Solved ESP32 LEDC timer and duty configuration parameters.
 */
typedef struct {
    uint32_t target_freq_hz;       /**< Requested frequency in Hz */
    uint32_t actual_freq_hz;       /**< Closest achievable LEDC frequency in Hz */
    uint8_t duty_resolution_bits;  /**< Optimal LEDC timer resolution in bits (1..20) */
    uint32_t max_duty_ticks;       /**< Full-scale duty count (1U << duty_resolution_bits) */
    uint32_t duty_ticks;           /**< Quantized duty count [0..max_duty_ticks] */
    float target_duty_pct;         /**< Requested duty cycle percentage (0.0..100.0) */
    float actual_duty_pct;         /**< Quantized achieved duty cycle percentage (0.0..100.0) */
} wavegen_ledc_params_t;

/**
 * @brief Solved ESP32 MCPWM timer and comparator tick parameters.
 */
typedef struct {
    uint32_t resolution_hz;        /**< Selected MCPWM timer tick clock in Hz */
    uint32_t period_ticks;         /**< Timer period in ticks (2..65535) */
    uint32_t compare_ticks;        /**< Comparator threshold in ticks (0..period_ticks) */
    uint32_t target_freq_hz;       /**< Requested frequency in Hz */
    uint32_t actual_freq_hz;       /**< Achieved frequency in Hz */
    float target_duty_pct;         /**< Requested duty cycle percentage (0.0..100.0) */
    float actual_duty_pct;         /**< Achieved duty cycle percentage (0.0..100.0) */
    float pulse_width_us;          /**< High pulse width in microseconds */
} wavegen_mcpwm_params_t;

/**
 * @brief Complete configuration state for `tool_wavegen`.
 */
typedef struct {
    gpio_num_t pin;                /**< Claimed output GPIO pin */
    wavegen_mode_t mode;           /**< Active generator waveform/peripheral mode */
    uint32_t freq_hz;              /**< Target frequency in Hz (for SQUARE / PWM modes) */
    float duty_pct;                /**< Target duty cycle percentage (0.0..100.0) */
    uint16_t servo_pulse_us;       /**< Target servo high pulse width in us (500..2500) */
    uint16_t burst_high_us;        /**< RMT burst high pulse duration in us (1..32767) */
    uint16_t burst_low_us;         /**< RMT burst low pulse duration in us (1..32767) */
    uint16_t burst_pulse_count;    /**< Number of pulses per RMT burst (1..64) */
    uint16_t burst_period_ms;      /**< Repetition period between bursts in ms (5..1000) */
    bool output_enabled;           /**< True when signal output is enabled */
    bool keep_in_bg;               /**< True to retain generator in background on Home/tool switch */
} wavegen_config_t;

/**
 * @brief Runtime telemetry and hardware status snapshot for `tool_wavegen`.
 */
typedef struct {
    wavegen_config_t cfg;          /**< Active generator configuration */
    uint32_t actual_freq_hz;       /**< Achieved output frequency in Hz */
    float actual_duty_pct;         /**< Achieved duty cycle percentage */
    uint8_t resolution_bits;       /**< Active LEDC duty resolution in bits (or 0 in MCPWM/RMT modes) */
    uint32_t period_ticks;         /**< Active MCPWM period ticks (or 0 in LEDC/RMT modes) */
    uint32_t compare_ticks;        /**< Active MCPWM compare ticks (or 0 in LEDC/RMT modes) */
    size_t rmt_symbol_count;       /**< Number of RMT symbols per burst/servo frame */
    uint32_t burst_frames_sent;    /**< Cumulative RMT burst/servo frames transmitted */
    bool running_fg;               /**< True if `tool_wavegen` is open in the foreground */
    bool running_bg;               /**< True if `tool_wavegen` is running in the background */
    bool hw_active;                /**< True if output peripheral is actively driving `cfg.pin` */
    esp_err_t last_error;          /**< Most recent peripheral error code (`ESP_OK` if nominal) */
} wavegen_status_t;

/* --------------------------------------------------------------------------
 * Pure Calculation & Synthesis Seams (wavegen_calc.c)
 * -------------------------------------------------------------------------- */

/**
 * @brief Solve for optimal ESP32 LEDC timer bit resolution, divider, and duty ticks.
 *
 * Selects the highest bit resolution `bits` in `[1, 20]` such that `apb_clk_hz / (target_freq_hz * 2^bits) >= 1`
 * and the 10-bit integer divider part does not overflow (`<= 1023`).
 *
 * @param[in]  apb_clk_hz     Source APB clock in Hz (typically `80000000`).
 * @param[in]  target_freq_hz Target PWM/square frequency in Hz (`1..1000000`).
 * @param[in]  duty_pct       Requested duty cycle percentage (`0.0..100.0`, clamped).
 * @param[out] out_params     Output structure receiving solved LEDC parameters.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if frequency is out of range or `out_params` is `NULL`.
 */
esp_err_t wavegen_calc_ledc_params(uint32_t apb_clk_hz,
                                   uint32_t target_freq_hz,
                                   float duty_pct,
                                   wavegen_ledc_params_t *out_params);

/**
 * @brief Solve for optimal ESP32 MCPWM timer resolution, period ticks, and comparator ticks.
 *
 * Selects a prescaled timer resolution (`10 MHz`, `1 MHz`, or `100 kHz`) so that `period_ticks`
 * fits within the 16-bit MCPWM counter range (`2..65535`) with maximum duty precision.
 *
 * @param[in]  target_freq_hz Target PWM frequency in Hz (`10..250000`).
 * @param[in]  duty_pct       Requested duty cycle percentage (`0.0..100.0`, clamped).
 * @param[out] out_params     Output structure receiving solved MCPWM parameters.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if frequency is out of range or `out_params` is `NULL`.
 */
esp_err_t wavegen_calc_mcpwm_params(uint32_t target_freq_hz,
                                    float duty_pct,
                                    wavegen_mcpwm_params_t *out_params);

/**
 * @brief Convert a servo angle in degrees (`0.0..180.0`) to a pulse width in microseconds (`500..2500`).
 *
 * @param[in] angle_deg Servo angle in degrees (clamped to `[0.0, 180.0]`).
 * @return Pulse width in microseconds (`500..2500`).
 */
uint16_t wavegen_calc_servo_angle_to_us(float angle_deg);

/**
 * @brief Convert a servo pulse width in microseconds (`500..2500`) to degrees (`0.0..180.0`).
 *
 * @param[in] pulse_us Servo pulse width in microseconds (clamped to `[500, 2500]`).
 * @return Servo angle in degrees (`0.0..180.0`).
 */
float wavegen_calc_servo_us_to_angle(uint16_t pulse_us);

/**
 * @brief Build RMT TX symbol(s) for a 50 Hz RC servo frame at 1 MHz (1 us) RMT resolution.
 *
 * Clamps `pulse_us` to `[500, 2500]`. If `period_us` is `0`, defaults to `20000 us` (`50 Hz`).
 * If the low duration `period_us - pulse_us` exceeds `32767 us`, splits the low interval across
 * two RMT symbols.
 *
 * @param[in]  pulse_us         High pulse width in microseconds (`500..2500`).
 * @param[in]  period_us        Total frame period in microseconds (`>= 3000`, `0` -> `20000`).
 * @param[out] out_symbols      Destination RMT symbol array.
 * @param[in]  max_symbols      Capacity of `out_symbols` (must be >= 1, or >= 2 if `period_us > 33267`).
 * @param[out] out_symbol_count Number of RMT symbols written.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t wavegen_calc_servo_rmt_symbol(uint16_t pulse_us,
                                        uint32_t period_us,
                                        rmt_symbol_word_t *out_symbols,
                                        size_t max_symbols,
                                        size_t *out_symbol_count);

/**
 * @brief Build an array of RMT TX symbols for an N-pulse digital burst at 1 MHz (1 us) resolution.
 *
 * @param[in]  high_us          High duration of each pulse in microseconds (`1..32767`).
 * @param[in]  low_us           Low duration of each pulse in microseconds (`1..32767`).
 * @param[in]  pulse_count      Number of pulses in the burst (`1..WAVEGEN_MAX_BURST_PULSES`).
 * @param[out] out_symbols      Destination RMT symbol array.
 * @param[in]  max_symbols      Capacity of `out_symbols` (must be `>= pulse_count`).
 * @param[out] out_symbol_count Number of RMT symbols written (`== pulse_count`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if any parameter is out of range or `NULL`.
 */
esp_err_t wavegen_calc_burst_rmt_symbols(uint16_t high_us,
                                         uint16_t low_us,
                                         uint16_t pulse_count,
                                         rmt_symbol_word_t *out_symbols,
                                         size_t max_symbols,
                                         size_t *out_symbol_count);

/**
 * @brief Synthesize a digital waveform preview array (`0` or `3300` mV) for LVGL chart rendering.
 *
 * @param[in]  cfg         Generator configuration to visualize.
 * @param[out] out_y       Destination array of `point_count` vertical chart points (`0..3300` mV).
 * @param[in]  point_count Number of points to generate (`>= 8`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t wavegen_build_preview_points(const wavegen_config_t *cfg,
                                       int32_t *out_y,
                                       size_t point_count);

/**
 * @brief Return a compact human-readable label for a `wavegen_mode_t`.
 *
 * @param[in] mode Wave generator mode.
 * @return Static null-terminated string (`"SQUARE"`, `"LEDC PWM"`, `"MCPWM"`, `"SERVO"`, `"BURST"`).
 */
const char *wavegen_mode_to_str(wavegen_mode_t mode);

/**
 * @brief Return the ESP32 hardware peripheral name associated with `mode`.
 *
 * @param[in] mode Wave generator mode.
 * @return Static null-terminated string (`"LEDC"`, `"MCPWM"`, or `"RMT TX"`).
 */
const char *wavegen_periph_to_str(wavegen_mode_t mode);

/* --------------------------------------------------------------------------
 * Runtime Instrument Configuration & Control API (tool_wavegen.c)
 * -------------------------------------------------------------------------- */

/**
 * @brief Check whether `pin` is a supported external output GPIO for `tool_wavegen`.
 *
 * Supported pins: `GPIO14`, `GPIO13`, `GPIO26`, `GPIO32`, `GPIO33`, `GPIO27`, `GPIO19`, `GPIO25`.
 * Input-only pins (`GPIO36`, `GPIO35`) and protected system pins return `false`.
 *
 * @param[in] pin GPIO number to inspect.
 * @return True if supported as a wave generator output pin, false otherwise.
 */
bool tool_wavegen_is_pin_supported(gpio_num_t pin);

/**
 * @brief Select the output GPIO pin for `tool_wavegen`.
 *
 * When `tool_wavegen` is active (in foreground or background), claims `pin` via
 * `bsp_pin_claim_ex(pin, WAVEGEN_TOOL_ID, true)` before releasing the previous pin and
 * rebinding the active hardware peripheral. If `pin` is already claimed by another tool,
 * returns `ESP_ERR_INVALID_STATE` and leaves the existing pin and output untouched.
 *
 * @param[in] pin Target output GPIO number.
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_ERR_NOT_SUPPORTED` if `pin` is input-only (`GPIO34..39`)
 *  - `ESP_ERR_NOT_ALLOWED` if `pin` is a protected internal M5Core2 pin
 *  - `ESP_ERR_INVALID_ARG` if `pin` is invalid
 *  - `ESP_ERR_INVALID_STATE` if `pin` is currently owned by another tool
 */
esp_err_t tool_wavegen_set_pin(gpio_num_t pin);

/**
 * @brief Get the currently configured output GPIO pin for `tool_wavegen`.
 *
 * @return Output `gpio_num_t`.
 */
gpio_num_t tool_wavegen_get_pin(void);

/**
 * @brief Set the active waveform/peripheral mode for `tool_wavegen`.
 *
 * @param[in] mode Target `wavegen_mode_t`.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `mode` is invalid.
 */
esp_err_t tool_wavegen_set_mode(wavegen_mode_t mode);

/**
 * @brief Get the currently configured waveform/peripheral mode.
 *
 * @return Current `wavegen_mode_t`.
 */
wavegen_mode_t tool_wavegen_get_mode(void);

/**
 * @brief Set the target output frequency in Hz (used by `SQUARE`, `PWM_LEDC`, and `PWM_MCPWM`).
 *
 * @param[in] freq_hz Target frequency in Hz (`1..1000000`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `freq_hz` is out of range for the active mode.
 */
esp_err_t tool_wavegen_set_freq_hz(uint32_t freq_hz);

/**
 * @brief Get the currently configured target frequency in Hz.
 *
 * @return Target frequency in Hz.
 */
uint32_t tool_wavegen_get_freq_hz(void);

/**
 * @brief Set the target PWM duty cycle percentage (`0.0..100.0`).
 *
 * @param[in] duty_pct Duty cycle percentage (`0.0..100.0`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `duty_pct` is outside `[0.0, 100.0]`.
 */
esp_err_t tool_wavegen_set_duty_pct(float duty_pct);

/**
 * @brief Get the currently configured target PWM duty cycle percentage.
 *
 * @return Target duty cycle percentage (`0.0..100.0`).
 */
float tool_wavegen_get_duty_pct(void);

/**
 * @brief Set the RC servo high pulse width in microseconds (`500..2500`).
 *
 * @param[in] pulse_us High pulse width in microseconds (`500..2500`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if outside `[500, 2500]`.
 */
esp_err_t tool_wavegen_set_servo_pulse_us(uint16_t pulse_us);

/**
 * @brief Get the currently configured RC servo high pulse width in microseconds.
 *
 * @return Servo pulse width in microseconds (`500..2500`).
 */
uint16_t tool_wavegen_get_servo_pulse_us(void);

/**
 * @brief Configure the RMT TX digital pulse burst parameters.
 *
 * @param[in] high_us     High pulse width in microseconds (`1..32767`).
 * @param[in] low_us      Low pulse width in microseconds (`1..32767`).
 * @param[in] pulse_count Number of pulses per burst (`1..WAVEGEN_MAX_BURST_PULSES`).
 * @param[in] period_ms   Repetition interval in milliseconds (`5..1000`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if any parameter is out of range.
 */
esp_err_t tool_wavegen_set_burst_config(uint16_t high_us,
                                        uint16_t low_us,
                                        uint16_t pulse_count,
                                        uint16_t period_ms);

/**
 * @brief Immediately transmit a single RMT pulse burst (or servo frame) when in RMT mode.
 *
 * @return `ESP_OK` on success, or error code if not active or not in an RMT mode.
 */
esp_err_t tool_wavegen_trigger_burst(void);

/**
 * @brief Enable or pause signal generation on the claimed output pin.
 *
 * @param[in] enabled True to drive the output signal, false to idle low.
 * @return `ESP_OK` on success.
 */
esp_err_t tool_wavegen_set_output_enabled(bool enabled);

/**
 * @brief Query whether signal output is enabled.
 *
 * @return True if output is enabled, false if paused.
 */
bool tool_wavegen_is_output_enabled(void);

/**
 * @brief Set the "Keep Running in Background" preference for `tool_wavegen`.
 *
 * When enabled (default `true`), returning Home via `ui_shell_go_home(true)` or
 * `tool_core_stop_active_tool(true)` retains the active hardware generator and GPIO ownership
 * in the background. When disabled (`false`), leaving the tool stops hardware output and
 * releases the GPIO pin.
 *
 * @param[in] keep_in_bg True to keep running in background when exiting with `keep_background=true`.
 * @return `ESP_OK` on success.
 */
esp_err_t tool_wavegen_set_keep_in_bg(bool keep_in_bg);

/**
 * @brief Query the "Keep Running in Background" preference for `tool_wavegen`.
 *
 * @return True if background retention is enabled, false otherwise.
 */
bool tool_wavegen_get_keep_in_bg(void);

/**
 * @brief Copy out a thread-safe snapshot of the current `wavegen_status_t`.
 *
 * @param[out] out_status Destination status structure.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `out_status` is `NULL`.
 */
esp_err_t tool_wavegen_get_status(wavegen_status_t *out_status);

/**
 * @brief Execute a single deterministic worker/refresh step for `tool_wavegen` (used in tests).
 *
 * @return `ESP_OK` on success, or error code.
 */
esp_err_t tool_wavegen_run_single_step(void);

/**
 * @brief Get the static `analyser_tool_t` descriptor for `tool_wavegen`.
 *
 * @return Pointer to static `analyser_tool_t`.
 */
const analyser_tool_t *tool_wavegen_get_descriptor(void);

/**
 * @brief Register the Digital Wave Generator tool (`id = "wavegen"`) and its self-test with `tool_core`.
 *
 * @return `ESP_OK` on success, or error code from `tool_registry_register()`.
 */
esp_err_t tool_wavegen_register(void);

#ifdef __cplusplus
}
#endif
