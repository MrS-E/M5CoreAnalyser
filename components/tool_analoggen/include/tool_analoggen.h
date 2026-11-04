/**
 * @file tool_analoggen.h
 * @brief 8-Bit DAC Analog Waveform Generator (`tool_analoggen`) for M5CoreAnalyser.
 *
 * Synthesizes 256-point lookup tables (Sine, Triangle, Sawtooth, Staircase, DC) in PSRAM/SRAM
 * and drives the ESP32 8-bit DAC on `GPIO25` (`DAC_CHAN_0`) or `GPIO26` (`DAC_CHAN_1`) using
 * task-driven `dac_oneshot_output_voltage()` updates on Core 1.
 *
 * Using task-driven DAC oneshot updates (rather than `dac_continuous` I2S0 DMA or high-rate
 * timer ISRs) avoids ESP32 `I2S0` / `ADC1` peripheral contention when running in the background
 * alongside `tool_scope` or `tool_voltmeter`, while maintaining cache/ISR safety and bounded
 * output frequencies (`1..1000 Hz`).
 *
 * Also supports software speaker audio monitoring via `bsp_spk_passthrough_push_u8()`,
 * automatically yielding speaker ownership whenever a speaker-capable foreground input tool
 * (`tool_scope` or `tool_voltmeter`) is active.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp_m5core2.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "tool_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name Analog DAC Generator Constants & Limits
 *  @{
 */
#define ANALOGGEN_TOOL_ID              "analoggen"
#define ANALOGGEN_DEFAULT_PIN          (BSP_PIN_DAC_OUT_CH1) /**< Default output pin: GPIO25 (DAC1) */
#define ANALOGGEN_ALT_PIN              (BSP_PIN_DAC_OUT_CH2) /**< Alternate output pin: GPIO26 (DAC2) */
#define ANALOGGEN_LUT_SIZE             (256U)                /**< 256-entry 8-bit waveform lookup table */
#define ANALOGGEN_MIN_FREQ_HZ          (1U)                  /**< Minimum waveform frequency (1 Hz) */
#define ANALOGGEN_MAX_FREQ_HZ          (1000U)               /**< Safe bounded max frequency for task-driven DAC (1 kHz) */
#define ANALOGGEN_DEFAULT_FREQ_HZ      (200U)                /**< Default waveform frequency (200 Hz) */
#define ANALOGGEN_MAX_MV               (3300U)               /**< Full-scale DAC reference voltage (3300 mV) */
#define ANALOGGEN_DEFAULT_AMP_MV       (2000U)               /**< Default peak-to-peak amplitude (2000 mV) */
#define ANALOGGEN_DEFAULT_OFFSET_MV    (1650U)               /**< Default DC center offset (1650 mV) */
#define ANALOGGEN_MIN_STAIR_STEPS      (2U)                  /**< Minimum staircase quantization steps */
#define ANALOGGEN_MAX_STAIR_STEPS      (16U)                 /**< Maximum staircase quantization steps */
#define ANALOGGEN_DEFAULT_STAIR_STEPS  (8U)                  /**< Default staircase steps (8) */
#define ANALOGGEN_FG_UPDATE_RATE_HZ    (8000U)               /**< Nominal task-driven DAC update rate in foreground */
#define ANALOGGEN_BG_UPDATE_RATE_HZ    (4000U)               /**< Nominal task-driven DAC update rate in background */
#define ANALOGGEN_SPK_CHUNK_SAMPLES    (256U)                /**< Audio samples pushed per speaker passthrough frame */
#define ANALOGGEN_CHART_POINTS         (128U)                /**< LVGL waveform preview chart points */
/** @} */

/**
 * @brief Supported analog waveform shapes for `tool_analoggen`.
 */
typedef enum {
    ANALOGGEN_WAVE_SINE = 0,  /**< Sinusoidal waveform */
    ANALOGGEN_WAVE_TRIANGLE,  /**< Symmetric linear triangle waveform */
    ANALOGGEN_WAVE_SAWTOOTH,  /**< Rising linear sawtooth ramp */
    ANALOGGEN_WAVE_STAIRCASE, /**< Quantized K-step staircase waveform */
    ANALOGGEN_WAVE_DC,        /**< Constant DC voltage level */
    ANALOGGEN_WAVE_COUNT
} analoggen_wave_t;

/**
 * @brief Complete configuration parameters for `tool_analoggen`.
 */
typedef struct {
    gpio_num_t pin;             /**< Selected DAC GPIO (`GPIO_NUM_25` or `GPIO_NUM_26`) */
    analoggen_wave_t waveform;  /**< Selected waveform shape */
    uint32_t freq_hz;           /**< Target waveform frequency in Hz (`1..1000`) */
    uint16_t amplitude_mv;      /**< Peak-to-peak amplitude in millivolts (`0..3300`) */
    uint16_t offset_mv;         /**< Center DC offset in millivolts (`0..3300`) */
    uint8_t stair_steps;        /**< Number of steps for `ANALOGGEN_WAVE_STAIRCASE` (`2..16`) */
    bool output_enabled;        /**< True when DAC output is active */
    bool keep_in_bg;            /**< True to keep running in background when returning Home */
} analoggen_config_t;

/**
 * @brief Computed statistics and rail-clamping telemetry for a synthesized 256-entry DAC LUT.
 */
typedef struct {
    uint8_t min_code;           /**< Minimum 8-bit DAC code in the LUT (`0..255`) */
    uint8_t max_code;           /**< Maximum 8-bit DAC code in the LUT (`0..255`) */
    uint16_t min_mv;            /**< Clamped minimum output voltage in mV (`0..3300`) */
    uint16_t max_mv;            /**< Clamped maximum output voltage in mV (`0..3300`) */
    uint16_t vpp_mv;            /**< Clamped peak-to-peak voltage swing in mV (`max_mv - min_mv`) */
    uint16_t mean_mv;           /**< Arithmetic mean voltage across the LUT in mV */
    bool clipped;               /**< True if `offset_mv +/- amplitude_mv/2` exceeded `[0, 3300]` mV */
} analoggen_lut_stats_t;

/**
 * @brief Runtime telemetry and hardware status snapshot for `tool_analoggen`.
 */
typedef struct {
    analoggen_config_t cfg;         /**< Active generator configuration */
    analoggen_lut_stats_t lut_stats;/**< Statistics of the active 256-entry LUT */
    uint32_t target_update_rate_hz; /**< Nominal task-driven DAC update rate in Hz */
    uint32_t dac_updates_total;     /**< Cumulative `dac_oneshot_output_voltage()` updates issued */
    uint8_t last_dac_code;          /**< Most recent 8-bit code written to the DAC */
    bool running_fg;                /**< True when open in the foreground */
    bool running_bg;                /**< True when retained in the background */
    bool hw_active;                 /**< True when `dac_oneshot` channel is registered and active */
    bool spk_streaming;             /**< True when actively pushing LUT audio to `bsp_spk_passthrough` */
    bool spk_yielded_to_fg;         /**< True when background `analoggen` yielded speaker to FG input tool */
    bool psram_lut_allocated;       /**< True when primary 256-byte LUT is allocated in PSRAM */
    esp_err_t last_error;           /**< Most recent hardware error (`ESP_OK` when nominal) */
} analoggen_status_t;

/* --------------------------------------------------------------------------
 * Pure LUT Synthesis, Clamping & Resampling Functions (analoggen_lut.c)
 * -------------------------------------------------------------------------- */

/**
 * @brief Clamp a requested analog generator frequency to `[ANALOGGEN_MIN_FREQ_HZ, ANALOGGEN_MAX_FREQ_HZ]`.
 *
 * @param[in]  requested_hz Requested frequency in Hz.
 * @param[out] out_clamped  Optional flag set to true if `requested_hz` was outside `[1, 1000]`.
 * @return Clamped frequency in Hz (`1..1000`).
 */
uint32_t analoggen_clamp_freq_hz(uint32_t requested_hz, bool *out_clamped);

/**
 * @brief Convert a target voltage in millivolts to an 8-bit unsigned DAC code (`0..255`) with rail clamping.
 *
 * @param[in]  mv          Target voltage in millivolts (clamped to `[0, 3300]`).
 * @param[out] out_clipped Optional pointer set to `true` if `mv < 0` or `mv > 3300`.
 * @return Clamped 8-bit DAC code (`0..255`).
 */
uint8_t analoggen_mv_to_dac_code(int32_t mv, bool *out_clipped);

/**
 * @brief Convert an 8-bit unsigned DAC code (`0..255`) to millivolts (`0..3300` mV).
 *
 * @param[in] code 8-bit DAC code (`0..255`).
 * @return Equivalent voltage in millivolts (`0..3300`).
 */
uint16_t analoggen_dac_code_to_mv(uint8_t code);

/**
 * @brief Synthesize a `lut_len`-point 8-bit DAC lookup table for `cfg` with rail clamping and statistics.
 *
 * @param[in]  cfg       Generator waveform, amplitude, offset, and staircase configuration.
 * @param[out] out_lut   Destination array of `lut_len` 8-bit DAC codes (`0..255`).
 * @param[in]  lut_len   Length of `out_lut` (typically `ANALOGGEN_LUT_SIZE = 256`, must be `>= 8`).
 * @param[out] out_stats Optional pointer to receive LUT min/max/Vpp/mean/clipping statistics.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t analoggen_lut_build(const analoggen_config_t *cfg,
                              uint8_t *out_lut,
                              size_t lut_len,
                              analoggen_lut_stats_t *out_stats);

/**
 * @brief Compute the LUT sample index `[0..lut_len-1]` at timestamp `time_us` for `freq_hz`.
 *
 * @param[in] time_us Timestamp in microseconds.
 * @param[in] freq_hz Output frequency in Hz (`>= 1`).
 * @param[in] lut_len Lookup table length (`>= 1`, typically `256`).
 * @return LUT index in `[0, lut_len - 1]`.
 */
size_t analoggen_calc_phase_index(uint64_t time_us, uint32_t freq_hz, size_t lut_len);

/**
 * @brief Resample a 256-entry DAC LUT into a phase-continuous 8-bit audio buffer for speaker monitoring.
 *
 * Uses a Q16.16 fixed-point phase accumulator (`inout_phase_q16`) so successive chunks streamed
 * to `bsp_spk_passthrough_push_u8()` have zero phase discontinuities at block boundaries.
 *
 * @param[in]     lut                Source lookup table of length `lut_len`.
 * @param[in]     lut_len            Number of entries in `lut` (`>= 2`).
 * @param[in]     wave_freq_hz       Waveform frequency in Hz (`0` for DC).
 * @param[in]     spk_sample_rate_hz Target speaker sample rate in Hz (e.g. `16000`).
 * @param[in,out] inout_phase_q16    Persistent Q16.16 phase accumulator across chunks.
 * @param[out]    out_audio_u8       Destination buffer of `audio_count` 8-bit samples.
 * @param[in]     audio_count        Number of audio samples to synthesize.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t analoggen_resample_lut_for_speaker(const uint8_t *lut,
                                             size_t lut_len,
                                             uint32_t wave_freq_hz,
                                             uint32_t spk_sample_rate_hz,
                                             uint32_t *inout_phase_q16,
                                             uint8_t *out_audio_u8,
                                             size_t audio_count);

/**
 * @brief Return a human-readable label for an `analoggen_wave_t`.
 *
 * @param[in] wave Waveform enum value.
 * @return Static null-terminated string (`"SINE"`, `"TRIANGLE"`, `"SAWTOOTH"`, `"STAIR"`, `"DC"`).
 */
const char *analoggen_wave_to_str(analoggen_wave_t wave);

/* --------------------------------------------------------------------------
 * Runtime Instrument Configuration & Control API (tool_analoggen.c)
 * -------------------------------------------------------------------------- */

/**
 * @brief Check whether `pin` is a valid ESP32 8-bit DAC output pin (`GPIO25` or `GPIO26`).
 *
 * @param[in] pin GPIO number to inspect.
 * @return True if `pin` is `GPIO_NUM_25` or `GPIO_NUM_26`, false otherwise.
 */
bool tool_analoggen_is_pin_supported(gpio_num_t pin);

/**
 * @brief Select the DAC output pin (`GPIO25` DAC1 or `GPIO26` DAC2).
 *
 * When active, claims `pin` via `bsp_pin_claim_ex(pin, ANALOGGEN_TOOL_ID, true)` before
 * releasing the previous pin and rebinding the `dac_oneshot` channel.
 *
 * @param[in] pin Target DAC GPIO (`GPIO_NUM_25` or `GPIO_NUM_26`).
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_ERR_NOT_ALLOWED` if `pin` is a protected system pin
 *  - `ESP_ERR_NOT_SUPPORTED` if `pin` is an input-only pin
 *  - `ESP_ERR_INVALID_ARG` if `pin` is not a DAC pin (`GPIO25`/`GPIO26`)
 *  - `ESP_ERR_INVALID_STATE` if `pin` is already claimed by another tool
 */
esp_err_t tool_analoggen_set_pin(gpio_num_t pin);

/**
 * @brief Get the currently selected DAC output pin (`GPIO25` or `GPIO26`).
 *
 * @return Active `gpio_num_t`.
 */
gpio_num_t tool_analoggen_get_pin(void);

/**
 * @brief Set the analog waveform shape and rebuild the PSRAM/SRAM lookup table.
 *
 * @param[in] waveform Target `analoggen_wave_t`.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if invalid.
 */
esp_err_t tool_analoggen_set_waveform(analoggen_wave_t waveform);

/**
 * @brief Get the currently configured analog waveform shape.
 *
 * @return Active `analoggen_wave_t`.
 */
analoggen_wave_t tool_analoggen_get_waveform(void);

/**
 * @brief Set the target output frequency in Hz (`1..ANALOGGEN_MAX_FREQ_HZ`).
 *
 * @param[in] freq_hz Target frequency in Hz (`1..1000`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if outside `[1, 1000]`.
 */
esp_err_t tool_analoggen_set_freq_hz(uint32_t freq_hz);

/**
 * @brief Get the currently configured output frequency in Hz.
 *
 * @return Target frequency in Hz.
 */
uint32_t tool_analoggen_get_freq_hz(void);

/**
 * @brief Set the peak-to-peak amplitude in millivolts (`0..3300`) and rebuild the LUT.
 *
 * @param[in] amplitude_mv Peak-to-peak amplitude in mV (`0..3300`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `> 3300`.
 */
esp_err_t tool_analoggen_set_amplitude_mv(uint16_t amplitude_mv);

/**
 * @brief Get the currently configured peak-to-peak amplitude in millivolts.
 *
 * @return Amplitude in mV (`0..3300`).
 */
uint16_t tool_analoggen_get_amplitude_mv(void);

/**
 * @brief Set the DC center offset in millivolts (`0..3300`) and rebuild the LUT.
 *
 * @param[in] offset_mv DC offset in mV (`0..3300`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `> 3300`.
 */
esp_err_t tool_analoggen_set_offset_mv(uint16_t offset_mv);

/**
 * @brief Get the currently configured DC center offset in millivolts.
 *
 * @return DC offset in mV (`0..3300`).
 */
uint16_t tool_analoggen_get_offset_mv(void);

/**
 * @brief Set the number of quantization steps (`2..16`) for `ANALOGGEN_WAVE_STAIRCASE`.
 *
 * @param[in] steps Staircase step count (`2..16`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if outside `[2, 16]`.
 */
esp_err_t tool_analoggen_set_stair_steps(uint8_t steps);

/**
 * @brief Get the currently configured staircase step count (`2..16`).
 *
 * @return Staircase step count.
 */
uint8_t tool_analoggen_get_stair_steps(void);

/**
 * @brief Enable or pause DAC output on the claimed GPIO pin.
 *
 * @param[in] enabled True to drive the DAC LUT, false to output 0 V (`code = 0`).
 * @return `ESP_OK` on success.
 */
esp_err_t tool_analoggen_set_output_enabled(bool enabled);

/**
 * @brief Query whether DAC output is enabled.
 *
 * @return True if enabled, false if paused.
 */
bool tool_analoggen_is_output_enabled(void);

/**
 * @brief Set the "Keep Running in Background" preference for `tool_analoggen`.
 *
 * @param[in] keep_in_bg True to retain DAC output in background when returning Home.
 * @return `ESP_OK` on success.
 */
esp_err_t tool_analoggen_set_keep_in_bg(bool keep_in_bg);

/**
 * @brief Query the "Keep Running in Background" preference for `tool_analoggen`.
 *
 * @return True if background retention is enabled, false otherwise.
 */
bool tool_analoggen_get_keep_in_bg(void);

/**
 * @brief Query whether `tool_analoggen` currently owns the speaker audio passthrough feed.
 *
 * Returns `true` when `tool_analoggen` is active in the foreground, or when it is running in the
 * background while no speaker-capable foreground input tool (`tool_scope`, `tool_voltmeter`) is open.
 * Returns `false` when a speaker-capable foreground input tool is open so the foreground tool
 * has exclusive ownership of `bsp_spk_passthrough`.
 *
 * @return True if `tool_analoggen` owns the speaker stream, false if yielded or inactive.
 */
bool tool_analoggen_owns_speaker(void);

/**
 * @brief Copy out the current 256-entry 8-bit lookup table under lock.
 *
 * @param[out] out_lut Destination buffer.
 * @param[in]  max_len Maximum bytes to copy (up to `ANALOGGEN_LUT_SIZE`).
 * @return Number of bytes copied (`0` if `out_lut == NULL`).
 */
size_t tool_analoggen_copy_lut(uint8_t *out_lut, size_t max_len);

/**
 * @brief Copy out a thread-safe snapshot of the current `analoggen_status_t`.
 *
 * @param[out] out_status Destination status structure.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `out_status` is `NULL`.
 */
esp_err_t tool_analoggen_get_status(analoggen_status_t *out_status);

/**
 * @brief Execute a single deterministic DAC + speaker worker step (used in tests).
 *
 * @return `ESP_OK` on success, or error code.
 */
esp_err_t tool_analoggen_run_single_step(void);

/**
 * @brief Get the static `analyser_tool_t` descriptor for `tool_analoggen`.
 *
 * @return Pointer to static `analyser_tool_t`.
 */
const analyser_tool_t *tool_analoggen_get_descriptor(void);

/**
 * @brief Register the Analog Generator tool (`id = "analoggen"`) and its self-test with `tool_core`.
 *
 * @return `ESP_OK` on success, or error code from `tool_registry_register()`.
 */
esp_err_t tool_analoggen_register(void);

#ifdef __cplusplus
}
#endif
