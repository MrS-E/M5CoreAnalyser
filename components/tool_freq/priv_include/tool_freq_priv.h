/**
 * @file tool_freq_priv.h
 * @brief Private test seams for deterministic unit and integration testing of `tool_freq`.
 */

#pragma once

#include "tool_freq.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Custom hardware capture seam for deterministic testing of `tool_freq`.
 *
 * Allows tests to inject synthetic RMT symbol arrays and/or PCNT gate pulse counts
 * without requiring an external physical signal generator wired to the ESP32 GPIO.
 */
typedef esp_err_t (*freq_capture_source_fn_t)(gpio_num_t pin,
                                              uint32_t gate_window_us,
                                              rmt_symbol_word_t *out_symbols,
                                              size_t max_symbols,
                                              size_t *out_num_symbols,
                                              uint64_t *out_pcnt_pulses,
                                              uint64_t *out_actual_gate_us,
                                              freq_pcnt_edge_mode_t *out_edge_mode,
                                              void *user_ctx);

/**
 * @brief Override hardware RMT/PCNT capture with a deterministic test source (pass `NULL` to restore hardware).
 *
 * @param[in] fn       Custom capture source callback, or `NULL`.
 * @param[in] user_ctx Opaque context pointer passed to `fn`.
 */
void tool_freq_set_capture_source(freq_capture_source_fn_t fn, void *user_ctx);

/**
 * @brief Execute one Core 1 Frequency Meter acquisition and calculation step synchronously.
 *
 * @return `ESP_OK` on success, or error code.
 */
esp_err_t tool_freq_run_single_step(void);

/**
 * @brief Record a GPIO edge transition directly into the `IRAM_ATTR` internal SRAM edge ring buffer
 *        during an active capture window (test seam for QEMU where GPIO pad output is not externally looped).
 *
 * @param[in] pin          GPIO pin number where the transition occurred.
 * @param[in] timestamp_us Edge timestamp in microseconds (`esp_timer_get_time()`).
 * @param[in] level        Logic level (`0` or `1`) immediately after the transition.
 */
void tool_freq_record_edge_isr_for_test(gpio_num_t pin, uint64_t timestamp_us, uint8_t level);

/**
 * @brief Inject a simulated ESP-IDF peripheral driver error into `ensure_freq_peripherals()`
 *        to verify error propagation and recovery (`ESP_OK` to clear).
 *
 * @param[in] err Simulated driver error code (`ESP_OK` for normal hardware operation).
 */
void tool_freq_set_driver_error_for_test(esp_err_t err);

/**
 * @brief Return the minimum observed FreeRTOS stack high-water mark in bytes on the `freq_c1` worker task.
 *
 * @return Minimum free stack bytes observed on `freq_c1` (`0` if worker has not stepped yet).
 */
uint32_t tool_freq_get_worker_stack_hwm_bytes(void);

/**
 * @brief Return the number of measurement steps completed directly by the `freq_c1` background worker task.
 *
 * @return Count of completed worker steps since `on_start`.
 */
uint32_t tool_freq_get_worker_completed_steps(void);

#ifdef __cplusplus
}
#endif
