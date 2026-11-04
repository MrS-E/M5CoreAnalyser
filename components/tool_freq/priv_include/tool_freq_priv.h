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

#ifdef __cplusplus
}
#endif
