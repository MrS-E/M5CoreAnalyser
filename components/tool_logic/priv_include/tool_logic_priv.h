/**
 * @file tool_logic_priv.h
 * @brief Private test seams for deterministic unit and integration testing of `tool_logic`.
 */

#pragma once

#include "tool_logic.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Custom sample/edge generator callback for deterministic testing of `tool_logic`.
 */
typedef esp_err_t (*logic_capture_source_fn_t)(logic_capture_mode_t cap_mode,
                                               const gpio_num_t channel_pins[LOGIC_CHANNEL_COUNT],
                                               uint32_t requested_rate_hz,
                                               uint8_t *out_samples,
                                               size_t max_samples,
                                               size_t *out_sample_count,
                                               uint32_t *out_actual_rate_hz,
                                               void *user_ctx);

/**
 * @brief Override hardware GPIO/RMT capture with a deterministic test source (pass `NULL` to restore).
 *
 * @param[in] fn       Custom capture callback, or `NULL`.
 * @param[in] user_ctx Opaque context pointer passed to `fn`.
 */
void tool_logic_set_capture_source(logic_capture_source_fn_t fn, void *user_ctx);

/**
 * @brief Execute one Core 1 Logic Analyzer capture, protocol decode, and frame swap step synchronously.
 *
 * @return `ESP_OK` on success, or error code.
 */
esp_err_t tool_logic_run_single_step(void);

#ifdef __cplusplus
}
#endif
