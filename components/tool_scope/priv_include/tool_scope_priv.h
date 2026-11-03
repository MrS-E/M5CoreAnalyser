/**
 * @file tool_scope_priv.h
 * @brief Private test seams for deterministic unit and integration testing of `tool_scope`.
 */

#pragma once

#include "tool_scope.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Custom sample generator callback for deterministic testing of `tool_scope`.
 */
typedef esp_err_t (*scope_sample_source_fn_t)(gpio_num_t pin,
                                              uint16_t *out_raw_adc,
                                              uint16_t *out_mv,
                                              size_t count,
                                              uint32_t *out_sample_rate_hz,
                                              void *user_ctx);

/**
 * @brief Override hardware ADC1 capture with a deterministic test sample source (pass `NULL` to restore).
 *
 * @param[in] fn       Custom sample source callback, or `NULL`.
 * @param[in] user_ctx Opaque context passed to `fn`.
 */
void tool_scope_set_sample_source(scope_sample_source_fn_t fn, void *user_ctx);

/**
 * @brief Execute one Core 1 oscilloscope acquisition, trigger, and DSP step synchronously.
 *
 * @return `ESP_OK` on success, or error code.
 */
esp_err_t tool_scope_run_single_step(void);

#ifdef __cplusplus
}
#endif
