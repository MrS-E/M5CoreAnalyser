/**
 * @file tool_voltmeter_priv.h
 * @brief Private test seams for deterministic unit and integration testing of `tool_voltmeter`.
 */

#pragma once

#include "tool_voltmeter.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Optional custom ADC batch reader callback for deterministic unit testing.
 */
typedef esp_err_t (*voltmeter_sample_source_fn_t)(gpio_num_t pin,
                                                  uint16_t *out_raw,
                                                  float *out_uncal_mv,
                                                  size_t count,
                                                  void *user_ctx);

/**
 * @brief Override the hardware ADC1 batch reader with a custom test callback (pass `NULL` to restore hardware ADC1).
 *
 * @param[in] fn       Custom batch reader callback, or `NULL`.
 * @param[in] user_ctx Opaque context passed to `fn`.
 */
void tool_voltmeter_set_sample_source(voltmeter_sample_source_fn_t fn, void *user_ctx);

/**
 * @brief Execute one Core 1 acquisition/DSP step synchronously for deterministic testing.
 *
 * @return `ESP_OK` on success, or error code.
 */
esp_err_t tool_voltmeter_run_single_step(void);

#ifdef __cplusplus
}
#endif
