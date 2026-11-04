/**
 * @file tool_analoggen_priv.h
 * @brief Private test seams for `tool_analoggen`.
 */

#pragma once

#include "esp_err.h"
#include "tool_analoggen.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Reset `tool_analoggen` state to power-on defaults for deterministic unit test isolation.
 */
void tool_analoggen_reset_for_test(void);

/**
 * @brief Inject a simulated hardware error on the next DAC channel initialization.
 *
 * @param[in] err Error code to inject (`ESP_OK` to clear).
 */
void tool_analoggen_force_hw_error_for_test(esp_err_t err);

#ifdef __cplusplus
}
#endif
