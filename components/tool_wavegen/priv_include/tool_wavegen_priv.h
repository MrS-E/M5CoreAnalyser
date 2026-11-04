/**
 * @file tool_wavegen_priv.h
 * @brief Private test seams for `tool_wavegen`.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "tool_wavegen.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Reset `tool_wavegen` state to power-on defaults for deterministic unit test isolation.
 */
void tool_wavegen_reset_for_test(void);

/**
 * @brief Inject a simulated peripheral error on the next hardware apply/start operation.
 *
 * @param[in] err Error code to return (`ESP_OK` to clear).
 */
void tool_wavegen_force_hw_error_for_test(esp_err_t err);

/**
 * @brief Copy out the most recentlystaged RMT TX symbols from internal SRAM bounce buffer.
 *
 * @param[out] out_symbols Destination symbol array.
 * @param[in]  max_symbols Maximum symbols to copy.
 * @return Number of symbols copied.
 */
size_t tool_wavegen_copy_last_rmt_symbols_for_test(rmt_symbol_word_t *out_symbols,
                                                   size_t max_symbols);

#ifdef __cplusplus
}
#endif
