/**
 * @file tool_core_priv.h
 * @brief Private unit-test reset and seam helpers for tool_core.
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Stop any active/background tools, clear the tool registry, and reset registered
 *        self-tests to initial state (unit-test helper).
 */
void tool_registry_test_reset(void);

/**
 * @brief Force `psram_buf_calloc()` to simulate allocation failure for testing OOM recovery.
 *
 * @param[in] fail_after_count Number of allocations to allow before returning `NULL` (`-1` disables fault injection).
 */
void psram_buf_test_set_fail_after(int fail_after_count);

#ifdef __cplusplus
}
#endif
