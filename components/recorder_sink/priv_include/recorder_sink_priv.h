/**
 * @file recorder_sink_priv.h
 * @brief Private unit-test reset helper for recorder_sink.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Reset recorder sink state, mode (`REC_MODE_OFF`), transport (`REC_MODE_USB_STUB`),
 *        and all counters to power-on defaults for deterministic unit testing.
 */
void recorder_sink_test_reset(void);

#ifdef __cplusplus
}
#endif
