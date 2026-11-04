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

/**
 * @brief Optional IRAM-safe per-sample generator invoked inside `on_logic_gptimer_alarm_isr` for testing.
 */
typedef uint8_t (*logic_isr_sample_fn_t)(size_t sample_idx, int64_t now_us, void *user_ctx);

/**
 * @brief Install or clear an IRAM-safe sample callback invoked on each real GPTimer alarm ISR tick.
 *
 * Pass `NULL` to restore normal hardware `GPIO.in` / `GPIO.in1.val` register sampling.
 *
 * @param[in] fn       Callback invoked per GPTimer ISR sample, or `NULL`.
 * @param[in] user_ctx Opaque pointer in internal RAM passed to `fn`.
 */
void tool_logic_set_isr_sample_cb_for_test(logic_isr_sample_fn_t fn, void *user_ctx);

/**
 * @brief Inject a single GPIO edge transition into the `DRAM_ATTR` edge bounce buffer (test seam).
 *
 * Exercises the exact `IRAM_ATTR` edge recording path (`record_logic_edge_iram`) used by
 * `on_logic_gpio_edge_isr` during `LOGIC_CAP_MODE_RMT_EDGE`.
 *
 * @param[in] timestamp_us Transition timestamp in microseconds.
 * @param[in] level        Logic level (`0` or `1`) after the transition.
 */
void tool_logic_record_edge_isr_for_test(uint32_t timestamp_us, uint8_t level);

/**
 * @brief Force the hardware GPTimer sampled capture path to simulate a stalled timer timeout (test seam).
 *
 * @param[in] force_timeout True to suppress timer start so `capture_logic_buffer()` times out with `ESP_ERR_TIMEOUT`.
 */
void tool_logic_force_gptimer_timeout_for_test(bool force_timeout);

/**
 * @brief Diagnostic counters captured from the most recent `tool_logic` acquisition window.
 */
typedef struct {
    uint32_t gptimer_isr_count;     /**< Total GPTimer alarm ISR invocations in the window */
    uint32_t bounce_overruns;       /**< Internal SRAM bounce ring overruns (expected `0`) */
    uint32_t edge_isr_invocations;  /**< Total GPIO edge ISR entries in the window */
    uint32_t edge_stored_count;     /**< Valid edge transitions stored in the bounce buffer */
    bool edge_isr_capped;           /**< True if the edge ISR disabled itself at its hard cap */
    uint32_t rmt_symbol_count;      /**< Symbols returned by RMT RX or converted from edge ISR */
    uint32_t capture_elapsed_us;    /**< Measured capture span in microseconds */
} logic_window_stats_t;

/**
 * @brief Copy out diagnostic statistics from the most recent acquisition window.
 *
 * @param[out] out_stats Destination structure.
 */
void tool_logic_get_last_window_stats(logic_window_stats_t *out_stats);

/**
 * @brief Return the minimum free stack bytes observed on the `logic_c1` worker task.
 *
 * @return Free stack high-water mark in bytes (`0` if worker has not executed a step yet).
 */
uint32_t tool_logic_get_worker_stack_hwm(void);

#ifdef __cplusplus
}
#endif
