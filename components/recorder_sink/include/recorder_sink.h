/**
 * @file recorder_sink.h
 * @brief Stubbed sample frame export sink for future USB CDC and Bluetooth streaming.
 *
 * Provides a thread-safe recording state machine (`REC_MODE_OFF`, `REC_MODE_USB_STUB`,
 * `REC_MODE_BT_STUB`) and sample frame accounting contract (`recorder_sink_push()`) used
 * by Core 1 instrument workers and Core 0 UI header controls.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum length (including null terminator) of a recorded frame's source tool ID. */
#define RECORDER_TOOL_ID_MAX_LEN (24)

/**
 * @brief Operating modes for the sample recorder sink.
 */
typedef enum {
    REC_MODE_OFF = 0,  /**< Recording disabled; incoming frames are ignored */
    REC_MODE_USB_STUB, /**< Stubbed USB capture streaming mode */
    REC_MODE_BT_STUB,  /**< Stubbed Bluetooth capture streaming mode */
} recorder_mode_t;

/**
 * @brief Sample frame descriptor submitted by active measurement instruments.
 */
typedef struct {
    const char *tool_id;     /**< Non-empty source tool identifier (e.g., "scope", "voltmeter") */
    uint32_t sample_rate_hz; /**< Acquisition sample rate in Hz */
    uint64_t timestamp_us;   /**< Capture timestamp in microseconds */
    const void *samples;     /**< Pointer to contiguous sample payload buffer */
    size_t sample_count;     /**< Number of samples in this frame (> 0) */
    size_t bytes_per_sample; /**< Size of each sample in bytes (> 0) */
} recorder_frame_t;

/**
 * @brief Thread-safe snapshot of recorder sink state and frame/byte counters.
 */
typedef struct {
    recorder_mode_t mode;                        /**< Current active recording mode */
    recorder_mode_t selected_transport;          /**< Selected transport (`REC_MODE_USB_STUB` or `REC_MODE_BT_STUB`) */
    uint32_t frames_pushed;                      /**< Total frames accepted while recording was active */
    uint32_t usb_frames;                         /**< Total frames accepted in `REC_MODE_USB_STUB` */
    uint32_t bt_frames;                          /**< Total frames accepted in `REC_MODE_BT_STUB` */
    uint32_t dropped_frames;                     /**< Frames rejected while `REC_MODE_OFF` */
    uint64_t samples_pushed;                     /**< Total samples accepted across active frames */
    uint64_t bytes_pushed;                       /**< Total payload bytes accepted across active frames */
    uint32_t last_sample_rate_hz;                /**< Sample rate (Hz) of most recently accepted frame */
    uint64_t last_timestamp_us;                  /**< Timestamp (us) of most recently accepted frame */
    size_t last_sample_count;                    /**< Sample count of most recently accepted frame */
    size_t last_bytes_per_sample;                /**< Sample byte width of most recently accepted frame */
    char last_tool_id[RECORDER_TOOL_ID_MAX_LEN]; /**< Null-terminated tool ID of most recently accepted frame */
} recorder_sink_stats_t;

/**
 * @brief Initialize the recorder sink subsystem to default state (`REC_MODE_OFF`, transport `USB`).
 *
 * @return `ESP_OK` on success.
 */
esp_err_t recorder_sink_init(void);

/**
 * @brief Set the active recorder sink mode.
 *
 * Setting `REC_MODE_USB_STUB` or `REC_MODE_BT_STUB` activates frame capture and updates
 * the remembered transport selection. Setting `REC_MODE_OFF` stops frame capture while
 * preserving the remembered transport selection and accumulated counters.
 *
 * @param[in] mode Target `recorder_mode_t` value.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `mode` is out of range.
 */
esp_err_t recorder_sink_set_mode(recorder_mode_t mode);

/**
 * @brief Get the current active recorder sink mode.
 *
 * @return Current `recorder_mode_t` value.
 */
recorder_mode_t recorder_sink_get_mode(void);

/**
 * @brief Select the target transport (`REC_MODE_USB_STUB` or `REC_MODE_BT_STUB`).
 *
 * If recording is currently active (`mode != REC_MODE_OFF`), switches the active mode
 * immediately to `transport`. If recording is off, stores `transport` for the next
 * `recorder_sink_toggle()` call.
 *
 * @param[in] transport Either `REC_MODE_USB_STUB` or `REC_MODE_BT_STUB`.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `transport` is `REC_MODE_OFF` or invalid.
 */
esp_err_t recorder_sink_set_transport(recorder_mode_t transport);

/**
 * @brief Get the currently selected transport (`REC_MODE_USB_STUB` or `REC_MODE_BT_STUB`).
 *
 * @return Selected transport mode.
 */
recorder_mode_t recorder_sink_get_transport(void);

/**
 * @brief Toggle recording between `REC_MODE_OFF` and the currently selected transport.
 *
 * @return `ESP_OK` on success.
 */
esp_err_t recorder_sink_toggle(void);

/**
 * @brief Return true if recording is currently active (`REC_MODE_USB_STUB` or `REC_MODE_BT_STUB`).
 *
 * @return True if active, false if `REC_MODE_OFF`.
 */
bool recorder_sink_is_recording(void);

/**
 * @brief Push a captured sample frame into the recorder sink.
 *
 * Thread-safe across Core 0 and Core 1. Validates frame fields and checks for byte-count
 * multiplication overflow before accounting.
 *
 * @param[in] frame Pointer to sample frame descriptor.
 * @return
 *  - `ESP_OK` if recording is active and the frame was accepted
 *  - `ESP_ERR_INVALID_ARG` if `frame` or any required field is invalid/overflowing
 *  - `ESP_ERR_INVALID_STATE` if recorder mode is `REC_MODE_OFF`
 */
esp_err_t recorder_sink_push(const recorder_frame_t *frame);

/**
 * @brief Copy out a consistent snapshot of the recorder sink statistics under spinlock.
 *
 * @param[out] out_stats Destination structure to populate.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `out_stats` is `NULL`.
 */
esp_err_t recorder_sink_get_stats(recorder_sink_stats_t *out_stats);

/**
 * @brief Reset accumulated frame/sample/byte counters without changing the active mode.
 *
 * @return `ESP_OK` on success.
 */
esp_err_t recorder_sink_reset_stats(void);

/**
 * @brief Convert a `recorder_mode_t` value to a short UI badge string (`"OFF"`, `"USB"`, `"BT"`).
 *
 * @param[in] mode Recorder mode enum value.
 * @return Static null-terminated string.
 */
const char *recorder_sink_mode_to_str(recorder_mode_t mode);

#ifdef __cplusplus
}
#endif
