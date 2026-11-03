/**
 * @file recorder_sink.c
 * @brief Stubbed USB/Bluetooth sample frame recorder sink and telemetry counters.
 */

#include "recorder_sink.h"
#include "recorder_sink_priv.h"

#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "recorder_sink";

static portMUX_TYPE s_rec_lock = portMUX_INITIALIZER_UNLOCKED;

static recorder_sink_stats_t s_stats = {
    .mode = REC_MODE_OFF,
    .selected_transport = REC_MODE_USB_STUB,
};

static bool is_valid_mode(recorder_mode_t mode)
{
    return (mode == REC_MODE_OFF) ||
           (mode == REC_MODE_USB_STUB) ||
           (mode == REC_MODE_BT_STUB);
}

static void reset_counters_locked(void)
{
    s_stats.frames_pushed = 0;
    s_stats.usb_frames = 0;
    s_stats.bt_frames = 0;
    s_stats.dropped_frames = 0;
    s_stats.samples_pushed = 0;
    s_stats.bytes_pushed = 0;
    s_stats.last_sample_rate_hz = 0;
    s_stats.last_timestamp_us = 0;
    s_stats.last_sample_count = 0;
    s_stats.last_bytes_per_sample = 0;
    s_stats.last_tool_id[0] = '\0';
}

void recorder_sink_test_reset(void)
{
    portENTER_CRITICAL(&s_rec_lock);
    s_stats.mode = REC_MODE_OFF;
    s_stats.selected_transport = REC_MODE_USB_STUB;
    reset_counters_locked();
    portEXIT_CRITICAL(&s_rec_lock);
}

esp_err_t recorder_sink_init(void)
{
    recorder_sink_test_reset();
    return ESP_OK;
}

esp_err_t recorder_sink_set_mode(recorder_mode_t mode)
{
    if (!is_valid_mode(mode)) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_rec_lock);
    s_stats.mode = mode;
    if (mode == REC_MODE_USB_STUB || mode == REC_MODE_BT_STUB) {
        s_stats.selected_transport = mode;
    }
    portEXIT_CRITICAL(&s_rec_lock);

    ESP_LOGI(TAG, "Recorder mode set to %s", recorder_sink_mode_to_str(mode));
    return ESP_OK;
}

recorder_mode_t recorder_sink_get_mode(void)
{
    recorder_mode_t mode;
    portENTER_CRITICAL(&s_rec_lock);
    mode = s_stats.mode;
    portEXIT_CRITICAL(&s_rec_lock);
    return mode;
}

esp_err_t recorder_sink_set_transport(recorder_mode_t transport)
{
    if (transport != REC_MODE_USB_STUB && transport != REC_MODE_BT_STUB) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_rec_lock);
    s_stats.selected_transport = transport;
    if (s_stats.mode != REC_MODE_OFF) {
        s_stats.mode = transport;
    }
    portEXIT_CRITICAL(&s_rec_lock);

    ESP_LOGI(TAG, "Recorder transport selected: %s", recorder_sink_mode_to_str(transport));
    return ESP_OK;
}

recorder_mode_t recorder_sink_get_transport(void)
{
    recorder_mode_t transport;
    portENTER_CRITICAL(&s_rec_lock);
    transport = s_stats.selected_transport;
    portEXIT_CRITICAL(&s_rec_lock);
    return transport;
}

esp_err_t recorder_sink_toggle(void)
{
    recorder_mode_t next_mode;
    portENTER_CRITICAL(&s_rec_lock);
    if (s_stats.mode == REC_MODE_OFF) {
        if (s_stats.selected_transport != REC_MODE_USB_STUB &&
            s_stats.selected_transport != REC_MODE_BT_STUB) {
            s_stats.selected_transport = REC_MODE_USB_STUB;
        }
        s_stats.mode = s_stats.selected_transport;
    } else {
        s_stats.mode = REC_MODE_OFF;
    }
    next_mode = s_stats.mode;
    portEXIT_CRITICAL(&s_rec_lock);

    ESP_LOGI(TAG, "Recorder toggled to %s", recorder_sink_mode_to_str(next_mode));
    return ESP_OK;
}

bool recorder_sink_is_recording(void)
{
    return recorder_sink_get_mode() != REC_MODE_OFF;
}

esp_err_t recorder_sink_push(const recorder_frame_t *frame)
{
    if (frame == NULL ||
        frame->tool_id == NULL ||
        frame->tool_id[0] == '\0' ||
        frame->samples == NULL ||
        frame->sample_count == 0 ||
        frame->bytes_per_sample == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (frame->sample_count > (SIZE_MAX / frame->bytes_per_sample)) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t frame_bytes = frame->sample_count * frame->bytes_per_sample;

    portENTER_CRITICAL(&s_rec_lock);
    if (s_stats.mode == REC_MODE_OFF) {
        s_stats.dropped_frames++;
        portEXIT_CRITICAL(&s_rec_lock);
        return ESP_ERR_INVALID_STATE;
    }

    s_stats.frames_pushed++;
    if (s_stats.mode == REC_MODE_USB_STUB) {
        s_stats.usb_frames++;
    } else if (s_stats.mode == REC_MODE_BT_STUB) {
        s_stats.bt_frames++;
    }

    s_stats.samples_pushed += (uint64_t)frame->sample_count;
    s_stats.bytes_pushed += (uint64_t)frame_bytes;
    s_stats.last_sample_rate_hz = frame->sample_rate_hz;
    s_stats.last_timestamp_us = frame->timestamp_us;
    s_stats.last_sample_count = frame->sample_count;
    s_stats.last_bytes_per_sample = frame->bytes_per_sample;
    strncpy(s_stats.last_tool_id, frame->tool_id, sizeof(s_stats.last_tool_id) - 1);
    s_stats.last_tool_id[sizeof(s_stats.last_tool_id) - 1] = '\0';
    portEXIT_CRITICAL(&s_rec_lock);

    return ESP_OK;
}

esp_err_t recorder_sink_get_stats(recorder_sink_stats_t *out_stats)
{
    if (out_stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_rec_lock);
    *out_stats = s_stats;
    portEXIT_CRITICAL(&s_rec_lock);
    return ESP_OK;
}

esp_err_t recorder_sink_reset_stats(void)
{
    portENTER_CRITICAL(&s_rec_lock);
    reset_counters_locked();
    portEXIT_CRITICAL(&s_rec_lock);
    return ESP_OK;
}

const char *recorder_sink_mode_to_str(recorder_mode_t mode)
{
    switch (mode) {
    case REC_MODE_OFF:
        return "OFF";
    case REC_MODE_USB_STUB:
        return "USB";
    case REC_MODE_BT_STUB:
        return "BT";
    default:
        return "INVALID";
    }
}
