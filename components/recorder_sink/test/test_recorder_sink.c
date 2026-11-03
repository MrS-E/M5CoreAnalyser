/**
 * @file test_recorder_sink.c
 * @brief Unity unit tests for recorder_sink mode transitions, frame/byte accounting,
 *        and argument validation.
 */

#include <stdint.h>
#include <string.h>

#include "recorder_sink.h"
#include "../priv_include/recorder_sink_priv.h"
#include "unity.h"

TEST_CASE("given recorder_sink when mode and transport switched then state transitions and badges update", "[recorder_sink]")
{
    recorder_sink_test_reset();

    TEST_ASSERT_EQUAL(REC_MODE_OFF, recorder_sink_get_mode());
    TEST_ASSERT_EQUAL(REC_MODE_USB_STUB, recorder_sink_get_transport());
    TEST_ASSERT_FALSE(recorder_sink_is_recording());
    TEST_ASSERT_EQUAL_STRING("OFF", recorder_sink_mode_to_str(REC_MODE_OFF));
    TEST_ASSERT_EQUAL_STRING("USB", recorder_sink_mode_to_str(REC_MODE_USB_STUB));
    TEST_ASSERT_EQUAL_STRING("BT", recorder_sink_mode_to_str(REC_MODE_BT_STUB));

    /* Invalid enum arguments are rejected */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, recorder_sink_set_mode((recorder_mode_t)99));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, recorder_sink_set_transport(REC_MODE_OFF));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, recorder_sink_set_transport((recorder_mode_t)42));

    /* Selecting BT while OFF updates remembered transport without starting recording */
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_set_transport(REC_MODE_BT_STUB));
    TEST_ASSERT_EQUAL(REC_MODE_OFF, recorder_sink_get_mode());
    TEST_ASSERT_EQUAL(REC_MODE_BT_STUB, recorder_sink_get_transport());
    TEST_ASSERT_FALSE(recorder_sink_is_recording());

    /* Toggling ON activates the selected BT transport */
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_toggle());
    TEST_ASSERT_EQUAL(REC_MODE_BT_STUB, recorder_sink_get_mode());
    TEST_ASSERT_TRUE(recorder_sink_is_recording());

    /* Switching transport while active immediately switches active mode to USB */
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_set_transport(REC_MODE_USB_STUB));
    TEST_ASSERT_EQUAL(REC_MODE_USB_STUB, recorder_sink_get_mode());
    TEST_ASSERT_EQUAL(REC_MODE_USB_STUB, recorder_sink_get_transport());

    /* Toggling OFF stops recording while keeping USB as remembered transport */
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_toggle());
    TEST_ASSERT_EQUAL(REC_MODE_OFF, recorder_sink_get_mode());
    TEST_ASSERT_EQUAL(REC_MODE_USB_STUB, recorder_sink_get_transport());
    TEST_ASSERT_FALSE(recorder_sink_is_recording());
}

TEST_CASE("given recorder_sink across OFF USB and BT modes when frames pushed then counters account accurately", "[recorder_sink]")
{
    recorder_sink_test_reset();

    uint16_t adc_samples[128] = {2048};
    uint8_t logic_samples[64] = {0xAA};

    recorder_frame_t scope_frame = {
        .tool_id = "scope",
        .sample_rate_hz = 20000,
        .timestamp_us = 100500ULL,
        .samples = adc_samples,
        .sample_count = 128,
        .bytes_per_sample = sizeof(uint16_t),
    };

    recorder_frame_t logic_frame = {
        .tool_id = "logic",
        .sample_rate_hz = 100000,
        .timestamp_us = 250000ULL,
        .samples = logic_samples,
        .sample_count = 64,
        .bytes_per_sample = sizeof(uint8_t),
    };

    /* Pushing while REC_MODE_OFF returns ESP_ERR_INVALID_STATE and increments dropped_frames only */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, recorder_sink_push(&scope_frame));

    recorder_sink_stats_t stats = {0};
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_get_stats(&stats));
    TEST_ASSERT_EQUAL_UINT32(0, stats.frames_pushed);
    TEST_ASSERT_EQUAL_UINT32(1, stats.dropped_frames);
    TEST_ASSERT_EQUAL_UINT64(0, stats.samples_pushed);
    TEST_ASSERT_EQUAL_UINT64(0, stats.bytes_pushed);

    /* Switch to REC_MODE_USB_STUB and push two scope frames (128 * 2 = 256 bytes each) */
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_set_mode(REC_MODE_USB_STUB));
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_push(&scope_frame));
    scope_frame.timestamp_us = 106900ULL;
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_push(&scope_frame));

    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_get_stats(&stats));
    TEST_ASSERT_EQUAL(REC_MODE_USB_STUB, stats.mode);
    TEST_ASSERT_EQUAL_UINT32(2, stats.frames_pushed);
    TEST_ASSERT_EQUAL_UINT32(2, stats.usb_frames);
    TEST_ASSERT_EQUAL_UINT32(0, stats.bt_frames);
    TEST_ASSERT_EQUAL_UINT32(1, stats.dropped_frames);
    TEST_ASSERT_EQUAL_UINT64(256, stats.samples_pushed);
    TEST_ASSERT_EQUAL_UINT64(512, stats.bytes_pushed);
    TEST_ASSERT_EQUAL_STRING("scope", stats.last_tool_id);
    TEST_ASSERT_EQUAL_UINT32(20000, stats.last_sample_rate_hz);
    TEST_ASSERT_EQUAL_UINT64(106900ULL, stats.last_timestamp_us);
    TEST_ASSERT_EQUAL_UINT32(128, (uint32_t)stats.last_sample_count);
    TEST_ASSERT_EQUAL_UINT32(2, (uint32_t)stats.last_bytes_per_sample);

    /* Switch to REC_MODE_BT_STUB and push a logic frame (64 * 1 = 64 bytes) */
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_set_mode(REC_MODE_BT_STUB));
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_push(&logic_frame));

    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_get_stats(&stats));
    TEST_ASSERT_EQUAL(REC_MODE_BT_STUB, stats.mode);
    TEST_ASSERT_EQUAL_UINT32(3, stats.frames_pushed);
    TEST_ASSERT_EQUAL_UINT32(2, stats.usb_frames);
    TEST_ASSERT_EQUAL_UINT32(1, stats.bt_frames);
    TEST_ASSERT_EQUAL_UINT64(320, stats.samples_pushed);
    TEST_ASSERT_EQUAL_UINT64(576, stats.bytes_pushed);
    TEST_ASSERT_EQUAL_STRING("logic", stats.last_tool_id);
    TEST_ASSERT_EQUAL_UINT32(100000, stats.last_sample_rate_hz);
    TEST_ASSERT_EQUAL_UINT64(250000ULL, stats.last_timestamp_us);

    /* Resetting stats zeroes counters while preserving active mode and transport */
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_reset_stats());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_get_stats(&stats));
    TEST_ASSERT_EQUAL(REC_MODE_BT_STUB, stats.mode);
    TEST_ASSERT_EQUAL(REC_MODE_BT_STUB, stats.selected_transport);
    TEST_ASSERT_EQUAL_UINT32(0, stats.frames_pushed);
    TEST_ASSERT_EQUAL_UINT64(0, stats.bytes_pushed);
    TEST_ASSERT_EQUAL_STRING("", stats.last_tool_id);
}

TEST_CASE("given invalid or overflowing frame descriptors when recorder_sink_push called then returns ESP_ERR_INVALID_ARG", "[recorder_sink]")
{
    recorder_sink_test_reset();
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_set_mode(REC_MODE_USB_STUB));

    uint16_t dummy_buf[4] = {1, 2, 3, 4};
    recorder_frame_t bad_frame = {
        .tool_id = "voltmeter",
        .sample_rate_hz = 1000,
        .timestamp_us = 100ULL,
        .samples = dummy_buf,
        .sample_count = 4,
        .bytes_per_sample = 2,
    };

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, recorder_sink_push(NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, recorder_sink_get_stats(NULL));

    bad_frame.tool_id = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, recorder_sink_push(&bad_frame));

    bad_frame.tool_id = "";
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, recorder_sink_push(&bad_frame));

    bad_frame.tool_id = "voltmeter";
    bad_frame.samples = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, recorder_sink_push(&bad_frame));

    bad_frame.samples = dummy_buf;
    bad_frame.sample_count = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, recorder_sink_push(&bad_frame));

    bad_frame.sample_count = 4;
    bad_frame.bytes_per_sample = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, recorder_sink_push(&bad_frame));

    /* Multiplication overflow check */
    bad_frame.sample_count = (SIZE_MAX / 2U) + 2U;
    bad_frame.bytes_per_sample = 4U;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, recorder_sink_push(&bad_frame));

    /* Long tool ID is safely truncated without buffer overrun */
    bad_frame.tool_id = "tool_with_extremely_long_identifier_name_12345";
    bad_frame.sample_count = 4;
    bad_frame.bytes_per_sample = 2;
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_push(&bad_frame));

    recorder_sink_stats_t stats = {0};
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_get_stats(&stats));
    TEST_ASSERT_EQUAL_UINT32(1, stats.frames_pushed);
    TEST_ASSERT_EQUAL_UINT32(RECORDER_TOOL_ID_MAX_LEN - 1, strlen(stats.last_tool_id));
}
