/**
 * @file test_ui_shell.c
 * @brief Unity unit tests for ui_shell header badge formatting, view-model state transitions,
 *        tool launching, background generator retention, and conflict status handling.
 */

#include <string.h>

#include "bsp_m5core2.h"
#include "../../bsp_m5core2/priv_include/bsp_m5core2_priv.h"
#include "recorder_sink.h"
#include "../../recorder_sink/priv_include/recorder_sink_priv.h"
#include "tool_core.h"
#include "../../tool_core/priv_include/tool_core_priv.h"
#include "ui_shell.h"
#include "unity.h"

typedef struct {
    const char *id;
    gpio_num_t pin;
    bool require_output;
    bool bg_active;
    int ui_refresh_count;
} ui_mock_tool_ctx_t;

static esp_err_t ui_mock_start(lv_obj_t *parent_cont, void *ctx)
{
    ui_mock_tool_ctx_t *m = (ui_mock_tool_ctx_t *)ctx;
    if (m->pin != GPIO_NUM_NC) {
        esp_err_t err = bsp_pin_claim_ex(m->pin, m->id, m->require_output);
        if (err != ESP_OK) {
            return err;
        }
    }
    if (parent_cont != NULL) {
        lv_obj_t *lbl = lv_label_create(parent_cont);
        lv_label_set_text(lbl, m->id);
    }
    m->bg_active = false;
    return ESP_OK;
}

static esp_err_t ui_mock_stop(bool keep_background, void *ctx)
{
    ui_mock_tool_ctx_t *m = (ui_mock_tool_ctx_t *)ctx;
    m->bg_active = keep_background;
    if (!keep_background && m->pin != GPIO_NUM_NC) {
        (void)bsp_pin_release(m->pin, m->id);
    }
    return ESP_OK;
}

static void ui_mock_refresh(void *ctx)
{
    ui_mock_tool_ctx_t *m = (ui_mock_tool_ctx_t *)ctx;
    m->ui_refresh_count++;
}

static bool ui_mock_is_bg(void *ctx)
{
    ui_mock_tool_ctx_t *m = (ui_mock_tool_ctx_t *)ctx;
    return m->bg_active;
}

static esp_err_t mock_ui_i2s_init(uint32_t rate)
{
    (void)rate;
    return ESP_OK;
}

static esp_err_t mock_ui_i2s_write(const int16_t *pcm, size_t len, size_t *out_written, uint32_t timeout_ms)
{
    (void)pcm;
    (void)timeout_ms;
    if (out_written != NULL) {
        *out_written = len;
    }
    return ESP_OK;
}

static esp_err_t mock_ui_i2s_disable(void)
{
    return ESP_OK;
}

static const bsp_spk_i2s_ops_t s_mock_ui_i2s_ops = {
    .ensure_init = mock_ui_i2s_init,
    .write_pcm = mock_ui_i2s_write,
    .disable = mock_ui_i2s_disable,
};

TEST_CASE("given battery background speaker and recorder states when formatted then header strings match specification", "[ui_shell][header]")
{
    char buf[32] = {0};
    char mode_buf[16] = {0};

    /* Battery formatting */
    ui_header_format_battery(3850, 68, false, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("3.85V 68%", buf);

    ui_header_format_battery(4150, 100, true, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("4.15V 100%+", buf);

    /* Background generator pill formatting */
    ui_header_format_bg_pill(0, NULL, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("", buf);

    ui_header_format_bg_pill(1, "wavegen", buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("BG:wavegen", buf);

    ui_header_format_bg_pill(2, "wavegen", buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("BG GEN(2)", buf);

    /* Speaker monitor badge formatting */
    ui_header_format_spk_badge(false, false, false, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("SPK N/A", buf);

    ui_header_format_spk_badge(true, false, false, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("SPK OFF", buf);

    ui_header_format_spk_badge(true, true, false, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("SPK ON", buf);

    ui_header_format_spk_badge(true, true, true, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("SPK MUTE", buf);

    /* Recorder badge & transport formatting */
    ui_header_format_rec_badge(REC_MODE_OFF, REC_MODE_USB_STUB, 0, buf, sizeof(buf), mode_buf, sizeof(mode_buf));
    TEST_ASSERT_EQUAL_STRING("REC OFF", buf);
    TEST_ASSERT_EQUAL_STRING("USB", mode_buf);

    ui_header_format_rec_badge(REC_MODE_USB_STUB, REC_MODE_USB_STUB, 12, buf, sizeof(buf), mode_buf, sizeof(mode_buf));
    TEST_ASSERT_EQUAL_STRING("REC USB #12", buf);
    TEST_ASSERT_EQUAL_STRING("USB", mode_buf);

    ui_header_format_rec_badge(REC_MODE_BT_STUB, REC_MODE_BT_STUB, 5, buf, sizeof(buf), mode_buf, sizeof(mode_buf));
    TEST_ASSERT_EQUAL_STRING("REC BT #5", buf);
    TEST_ASSERT_EQUAL_STRING("BT", mode_buf);
}

TEST_CASE("given ui_shell and registered tools when navigating and toggling controls then header state and LVGL views update", "[ui_shell][navigation]")
{
    tool_registry_test_reset();
    recorder_sink_test_reset();
    bsp_pin_manager_reset();
    bsp_spk_passthrough_set_i2s_ops(&s_mock_ui_i2s_ops);

    TEST_ASSERT_EQUAL(ESP_OK, tool_core_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());

    ui_mock_tool_ctx_t wave_ctx  = {.id = "wavegen", .pin = BSP_PIN_PORT_C_TXD2, .require_output = true};
    ui_mock_tool_ctx_t scope_ctx = {.id = "scope", .pin = BSP_PIN_ANALOG_IN_DEFAULT, .require_output = false};
    ui_mock_tool_ctx_t logic_ctx = {.id = "logic", .pin = BSP_PIN_PORT_C_TXD2, .require_output = false};

    const analyser_tool_t wave_tool = {
        .id = "wavegen",
        .title = "Wave Generator",
        .symbol = LV_SYMBOL_REFRESH,
        .category = TOOL_CATEGORY_GENERATOR,
        .supports_background = true,
        .supports_recording = false,
        .supports_spk_passthrough = false,
        .on_start = ui_mock_start,
        .on_stop = ui_mock_stop,
        .is_running_bg = ui_mock_is_bg,
        .user_ctx = &wave_ctx,
    };

    const analyser_tool_t scope_tool = {
        .id = "scope",
        .title = "Oscilloscope",
        .symbol = LV_SYMBOL_IMAGE,
        .category = TOOL_CATEGORY_ANALOG_IN,
        .supports_background = false,
        .supports_recording = true,
        .supports_spk_passthrough = true,
        .on_start = ui_mock_start,
        .on_stop = ui_mock_stop,
        .on_ui_refresh = ui_mock_refresh,
        .user_ctx = &scope_ctx,
    };

    const analyser_tool_t logic_tool = {
        .id = "logic",
        .title = "Logic Analyzer",
        .symbol = LV_SYMBOL_LIST,
        .category = TOOL_CATEGORY_DIGITAL_IN,
        .supports_background = false,
        .on_start = ui_mock_start,
        .on_stop = ui_mock_stop,
        .user_ctx = &logic_ctx,
    };

    TEST_ASSERT_EQUAL(ESP_OK, tool_registry_register(&wave_tool));
    TEST_ASSERT_EQUAL(ESP_OK, tool_registry_register(&scope_tool));
    TEST_ASSERT_EQUAL(ESP_OK, tool_registry_register(&logic_tool));

    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_init());
    TEST_ASSERT_TRUE(ui_shell_is_initialized());
    TEST_ASSERT_NOT_NULL(ui_shell_get_tool_container());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_rebuild_launcher());

    ui_header_state_t hstate = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_get_header_state(&hstate));
    TEST_ASSERT_FALSE(hstate.home_enabled);
    TEST_ASSERT_EQUAL_STRING("M5CoreAnalyser", hstate.title);
    TEST_ASSERT_FALSE(hstate.bg_pill_visible);
    TEST_ASSERT_EQUAL_STRING("SPK N/A", hstate.spk_badge_text);
    TEST_ASSERT_EQUAL_STRING("REC OFF", hstate.rec_badge_text);

    /* Toggling speaker while no speaker-capable tool is open returns ESP_ERR_NOT_SUPPORTED */
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, ui_shell_toggle_speaker());

    /* Open Wave Generator and return Home with keep_background = true */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool("wavegen"));
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_get_header_state(&hstate));
    TEST_ASSERT_TRUE(hstate.home_enabled);
    TEST_ASSERT_EQUAL_STRING("Wave Generator", hstate.title);

    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(true));
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_get_header_state(&hstate));
    TEST_ASSERT_FALSE(hstate.home_enabled);
    TEST_ASSERT_TRUE(hstate.bg_pill_visible);
    TEST_ASSERT_EQUAL_UINT32(1, (uint32_t)hstate.bg_tool_count);
    TEST_ASSERT_EQUAL_STRING("BG:wavegen", hstate.bg_pill_text);

    /* Open Oscilloscope while Wave Generator runs in background */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool("scope"));
    ui_shell_step();
    TEST_ASSERT_GREATER_THAN(0, scope_ctx.ui_refresh_count);

    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_get_header_state(&hstate));
    TEST_ASSERT_TRUE(hstate.home_enabled);
    TEST_ASSERT_EQUAL_STRING("Oscilloscope", hstate.title);
    TEST_ASSERT_TRUE(hstate.bg_pill_visible);
    TEST_ASSERT_TRUE(hstate.spk_supported);
    TEST_ASSERT_EQUAL_STRING("SPK OFF", hstate.spk_badge_text);

    /* Toggle speaker ON and then MUTE */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_toggle_speaker());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_get_header_state(&hstate));
    TEST_ASSERT_TRUE(hstate.spk_enabled);
    TEST_ASSERT_EQUAL_STRING("SPK ON", hstate.spk_badge_text);

    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_toggle_speaker());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_get_header_state(&hstate));
    TEST_ASSERT_FALSE(hstate.spk_enabled);
    TEST_ASSERT_TRUE(hstate.spk_muted);
    TEST_ASSERT_EQUAL_STRING("SPK MUTE", hstate.spk_badge_text);

    /* Toggle recorder and cycle transport USB -> BT */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_toggle_recorder());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_cycle_recorder_transport());
    uint16_t dummy_samples[10] = {1024};
    recorder_frame_t frame = {
        .tool_id = "scope",
        .sample_rate_hz = 16000,
        .timestamp_us = 500ULL,
        .samples = dummy_samples,
        .sample_count = 10,
        .bytes_per_sample = sizeof(uint16_t),
    };
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_push(&frame));

    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_get_header_state(&hstate));
    TEST_ASSERT_EQUAL(REC_MODE_BT_STUB, hstate.rec_mode);
    TEST_ASSERT_EQUAL_UINT32(1, hstate.rec_frames);
    TEST_ASSERT_EQUAL_STRING("REC BT #1", hstate.rec_badge_text);
    TEST_ASSERT_EQUAL_STRING("BT", hstate.transport_badge_text);

    /* Attempt to open Logic Analyzer on conflicting GPIO14 -> fails with clear status message */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(false));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ui_shell_open_tool("logic"));
    char status_msg[UI_SHELL_STATUS_MSG_MAX] = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_get_last_status_msg(status_msg, sizeof(status_msg)));
    TEST_ASSERT_NOT_NULL(strstr(status_msg, "Pin conflict"));

    /* Cleanup */
    (void)tool_core_stop_all_background_tools();
    (void)bsp_spk_passthrough_enable(false, 0);
    bsp_spk_passthrough_set_i2s_ops(NULL);
}
