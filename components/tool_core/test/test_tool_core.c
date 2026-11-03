/**
 * @file test_tool_core.c
 * @brief Unity unit tests for tool_core registry, foreground/background generator concurrency,
 *        PSRAM ping-pong double buffering, Core 1 worker tasks, and self-test runner.
 */

#include <stdint.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "bsp_m5core2_priv.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tool_core.h"
#include "tool_core_priv.h"
#include "unity.h"

typedef struct {
    const char *id;
    gpio_num_t pin;
    bool require_output;
    int init_calls;
    int start_calls;
    int stop_calls;
    int refresh_calls;
    bool last_keep_bg;
    bool bg_active;
    esp_err_t force_init_err;
    esp_err_t force_start_err;
} mock_tool_ctx_t;

static esp_err_t mock_spk_i2s_init(uint32_t rate)
{
    (void)rate;
    return ESP_OK;
}

static esp_err_t mock_spk_i2s_write(const int16_t *pcm, size_t len, size_t *out_written, uint32_t timeout_ms)
{
    (void)pcm;
    (void)timeout_ms;
    if (out_written != NULL) {
        *out_written = len;
    }
    return ESP_OK;
}

static esp_err_t mock_spk_i2s_disable(void)
{
    return ESP_OK;
}

static const bsp_spk_i2s_ops_t s_mock_spk_i2s_ops = {
    .ensure_init = mock_spk_i2s_init,
    .write_pcm = mock_spk_i2s_write,
    .disable = mock_spk_i2s_disable,
};

static esp_err_t mock_tool_on_init(void *ctx)
{
    mock_tool_ctx_t *m = (mock_tool_ctx_t *)ctx;
    m->init_calls++;
    return m->force_init_err;
}

static esp_err_t mock_tool_on_start(lv_obj_t *parent_cont, void *ctx)
{
    (void)parent_cont;
    mock_tool_ctx_t *m = (mock_tool_ctx_t *)ctx;
    m->start_calls++;
    if (m->force_start_err != ESP_OK) {
        return m->force_start_err;
    }
    if (m->pin != GPIO_NUM_NC) {
        esp_err_t err = bsp_pin_claim_ex(m->pin, m->id, m->require_output);
        if (err != ESP_OK) {
            return err;
        }
    }
    m->bg_active = false;
    return ESP_OK;
}

static esp_err_t mock_tool_on_stop(bool keep_background, void *ctx)
{
    mock_tool_ctx_t *m = (mock_tool_ctx_t *)ctx;
    m->stop_calls++;
    m->last_keep_bg = keep_background;
    m->bg_active = keep_background;
    if (!keep_background && m->pin != GPIO_NUM_NC) {
        (void)bsp_pin_release(m->pin, m->id);
    }
    return ESP_OK;
}

static void mock_tool_on_refresh(void *ctx)
{
    mock_tool_ctx_t *m = (mock_tool_ctx_t *)ctx;
    m->refresh_calls++;
}

static bool mock_tool_is_running_bg(void *ctx)
{
    mock_tool_ctx_t *m = (mock_tool_ctx_t *)ctx;
    return m->bg_active;
}

TEST_CASE("given tool descriptors when registered then duplicates and invalid fields rejected and categories filter accurately", "[tool_core][registry]")
{
    tool_registry_test_reset();
    bsp_pin_manager_reset();
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_init());

    mock_tool_ctx_t scope_ctx = {.id = "scope", .pin = BSP_PIN_ANALOG_IN_DEFAULT};
    mock_tool_ctx_t logic_ctx = {.id = "logic", .pin = BSP_PIN_PORT_C_RXD2};
    mock_tool_ctx_t wave_ctx  = {.id = "wavegen", .pin = BSP_PIN_PORT_C_TXD2, .require_output = true};
    mock_tool_ctx_t bad_ctx   = {.id = "bad", .force_init_err = ESP_ERR_INVALID_STATE};

    const analyser_tool_t scope_tool = {
        .id = "scope",
        .title = "Oscilloscope",
        .symbol = LV_SYMBOL_IMAGE,
        .category = TOOL_CATEGORY_ANALOG_IN,
        .supports_background = false,
        .supports_recording = true,
        .supports_spk_passthrough = true,
        .on_init = mock_tool_on_init,
        .on_start = mock_tool_on_start,
        .on_stop = mock_tool_on_stop,
        .on_ui_refresh = mock_tool_on_refresh,
        .user_ctx = &scope_ctx,
    };

    const analyser_tool_t logic_tool = {
        .id = "logic",
        .title = "Logic Analyzer",
        .symbol = LV_SYMBOL_LIST,
        .category = TOOL_CATEGORY_DIGITAL_IN,
        .supports_background = false,
        .supports_recording = true,
        .supports_spk_passthrough = false,
        .on_init = mock_tool_on_init,
        .on_start = mock_tool_on_start,
        .on_stop = mock_tool_on_stop,
        .user_ctx = &logic_ctx,
    };

    const analyser_tool_t wave_tool = {
        .id = "wavegen",
        .title = "Wave Generator",
        .symbol = LV_SYMBOL_REFRESH,
        .category = TOOL_CATEGORY_GENERATOR,
        .supports_background = true,
        .supports_recording = false,
        .supports_spk_passthrough = false,
        .on_init = mock_tool_on_init,
        .on_start = mock_tool_on_start,
        .on_stop = mock_tool_on_stop,
        .is_running_bg = mock_tool_is_running_bg,
        .user_ctx = &wave_ctx,
    };

    const analyser_tool_t failing_init_tool = {
        .id = "bad_init",
        .title = "Failing Init",
        .symbol = LV_SYMBOL_WARNING,
        .category = TOOL_CATEGORY_ANALOG_IN,
        .on_init = mock_tool_on_init,
        .on_start = mock_tool_on_start,
        .on_stop = mock_tool_on_stop,
        .user_ctx = &bad_ctx,
    };

    /* Invalid descriptor fields rejected */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, tool_registry_register(NULL));
    analyser_tool_t invalid_tool = scope_tool;
    invalid_tool.id = "";
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, tool_registry_register(&invalid_tool));
    invalid_tool.id = "valid";
    invalid_tool.on_start = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, tool_registry_register(&invalid_tool));

    /* Failing on_init prevents registration */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, tool_registry_register(&failing_init_tool));
    TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)tool_registry_get_all(NULL, 0));

    /* Register valid tools */
    TEST_ASSERT_EQUAL(ESP_OK, tool_registry_register(&scope_tool));
    TEST_ASSERT_EQUAL(ESP_OK, tool_registry_register(&logic_tool));
    TEST_ASSERT_EQUAL(ESP_OK, tool_registry_register(&wave_tool));
    TEST_ASSERT_EQUAL(1, scope_ctx.init_calls);
    TEST_ASSERT_EQUAL(1, logic_ctx.init_calls);
    TEST_ASSERT_EQUAL(1, wave_ctx.init_calls);

    /* Duplicate ID is rejected */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, tool_registry_register(&scope_tool));

    /* Verify lookup and category filtering */
    const analyser_tool_t *all[4] = {0};
    TEST_ASSERT_EQUAL_UINT32(3, (uint32_t)tool_registry_get_all(all, 4));
    TEST_ASSERT_EQUAL_PTR(&scope_tool, all[0]);
    TEST_ASSERT_EQUAL_PTR(&logic_tool, all[1]);
    TEST_ASSERT_EQUAL_PTR(&wave_tool, all[2]);

    const analyser_tool_t *gen_tools[2] = {0};
    TEST_ASSERT_EQUAL_UINT32(1, (uint32_t)tool_registry_get_by_category(TOOL_CATEGORY_GENERATOR, gen_tools, 2));
    TEST_ASSERT_EQUAL_PTR(&wave_tool, gen_tools[0]);
    TEST_ASSERT_EQUAL_PTR(&scope_tool, tool_registry_find("scope"));
    TEST_ASSERT_NULL(tool_registry_find("nonexistent"));
    TEST_ASSERT_EQUAL_STRING("Analog In", tool_category_to_str(TOOL_CATEGORY_ANALOG_IN));
    TEST_ASSERT_EQUAL_STRING("Digital In", tool_category_to_str(TOOL_CATEGORY_DIGITAL_IN));
    TEST_ASSERT_EQUAL_STRING("Generator", tool_category_to_str(TOOL_CATEGORY_GENERATOR));
}

TEST_CASE("given background generator and foreground tools when switched then concurrency pin ownership and speaker gating transition safely", "[tool_core][lifecycle]")
{
    tool_registry_test_reset();
    bsp_pin_manager_reset();
    bsp_spk_passthrough_set_i2s_ops(&s_mock_spk_i2s_ops);

    mock_tool_ctx_t wave_ctx   = {.id = "wavegen", .pin = BSP_PIN_PORT_C_TXD2, .require_output = true};
    mock_tool_ctx_t analog_ctx = {.id = "analoggen", .pin = BSP_PIN_DAC_OUT_CH1, .require_output = true};
    mock_tool_ctx_t scope_ctx  = {.id = "scope", .pin = BSP_PIN_ANALOG_IN_DEFAULT, .require_output = false};
    mock_tool_ctx_t logic_ctx  = {.id = "logic", .pin = BSP_PIN_PORT_C_TXD2, .require_output = false};

    const analyser_tool_t wave_tool = {
        .id = "wavegen",
        .title = "Wave Generator",
        .symbol = LV_SYMBOL_REFRESH,
        .category = TOOL_CATEGORY_GENERATOR,
        .supports_background = true,
        .supports_spk_passthrough = false,
        .on_start = mock_tool_on_start,
        .on_stop = mock_tool_on_stop,
        .is_running_bg = mock_tool_is_running_bg,
        .user_ctx = &wave_ctx,
    };

    const analyser_tool_t analog_tool = {
        .id = "analoggen",
        .title = "Analog Generator",
        .symbol = LV_SYMBOL_AUDIO,
        .category = TOOL_CATEGORY_GENERATOR,
        .supports_background = true,
        .supports_spk_passthrough = true,
        .on_start = mock_tool_on_start,
        .on_stop = mock_tool_on_stop,
        .is_running_bg = mock_tool_is_running_bg,
        .user_ctx = &analog_ctx,
    };

    const analyser_tool_t scope_tool = {
        .id = "scope",
        .title = "Oscilloscope",
        .symbol = LV_SYMBOL_IMAGE,
        .category = TOOL_CATEGORY_ANALOG_IN,
        .supports_background = false,
        .supports_recording = true,
        .supports_spk_passthrough = true,
        .on_start = mock_tool_on_start,
        .on_stop = mock_tool_on_stop,
        .on_ui_refresh = mock_tool_on_refresh,
        .user_ctx = &scope_ctx,
    };

    const analyser_tool_t logic_tool_conflicting = {
        .id = "logic",
        .title = "Logic Analyzer",
        .symbol = LV_SYMBOL_LIST,
        .category = TOOL_CATEGORY_DIGITAL_IN,
        .supports_background = false,
        .on_start = mock_tool_on_start,
        .on_stop = mock_tool_on_stop,
        .user_ctx = &logic_ctx,
    };

    TEST_ASSERT_EQUAL(ESP_OK, tool_registry_register(&wave_tool));
    TEST_ASSERT_EQUAL(ESP_OK, tool_registry_register(&analog_tool));
    TEST_ASSERT_EQUAL(ESP_OK, tool_registry_register(&scope_tool));
    TEST_ASSERT_EQUAL(ESP_OK, tool_registry_register(&logic_tool_conflicting));

    /* 1. Start wavegen in foreground on GPIO14 */
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_start_tool("wavegen", NULL));
    TEST_ASSERT_EQUAL_PTR(&wave_tool, tool_core_get_active_tool());
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_TXD2, "wavegen"));
    TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)tool_core_get_background_count());

    /* 2. Stop wavegen with keep_background=true -> moves to background and keeps GPIO14 */
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_stop_active_tool(true));
    TEST_ASSERT_NULL(tool_core_get_active_tool());
    TEST_ASSERT_TRUE(tool_core_is_tool_running_bg("wavegen"));
    TEST_ASSERT_EQUAL_UINT32(1, (uint32_t)tool_core_get_background_count());
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_TXD2, "wavegen"));

    /* 3. Open scope on GPIO36 while wavegen runs in background on GPIO14; enable speaker */
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_start_tool("scope", NULL));
    TEST_ASSERT_EQUAL_PTR(&scope_tool, tool_core_get_active_tool());
    TEST_ASSERT_TRUE(tool_core_is_tool_running_bg("wavegen"));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_TXD2, "wavegen"));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_DEFAULT, "scope"));

    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_enable(true, BSP_SPK_DEFAULT_RATE_HZ));
    TEST_ASSERT_TRUE(bsp_spk_passthrough_is_enabled());

    tool_core_ui_refresh_active();
    TEST_ASSERT_EQUAL(1, scope_ctx.refresh_calls);

    /* 4. Stopping scope (when only non-speaker wavegen is in BG) automatically disables speaker */
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_stop_active_tool(true));
    TEST_ASSERT_FALSE(tool_core_is_tool_running_bg("scope"));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_DEFAULT, "scope"));
    TEST_ASSERT_TRUE(tool_core_is_tool_running_bg("wavegen"));
    TEST_ASSERT_FALSE(bsp_spk_passthrough_is_enabled());

    /* 5. Start analoggen (supports_background=true, supports_spk_passthrough=true) and enable speaker */
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_start_tool("analoggen", NULL));
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_enable(true, BSP_SPK_DEFAULT_RATE_HZ));
    TEST_ASSERT_TRUE(bsp_spk_passthrough_is_enabled());

    /* Move analoggen to background -> speaker stays enabled */
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_stop_active_tool(true));
    TEST_ASSERT_TRUE(tool_core_is_tool_running_bg("analoggen"));
    TEST_ASSERT_TRUE(bsp_spk_passthrough_is_enabled());

    /* Open and exit scope while analoggen is in background -> speaker remains enabled for analoggen */
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_start_tool("scope", NULL));
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_stop_active_tool(false));
    TEST_ASSERT_TRUE(bsp_spk_passthrough_is_enabled());

    /* Stopping background analoggen disables speaker passthrough */
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_stop_background_tool("analoggen"));
    TEST_ASSERT_FALSE(tool_core_is_tool_running_bg("analoggen"));
    TEST_ASSERT_FALSE(bsp_spk_passthrough_is_enabled());

    /* Also verify stop_all_background_tools gates off speaker when analoggen is in background */
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_start_tool("analoggen", NULL));
    TEST_ASSERT_EQUAL(ESP_OK, bsp_spk_passthrough_enable(true, BSP_SPK_DEFAULT_RATE_HZ));
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_stop_active_tool(true));
    TEST_ASSERT_TRUE(bsp_spk_passthrough_is_enabled());
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_stop_all_background_tools());
    TEST_ASSERT_FALSE(bsp_spk_passthrough_is_enabled());

    /* Restart wavegen in background to verify pin conflict protection */
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_start_tool("wavegen", NULL));
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_stop_active_tool(true));

    /* 6. Starting logic analyzer on conflicting pin GPIO14 fails without stealing wavegen's pin */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, tool_core_start_tool("logic", NULL));
    TEST_ASSERT_NULL(tool_core_get_active_tool());
    TEST_ASSERT_TRUE(tool_core_is_tool_running_bg("wavegen"));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_TXD2, "wavegen"));

    /* 7. Re-opening wavegen promotes it from background back to foreground */
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_start_tool("wavegen", NULL));
    TEST_ASSERT_EQUAL_PTR(&wave_tool, tool_core_get_active_tool());
    TEST_ASSERT_FALSE(tool_core_is_tool_running_bg("wavegen"));
    TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)tool_core_get_background_count());

    /* 8. Stopping wavegen in background releases GPIO14 so logic analyzer can now start */
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_stop_active_tool(true));
    TEST_ASSERT_TRUE(tool_core_is_tool_running_bg("wavegen"));
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_stop_background_tool("wavegen"));
    TEST_ASSERT_FALSE(tool_core_is_tool_running_bg("wavegen"));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_TXD2, "wavegen"));

    TEST_ASSERT_EQUAL(ESP_OK, tool_core_start_tool("logic", NULL));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_TXD2, "logic"));
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_stop_active_tool(false));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_TXD2, "logic"));

    bsp_spk_passthrough_set_i2s_ops(NULL);
}

TEST_CASE("given psram_double_buf when allocated swapped acquired and copied then reader slot is tear-free and OOM rolls back", "[tool_core][psram]")
{
    psram_buf_test_set_fail_after(-1);

    psram_double_buf_t dbuf = {0};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, psram_double_buf_alloc(NULL, 64, 2));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, psram_double_buf_alloc(&dbuf, 0, 2));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, psram_double_buf_alloc(&dbuf, 64, 0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, psram_double_buf_alloc(&dbuf, (SIZE_MAX / 2U) + 2U, 4));

    /* Inject OOM on 1st, 2nd, and 3rd slot allocations to verify clean rollback */
    psram_buf_test_set_fail_after(0);
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, psram_double_buf_alloc(&dbuf, 128, sizeof(uint16_t)));
    TEST_ASSERT_NULL(dbuf.buf_a);
    TEST_ASSERT_NULL(dbuf.buf_b);
    TEST_ASSERT_NULL(dbuf.buf_c);

    psram_buf_test_set_fail_after(1);
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, psram_double_buf_alloc(&dbuf, 128, sizeof(uint16_t)));
    TEST_ASSERT_NULL(dbuf.buf_a);
    TEST_ASSERT_NULL(dbuf.buf_b);
    TEST_ASSERT_NULL(dbuf.buf_c);

    psram_buf_test_set_fail_after(2);
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, psram_double_buf_alloc(&dbuf, 128, sizeof(uint16_t)));
    TEST_ASSERT_NULL(dbuf.buf_a);
    TEST_ASSERT_NULL(dbuf.buf_b);
    TEST_ASSERT_NULL(dbuf.buf_c);
    psram_buf_test_set_fail_after(-1);

    /* Allocate 300-sample 3-slot buffer */
    TEST_ASSERT_EQUAL(ESP_OK, psram_double_buf_alloc(&dbuf, 300, sizeof(uint16_t)));
    TEST_ASSERT_NOT_NULL(dbuf.buf_a);
    TEST_ASSERT_NOT_NULL(dbuf.buf_b);
    TEST_ASSERT_NOT_NULL(dbuf.buf_c);
    TEST_ASSERT_NOT_EQUAL(dbuf.buf_a, dbuf.buf_b);
    TEST_ASSERT_NOT_EQUAL(dbuf.buf_b, dbuf.buf_c);
    TEST_ASSERT_NOT_EQUAL(dbuf.buf_a, dbuf.buf_c);
    TEST_ASSERT_EQUAL_UINT8(0U, dbuf.active_write_idx);
    TEST_ASSERT_EQUAL_UINT8(1U, dbuf.ready_idx);
    TEST_ASSERT_EQUAL_UINT8(2U, dbuf.active_read_idx);
    TEST_ASSERT_FALSE(dbuf.frame_ready);

    /* Initially write_ptr is buf_a (0) and read_ptr is buf_c (2) */
    uint16_t *w0 = (uint16_t *)psram_double_buf_get_write_ptr(&dbuf);
    const uint16_t *r0 = (const uint16_t *)psram_double_buf_get_read_ptr(&dbuf);
    TEST_ASSERT_EQUAL_PTR(dbuf.buf_a, w0);
    TEST_ASSERT_EQUAL_PTR(dbuf.buf_c, r0);

    for (size_t i = 0; i < 300; ++i) {
        w0[i] = (uint16_t)(1000U + i);
    }

    const void *ready_ptr = NULL;
    TEST_ASSERT_FALSE(psram_double_buf_consume_ready(&dbuf, &ready_ptr));
    TEST_ASSERT_FALSE(psram_double_buf_acquire_read(&dbuf, &ready_ptr));

    /* Swap 1: write_idx becomes 1 (buf_b), ready_idx becomes 0 (buf_a) */
    TEST_ASSERT_EQUAL(ESP_OK, psram_double_buf_swap(&dbuf));
    TEST_ASSERT_EQUAL_UINT8(1U, dbuf.active_write_idx);
    TEST_ASSERT_EQUAL_UINT8(0U, dbuf.ready_idx);
    TEST_ASSERT_TRUE(dbuf.frame_ready);
    TEST_ASSERT_EQUAL_PTR(dbuf.buf_b, psram_double_buf_get_write_ptr(&dbuf));

    /* Consumer latches ready_idx (buf_a) into active_read_idx */
    TEST_ASSERT_TRUE(psram_double_buf_consume_ready(&dbuf, &ready_ptr));
    TEST_ASSERT_EQUAL_PTR(dbuf.buf_a, ready_ptr);
    TEST_ASSERT_EQUAL_PTR(dbuf.buf_a, psram_double_buf_get_read_ptr(&dbuf));
    TEST_ASSERT_FALSE(dbuf.frame_ready);

    /* Verify tear-free guarantee: even if producer writes & swaps multiple times while
     * consumer is reading ready_ptr (buf_a), producer ONLY alternates between buf_b and buf_c! */
    for (uint16_t round = 0; round < 5; ++round) {
        uint16_t *wn = (uint16_t *)psram_double_buf_get_write_ptr(&dbuf);
        TEST_ASSERT_NOT_EQUAL(ready_ptr, wn);
        for (size_t i = 0; i < 300; ++i) {
            wn[i] = (uint16_t)(9000U + round);
        }
        TEST_ASSERT_EQUAL(ESP_OK, psram_double_buf_swap(&dbuf));
    }
    const uint16_t *latched_samples = (const uint16_t *)ready_ptr;
    TEST_ASSERT_EQUAL_UINT16(1000U, latched_samples[0]);
    TEST_ASSERT_EQUAL_UINT16(1299U, latched_samples[299]);

    /* Verify explicit acquire_read / release_read and reader_busy protection */
    const void *acq_ptr = NULL;
    TEST_ASSERT_TRUE(psram_double_buf_acquire_read(&dbuf, &acq_ptr));
    TEST_ASSERT_NOT_NULL(acq_ptr);
    TEST_ASSERT_TRUE(dbuf.reader_busy);
    /* Second acquire or read_copy while reader_busy is rejected */
    const void *acq_second = NULL;
    uint16_t copy_dst[300] = {0};
    TEST_ASSERT_FALSE(psram_double_buf_acquire_read(&dbuf, &acq_second));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, psram_double_buf_read_copy(&dbuf, copy_dst, 300, NULL));
    psram_double_buf_release_read(&dbuf);
    TEST_ASSERT_FALSE(dbuf.reader_busy);

    /* Write new frame, swap, and copy out via psram_double_buf_read_copy (outside spinlock) */
    uint16_t *w_final = (uint16_t *)psram_double_buf_get_write_ptr(&dbuf);
    for (size_t i = 0; i < 300; ++i) {
        w_final[i] = (uint16_t)(2000U + i);
    }
    TEST_ASSERT_EQUAL(ESP_OK, psram_double_buf_swap(&dbuf));

    size_t copied = 0;
    TEST_ASSERT_EQUAL(ESP_OK, psram_double_buf_read_copy(&dbuf, copy_dst, 300, &copied));
    TEST_ASSERT_EQUAL_UINT32(300, (uint32_t)copied);
    TEST_ASSERT_EQUAL_UINT16(2000, copy_dst[0]);
    TEST_ASSERT_EQUAL_UINT16(2299, copy_dst[299]);
    TEST_ASSERT_FALSE(dbuf.frame_ready);

    psram_double_buf_free(&dbuf);
    TEST_ASSERT_NULL(dbuf.buf_a);
    TEST_ASSERT_NULL(dbuf.buf_b);
    TEST_ASSERT_NULL(dbuf.buf_c);
}

#define CONCURRENT_FRAME_SAMPLES (256U)

typedef struct {
    psram_double_buf_t *dbuf;
    volatile bool stop;
    volatile uint32_t frames_written;
    SemaphoreHandle_t done_sem;
} concurrent_producer_ctx_t;

static void concurrent_producer_task(void *arg)
{
    concurrent_producer_ctx_t *ctx = (concurrent_producer_ctx_t *)arg;
    uint32_t seq = 1U;
    while (!ctx->stop) {
        uint32_t *wptr = (uint32_t *)psram_double_buf_get_write_ptr(ctx->dbuf);
        if (wptr != NULL) {
            for (size_t i = 0; i < CONCURRENT_FRAME_SAMPLES; ++i) {
                wptr[i] = (seq << 12) | (uint32_t)i;
            }
            (void)psram_double_buf_swap(ctx->dbuf);
            ctx->frames_written = seq;
            seq++;
        }
        if ((seq & 0x1FU) == 0U) {
            taskYIELD();
        }
    }
    (void)xSemaphoreGive(ctx->done_sem);
    vTaskDelete(NULL);
}

TEST_CASE("given concurrent Core 1 producer and Core 0 consumer when hammering swaps and reads then zero torn frames occur", "[tool_core][psram][concurrency]")
{
    psram_double_buf_t dbuf = {0};
    TEST_ASSERT_EQUAL(ESP_OK, psram_double_buf_alloc(&dbuf, CONCURRENT_FRAME_SAMPLES, sizeof(uint32_t)));

    StaticSemaphore_t sem_buf;
    concurrent_producer_ctx_t pctx = {
        .dbuf = &dbuf,
        .stop = false,
        .frames_written = 0,
        .done_sem = xSemaphoreCreateBinaryStatic(&sem_buf),
    };

    BaseType_t prod_core = (portNUM_PROCESSORS > 1) ? 1 : tskNO_AFFINITY;
    TaskHandle_t prod_task = NULL;
    TEST_ASSERT_EQUAL(pdPASS,
                      xTaskCreatePinnedToCore(
                          concurrent_producer_task, "dbuf_prod_c1", 4096, &pctx, 5, &prod_task, prod_core));

    uint32_t copy_buf[CONCURRENT_FRAME_SAMPLES] = {0};
    uint32_t last_seq = 0;
    uint32_t verified_frames = 0;
    uint32_t attempts = 0;

    while (verified_frames < 120U && attempts < 20000U) {
        attempts++;
        if ((verified_frames & 1U) == 0U) {
            const void *rptr = NULL;
            if (psram_double_buf_acquire_read(&dbuf, &rptr)) {
                const uint32_t *samples = (const uint32_t *)rptr;
                uint32_t seq = samples[0] >> 12;
                TEST_ASSERT_GREATER_OR_EQUAL_UINT32(last_seq, seq);
                for (size_t i = 0; i < CONCURRENT_FRAME_SAMPLES; ++i) {
                    TEST_ASSERT_EQUAL_UINT32((seq << 12) | (uint32_t)i, samples[i]);
                }
                last_seq = seq;
                verified_frames++;
                psram_double_buf_release_read(&dbuf);
            }
        } else {
            if (dbuf.frame_ready) {
                size_t copied = 0;
                TEST_ASSERT_EQUAL(ESP_OK,
                                  psram_double_buf_read_copy(&dbuf, copy_buf, CONCURRENT_FRAME_SAMPLES, &copied));
                TEST_ASSERT_EQUAL_UINT32(CONCURRENT_FRAME_SAMPLES, (uint32_t)copied);
                uint32_t seq = copy_buf[0] >> 12;
                TEST_ASSERT_GREATER_OR_EQUAL_UINT32(last_seq, seq);
                for (size_t i = 0; i < CONCURRENT_FRAME_SAMPLES; ++i) {
                    TEST_ASSERT_EQUAL_UINT32((seq << 12) | (uint32_t)i, copy_buf[i]);
                }
                last_seq = seq;
                verified_frames++;
            }
        }
        if ((attempts & 0x0FU) == 0U) {
            taskYIELD();
        }
    }

    pctx.stop = true;
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(pctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(120U, verified_frames);
    TEST_ASSERT_GREATER_THAN_UINT32(0U, pctx.frames_written);

    psram_double_buf_free(&dbuf);
}

typedef struct {
    volatile uint32_t iterations;
    BaseType_t observed_core;
    esp_err_t return_err;
    uint32_t sleep_ms;
} worker_test_ctx_t;

static esp_err_t test_worker_step(void *user_ctx)
{
    worker_test_ctx_t *ctx = (worker_test_ctx_t *)user_ctx;
    ctx->observed_core = xPortGetCoreID();
    ctx->iterations++;
    if (ctx->sleep_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(ctx->sleep_ms));
    }
    return ctx->return_err;
}

static esp_err_t sample_selftest_ok(void)
{
    return ESP_OK;
}

TEST_CASE("given tool_worker and selftest runner when started stopped or failing then worker handles Core 1 errors and timeouts cleanly", "[tool_core][worker]")
{
    tool_registry_test_reset();

    worker_test_ctx_t wctx = {
        .iterations = 0,
        .observed_core = -1,
        .return_err = ESP_OK,
        .sleep_ms = 0,
    };

    tool_worker_config_t cfg = {
        .name = "test_c1_worker",
        .stack_size_bytes = 4096,
        .priority = 5,
        .core_id = TOOL_WORKER_DEFAULT_CORE,
        .period_ms = 2,
        .step_fn = test_worker_step,
        .user_ctx = &wctx,
    };

    tool_worker_t *worker = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, tool_worker_start(&worker, &cfg));
    TEST_ASSERT_NOT_NULL(worker);
    TEST_ASSERT_TRUE(tool_worker_is_running(worker));

    vTaskDelay(pdMS_TO_TICKS(20));
    TEST_ASSERT_GREATER_THAN_UINT32(0, wctx.iterations);
#if (portNUM_PROCESSORS > 1)
    TEST_ASSERT_EQUAL(1, wctx.observed_core);
#endif

    TEST_ASSERT_EQUAL(ESP_OK, tool_worker_stop(worker, 500));

    /* Verify worker step returning an error exits early and sets running = false */
    worker_test_ctx_t err_ctx = {
        .iterations = 0,
        .observed_core = -1,
        .return_err = ESP_FAIL,
        .sleep_ms = 0,
    };
    cfg.user_ctx = &err_ctx;
    worker = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, tool_worker_start(&worker, &cfg));
    vTaskDelay(pdMS_TO_TICKS(20));
    TEST_ASSERT_EQUAL_UINT32(1U, err_ctx.iterations);
    TEST_ASSERT_FALSE(tool_worker_is_running(worker));
    TEST_ASSERT_EQUAL(ESP_OK, tool_worker_stop(worker, 100));

    /* Verify worker stop timeout returns ESP_ERR_TIMEOUT and keeps handle valid for retry */
    worker_test_ctx_t slow_ctx = {
        .iterations = 0,
        .observed_core = -1,
        .return_err = ESP_OK,
        .sleep_ms = 80,
    };
    cfg.user_ctx = &slow_ctx;
    worker = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, tool_worker_start(&worker, &cfg));
    vTaskDelay(pdMS_TO_TICKS(10));
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, tool_worker_stop(worker, 5));
    TEST_ASSERT_TRUE(tool_worker_stop_requested(worker));
    TEST_ASSERT_EQUAL(ESP_OK, tool_worker_stop(worker, 500));

    /* Verify non-disruptive self-test runner */
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_register_selftest("sample_ok", sample_selftest_ok));
    size_t passed = 0;
    size_t failed = 0;
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_run_selftests(&passed, &failed));
    TEST_ASSERT_EQUAL_UINT32(2, (uint32_t)passed);
    TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)failed);
}
