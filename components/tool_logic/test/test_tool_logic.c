/**
 * @file test_tool_logic.c
 * @brief Unity unit & integration tests for `tool_logic`: UART 8N1 (9600/115200 + framing error),
 *        I²C (START/RESTART/STOP, addr 0x34, R/W, ACK/NACK, data), SPI (CS/SCK/MOSI/MISO Mode 0/1),
 *        PWM (1 kHz 25% duty), RMT symbol expansion, multi-pin ownership, and tool lifecycle.
 */

#include <math.h>
#include <string.h>

#include "bsp_m5core2.h"
#include "bsp_m5core2_priv.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "recorder_sink.h"
#include "recorder_sink_priv.h"
#include "tool_core.h"
#include "tool_core_priv.h"
#include "tool_logic.h"
#include "tool_logic_priv.h"
#include "ui_shell.h"
#include "unity.h"

/* Helper to synthesize an 8N1 UART frame into `buf` on bit `ch` */
static size_t synth_uart_byte(uint8_t *buf,
                              size_t max_len,
                              size_t pos,
                              uint8_t ch,
                              uint8_t byte_val,
                              size_t samples_per_bit,
                              bool stop_bit_high,
                              bool invert)
{
    uint8_t mask = (uint8_t)(1U << ch);
    /* Start bit (Low = 0), 8 data bits LSB-first, Stop bit (1 = valid, 0 = framing error) */
    uint8_t bits[10];
    bits[0] = 0U;
    for (uint8_t b = 0U; b < 8U; ++b) {
        bits[1U + b] = (byte_val >> b) & 1U;
    }
    bits[9] = stop_bit_high ? 1U : 0U;

    for (size_t bit_idx = 0; bit_idx < 10U; ++bit_idx) {
        uint8_t lvl = invert ? (uint8_t)(bits[bit_idx] ^ 1U) : bits[bit_idx];
        for (size_t s = 0; s < samples_per_bit && pos < max_len; ++s) {
            if (lvl != 0U) {
                buf[pos] |= mask;
            } else {
                buf[pos] &= (uint8_t)(~mask);
            }
            pos++;
        }
    }
    return pos;
}

TEST_CASE("given UART 8N1 waveforms at 9600 and 115200 baud with framing errors when decoded then bytes ASCII and error counts match", "[tool_logic][uart]")
{
    uint8_t buf[600];
    /* Initialize idle High on CH0 (0x01) */
    memset(buf, 0x01, sizeof(buf));

    /* Single-sample glitch low at index 5 (must be rejected) */
    buf[5] = 0x00;

    /* Synthesize "OK" ('O' = 0x4F, 'K' = 0x4B) at 10 samples/bit, followed by 0x55 with framing error (stop_bit = 0) */
    size_t pos = 20U;
    pos = synth_uart_byte(buf, sizeof(buf), pos, 0U, (uint8_t)'O', 10U, true, false);
    pos += 15U; /* Inter-frame idle High */
    pos = synth_uart_byte(buf, sizeof(buf), pos, 0U, (uint8_t)'K', 10U, true, false);
    pos += 15U;
    pos = synth_uart_byte(buf, sizeof(buf), pos, 0U, 0x55U, 10U, false, false);
    /* Keep line low briefly after framing error, then return idle High */
    for (size_t k = 0; k < 10U && pos < sizeof(buf); ++k) {
        buf[pos++] = 0x00U;
    }

    logic_uart_result_t ures = {0};
    /* 1. Test at 9600 baud (sample rate = 96,000 Hz -> 10 samples/bit) */
    TEST_ASSERT_EQUAL(ESP_OK, logic_decode_uart(buf, sizeof(buf), 96000U, 0U, 9600U, false, &ures));
    TEST_ASSERT_EQUAL_UINT32(3U, (uint32_t)ures.frame_count);
    TEST_ASSERT_EQUAL_UINT32(1U, (uint32_t)ures.error_count);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)'O', ures.frames[0].data_byte);
    TEST_ASSERT_FALSE(ures.frames[0].framing_error);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)'K', ures.frames[1].data_byte);
    TEST_ASSERT_FALSE(ures.frames[1].framing_error);
    TEST_ASSERT_EQUAL_HEX8(0x55U, ures.frames[2].data_byte);
    TEST_ASSERT_TRUE(ures.frames[2].framing_error);
    TEST_ASSERT_EQUAL_STRING("OK", ures.ascii_str);

    /* 2. Test at 115200 baud (sample rate = 1,152,000 Hz -> 10 samples/bit) */
    memset(&ures, 0, sizeof(ures));
    TEST_ASSERT_EQUAL(ESP_OK,
                      logic_decode_uart(buf, sizeof(buf), 1152000U, 0U, 115200U, false, &ures));
    TEST_ASSERT_EQUAL_UINT32(3U, (uint32_t)ures.frame_count);
    TEST_ASSERT_EQUAL_STRING("OK", ures.ascii_str);
}

/* Helpers to synthesize I2C bus waveforms (CH0 = SDA, CH1 = SCL) */
static void i2c_append_state(uint8_t *buf, size_t max_len, size_t *pos, uint8_t sda, uint8_t scl, size_t count)
{
    uint8_t val = (uint8_t)((sda & 1U) | ((scl & 1U) << 1U));
    for (size_t i = 0; i < count && *pos < max_len; ++i) {
        buf[(*pos)++] = val;
    }
}

static void i2c_append_byte(uint8_t *buf, size_t max_len, size_t *pos, uint8_t byte_val, bool ack)
{
    for (int b = 7; b >= 0; --b) {
        uint8_t bit = (byte_val >> b) & 1U;
        i2c_append_state(buf, max_len, pos, bit, 0U, 2U); /* SCL Low, setup SDA */
        i2c_append_state(buf, max_len, pos, bit, 1U, 2U); /* SCL High, sample SDA */
        i2c_append_state(buf, max_len, pos, bit, 0U, 2U); /* SCL Low */
    }
    uint8_t ack_sda = ack ? 0U : 1U;
    i2c_append_state(buf, max_len, pos, ack_sda, 0U, 2U);
    i2c_append_state(buf, max_len, pos, ack_sda, 1U, 2U);
    i2c_append_state(buf, max_len, pos, ack_sda, 0U, 2U);
}

TEST_CASE("given I2C transaction with START addr 0x34 ACK data 0x01 RESTART read NACK and STOP when decoded then all events match", "[tool_logic][i2c]")
{
    uint8_t buf[512];
    size_t pos = 0U;

    /* Idle bus: SDA=1, SCL=1 */
    i2c_append_state(buf, sizeof(buf), &pos, 1U, 1U, 4U);
    /* START condition: SDA 1->0 while SCL=1 */
    i2c_append_state(buf, sizeof(buf), &pos, 0U, 1U, 2U);
    i2c_append_state(buf, sizeof(buf), &pos, 0U, 0U, 2U);

    /* Address 0x34 Write (0x68) + ACK */
    i2c_append_byte(buf, sizeof(buf), &pos, (0x34U << 1U) | 0U, true);
    /* Register byte 0x01 + ACK */
    i2c_append_byte(buf, sizeof(buf), &pos, 0x01U, true);

    /* Repeated START (RESTART): SDA=1 while SCL=0, then SCL=1, then SDA 1->0 while SCL=1 */
    i2c_append_state(buf, sizeof(buf), &pos, 1U, 0U, 2U);
    i2c_append_state(buf, sizeof(buf), &pos, 1U, 1U, 2U);
    i2c_append_state(buf, sizeof(buf), &pos, 0U, 1U, 2U);
    i2c_append_state(buf, sizeof(buf), &pos, 0U, 0U, 2U);

    /* Address 0x34 Read (0x69) + ACK */
    i2c_append_byte(buf, sizeof(buf), &pos, (0x34U << 1U) | 1U, true);
    /* Read byte 0xA5 + NACK (master ends read) */
    i2c_append_byte(buf, sizeof(buf), &pos, 0xA5U, false);

    /* STOP condition: SDA=0 while SCL=0 -> SCL=1 -> SDA 0->1 while SCL=1 */
    i2c_append_state(buf, sizeof(buf), &pos, 0U, 0U, 2U);
    i2c_append_state(buf, sizeof(buf), &pos, 0U, 1U, 2U);
    i2c_append_state(buf, sizeof(buf), &pos, 1U, 1U, 4U);

    logic_i2c_result_t ires = {0};
    TEST_ASSERT_EQUAL(ESP_OK, logic_decode_i2c(buf, pos, 1U, 0U, &ires));
    TEST_ASSERT_EQUAL_UINT32(7U, (uint32_t)ires.event_count);
    TEST_ASSERT_EQUAL_UINT32(2U, (uint32_t)ires.transaction_count);
    TEST_ASSERT_EQUAL_UINT32(3U, (uint32_t)ires.ack_count);
    TEST_ASSERT_EQUAL_UINT32(1U, (uint32_t)ires.nack_count);
    TEST_ASSERT_EQUAL_HEX8(0x34U, ires.last_addr_7bit);

    TEST_ASSERT_EQUAL(LOGIC_I2C_EVENT_START, ires.events[0].type);
    TEST_ASSERT_EQUAL(LOGIC_I2C_EVENT_ADDR, ires.events[1].type);
    TEST_ASSERT_EQUAL_HEX8(0x34U, ires.events[1].addr_7bit);
    TEST_ASSERT_FALSE(ires.events[1].is_read);
    TEST_ASSERT_TRUE(ires.events[1].ack);

    TEST_ASSERT_EQUAL(LOGIC_I2C_EVENT_DATA, ires.events[2].type);
    TEST_ASSERT_EQUAL_HEX8(0x01U, ires.events[2].data_byte);
    TEST_ASSERT_TRUE(ires.events[2].ack);

    TEST_ASSERT_EQUAL(LOGIC_I2C_EVENT_RESTART, ires.events[3].type);
    TEST_ASSERT_EQUAL(LOGIC_I2C_EVENT_ADDR, ires.events[4].type);
    TEST_ASSERT_EQUAL_HEX8(0x34U, ires.events[4].addr_7bit);
    TEST_ASSERT_TRUE(ires.events[4].is_read);
    TEST_ASSERT_TRUE(ires.events[4].ack);

    TEST_ASSERT_EQUAL(LOGIC_I2C_EVENT_DATA, ires.events[5].type);
    TEST_ASSERT_EQUAL_HEX8(0xA5U, ires.events[5].data_byte);
    TEST_ASSERT_FALSE(ires.events[5].ack);

    TEST_ASSERT_EQUAL(LOGIC_I2C_EVENT_STOP, ires.events[6].type);
}

TEST_CASE("given SPI 4-wire transfer in Mode 0 and Mode 1 when decoded then MOSI and MISO bytes and CS framing are exact", "[tool_logic][spi]")
{
    /* Channels: CH0 = CS (active low), CH1 = SCK, CH2 = MOSI, CH3 = MISO */
    uint8_t buf[256];
    size_t pos = 0U;

    /* 1. CS High (inactive): clock pulses must be ignored */
    for (int i = 0; i < 8; ++i) {
        buf[pos++] = (1U << 0) | (0U << 1) | (1U << 2) | (1U << 3);
        buf[pos++] = (1U << 0) | (1U << 1) | (1U << 2) | (1U << 3);
    }

    /* 2. Assert CS Low and clock 2 bytes: MOSI = {0x9F, 0xA5}, MISO = {0x5A, 0x3C} in Mode 0 (rising SCK) */
    const uint8_t mosi_bytes[2] = {0x9FU, 0xA5U};
    const uint8_t miso_bytes[2] = {0x5AU, 0x3CU};
    buf[pos++] = 0x00U; /* CS=0, SCK=0 */

    for (size_t w = 0; w < 2U; ++w) {
        for (int b = 7; b >= 0; --b) {
            uint8_t mo = (mosi_bytes[w] >> b) & 1U;
            uint8_t mi = (miso_bytes[w] >> b) & 1U;
            uint8_t data_bits = (uint8_t)((mo << 2U) | (mi << 3U));
            buf[pos++] = data_bits | (0U << 1U); /* SCK Low */
            buf[pos++] = data_bits | (1U << 1U); /* SCK Rising -> sample in Mode 0 */
            buf[pos++] = data_bits | (0U << 1U); /* SCK Falling -> sample in Mode 1 */
        }
    }
    /* Deassert CS High */
    buf[pos++] = (1U << 0);

    logic_spi_result_t sres = {0};
    TEST_ASSERT_EQUAL(ESP_OK, logic_decode_spi(buf, pos, 0U, 1U, 2U, 3U, 0U, 0U, true, &sres));
    TEST_ASSERT_EQUAL_UINT32(2U, (uint32_t)sres.word_count);
    TEST_ASSERT_EQUAL_HEX8(0x9FU, sres.words[0].mosi_byte);
    TEST_ASSERT_EQUAL_HEX8(0x5AU, sres.words[0].miso_byte);
    TEST_ASSERT_EQUAL_HEX8(0xA5U, sres.words[1].mosi_byte);
    TEST_ASSERT_EQUAL_HEX8(0x3CU, sres.words[1].miso_byte);

    /* Verify Mode 1 (CPOL=0, CPHA=1, falling SCK edge) also decodes the same held data bits */
    memset(&sres, 0, sizeof(sres));
    TEST_ASSERT_EQUAL(ESP_OK, logic_decode_spi(buf, pos, 0U, 1U, 2U, 3U, 0U, 1U, true, &sres));
    TEST_ASSERT_EQUAL_UINT32(2U, (uint32_t)sres.word_count);
    TEST_ASSERT_EQUAL_HEX8(0x9FU, sres.words[0].mosi_byte);
    TEST_ASSERT_EQUAL_HEX8(0x5AU, sres.words[0].miso_byte);
}

TEST_CASE("given RMT edge symbols and PWM waveform when expanded and decoded then 1kHz 25-percent duty and zoom viewport are accurate", "[tool_logic][pwm]")
{
    /* Expand 4 periods of 1 kHz 25% duty cycle RMT symbols (250 us high, 750 us low at 1 MHz) into a 100 kHz sample buffer */
    rmt_symbol_word_t syms[4];
    for (size_t i = 0; i < 4U; ++i) {
        syms[i].duration0 = 250U;
        syms[i].level0 = 1U;
        syms[i].duration1 = 750U;
        syms[i].level1 = 0U;
    }

    uint8_t buf[512] = {0};
    size_t written = 0U;
    TEST_ASSERT_EQUAL(ESP_OK,
                      logic_expand_rmt_to_samples(syms, 4U, 1000000U, 100000U, 0U, buf, sizeof(buf), &written));
    TEST_ASSERT_EQUAL_UINT32(400U, (uint32_t)written);

    logic_pwm_result_t pres = {0};
    TEST_ASSERT_EQUAL(ESP_OK, logic_decode_pwm(buf, written, 100000U, 0U, &pres));
    TEST_ASSERT_TRUE(pres.valid);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 1000.0f, pres.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 1000.0f, pres.period_us);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 250.0f, pres.high_width_us);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 750.0f, pres.low_width_us);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 25.0f, pres.duty_pct);

    /* Verify digital trigger search and 4x zoom viewport extraction */
    bool trig = false;
    size_t trig_idx = 0U;
    TEST_ASSERT_EQUAL(ESP_OK,
                      logic_find_digital_trigger(buf, written, 0U, LOGIC_TRIG_FALLING, &trig, &trig_idx));
    TEST_ASSERT_TRUE(trig);
    TEST_ASSERT_EQUAL_UINT32(25U, (uint32_t)trig_idx);

    uint8_t view[LOGIC_VIEW_POINTS] = {0};
    TEST_ASSERT_EQUAL(ESP_OK,
                      logic_extract_viewport(buf, written, LOGIC_ZOOM_4X, 20U, view, LOGIC_VIEW_POINTS));
    /* At pan=20 and 4x zoom: samples 20..24 are High (1) -> view[0..19] == 1, sample 25 is Low (0) -> view[20..23] == 0 */
    TEST_ASSERT_EQUAL_HEX8(0x01U, view[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01U, view[19]);
    TEST_ASSERT_EQUAL_HEX8(0x00U, view[20]);
}

static esp_err_t synth_logic_uart_source_cb(logic_capture_mode_t cap_mode,
                                            const gpio_num_t channel_pins[LOGIC_CHANNEL_COUNT],
                                            uint32_t requested_rate_hz,
                                            uint8_t *out_samples,
                                            size_t max_samples,
                                            size_t *out_sample_count,
                                            uint32_t *out_actual_rate_hz,
                                            void *user_ctx)
{
    (void)cap_mode;
    (void)channel_pins;
    (void)requested_rate_hz;
    (void)user_ctx;

    memset(out_samples, 0x01, max_samples);
    size_t pos = 10U;
    pos = synth_uart_byte(out_samples, max_samples, pos, 0U, (uint8_t)'O', 10U, true, false);
    pos += 10U;
    pos = synth_uart_byte(out_samples, max_samples, pos, 0U, (uint8_t)'K', 10U, true, false);
    *out_sample_count = max_samples;
    *out_actual_rate_hz = 96000U;
    return ESP_OK;
}

TEST_CASE("given registered tool_logic when started configured and stepped then multi-pin ownership UI decode table and recorder hooks work", "[tool_logic][lifecycle]")
{
    tool_registry_test_reset();
    recorder_sink_test_reset();
    bsp_pin_manager_reset();

    TEST_ASSERT_EQUAL(ESP_OK, tool_core_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_channel_pin(0U, BSP_PIN_PORT_C_RXD2));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_channel_pin(1U, BSP_PIN_PORT_C_TXD2));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_channel_pin(2U, BSP_PIN_PORT_A_SDA));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_channel_pin(3U, BSP_PIN_PORT_A_SCL));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_capture_mode(LOGIC_CAP_MODE_SAMPLED));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_sample_rate(LOGIC_RATE_100KHZ));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_protocol(LOGIC_PROTO_UART));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_uart_baud(9600U));

    tool_logic_set_capture_source(synth_logic_uart_source_cb, NULL);

    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_register());
    size_t passed = 0, failed = 0;
    TEST_ASSERT_EQUAL(ESP_OK, tool_core_run_selftests(&passed, &failed));
    TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)failed);

    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_init());

    /* Verify atomic rollback on start if a channel pin is already claimed by a background generator */
    TEST_ASSERT_EQUAL(ESP_OK, bsp_pin_claim(BSP_PIN_PORT_A_SCL, "wavegen"));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ui_shell_open_tool(LOGIC_TOOL_ID));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_RXD2, LOGIC_TOOL_ID));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_TXD2, LOGIC_TOOL_ID));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_PORT_A_SDA, LOGIC_TOOL_ID));
    TEST_ASSERT_EQUAL(ESP_OK, bsp_pin_release(BSP_PIN_PORT_A_SCL, "wavegen"));

    /* Now open tool_logic cleanly */
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(LOGIC_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_RXD2, LOGIC_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_TXD2, LOGIC_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_PORT_A_SDA, LOGIC_TOOL_ID));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_PORT_A_SCL, LOGIC_TOOL_ID));

    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_set_mode(REC_MODE_USB_STUB));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_run_single_step());
    ui_shell_step();

    static logic_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_get_latest_frame(&frame));
    TEST_ASSERT_EQUAL_UINT32(2U, (uint32_t)frame.decoded.uart.frame_count);
    TEST_ASSERT_EQUAL_STRING("OK", frame.decoded.uart.ascii_str);
    TEST_ASSERT_EQUAL_UINT32(2U, (uint32_t)frame.decoded.row_count);

    recorder_sink_stats_t rstats = {0};
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_get_stats(&rstats));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1U, rstats.frames_pushed);
    TEST_ASSERT_EQUAL_STRING(LOGIC_TOOL_ID, rstats.last_tool_id);
    TEST_ASSERT_EQUAL_UINT32(96000U, rstats.last_sample_rate_hz);

    /* Duplicate pin assignment across channels is rejected; remapping CH0 -> GPIO36 succeeds */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, tool_logic_set_channel_pin(0U, BSP_PIN_PORT_C_TXD2));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_channel_pin(0U, BSP_PIN_ANALOG_IN_DEFAULT));
    TEST_ASSERT_TRUE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_DEFAULT, LOGIC_TOOL_ID));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_RXD2, LOGIC_TOOL_ID));

    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(false));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_ANALOG_IN_DEFAULT, LOGIC_TOOL_ID));
    TEST_ASSERT_FALSE(bsp_pin_is_owned_by(BSP_PIN_PORT_C_TXD2, LOGIC_TOOL_ID));

    /* Test real hardware capture paths (SAMPLED & RMT_EDGE) and cooperative stop */
    tool_logic_set_capture_source(NULL, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_channel_pin(0U, BSP_PIN_PORT_C_RXD2));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_sample_rate(LOGIC_RATE_500KHZ));
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(LOGIC_TOOL_ID));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_run_single_step());

    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_capture_mode(LOGIC_CAP_MODE_RMT_EDGE));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_run_single_step());

    /* Switch to slow 10 kHz sampled mode (~102 ms capture) and verify immediate cooperative stop */
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_capture_mode(LOGIC_CAP_MODE_SAMPLED));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_sample_rate(LOGIC_RATE_10KHZ));
    vTaskDelay(pdMS_TO_TICKS(25));

    int64_t stop_start_us = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(false));
    int64_t stop_elapsed_us = esp_timer_get_time() - stop_start_us;
    TEST_ASSERT_LESS_THAN_INT32(250000, (int32_t)stop_elapsed_us);
    TEST_ASSERT_TRUE(heap_caps_check_integrity_all(true));

    (void)tool_logic_set_sample_rate(LOGIC_RATE_100KHZ);
}

TEST_CASE("given sample rate presets and edge timestamps when clamped and converted then rate caps and RMT symbols are accurate", "[tool_logic][decoders]")
{
    /* 1. Sampled mode clamps presets above 100 kHz to 100 kHz */
    TEST_ASSERT_EQUAL(LOGIC_RATE_10KHZ, logic_clamp_sample_rate(LOGIC_CAP_MODE_SAMPLED, LOGIC_RATE_10KHZ));
    TEST_ASSERT_EQUAL(LOGIC_RATE_50KHZ, logic_clamp_sample_rate(LOGIC_CAP_MODE_SAMPLED, LOGIC_RATE_50KHZ));
    TEST_ASSERT_EQUAL(LOGIC_RATE_100KHZ, logic_clamp_sample_rate(LOGIC_CAP_MODE_SAMPLED, LOGIC_RATE_100KHZ));
    TEST_ASSERT_EQUAL(LOGIC_RATE_100KHZ, logic_clamp_sample_rate(LOGIC_CAP_MODE_SAMPLED, LOGIC_RATE_250KHZ));
    TEST_ASSERT_EQUAL(LOGIC_RATE_100KHZ, logic_clamp_sample_rate(LOGIC_CAP_MODE_SAMPLED, LOGIC_RATE_500KHZ));
    TEST_ASSERT_EQUAL(LOGIC_RATE_100KHZ, logic_clamp_sample_rate(LOGIC_CAP_MODE_SAMPLED, LOGIC_RATE_1MHZ));

    TEST_ASSERT_EQUAL_UINT32(10000U, logic_effective_sample_rate_hz(LOGIC_CAP_MODE_SAMPLED, LOGIC_RATE_10KHZ));
    TEST_ASSERT_EQUAL_UINT32(50000U, logic_effective_sample_rate_hz(LOGIC_CAP_MODE_SAMPLED, LOGIC_RATE_50KHZ));
    TEST_ASSERT_EQUAL_UINT32(100000U, logic_effective_sample_rate_hz(LOGIC_CAP_MODE_SAMPLED, LOGIC_RATE_100KHZ));
    TEST_ASSERT_EQUAL_UINT32(100000U, logic_effective_sample_rate_hz(LOGIC_CAP_MODE_SAMPLED, LOGIC_RATE_250KHZ));
    TEST_ASSERT_EQUAL_UINT32(100000U, logic_effective_sample_rate_hz(LOGIC_CAP_MODE_SAMPLED, LOGIC_RATE_500KHZ));
    TEST_ASSERT_EQUAL_UINT32(100000U, logic_effective_sample_rate_hz(LOGIC_CAP_MODE_SAMPLED, LOGIC_RATE_1MHZ));

    TEST_ASSERT_TRUE(logic_capture_mode_channel_available(LOGIC_CAP_MODE_SAMPLED, 0U));
    TEST_ASSERT_TRUE(logic_capture_mode_channel_available(LOGIC_CAP_MODE_SAMPLED, 3U));
    TEST_ASSERT_TRUE(logic_capture_mode_channel_available(LOGIC_CAP_MODE_RMT_EDGE, 0U));
    TEST_ASSERT_FALSE(logic_capture_mode_channel_available(LOGIC_CAP_MODE_RMT_EDGE, 1U));
    TEST_ASSERT_FALSE(logic_capture_mode_channel_available(LOGIC_CAP_MODE_RMT_EDGE, 3U));
    TEST_ASSERT_FALSE(logic_capture_mode_channel_available(LOGIC_CAP_MODE_COUNT, 0U));
    TEST_ASSERT_FALSE(logic_capture_mode_channel_available(LOGIC_CAP_MODE_SAMPLED, LOGIC_CHANNEL_COUNT));

    /* 2. RMT_EDGE mode allows all presets up to 1 MHz */
    TEST_ASSERT_EQUAL(LOGIC_RATE_250KHZ, logic_clamp_sample_rate(LOGIC_CAP_MODE_RMT_EDGE, LOGIC_RATE_250KHZ));
    TEST_ASSERT_EQUAL(LOGIC_RATE_500KHZ, logic_clamp_sample_rate(LOGIC_CAP_MODE_RMT_EDGE, LOGIC_RATE_500KHZ));
    TEST_ASSERT_EQUAL(LOGIC_RATE_1MHZ, logic_clamp_sample_rate(LOGIC_CAP_MODE_RMT_EDGE, LOGIC_RATE_1MHZ));
    TEST_ASSERT_EQUAL_UINT32(1000000U, logic_effective_sample_rate_hz(LOGIC_CAP_MODE_RMT_EDGE, LOGIC_RATE_1MHZ));
    TEST_ASSERT_EQUAL_UINT32(0U, logic_effective_sample_rate_hz(LOGIC_CAP_MODE_SAMPLED, LOGIC_RATE_COUNT));

    /* 3. Continuous edge timestamps -> RMT symbols with duplicate-level & 0-us glitch rejection */
    logic_edge_sample_t edges[12];
    size_t n = 0U;
    uint32_t t = 1000U;
    for (int cyc = 0; cyc < 4; ++cyc) {
        edges[n].timestamp_us = t;
        edges[n].level = 1U;
        n++;
        if (cyc == 1) {
            /* Inject zero-duration glitch at `t` and duplicate-level bounce at `t + 5` */
            edges[n].timestamp_us = t;
            edges[n].level = 0U;
            n++;
            edges[n].timestamp_us = t;
            edges[n].level = 1U;
            n++;
            edges[n].timestamp_us = t + 5U;
            edges[n].level = 1U;
            n++;
        }
        t += 250U;
        edges[n].timestamp_us = t;
        edges[n].level = 0U;
        n++;
        t += 750U;
    }
    edges[n].timestamp_us = t;
    edges[n].level = 1U;
    n++;

    rmt_symbol_word_t syms[16] = {0};
    size_t sym_cnt = 0U;
    TEST_ASSERT_EQUAL(ESP_OK, logic_edge_timestamps_to_rmt_symbols(edges, n, syms, 16U, &sym_cnt));
    TEST_ASSERT_EQUAL_UINT32(4U, (uint32_t)sym_cnt);

    uint8_t buf[512] = {0};
    size_t written = 0U;
    TEST_ASSERT_EQUAL(ESP_OK,
                      logic_expand_rmt_to_samples(syms, sym_cnt, 1000000U, 100000U, 0U, buf, sizeof(buf), &written));
    TEST_ASSERT_EQUAL_UINT32(400U, (uint32_t)written);

    logic_pwm_result_t pres = {0};
    TEST_ASSERT_EQUAL(ESP_OK, logic_decode_pwm(buf, written, 100000U, 0U, &pres));
    TEST_ASSERT_TRUE(pres.valid);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 1000.0f, pres.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 25.0f, pres.duty_pct);

    /* 4. Long pulse (> 32767 us) splitting across RMT half-words and < 2 edge handling */
    logic_edge_sample_t long_edges[2] = {
        {.timestamp_us = 100U, .level = 1U},
        {.timestamp_us = 50100U, .level = 0U},
    };
    TEST_ASSERT_EQUAL(ESP_OK, logic_edge_timestamps_to_rmt_symbols(long_edges, 2U, syms, 16U, &sym_cnt));
    TEST_ASSERT_EQUAL_UINT32(1U, (uint32_t)sym_cnt);
    TEST_ASSERT_EQUAL_UINT32(32767U, (uint32_t)syms[0].duration0);
    TEST_ASSERT_EQUAL_UINT32(17233U, (uint32_t)syms[0].duration1);
    TEST_ASSERT_EQUAL_UINT32(1U, (uint32_t)syms[0].level0);
    TEST_ASSERT_EQUAL_UINT32(1U, (uint32_t)syms[0].level1);

    TEST_ASSERT_EQUAL(ESP_OK, logic_edge_timestamps_to_rmt_symbols(long_edges, 1U, syms, 16U, &sym_cnt));
    TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)sym_cnt);
}

static uint8_t IRAM_ATTR test_isr_square_wave_cb(size_t sample_idx, int64_t now_us, void *user_ctx)
{
    (void)now_us;
    uint32_t *counter = (uint32_t *)user_ctx;
    if (counter != NULL) {
        (*counter)++;
    }
    /* 8-sample period on CH0 (4 High, 4 Low = 50% duty cycle) */
    return ((sample_idx & 7U) < 4U) ? 0x01U : 0x00U;
}

TEST_CASE("given sampled mode at 10kHz 50kHz 100kHz and capped 250kHz-1MHz when stepped then real GPTimer ISR fires and measures rate", "[tool_logic][sampled_rate]")
{
    tool_registry_test_reset();
    recorder_sink_test_reset();
    bsp_pin_manager_reset();

    TEST_ASSERT_EQUAL(ESP_OK, tool_core_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_register());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_init());

    tool_logic_set_capture_source(NULL, NULL);
    tool_logic_force_gptimer_timeout_for_test(false);
    uint32_t cb_calls = 0U;
    tool_logic_set_isr_sample_cb_for_test(test_isr_square_wave_cb, &cb_calls);

    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_capture_mode(LOGIC_CAP_MODE_SAMPLED));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_protocol(LOGIC_PROTO_PWM));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_sample_rate(LOGIC_RATE_100KHZ));
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(LOGIC_TOOL_ID));
    /* Pause background worker so synchronous steps have exclusive timer access */
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(false));
    vTaskDelay(pdMS_TO_TICKS(10));

    static logic_frame_t frame;
    logic_window_stats_t stats = {0};

    /* 1. 10 kHz sampled capture (1024 samples * 100 us = ~102.3 ms) */
    cb_calls = 0U;
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_sample_rate(LOGIC_RATE_10KHZ));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(true));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(false));

    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_get_latest_frame(&frame));
    tool_logic_get_last_window_stats(&stats);
    TEST_ASSERT_EQUAL(ESP_OK, frame.hw_error);
    TEST_ASSERT_FALSE(frame.no_capture);
    TEST_ASSERT_FALSE(frame.rate_capped);
    TEST_ASSERT_EQUAL_UINT32(10000U, frame.requested_rate_hz);
    TEST_ASSERT_EQUAL_UINT32(10000U, frame.effective_rate_hz);
    TEST_ASSERT_EQUAL_UINT32(LOGIC_DEFAULT_CAPTURE_SAMPLES, (uint32_t)frame.total_captured);
    TEST_ASSERT_EQUAL_UINT32(LOGIC_DEFAULT_CAPTURE_SAMPLES, stats.gptimer_isr_count);
    TEST_ASSERT_EQUAL_UINT32(0U, stats.bounce_overruns);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(LOGIC_DEFAULT_CAPTURE_SAMPLES, cb_calls);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(70000U, frame.capture_elapsed_us);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(220000U, frame.capture_elapsed_us);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(4500U, frame.sample_rate_hz);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(14500U, frame.sample_rate_hz);
    TEST_ASSERT_TRUE(frame.decoded.pwm.valid);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 50.0f, frame.decoded.pwm.duty_pct);

    /* 2. 50 kHz sampled capture (1024 samples * 20 us = ~20.5 ms) */
    cb_calls = 0U;
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_sample_rate(LOGIC_RATE_50KHZ));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(true));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(false));

    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_get_latest_frame(&frame));
    tool_logic_get_last_window_stats(&stats);
    TEST_ASSERT_EQUAL(ESP_OK, frame.hw_error);
    TEST_ASSERT_FALSE(frame.rate_capped);
    TEST_ASSERT_EQUAL_UINT32(50000U, frame.requested_rate_hz);
    TEST_ASSERT_EQUAL_UINT32(50000U, frame.effective_rate_hz);
    TEST_ASSERT_EQUAL_UINT32(LOGIC_DEFAULT_CAPTURE_SAMPLES, stats.gptimer_isr_count);
    TEST_ASSERT_EQUAL_UINT32(0U, stats.bounce_overruns);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(12000U, frame.capture_elapsed_us);
    TEST_ASSERT_GREATER_THAN_UINT32(0U, frame.sample_rate_hz);

    /* 3. 100 kHz sampled capture (1024 samples * 10 us = ~10.2 ms) */
    cb_calls = 0U;
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_sample_rate(LOGIC_RATE_100KHZ));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(true));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(false));

    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_get_latest_frame(&frame));
    tool_logic_get_last_window_stats(&stats);
    TEST_ASSERT_EQUAL(ESP_OK, frame.hw_error);
    TEST_ASSERT_FALSE(frame.rate_capped);
    TEST_ASSERT_EQUAL_UINT32(100000U, frame.requested_rate_hz);
    TEST_ASSERT_EQUAL_UINT32(100000U, frame.effective_rate_hz);
    TEST_ASSERT_EQUAL_UINT32(LOGIC_DEFAULT_CAPTURE_SAMPLES, stats.gptimer_isr_count);
    TEST_ASSERT_EQUAL_UINT32(0U, stats.bounce_overruns);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(5000U, frame.capture_elapsed_us);
    TEST_ASSERT_GREATER_THAN_UINT32(0U, frame.sample_rate_hz);

    /* 4. Presets > 100 kHz (250 kHz, 500 kHz, 1 MHz) are capped to 100 kHz and report measured rate to recorder */
    const logic_sample_rate_t high_rates[] = {LOGIC_RATE_250KHZ, LOGIC_RATE_500KHZ, LOGIC_RATE_1MHZ};
    const uint32_t req_hz[] = {250000U, 500000U, 1000000U};
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_set_mode(REC_MODE_USB_STUB));

    for (size_t i = 0; i < 3U; ++i) {
        recorder_sink_test_reset();
        TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
        TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_set_mode(REC_MODE_USB_STUB));

        TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_sample_rate(high_rates[i]));
        TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(true));
        TEST_ASSERT_EQUAL(ESP_OK, tool_logic_run_single_step());
        TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(false));
        ui_shell_step();

        TEST_ASSERT_EQUAL(ESP_OK, tool_logic_get_latest_frame(&frame));
        tool_logic_get_last_window_stats(&stats);
        TEST_ASSERT_EQUAL(ESP_OK, frame.hw_error);
        TEST_ASSERT_TRUE(frame.rate_capped);
        TEST_ASSERT_EQUAL_UINT32(req_hz[i], frame.requested_rate_hz);
        TEST_ASSERT_EQUAL_UINT32(LOGIC_SAMPLED_MAX_RATE_HZ, frame.effective_rate_hz);
        TEST_ASSERT_EQUAL_UINT32(LOGIC_DEFAULT_CAPTURE_SAMPLES, stats.gptimer_isr_count);
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(150000U, frame.sample_rate_hz);

        recorder_sink_stats_t rstats = {0};
        TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_get_stats(&rstats));
        TEST_ASSERT_EQUAL_UINT32(1U, rstats.frames_pushed);
        TEST_ASSERT_EQUAL_UINT32(frame.sample_rate_hz, rstats.last_sample_rate_hz);
    }

    tool_logic_set_isr_sample_cb_for_test(NULL, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(false));
    (void)tool_logic_set_sample_rate(LOGIC_RATE_100KHZ);
}

TEST_CASE("given sampled mode when GPTimer stalls then capture times out cleanly without recorder push", "[tool_logic][timeout]")
{
    tool_registry_test_reset();
    recorder_sink_test_reset();
    bsp_pin_manager_reset();

    TEST_ASSERT_EQUAL(ESP_OK, tool_core_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_register());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_init());

    tool_logic_set_capture_source(NULL, NULL);
    tool_logic_set_isr_sample_cb_for_test(NULL, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_capture_mode(LOGIC_CAP_MODE_SAMPLED));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_sample_rate(LOGIC_RATE_100KHZ));
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(LOGIC_TOOL_ID));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(false));
    vTaskDelay(pdMS_TO_TICKS(10));

    recorder_sink_test_reset();
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_set_mode(REC_MODE_USB_STUB));

    tool_logic_force_gptimer_timeout_for_test(true);
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(true));
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, tool_logic_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(false));
    tool_logic_force_gptimer_timeout_for_test(false);
    ui_shell_step();

    static logic_frame_t frame;
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_get_latest_frame(&frame));
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, frame.hw_error);
    TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)frame.total_captured);
    TEST_ASSERT_EQUAL_UINT32(0U, frame.sample_rate_hz);

    recorder_sink_stats_t rstats = {0};
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_get_stats(&rstats));
    TEST_ASSERT_EQUAL_UINT32(0U, rstats.frames_pushed);

    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(false));
}

typedef struct {
    uint32_t sim_ts_us;
    uint32_t half_period_us;
    uint8_t level;
    bool flood_same_level;
} logic_edge_feeder_ctx_t;

static void logic_edge_feeder_timer_cb(void *arg)
{
    logic_edge_feeder_ctx_t *ctx = (logic_edge_feeder_ctx_t *)arg;
    if (ctx == NULL) {
        return;
    }
    if (ctx->flood_same_level) {
        for (int i = 0; i < 64; ++i) {
            ctx->sim_ts_us += 2U;
            tool_logic_record_edge_isr_for_test(ctx->sim_ts_us, 1U);
        }
    } else {
        for (int i = 0; i < 4; ++i) {
            ctx->level ^= 1U;
            ctx->sim_ts_us += ctx->half_period_us;
            tool_logic_record_edge_isr_for_test(ctx->sim_ts_us, ctx->level);
        }
    }
}

TEST_CASE("given RMT_EDGE mode when idle or driven by continuous edges or flooded then no_capture continuous decode and ISR cap work", "[tool_logic][rmt_edge]")
{
    tool_registry_test_reset();
    recorder_sink_test_reset();
    bsp_pin_manager_reset();

    TEST_ASSERT_EQUAL(ESP_OK, tool_core_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_register());
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_init());

    tool_logic_set_capture_source(NULL, NULL);
    tool_logic_set_isr_sample_cb_for_test(NULL, NULL);
    tool_logic_force_gptimer_timeout_for_test(false);
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_edge_timeout_us(15000U));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_capture_mode(LOGIC_CAP_MODE_RMT_EDGE));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_sample_rate(LOGIC_RATE_100KHZ));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_protocol(LOGIC_PROTO_PWM));

    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_open_tool(LOGIC_TOOL_ID));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(false));
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Subcase A: Idle line in RMT_EDGE mode reports distinct no_capture and skips recorder push */
    recorder_sink_test_reset();
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_init());
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_set_mode(REC_MODE_USB_STUB));

    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(true));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(false));
    ui_shell_step();

    static logic_frame_t frame;
    logic_window_stats_t stats = {0};
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_get_latest_frame(&frame));
    tool_logic_get_last_window_stats(&stats);
    TEST_ASSERT_EQUAL(ESP_OK, frame.hw_error);
    TEST_ASSERT_TRUE(frame.no_capture);
    TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)frame.total_captured);
    TEST_ASSERT_EQUAL_UINT32(0U, frame.sample_rate_hz);
    TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)frame.decoded.row_count);
    TEST_ASSERT_EQUAL_STRING("RMT EDGE: No transitions captured", frame.decoded.summary);

    recorder_sink_stats_t rstats = {0};
    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_get_stats(&rstats));
    TEST_ASSERT_EQUAL_UINT32(0U, rstats.frames_pushed);

    /* Subcase B: Continuous 1 kHz 50% duty square wave without RMT idle gap is captured and decoded */
    logic_edge_feeder_ctx_t feed_ctx = {
        .sim_ts_us = 1000U,
        .half_period_us = 500U,
        .level = 0U,
        .flood_same_level = false,
    };
    esp_timer_handle_t feed_timer = NULL;
    const esp_timer_create_args_t targs = {
        .callback = logic_edge_feeder_timer_cb,
        .arg = &feed_ctx,
        .name = "logic_edge_feed",
    };
    TEST_ASSERT_EQUAL(ESP_OK, esp_timer_create(&targs, &feed_timer));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_edge_timeout_us(30000U));
    TEST_ASSERT_EQUAL(ESP_OK, esp_timer_start_periodic(feed_timer, 1000U));

    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(true));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(false));
    (void)esp_timer_stop(feed_timer);
    ui_shell_step();

    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_get_latest_frame(&frame));
    tool_logic_get_last_window_stats(&stats);
    TEST_ASSERT_EQUAL(ESP_OK, frame.hw_error);
    TEST_ASSERT_FALSE(frame.no_capture);
    TEST_ASSERT_GREATER_THAN_UINT32(0U, (uint32_t)frame.total_captured);
    TEST_ASSERT_EQUAL_UINT32(100000U, frame.sample_rate_hz);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(4U, frame.edge_count);
    TEST_ASSERT_TRUE(frame.decoded.pwm.valid);
    TEST_ASSERT_FLOAT_WITHIN(50.0f, 1000.0f, frame.decoded.pwm.freq_hz);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 50.0f, frame.decoded.pwm.duty_pct);

    TEST_ASSERT_EQUAL(ESP_OK, recorder_sink_get_stats(&rstats));
    TEST_ASSERT_EQUAL_UINT32(1U, rstats.frames_pushed);
    TEST_ASSERT_EQUAL_UINT32(100000U, rstats.last_sample_rate_hz);

    /* Subcase C: Same-level edge storm hits 512-invocation hard cap and terminates early */
    feed_ctx.sim_ts_us = 1000U;
    feed_ctx.flood_same_level = true;
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_edge_timeout_us(200000U));
    TEST_ASSERT_EQUAL(ESP_OK, esp_timer_start_periodic(feed_timer, 1000U));

    int64_t flood_start_us = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(true));
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_run_single_step());
    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_set_running(false));
    int64_t flood_elapsed_us = esp_timer_get_time() - flood_start_us;
    (void)esp_timer_stop(feed_timer);
    (void)esp_timer_delete(feed_timer);

    TEST_ASSERT_EQUAL(ESP_OK, tool_logic_get_latest_frame(&frame));
    tool_logic_get_last_window_stats(&stats);
    TEST_ASSERT_TRUE(stats.edge_isr_capped);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(LOGIC_EDGE_ISR_MAX_INVOCATIONS, stats.edge_isr_invocations);
    TEST_ASSERT_EQUAL_UINT32(1U, stats.edge_stored_count);
    TEST_ASSERT_TRUE(frame.no_capture);
    TEST_ASSERT_LESS_THAN_INT32(120000, (int32_t)flood_elapsed_us);

    (void)tool_logic_set_edge_timeout_us(LOGIC_EDGE_DEFAULT_TIMEOUT_US);
    (void)tool_logic_set_capture_mode(LOGIC_CAP_MODE_SAMPLED);
    (void)tool_logic_set_sample_rate(LOGIC_RATE_100KHZ);
    (void)tool_logic_set_protocol(LOGIC_PROTO_UART);
    TEST_ASSERT_EQUAL(ESP_OK, ui_shell_go_home(false));
    TEST_ASSERT_TRUE(heap_caps_check_integrity_all(true));
}
