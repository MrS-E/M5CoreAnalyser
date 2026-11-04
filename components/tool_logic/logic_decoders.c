/**
 * @file logic_decoders.c
 * @brief Pure software protocol decoders (UART 8N1, I²C, SPI, PWM/Pulse), RMT edge-to-sample
 *        expansion, digital trigger search, and viewport zoom/pan extraction for `tool_logic`.
 */

#include "tool_logic.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t logic_sample_rate_to_hz(logic_sample_rate_t rate)
{
    switch (rate) {
        case LOGIC_RATE_10KHZ:
            return 10000U;
        case LOGIC_RATE_50KHZ:
            return 50000U;
        case LOGIC_RATE_100KHZ:
            return 100000U;
        case LOGIC_RATE_250KHZ:
            return 250000U;
        case LOGIC_RATE_500KHZ:
            return 500000U;
        case LOGIC_RATE_1MHZ:
            return 1000000U;
        default:
            return 0U;
    }
}

logic_sample_rate_t logic_clamp_sample_rate(logic_capture_mode_t mode, logic_sample_rate_t rate)
{
    if (rate < 0 || rate >= LOGIC_RATE_COUNT) {
        return LOGIC_RATE_100KHZ;
    }
    if (mode == LOGIC_CAP_MODE_SAMPLED && rate > LOGIC_SAMPLED_MAX_RATE) {
        return LOGIC_SAMPLED_MAX_RATE;
    }
    return rate;
}

uint32_t logic_effective_sample_rate_hz(logic_capture_mode_t mode, logic_sample_rate_t rate)
{
    if (mode < 0 || mode >= LOGIC_CAP_MODE_COUNT || rate < 0 || rate >= LOGIC_RATE_COUNT) {
        return 0U;
    }
    logic_sample_rate_t clamped = logic_clamp_sample_rate(mode, rate);
    return logic_sample_rate_to_hz(clamped);
}

esp_err_t logic_edge_timestamps_to_rmt_symbols(const logic_edge_sample_t *edges,
                                               size_t edge_count,
                                               rmt_symbol_word_t *out_symbols,
                                               size_t max_symbols,
                                               size_t *out_symbol_count)
{
    if (out_symbols == NULL || max_symbols == 0U || out_symbol_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_symbol_count = 0U;
    if (edges == NULL || edge_count < 2U) {
        return ESP_OK;
    }

    bool have_anchor = false;
    uint32_t anchor_ts = 0U;
    uint8_t anchor_lvl = 0U;
    size_t sym_idx = 0U;
    bool half_filled = false;

    for (size_t i = 0; i < edge_count && sym_idx < max_symbols; ++i) {
        uint8_t lvl = edges[i].level & 1U;
        uint32_t ts = edges[i].timestamp_us;

        if (!have_anchor) {
            anchor_ts = ts;
            anchor_lvl = lvl;
            have_anchor = true;
            continue;
        }
        if (lvl == anchor_lvl) {
            /* Ignore duplicate same-level bounce */
            continue;
        }
        if (ts <= anchor_ts) {
            /* Zero-duration sub-microsecond glitch: update level without emitting 0-tick pulse */
            anchor_lvl = lvl;
            continue;
        }

        uint32_t dt = ts - anchor_ts;
        while (dt > 0U && sym_idx < max_symbols) {
            uint16_t chunk = (dt > 32767U) ? 32767U : (uint16_t)dt;
            if (!half_filled) {
                out_symbols[sym_idx].duration0 = chunk;
                out_symbols[sym_idx].level0 = anchor_lvl;
                out_symbols[sym_idx].duration1 = 0U;
                out_symbols[sym_idx].level1 = 0U;
                half_filled = true;
            } else {
                out_symbols[sym_idx].duration1 = chunk;
                out_symbols[sym_idx].level1 = anchor_lvl;
                sym_idx++;
                half_filled = false;
            }
            dt -= (uint32_t)chunk;
        }

        anchor_ts = ts;
        anchor_lvl = lvl;
    }

    if (half_filled && sym_idx < max_symbols) {
        sym_idx++;
    }

    *out_symbol_count = sym_idx;
    return ESP_OK;
}

const char *logic_sample_rate_to_str(logic_sample_rate_t rate)
{
    switch (rate) {
        case LOGIC_RATE_10KHZ:
            return "10kHz";
        case LOGIC_RATE_50KHZ:
            return "50kHz";
        case LOGIC_RATE_100KHZ:
            return "100kHz";
        case LOGIC_RATE_250KHZ:
            return "250kHz";
        case LOGIC_RATE_500KHZ:
            return "500kHz";
        case LOGIC_RATE_1MHZ:
            return "1MHz";
        default:
            return "?";
    }
}

const char *logic_protocol_to_str(logic_protocol_t proto)
{
    switch (proto) {
        case LOGIC_PROTO_UART:
            return "UART";
        case LOGIC_PROTO_I2C:
            return "I2C";
        case LOGIC_PROTO_SPI:
            return "SPI";
        case LOGIC_PROTO_PWM:
            return "PWM";
        default:
            return "?";
    }
}

uint32_t logic_zoom_factor(logic_zoom_t zoom)
{
    switch (zoom) {
        case LOGIC_ZOOM_1X:
            return 1U;
        case LOGIC_ZOOM_2X:
            return 2U;
        case LOGIC_ZOOM_4X:
            return 4U;
        case LOGIC_ZOOM_8X:
            return 8U;
        default:
            return 1U;
    }
}

esp_err_t logic_expand_rmt_to_samples(const rmt_symbol_word_t *symbols,
                                      size_t num_symbols,
                                      uint32_t rmt_resolution_hz,
                                      uint32_t target_sample_rate_hz,
                                      uint8_t channel,
                                      uint8_t *out_samples,
                                      size_t max_samples,
                                      size_t *out_written)
{
    if (out_samples == NULL || max_samples == 0U || rmt_resolution_hz == 0U ||
        target_sample_rate_hz == 0U || channel >= LOGIC_CHANNEL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t pos = 0U;
    uint8_t bit_mask = (uint8_t)(1U << channel);

    if (symbols != NULL && num_symbols > 0U) {
        for (size_t i = 0; i < num_symbols && pos < max_samples; ++i) {
            uint16_t d[2] = {symbols[i].duration0, symbols[i].duration1};
            uint16_t l[2] = {symbols[i].level0, symbols[i].level1};

            for (int part = 0; part < 2 && pos < max_samples; ++part) {
                if (d[part] == 0U) {
                    i = num_symbols;
                    break;
                }
                double scaled = ((double)d[part] * (double)target_sample_rate_hz) /
                                (double)rmt_resolution_hz;
                size_t count = (size_t)llround(scaled);
                if (count == 0U) {
                    count = 1U;
                }
                for (size_t k = 0; k < count && pos < max_samples; ++k) {
                    if (l[part] != 0U) {
                        out_samples[pos] |= bit_mask;
                    } else {
                        out_samples[pos] &= (uint8_t)(~bit_mask);
                    }
                    pos++;
                }
            }
        }
    }

    if (out_written != NULL) {
        *out_written = pos;
    }
    return ESP_OK;
}

esp_err_t logic_find_digital_trigger(const uint8_t *samples,
                                     size_t sample_count,
                                     uint8_t channel,
                                     logic_trig_mode_t trig_mode,
                                     bool *out_triggered,
                                     size_t *out_trigger_index)
{
    if (samples == NULL || sample_count == 0U || channel >= LOGIC_CHANNEL_COUNT ||
        out_triggered == NULL || out_trigger_index == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_triggered = false;
    *out_trigger_index = 0U;

    uint8_t mask = (uint8_t)(1U << channel);
    for (size_t i = 1U; i < sample_count; ++i) {
        bool prev_hi = (samples[i - 1U] & mask) != 0U;
        bool cur_hi = (samples[i] & mask) != 0U;
        if (trig_mode == LOGIC_TRIG_RISING && !prev_hi && cur_hi) {
            *out_triggered = true;
            *out_trigger_index = i;
            return ESP_OK;
        }
        if (trig_mode == LOGIC_TRIG_FALLING && prev_hi && !cur_hi) {
            *out_triggered = true;
            *out_trigger_index = i;
            return ESP_OK;
        }
        if (trig_mode == LOGIC_TRIG_AUTO && (prev_hi != cur_hi)) {
            *out_triggered = true;
            *out_trigger_index = i;
            return ESP_OK;
        }
    }

    return ESP_OK;
}

esp_err_t logic_extract_viewport(const uint8_t *samples,
                                 size_t sample_count,
                                 logic_zoom_t zoom,
                                 size_t pan_offset,
                                 uint8_t *out_view,
                                 size_t view_points)
{
    if (samples == NULL || sample_count == 0U || out_view == NULL || view_points == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t zf = logic_zoom_factor(zoom);
    if (pan_offset >= sample_count) {
        pan_offset = sample_count - 1U;
    }

    for (size_t i = 0; i < view_points; ++i) {
        size_t src_idx = pan_offset + (i / (size_t)zf);
        if (src_idx >= sample_count) {
            src_idx = sample_count - 1U;
        }
        out_view[i] = samples[src_idx];
    }
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * 1. UART 8N1 Protocol Decoder
 * -------------------------------------------------------------------------- */

static inline uint8_t uart_bit_at(const uint8_t *samples,
                                  size_t idx,
                                  uint8_t rx_channel,
                                  bool invert)
{
    uint8_t raw = (samples[idx] >> rx_channel) & 1U;
    return invert ? (uint8_t)(raw ^ 1U) : raw;
}

esp_err_t logic_decode_uart(const uint8_t *samples,
                            size_t sample_count,
                            uint32_t sample_rate_hz,
                            uint8_t rx_channel,
                            uint32_t baud_rate,
                            bool invert,
                            logic_uart_result_t *out_result)
{
    if (samples == NULL || out_result == NULL || sample_count < 10U ||
        rx_channel >= LOGIC_CHANNEL_COUNT || baud_rate == 0U ||
        sample_rate_hz < baud_rate * 3U) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_result, 0, sizeof(*out_result));
    out_result->baud_rate = baud_rate;

    const double bit_samples = (double)sample_rate_hz / (double)baud_rate;
    size_t ascii_pos = 0U;

    size_t i = 1U;
    while (i < sample_count && out_result->frame_count < LOGIC_MAX_DECODED_ITEMS) {
        uint8_t prev_lvl = uart_bit_at(samples, i - 1U, rx_channel, invert);
        uint8_t cur_lvl = uart_bit_at(samples, i, rx_channel, invert);

        /* Detect Start-bit falling edge (Idle High 1 -> Start Low 0) */
        if (prev_lvl == 1U && cur_lvl == 0U) {
            size_t start_mid = (size_t)llround((double)i + 0.5 * bit_samples);
            size_t stop_mid = (size_t)llround((double)i + 9.5 * bit_samples);
            if (stop_mid >= sample_count) {
                break;
            }

            /* Verify start bit is still Low at bit center to reject single-sample glitches */
            if (uart_bit_at(samples, start_mid, rx_channel, invert) != 0U) {
                i++;
                continue;
            }

            uint8_t byte_val = 0U;
            for (uint8_t b = 0U; b < 8U; ++b) {
                size_t bit_idx = (size_t)llround((double)i + (1.5 + (double)b) * bit_samples);
                if (bit_idx >= sample_count) {
                    bit_idx = sample_count - 1U;
                }
                uint8_t bval = uart_bit_at(samples, bit_idx, rx_channel, invert);
                byte_val |= (uint8_t)(bval << b);
            }

            uint8_t stop_lvl = uart_bit_at(samples, stop_mid, rx_channel, invert);
            bool framing_err = (stop_lvl == 0U);

            logic_uart_frame_t *frm = &out_result->frames[out_result->frame_count++];
            frm->data_byte = byte_val;
            frm->framing_error = framing_err;
            frm->start_sample = i;
            frm->end_sample = stop_mid;

            if (framing_err) {
                out_result->error_count++;
                /* Advance until line returns to idle High (1) before hunting next start bit */
                size_t k = stop_mid + 1U;
                while (k < sample_count && uart_bit_at(samples, k, rx_channel, invert) == 0U) {
                    k++;
                }
                i = k + 1U;
            } else {
                if (ascii_pos + 1U < LOGIC_UART_ASCII_MAX) {
                    char ch = (byte_val >= 32U && byte_val <= 126U) ? (char)byte_val : '.';
                    out_result->ascii_str[ascii_pos++] = ch;
                    out_result->ascii_str[ascii_pos] = '\0';
                }
                i = stop_mid + 1U;
            }
            continue;
        }
        i++;
    }

    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * 2. I²C Bus Protocol Decoder
 * -------------------------------------------------------------------------- */

static void push_i2c_event(logic_i2c_result_t *res,
                           logic_i2c_event_type_t type,
                           uint8_t addr_7bit,
                           bool is_read,
                           uint8_t data_byte,
                           bool ack,
                           size_t sample_idx)
{
    if (res->event_count >= LOGIC_MAX_DECODED_ITEMS) {
        return;
    }
    logic_i2c_event_t *ev = &res->events[res->event_count++];
    ev->type = type;
    ev->addr_7bit = addr_7bit;
    ev->is_read = is_read;
    ev->data_byte = data_byte;
    ev->ack = ack;
    ev->sample_index = sample_idx;
}

esp_err_t logic_decode_i2c(const uint8_t *samples,
                           size_t sample_count,
                           uint8_t scl_channel,
                           uint8_t sda_channel,
                           logic_i2c_result_t *out_result)
{
    if (samples == NULL || out_result == NULL || sample_count < 2U ||
        scl_channel >= LOGIC_CHANNEL_COUNT || sda_channel >= LOGIC_CHANNEL_COUNT ||
        scl_channel == sda_channel) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_result, 0, sizeof(*out_result));

    bool in_transaction = false;
    bool expecting_addr = false;
    uint8_t bit_count = 0U;
    uint8_t shift_reg = 0U;

    for (size_t i = 1U; i < sample_count; ++i) {
        uint8_t prev_scl = (samples[i - 1U] >> scl_channel) & 1U;
        uint8_t scl = (samples[i] >> scl_channel) & 1U;
        uint8_t prev_sda = (samples[i - 1U] >> sda_channel) & 1U;
        uint8_t sda = (samples[i] >> sda_channel) & 1U;

        /* SDA transition while SCL is stable High -> START / RESTART / STOP */
        if (prev_scl == 1U && scl == 1U && prev_sda != sda) {
            if (prev_sda == 1U && sda == 0U) {
                /* START or Repeated START */
                logic_i2c_event_type_t etype = in_transaction
                                                   ? LOGIC_I2C_EVENT_RESTART
                                                   : LOGIC_I2C_EVENT_START;
                push_i2c_event(out_result, etype, 0U, false, 0U, false, i);
                out_result->transaction_count++;
                in_transaction = true;
                expecting_addr = true;
                bit_count = 0U;
                shift_reg = 0U;
            } else {
                /* STOP (SDA 0 -> 1 while SCL High) */
                push_i2c_event(out_result, LOGIC_I2C_EVENT_STOP, 0U, false, 0U, false, i);
                in_transaction = false;
                expecting_addr = false;
                bit_count = 0U;
                shift_reg = 0U;
            }
            continue;
        }

        /* Sample SDA on rising edge of SCL during an active transaction */
        if (in_transaction && prev_scl == 0U && scl == 1U) {
            if (bit_count < 8U) {
                shift_reg = (uint8_t)((shift_reg << 1U) | sda);
                bit_count++;
            } else {
                /* 9th SCL rising edge: ACK (SDA == 0) or NACK (SDA == 1) */
                bool ack = (sda == 0U);
                if (ack) {
                    out_result->ack_count++;
                } else {
                    out_result->nack_count++;
                }

                if (expecting_addr) {
                    uint8_t addr7 = (shift_reg >> 1U) & 0x7FU;
                    bool is_read = (shift_reg & 0x01U) != 0U;
                    out_result->last_addr_7bit = addr7;
                    push_i2c_event(
                        out_result, LOGIC_I2C_EVENT_ADDR, addr7, is_read, shift_reg, ack, i);
                    expecting_addr = false;
                } else {
                    push_i2c_event(
                        out_result, LOGIC_I2C_EVENT_DATA, out_result->last_addr_7bit, false, shift_reg, ack, i);
                }
                bit_count = 0U;
                shift_reg = 0U;
            }
        }
    }

    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * 3. SPI 4-Wire Protocol Decoder
 * -------------------------------------------------------------------------- */

esp_err_t logic_decode_spi(const uint8_t *samples,
                           size_t sample_count,
                           uint8_t cs_channel,
                           uint8_t sck_channel,
                           uint8_t mosi_channel,
                           uint8_t miso_channel,
                           uint8_t cpol,
                           uint8_t cpha,
                           bool cs_active_low,
                           logic_spi_result_t *out_result)
{
    if (samples == NULL || out_result == NULL || sample_count < 2U ||
        cs_channel >= LOGIC_CHANNEL_COUNT || sck_channel >= LOGIC_CHANNEL_COUNT ||
        mosi_channel >= LOGIC_CHANNEL_COUNT || miso_channel >= LOGIC_CHANNEL_COUNT ||
        cpol > 1U || cpha > 1U) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_result, 0, sizeof(*out_result));
    out_result->cpol = cpol;
    out_result->cpha = cpha;

    const bool sample_on_rising = ((cpol ^ cpha) == 0U);
    const uint8_t active_cs_lvl = cs_active_low ? 0U : 1U;

    uint8_t bit_count = 0U;
    uint8_t mosi_reg = 0U;
    uint8_t miso_reg = 0U;
    size_t word_start = 0U;

    for (size_t i = 1U; i < sample_count && out_result->word_count < LOGIC_MAX_DECODED_ITEMS; ++i) {
        uint8_t prev_cs = (samples[i - 1U] >> cs_channel) & 1U;
        uint8_t cs = (samples[i] >> cs_channel) & 1U;
        if (cs != active_cs_lvl || prev_cs != active_cs_lvl) {
            bit_count = 0U;
            mosi_reg = 0U;
            miso_reg = 0U;
            continue;
        }

        uint8_t prev_sck = (samples[i - 1U] >> sck_channel) & 1U;
        uint8_t sck = (samples[i] >> sck_channel) & 1U;
        bool active_edge = sample_on_rising ? (prev_sck == 0U && sck == 1U)
                                            : (prev_sck == 1U && sck == 0U);
        if (!active_edge) {
            continue;
        }

        uint8_t mosi = (samples[i] >> mosi_channel) & 1U;
        uint8_t miso = (samples[i] >> miso_channel) & 1U;
        if (bit_count == 0U) {
            word_start = i;
        }
        mosi_reg = (uint8_t)((mosi_reg << 1U) | mosi);
        miso_reg = (uint8_t)((miso_reg << 1U) | miso);
        bit_count++;

        if (bit_count == 8U) {
            logic_spi_word_t *w = &out_result->words[out_result->word_count++];
            w->mosi_byte = mosi_reg;
            w->miso_byte = miso_reg;
            w->start_sample = word_start;
            w->end_sample = i;
            bit_count = 0U;
            mosi_reg = 0U;
            miso_reg = 0U;
        }
    }

    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * 4. PWM / Pulse Protocol Decoder
 * -------------------------------------------------------------------------- */

esp_err_t logic_decode_pwm(const uint8_t *samples,
                           size_t sample_count,
                           uint32_t sample_rate_hz,
                           uint8_t channel,
                           logic_pwm_result_t *out_result)
{
    if (samples == NULL || out_result == NULL || sample_count < 2U ||
        sample_rate_hz == 0U || channel >= LOGIC_CHANNEL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_result, 0, sizeof(*out_result));

    bool have_first_rising = false;
    bool have_falling_after_rising = false;
    size_t last_rising_idx = 0U;
    size_t last_falling_idx = 0U;

    uint64_t sum_period_samples = 0ULL;
    uint64_t sum_high_samples = 0ULL;
    uint32_t cycles = 0U;

    for (size_t i = 1U; i < sample_count; ++i) {
        uint8_t prev = (samples[i - 1U] >> channel) & 1U;
        uint8_t cur = (samples[i] >> channel) & 1U;

        if (prev == 0U && cur == 1U) {
            out_result->rising_edges++;
            if (have_first_rising && have_falling_after_rising && i > last_rising_idx) {
                size_t period_s = i - last_rising_idx;
                size_t high_s = last_falling_idx - last_rising_idx;
                sum_period_samples += (uint64_t)period_s;
                sum_high_samples += (uint64_t)high_s;
                cycles++;
            }
            last_rising_idx = i;
            have_first_rising = true;
            have_falling_after_rising = false;
        } else if (prev == 1U && cur == 0U) {
            out_result->falling_edges++;
            if (have_first_rising && !have_falling_after_rising) {
                last_falling_idx = i;
                have_falling_after_rising = true;
            }
        }
    }

    out_result->complete_cycles = cycles;
    if (cycles == 0U || sum_period_samples == 0ULL) {
        return ESP_OK;
    }

    double avg_period_samples = (double)sum_period_samples / (double)cycles;
    double avg_high_samples = (double)sum_high_samples / (double)cycles;
    double period_us = (avg_period_samples * 1000000.0) / (double)sample_rate_hz;
    double high_us = (avg_high_samples * 1000000.0) / (double)sample_rate_hz;
    double low_us = period_us - high_us;
    if (low_us < 0.0) {
        low_us = 0.0;
    }
    double freq_hz = (period_us > 0.0) ? (1000000.0 / period_us) : 0.0;
    double duty_pct = (period_us > 0.0) ? ((high_us * 100.0) / period_us) : 0.0;

    out_result->valid = true;
    out_result->freq_hz = (float)freq_hz;
    out_result->period_us = (float)period_us;
    out_result->high_width_us = (float)high_us;
    out_result->low_width_us = (float)low_us;
    out_result->duty_pct = (float)duty_pct;
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Unified Protocol Decode Table Builder
 * -------------------------------------------------------------------------- */

esp_err_t logic_decode_run_protocol(const uint8_t *samples,
                                    size_t sample_count,
                                    uint32_t sample_rate_hz,
                                    logic_protocol_t protocol,
                                    uint32_t uart_baud,
                                    logic_decode_table_t *out_table)
{
    if (samples == NULL || out_table == NULL || sample_count == 0U || sample_rate_hz == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_table, 0, sizeof(*out_table));
    out_table->protocol = protocol;

    if (protocol == LOGIC_PROTO_UART) {
        uint32_t baud = (uart_baud > 0U) ? uart_baud : 9600U;
        if (sample_rate_hz < baud * 3U) {
            (void)snprintf(out_table->summary,
                           sizeof(out_table->summary),
                           "UART %lubps: Rate too low (<3x)",
                           (unsigned long)baud);
            return ESP_OK;
        }
        (void)logic_decode_uart(
            samples, sample_count, sample_rate_hz, 0U, baud, false, &out_table->uart);
        (void)snprintf(out_table->summary,
                       sizeof(out_table->summary),
                       "UART %lu 8N1: %uB err:%u \"%.16s\"",
                       (unsigned long)baud,
                       (unsigned)out_table->uart.frame_count,
                       (unsigned)out_table->uart.error_count,
                       out_table->uart.ascii_str);

        for (size_t i = 0; i < out_table->uart.frame_count && out_table->row_count < LOGIC_MAX_DECODED_ITEMS; ++i) {
            const logic_uart_frame_t *f = &out_table->uart.frames[i];
            logic_decoded_row_t *r = &out_table->rows[out_table->row_count++];
            r->sample_index = f->start_sample;
            char ch = (f->data_byte >= 32U && f->data_byte <= 126U) ? (char)f->data_byte : '.';
            if (f->framing_error) {
                (void)snprintf(r->text,
                               sizeof(r->text),
                               "@%u 0x%02X '%c' ERR:FRAME",
                               (unsigned)f->start_sample,
                               (unsigned)f->data_byte,
                               ch);
            } else {
                (void)snprintf(r->text,
                               sizeof(r->text),
                               "@%u 0x%02X '%c' OK",
                               (unsigned)f->start_sample,
                               (unsigned)f->data_byte,
                               ch);
            }
        }
        return ESP_OK;
    }

    if (protocol == LOGIC_PROTO_I2C) {
        /* Default I2C mapping: CH0 = SDA, CH1 = SCL */
        (void)logic_decode_i2c(samples, sample_count, 1U, 0U, &out_table->i2c);
        (void)snprintf(out_table->summary,
                       sizeof(out_table->summary),
                       "I2C(C0:SDA C1:SCL): %u ev ACK:%u NACK:%u",
                       (unsigned)out_table->i2c.event_count,
                       (unsigned)out_table->i2c.ack_count,
                       (unsigned)out_table->i2c.nack_count);

        for (size_t i = 0; i < out_table->i2c.event_count && out_table->row_count < LOGIC_MAX_DECODED_ITEMS; ++i) {
            const logic_i2c_event_t *ev = &out_table->i2c.events[i];
            logic_decoded_row_t *r = &out_table->rows[out_table->row_count++];
            r->sample_index = ev->sample_index;
            if (ev->type == LOGIC_I2C_EVENT_START) {
                (void)snprintf(r->text, sizeof(r->text), "@%u START", (unsigned)ev->sample_index);
            } else if (ev->type == LOGIC_I2C_EVENT_RESTART) {
                (void)snprintf(r->text, sizeof(r->text), "@%u RESTART", (unsigned)ev->sample_index);
            } else if (ev->type == LOGIC_I2C_EVENT_STOP) {
                (void)snprintf(r->text, sizeof(r->text), "@%u STOP", (unsigned)ev->sample_index);
            } else if (ev->type == LOGIC_I2C_EVENT_ADDR) {
                (void)snprintf(r->text,
                               sizeof(r->text),
                               "@%u ADDR 0x%02X %s %s",
                               (unsigned)ev->sample_index,
                               (unsigned)ev->addr_7bit,
                               ev->is_read ? "R" : "W",
                               ev->ack ? "ACK" : "NACK");
            } else {
                (void)snprintf(r->text,
                               sizeof(r->text),
                               "@%u DATA 0x%02X %s",
                               (unsigned)ev->sample_index,
                               (unsigned)ev->data_byte,
                               ev->ack ? "ACK" : "NACK");
            }
        }
        return ESP_OK;
    }

    if (protocol == LOGIC_PROTO_SPI) {
        /* Default SPI mapping: CH0 = CS, CH1 = SCK, CH2 = MOSI, CH3 = MISO (Mode 0, active-low CS) */
        (void)logic_decode_spi(samples, sample_count, 0U, 1U, 2U, 3U, 0U, 0U, true, &out_table->spi);
        (void)snprintf(out_table->summary,
                       sizeof(out_table->summary),
                       "SPI M0(CS:0 CK:1 MO:2 MI:3): %u words",
                       (unsigned)out_table->spi.word_count);

        for (size_t i = 0; i < out_table->spi.word_count && out_table->row_count < LOGIC_MAX_DECODED_ITEMS; ++i) {
            const logic_spi_word_t *w = &out_table->spi.words[i];
            logic_decoded_row_t *r = &out_table->rows[out_table->row_count++];
            r->sample_index = w->start_sample;
            (void)snprintf(r->text,
                           sizeof(r->text),
                           "@%u #%u MOSI:0x%02X MISO:0x%02X",
                           (unsigned)w->start_sample,
                           (unsigned)i,
                           (unsigned)w->mosi_byte,
                           (unsigned)w->miso_byte);
        }
        return ESP_OK;
    }

    /* LOGIC_PROTO_PWM on CH0 */
    (void)logic_decode_pwm(samples, sample_count, sample_rate_hz, 0U, &out_table->pwm);
    if (out_table->pwm.valid) {
        long f_tenths = lroundf(out_table->pwm.freq_hz * 10.0f);
        long d_tenths = lroundf(out_table->pwm.duty_pct * 10.0f);
        long hi_tenths = lroundf(out_table->pwm.high_width_us * 10.0f);
        long lo_tenths = lroundf(out_table->pwm.low_width_us * 10.0f);
        (void)snprintf(out_table->summary,
                       sizeof(out_table->summary),
                       "PWM CH0: %ld.%ldHz Duty:%ld.%ld%% (%u cyc)",
                       f_tenths / 10L, labs(f_tenths % 10L),
                       d_tenths / 10L, labs(d_tenths % 10L),
                       (unsigned)out_table->pwm.complete_cycles);

        logic_decoded_row_t *r0 = &out_table->rows[out_table->row_count++];
        r0->sample_index = 0U;
        (void)snprintf(r0->text,
                       sizeof(r0->text),
                       "PW+:%ld.%ldus PW-:%ld.%ldus R:%u F:%u",
                       hi_tenths / 10L, labs(hi_tenths % 10L),
                       lo_tenths / 10L, labs(lo_tenths % 10L),
                       (unsigned)out_table->pwm.rising_edges,
                       (unsigned)out_table->pwm.falling_edges);
    } else {
        (void)snprintf(out_table->summary,
                       sizeof(out_table->summary),
                       "PWM CH0: No complete cycle (R:%u F:%u)",
                       (unsigned)out_table->pwm.rising_edges,
                       (unsigned)out_table->pwm.falling_edges);
    }

    return ESP_OK;
}
