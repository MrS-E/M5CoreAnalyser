/**
 * @file tool_logic.h
 * @brief Multi-Channel Logic Analyzer & Protocol Decoder Suite (`tool_logic`) for M5CoreAnalyser.
 *
 * Supports dual capture modes:
 * 1. **Sampled Logic Mode (`LOGIC_CAP_MODE_SAMPLED`)**: Timer/ISR-driven multi-pin GPIO sampling
 *    (`GPIO.in` / `GPIO.in1.val`) via an internal SRAM bounce buffer into a 64 KB PSRAM capture buffer.
 * 2. **Edge-Timestamp Mode (`LOGIC_CAP_MODE_RMT_EDGE`)**: Hardware pulse duration capture via ESP32
 *    RMT RX into internal SRAM bounce memory, staged and expanded in PSRAM.
 *
 * Includes deterministic software protocol decoders for **UART** (8N1 ASCII/Hex + framing errors),
 * **I²C** (`START`/`RESTART`/`STOP`, 7-bit address, `R/W`, `ACK`/`NACK`, data bytes),
 * **SPI** (`CS` framing, `SCK` CPOL/CPHA edge sampling of `MOSI`/`MISO`), and **PWM/Pulse**
 * (frequency, period, high/low pulse widths, duty cycle).
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/rmt_types.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "tool_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name Logic Analyzer Constants & Capacities
 *  @{
 */
#define LOGIC_TOOL_ID                   "logic"
#define LOGIC_CHANNEL_COUNT             (4U)        /**< 4 simultaneous logic analyzer channels (CH0..CH3) */
#define LOGIC_VIEW_POINTS               (120U)      /**< Points per channel displayed in zoomed waveform viewport */
#define LOGIC_MAX_DECODED_ITEMS         (16U)       /**< Maximum decoded protocol items per frame */
#define LOGIC_DECODED_TEXT_MAX          (48U)       /**< Max characters per decoded packet table row */
#define LOGIC_UART_ASCII_MAX            (33U)       /**< Max assembled ASCII string length for UART decoder */
#define LOGIC_ISR_BOUNCE_SAMPLES        (1024U)     /**< Internal SRAM ring bounce buffer size for GPTimer ISR (power of 2) */
#define LOGIC_RMT_BOUNCE_SYMBOLS        (128U)      /**< Internal SRAM bounce buffer symbols for RMT RX ISR */
#define LOGIC_EDGE_BOUNCE_MAX           (256U)      /**< Max GPIO edge transitions in internal SRAM per edge window */
#define LOGIC_EDGE_ISR_MAX_INVOCATIONS  (512U)      /**< Hard cap on GPIO edge ISR invocations per window */
#define LOGIC_RMT_RESOLUTION_HZ         (1000000U)  /**< 1 MHz (1 us/tick) RMT RX resolution */
#define LOGIC_SAMPLED_MAX_RATE_HZ       (100000U)   /**< Max sustainable GPTimer ISR sampling rate (100 kHz = 10 us) */
#define LOGIC_EDGE_DEFAULT_TIMEOUT_US   (25000U)    /**< Default edge capture window timeout in microseconds (25 ms) */
#define LOGIC_EDGE_MIN_TIMEOUT_US       (5000U)     /**< Minimum configurable edge capture timeout (5 ms) */
#define LOGIC_EDGE_MAX_TIMEOUT_US       (500000U)   /**< Maximum configurable edge capture timeout (500 ms) */

#if CONFIG_SPIRAM
#define LOGIC_PSRAM_BUFFER_BYTES        (65536U)    /**< Full 64 KB PSRAM sample buffer on M5Stack Core2 */
#else
#define LOGIC_PSRAM_BUFFER_BYTES        (4096U)     /**< Compact sample buffer for PSRAM-less QEMU */
#endif

/** Default active capture window length per frame (up to `LOGIC_PSRAM_BUFFER_BYTES`). */
#define LOGIC_DEFAULT_CAPTURE_SAMPLES   (1024U)
/** @} */

/**
 * @brief Logic Analyzer capture modes.
 */
typedef enum {
    LOGIC_CAP_MODE_SAMPLED = 0, /**< Periodic multi-channel GPIO sampling into PSRAM */
    LOGIC_CAP_MODE_RMT_EDGE,    /**< RMT RX hardware edge-timestamp capture expanded into PSRAM */
    LOGIC_CAP_MODE_COUNT
} logic_capture_mode_t;

/**
 * @brief Selectable periodic sampling rates for `LOGIC_CAP_MODE_SAMPLED`.
 */
typedef enum {
    LOGIC_RATE_10KHZ = 0,
    LOGIC_RATE_50KHZ,
    LOGIC_RATE_100KHZ,
    LOGIC_RATE_250KHZ,
    LOGIC_RATE_500KHZ,
    LOGIC_RATE_1MHZ,
    LOGIC_RATE_COUNT
} logic_sample_rate_t;

/** Maximum sustainable periodic GPTimer ISR preset in `LOGIC_CAP_MODE_SAMPLED`. */
#define LOGIC_SAMPLED_MAX_RATE          LOGIC_RATE_100KHZ

/**
 * @brief Single GPIO edge transition timestamp recorded in internal SRAM (`DRAM_ATTR`).
 */
typedef struct {
    uint32_t timestamp_us;      /**< Timestamp in microseconds (`esp_timer_get_time()`) */
    uint8_t level;              /**< Digital logic level (`0` or `1`) AFTER the edge transition */
} logic_edge_sample_t;

/**
 * @brief Selectable protocol decoders.
 */
typedef enum {
    LOGIC_PROTO_UART = 0,       /**< UART 8N1 asynchronous serial decoder */
    LOGIC_PROTO_I2C,            /**< I2C 2-wire bus decoder (SCL + SDA) */
    LOGIC_PROTO_SPI,            /**< SPI 4-wire bus decoder (CS + SCK + MOSI + MISO) */
    LOGIC_PROTO_PWM,            /**< PWM / Pulse timing decoder */
    LOGIC_PROTO_COUNT
} logic_protocol_t;

/**
 * @brief Trigger condition for starting or aligning logic capture.
 */
typedef enum {
    LOGIC_TRIG_AUTO = 0,        /**< Trigger on any transition on CH0 with automatic fallback */
    LOGIC_TRIG_RISING,          /**< Trigger on rising edge (0 -> 1) on CH0 */
    LOGIC_TRIG_FALLING,         /**< Trigger on falling edge (1 -> 0) on CH0 */
    LOGIC_TRIG_COUNT
} logic_trig_mode_t;

/**
 * @brief Horizontal zoom factors for the multi-lane waveform display.
 */
typedef enum {
    LOGIC_ZOOM_1X = 0,
    LOGIC_ZOOM_2X,
    LOGIC_ZOOM_4X,
    LOGIC_ZOOM_8X,
    LOGIC_ZOOM_COUNT
} logic_zoom_t;

/* --------------------------------------------------------------------------
 * Protocol Decoder Data Structures (`logic_decoders.c`)
 * -------------------------------------------------------------------------- */

/**
 * @brief Single decoded UART 8N1 byte frame.
 */
typedef struct {
    uint8_t data_byte;          /**< Decoded 8-bit LSB-first data value */
    bool framing_error;         /**< True if stop bit was Low (0) instead of High (1) */
    size_t start_sample;        /**< Sample index of start-bit falling edge */
    size_t end_sample;          /**< Sample index of stop-bit center */
} logic_uart_frame_t;

/**
 * @brief Complete UART 8N1 decode result.
 */
typedef struct {
    logic_uart_frame_t frames[LOGIC_MAX_DECODED_ITEMS];
    size_t frame_count;         /**< Total UART frames decoded (valid + framing errors) */
    size_t error_count;         /**< Count of frames with `framing_error == true` */
    uint32_t baud_rate;         /**< Configured baud rate used for decoding */
    char ascii_str[LOGIC_UART_ASCII_MAX]; /**< Printable ASCII summary of valid decoded bytes */
} logic_uart_result_t;

/**
 * @brief Types of decoded I²C bus events.
 */
typedef enum {
    LOGIC_I2C_EVENT_START = 0,  /**< START condition (SDA falling while SCL high) */
    LOGIC_I2C_EVENT_RESTART,    /**< Repeated START condition */
    LOGIC_I2C_EVENT_ADDR,       /**< 7-bit slave address + R/W bit + ACK/NACK */
    LOGIC_I2C_EVENT_DATA,       /**< 8-bit data byte + ACK/NACK */
    LOGIC_I2C_EVENT_STOP,       /**< STOP condition (SDA rising while SCL high) */
} logic_i2c_event_type_t;

/**
 * @brief Single decoded I²C bus event.
 */
typedef struct {
    logic_i2c_event_type_t type;/**< Event type */
    uint8_t addr_7bit;          /**< 7-bit slave address (valid for `LOGIC_I2C_EVENT_ADDR`) */
    bool is_read;               /**< True for Read (`R/W = 1`), false for Write (`R/W = 0`) */
    uint8_t data_byte;          /**< 8-bit payload byte (valid for `LOGIC_I2C_EVENT_DATA`) */
    bool ack;                   /**< True if ACK (`SDA = 0` on 9th clock), false if NACK (`SDA = 1`) */
    size_t sample_index;        /**< Sample index where the event occurred */
} logic_i2c_event_t;

/**
 * @brief Complete I²C decode result.
 */
typedef struct {
    logic_i2c_event_t events[LOGIC_MAX_DECODED_ITEMS];
    size_t event_count;         /**< Total events recorded in `events` */
    size_t transaction_count;   /**< Count of START conditions observed */
    size_t ack_count;           /**< Count of ACKed address/data bytes */
    size_t nack_count;          /**< Count of NACKed address/data bytes */
    uint8_t last_addr_7bit;     /**< Most recent 7-bit slave address decoded */
} logic_i2c_result_t;

/**
 * @brief Single decoded SPI 8-bit full-duplex transfer word.
 */
typedef struct {
    uint8_t mosi_byte;          /**< 8-bit byte sampled on MOSI */
    uint8_t miso_byte;          /**< 8-bit byte sampled on MISO */
    size_t start_sample;        /**< Sample index of first SCK active edge */
    size_t end_sample;          /**< Sample index of 8th SCK active edge */
} logic_spi_word_t;

/**
 * @brief Complete SPI decode result.
 */
typedef struct {
    logic_spi_word_t words[LOGIC_MAX_DECODED_ITEMS];
    size_t word_count;          /**< Total 8-bit SPI words decoded while CS active */
    uint8_t cpol;               /**< Clock polarity used (`0` or `1`) */
    uint8_t cpha;               /**< Clock phase used (`0` or `1`) */
} logic_spi_result_t;

/**
 * @brief Decoded PWM / Pulse timing result.
 */
typedef struct {
    bool valid;                 /**< True if at least one complete period was measured */
    float freq_hz;              /**< Measured PWM frequency in Hz */
    float period_us;            /**< Average period in microseconds */
    float high_width_us;        /**< Average high pulse width in microseconds */
    float low_width_us;         /**< Average low pulse width in microseconds */
    float duty_pct;             /**< Positive duty cycle in percent (`0..100`) */
    uint32_t rising_edges;      /**< Total rising edges counted in the buffer */
    uint32_t falling_edges;     /**< Total falling edges counted in the buffer */
    uint32_t complete_cycles;   /**< Number of complete cycles averaged */
} logic_pwm_result_t;

/**
 * @brief Single formatted entry for the LVGL decoded packet table.
 */
typedef struct {
    size_t sample_index;
    char text[LOGIC_DECODED_TEXT_MAX];
} logic_decoded_row_t;

/**
 * @brief Unified protocol decode table and summary for the UI.
 */
typedef struct {
    logic_protocol_t protocol;
    char summary[80];
    logic_decoded_row_t rows[LOGIC_MAX_DECODED_ITEMS];
    size_t row_count;
    logic_uart_result_t uart;
    logic_i2c_result_t i2c;
    logic_spi_result_t spi;
    logic_pwm_result_t pwm;
} logic_decode_table_t;

/**
 * @brief Complete Logic Analyzer UI & decoder frame transferred from Core 1 to Core 0 via `psram_double_buf_t`.
 */
typedef struct {
    uint8_t view_samples[LOGIC_VIEW_POINTS]; /**< Zoomed & panned 4-channel bitmask samples for waveform lanes */
    size_t total_captured;                   /**< Total samples in PSRAM capture buffer (`0` when `no_capture` is true) */
    size_t trigger_index;                    /**< Detected trigger sample index */
    bool triggered;                          /**< True if edge trigger matched */
    uint32_t sample_rate_hz;                 /**< Measured actual sample rate in Hz (`0` when `no_capture` is true) */
    uint32_t requested_rate_hz;              /**< Nominal requested sample rate in Hz from preset */
    uint32_t effective_rate_hz;              /**< Target sample rate in Hz after mode-dependent safety cap */
    uint32_t capture_elapsed_us;             /**< Measured capture duration in microseconds */
    uint32_t edge_count;                     /**< Observed edge transition count in the capture window */
    bool rate_capped;                        /**< True if `requested_rate_hz` was capped to `effective_rate_hz` */
    bool no_capture;                         /**< True when `RMT_EDGE` observed no complete pulse before timeout */
    esp_err_t hw_error;                      /**< `ESP_OK` on normal operation, or hardware/timeout error code */
    logic_capture_mode_t cap_mode;           /**< Active capture mode (`SAMPLED` or `RMT_EDGE`) */
    logic_protocol_t protocol;               /**< Active protocol decoder */
    gpio_num_t channel_pins[LOGIC_CHANNEL_COUNT]; /**< Pin assignment for CH0..CH3 */
    logic_zoom_t zoom;                       /**< Active horizontal zoom factor */
    uint16_t pan_offset;                     /**< Horizontal pan offset in samples */
    logic_decode_table_t decoded;            /**< Decoded protocol packets & summary */
} logic_frame_t;

/* --------------------------------------------------------------------------
 * Pure Software Protocol Decoders & Buffer Helpers (`logic_decoders.c`)
 * -------------------------------------------------------------------------- */

/**
 * @brief Return the nominal sample rate in Hz for a `logic_sample_rate_t` enum.
 *
 * @param[in] rate Sample rate enum value.
 * @return Sample rate in Hz (`10000`..`1000000`), or `0` if invalid.
 */
uint32_t logic_sample_rate_to_hz(logic_sample_rate_t rate);

/**
 * @brief Clamp a requested `logic_sample_rate_t` preset to the maximum sustainable rate for `mode`.
 *
 * In `LOGIC_CAP_MODE_SAMPLED`, presets above `LOGIC_SAMPLED_MAX_RATE` (`100 kHz`) are clamped to
 * `LOGIC_RATE_100KHZ` because periodic GPTimer ISRs above 100 kHz (`< 10 us` period) would starve
 * the CPU. In `LOGIC_CAP_MODE_RMT_EDGE`, all presets up to `1 MHz` are permitted.
 *
 * @param[in] mode Capture mode (`SAMPLED` or `RMT_EDGE`).
 * @param[in] rate Requested sample rate preset.
 * @return Clamped `logic_sample_rate_t`.
 */
logic_sample_rate_t logic_clamp_sample_rate(logic_capture_mode_t mode, logic_sample_rate_t rate);

/**
 * @brief Return the effective target sample rate in Hz for `(mode, rate)` after applying safety caps.
 *
 * @param[in] mode Capture mode (`SAMPLED` or `RMT_EDGE`).
 * @param[in] rate Requested sample rate preset.
 * @return Effective target sample rate in Hz (`10000`..`100000` for `SAMPLED`, `10000`..`1000000` for `RMT_EDGE`), or `0` if invalid.
 */
uint32_t logic_effective_sample_rate_hz(logic_capture_mode_t mode, logic_sample_rate_t rate);

/**
 * @brief Return whether a channel has real signal data in the selected capture mode.
 *
 * `LOGIC_CAP_MODE_RMT_EDGE` captures only CH0; CH1..CH3 are unavailable rather than low.
 *
 * @param[in] mode Capture mode.
 * @param[in] channel Channel index (`0..LOGIC_CHANNEL_COUNT-1`).
 * @return True when the channel is captured, false for invalid inputs or unavailable channels.
 */
bool logic_capture_mode_channel_available(logic_capture_mode_t mode, uint8_t channel);

/**
 * @brief Convert an array of GPIO edge transition timestamps (`1 us` resolution) into ESP32 RMT symbol words.
 *
 * Rejects duplicate-level or zero-duration glitches and packs consecutive pulse durations (`t[k+1] - t[k]`,
 * clamped to `1..32767 us`) into `rmt_symbol_word_t` half-words so continuous edge streams without an RMT
 * idle gap can be expanded through `logic_expand_rmt_to_samples()`.
 *
 * @param[in]  edges            Array of `logic_edge_sample_t` transitions.
 * @param[in]  edge_count       Number of transitions in `edges` (at least `2` required to form 1 pulse).
 * @param[out] out_symbols      Destination `rmt_symbol_word_t` buffer.
 * @param[in]  max_symbols      Capacity of `out_symbols` in words.
 * @param[out] out_symbol_count Pointer to receive number of `rmt_symbol_word_t` words written.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on NULL/invalid pointers.
 */
esp_err_t logic_edge_timestamps_to_rmt_symbols(const logic_edge_sample_t *edges,
                                               size_t edge_count,
                                               rmt_symbol_word_t *out_symbols,
                                               size_t max_symbols,
                                               size_t *out_symbol_count);

/**
 * @brief Return a compact display string for `rate` (e.g. `"100kHz"`, `"1MHz"`).
 *
 * @param[in] rate Sample rate enum value.
 * @return Static null-terminated string.
 */
const char *logic_sample_rate_to_str(logic_sample_rate_t rate);

/**
 * @brief Return a compact display string for `proto` (`"UART"`, `"I2C"`, `"SPI"`, `"PWM"`).
 *
 * @param[in] proto Protocol enum value.
 * @return Static null-terminated string.
 */
const char *logic_protocol_to_str(logic_protocol_t proto);

/**
 * @brief Return the integer zoom factor (`1`, `2`, `4`, `8`) for `zoom`.
 *
 * @param[in] zoom Zoom enum value.
 * @return Integer zoom multiplier.
 */
uint32_t logic_zoom_factor(logic_zoom_t zoom);

/**
 * @brief Expand an array of ESP32 RMT RX edge-duration symbols into a uniform multi-channel sample buffer.
 *
 * Sets bit `channel` of each output byte in `out_samples` according to the symbol durations at
 * `rmt_resolution_hz` resampled to `target_sample_rate_hz`.
 *
 * @param[in]  symbols               Input array of `rmt_symbol_word_t` from RMT RX.
 * @param[in]  num_symbols           Number of symbols in `symbols`.
 * @param[in]  rmt_resolution_hz     RMT tick resolution in Hz (e.g. `1000000`).
 * @param[in]  target_sample_rate_hz Target uniform sample rate in Hz.
 * @param[in]  channel               Channel bit index (`0..3`) to populate in `out_samples`.
 * @param[out] out_samples           Output packed sample buffer.
 * @param[in]  max_samples           Capacity of `out_samples` in bytes.
 * @param[out] out_written           Optional pointer to receive number of samples written.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t logic_expand_rmt_to_samples(const rmt_symbol_word_t *symbols,
                                      size_t num_symbols,
                                      uint32_t rmt_resolution_hz,
                                      uint32_t target_sample_rate_hz,
                                      uint8_t channel,
                                      uint8_t *out_samples,
                                      size_t max_samples,
                                      size_t *out_written);

/**
 * @brief Search a packed multi-channel sample buffer for a trigger transition on `channel`.
 *
 * @param[in]  samples           Input packed sample buffer (`bit c` = level of `CHc`).
 * @param[in]  sample_count      Number of samples in `samples`.
 * @param[in]  channel           Channel index (`0..3`).
 * @param[in]  trig_mode         Trigger mode (`AUTO`, `RISING`, `FALLING`).
 * @param[out] out_triggered     True if a matching edge was found.
 * @param[out] out_trigger_index Sample index of the detected edge (or `0` on fallback).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t logic_find_digital_trigger(const uint8_t *samples,
                                     size_t sample_count,
                                     uint8_t channel,
                                     logic_trig_mode_t trig_mode,
                                     bool *out_triggered,
                                     size_t *out_trigger_index);

/**
 * @brief Extract a zoomed and panned viewport window of `view_points` samples from `samples`.
 *
 * At higher zoom factors (`2x`, `4x`, `8x`), each sample in `samples` is replicated across
 * `zoom_factor` display points so narrow digital pulses can be inspected clearly.
 *
 * @param[in]  samples      Input packed sample buffer.
 * @param[in]  sample_count Number of samples in `samples`.
 * @param[in]  zoom         Active zoom enum (`1x`..`8x`).
 * @param[in]  pan_offset   Starting sample offset into `samples`.
 * @param[out] out_view     Destination viewport array of length `view_points`.
 * @param[in]  view_points  Number of viewport points to write (`LOGIC_VIEW_POINTS`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t logic_extract_viewport(const uint8_t *samples,
                                 size_t sample_count,
                                 logic_zoom_t zoom,
                                 size_t pan_offset,
                                 uint8_t *out_view,
                                 size_t view_points);

/**
 * @brief Decode UART 8N1 asynchronous serial frames from a packed digital sample buffer.
 *
 * Synchronizes on the falling edge of each Start bit (`1 -> 0`), samples 8 LSB-first data bits
 * at `(1.5 .. 8.5) * T_bit`, and verifies the Stop bit at `9.5 * T_bit` (marking `framing_error = true`
 * if the Stop bit is Low).
 *
 * @param[in]  samples        Packed multi-channel sample buffer.
 * @param[in]  sample_count   Total samples in `samples`.
 * @param[in]  sample_rate_hz Sampling rate in Hz.
 * @param[in]  rx_channel     Channel index (`0..3`) carrying UART RX.
 * @param[in]  baud_rate      Target UART baud rate in bps (e.g. `9600`, `115200`).
 * @param[in]  invert         True for inverted UART polarity (idle Low), false for standard 8N1 (idle High).
 * @param[out] out_result     Destination UART decode result.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments or insufficient oversampling.
 */
esp_err_t logic_decode_uart(const uint8_t *samples,
                            size_t sample_count,
                            uint32_t sample_rate_hz,
                            uint8_t rx_channel,
                            uint32_t baud_rate,
                            bool invert,
                            logic_uart_result_t *out_result);

/**
 * @brief Decode I²C bus transactions (`START`, `RESTART`, 7-bit address + `R/W`, `ACK`/`NACK`, data, `STOP`).
 *
 * Detects `START` (`SDA` falling while `SCL` high) and `STOP` (`SDA` rising while `SCL` high),
 * and samples 8 MSB-first bits plus the 9th `ACK`/`NACK` bit on rising edges of `SCL`.
 *
 * @param[in]  samples      Packed multi-channel sample buffer.
 * @param[in]  sample_count Total samples in `samples`.
 * @param[in]  scl_channel  Channel index (`0..3`) carrying `SCL`.
 * @param[in]  sda_channel  Channel index (`0..3`) carrying `SDA` (must differ from `scl_channel`).
 * @param[out] out_result   Destination I²C decode result.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t logic_decode_i2c(const uint8_t *samples,
                           size_t sample_count,
                           uint8_t scl_channel,
                           uint8_t sda_channel,
                           logic_i2c_result_t *out_result);

/**
 * @brief Decode 4-wire SPI byte transfers (`CS`, `SCK`, `MOSI`, `MISO`) with configurable `CPOL`/`CPHA`.
 *
 * Frames transfers while `CS` is active (`0` when `cs_active_low == true`) and samples 8 MSB-first
 * bits on `MOSI` and `MISO` on the active `SCK` edge (`rising` when `cpol == cpha`, `falling` when `cpol != cpha`).
 *
 * @param[in]  samples       Packed multi-channel sample buffer.
 * @param[in]  sample_count  Total samples in `samples`.
 * @param[in]  cs_channel    Channel index (`0..3`) carrying `CS`.
 * @param[in]  sck_channel   Channel index (`0..3`) carrying `SCK`.
 * @param[in]  mosi_channel  Channel index (`0..3`) carrying `MOSI`.
 * @param[in]  miso_channel  Channel index (`0..3`) carrying `MISO`.
 * @param[in]  cpol          Clock polarity (`0` or `1`).
 * @param[in]  cpha          Clock phase (`0` or `1`).
 * @param[in]  cs_active_low True if `CS` is active-low (standard SPI), false if active-high.
 * @param[out] out_result    Destination SPI decode result.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t logic_decode_spi(const uint8_t *samples,
                           size_t sample_count,
                           uint8_t cs_channel,
                           uint8_t sck_channel,
                           uint8_t mosi_channel,
                           uint8_t miso_channel,
                           uint8_t cpol,
                           uint8_t cpha,
                           bool cs_active_low,
                           logic_spi_result_t *out_result);

/**
 * @brief Decode PWM / pulse period, frequency, high/low width, and duty cycle from a sampled channel.
 *
 * @param[in]  samples        Packed multi-channel sample buffer.
 * @param[in]  sample_count   Total samples in `samples`.
 * @param[in]  sample_rate_hz Sampling rate in Hz (`> 0`).
 * @param[in]  channel        Channel index (`0..3`) to analyze.
 * @param[out] out_result     Destination PWM decode result.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t logic_decode_pwm(const uint8_t *samples,
                           size_t sample_count,
                           uint32_t sample_rate_hz,
                           uint8_t channel,
                           logic_pwm_result_t *out_result);

/**
 * @brief Run the selected protocol decoder (`UART`, `I2C`, `SPI`, `PWM`) and populate a unified `logic_decode_table_t`.
 *
 * @param[in]  samples        Packed multi-channel sample buffer.
 * @param[in]  sample_count   Total samples in `samples`.
 * @param[in]  sample_rate_hz Sampling rate in Hz.
 * @param[in]  protocol       Active `logic_protocol_t`.
 * @param[in]  uart_baud      UART baud rate in bps (used when `protocol == LOGIC_PROTO_UART`).
 * @param[out] out_table      Destination unified decode table.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` on invalid arguments.
 */
esp_err_t logic_decode_run_protocol(const uint8_t *samples,
                                    size_t sample_count,
                                    uint32_t sample_rate_hz,
                                    logic_protocol_t protocol,
                                    uint32_t uart_baud,
                                    logic_decode_table_t *out_table);

/* --------------------------------------------------------------------------
 * Logic Analyzer Runtime Configuration & Control API (`tool_logic.c`)
 * -------------------------------------------------------------------------- */

/**
 * @brief Check whether `pin` is a valid safe external GPIO for the Logic Analyzer.
 *
 * Supported pins: `GPIO36`, `GPIO35`, `GPIO26`, `GPIO13`, `GPIO14`, `GPIO32`, `GPIO33`, `GPIO27`, `GPIO19`.
 *
 * @param[in] pin GPIO number to inspect.
 * @return True if supported, false otherwise.
 */
bool tool_logic_is_pin_supported(gpio_num_t pin);

/**
 * @brief Assign a safe external GPIO to Logic Analyzer channel `channel` (`0..3`).
 *
 * Rejects duplicate assignments across the 4 channels (`ESP_ERR_INVALID_ARG`) and atomically
 * claims the new pin via `bsp_pin_claim()` (releasing the previous pin) when the tool is active.
 *
 * @param[in] channel Channel index (`0..3`).
 * @param[in] pin     Supported GPIO number.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` or `ESP_ERR_INVALID_STATE` on conflict.
 */
esp_err_t tool_logic_set_channel_pin(uint8_t channel, gpio_num_t pin);

/**
 * @brief Get the GPIO currently assigned to `channel` (`0..3`).
 *
 * @param[in] channel Channel index (`0..3`).
 * @return Assigned `gpio_num_t`, or `GPIO_NUM_NC` if `channel >= LOGIC_CHANNEL_COUNT`.
 */
gpio_num_t tool_logic_get_channel_pin(uint8_t channel);

/**
 * @brief Set the active capture mode (`LOGIC_CAP_MODE_SAMPLED` or `LOGIC_CAP_MODE_RMT_EDGE`).
 *
 * @param[in] mode Target `logic_capture_mode_t`.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if out of range.
 */
esp_err_t tool_logic_set_capture_mode(logic_capture_mode_t mode);

/**
 * @brief Get the active capture mode.
 *
 * @return Current `logic_capture_mode_t`.
 */
logic_capture_mode_t tool_logic_get_capture_mode(void);

/**
 * @brief Set the periodic sampling rate preset (`10kHz`..`1MHz`).
 *
 * @param[in] rate Target `logic_sample_rate_t`.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if out of range.
 */
esp_err_t tool_logic_set_sample_rate(logic_sample_rate_t rate);

/**
 * @brief Get the active sampling rate preset.
 *
 * @return Current `logic_sample_rate_t`.
 */
logic_sample_rate_t tool_logic_get_sample_rate(void);

/**
 * @brief Set the active protocol decoder (`UART`, `I2C`, `SPI`, `PWM`).
 *
 * @param[in] proto Target `logic_protocol_t`.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if out of range.
 */
esp_err_t tool_logic_set_protocol(logic_protocol_t proto);

/**
 * @brief Get the active protocol decoder.
 *
 * @return Current `logic_protocol_t`.
 */
logic_protocol_t tool_logic_get_protocol(void);

/**
 * @brief Set the UART decoder baud rate in bps (e.g. `9600`, `115200`).
 *
 * @param[in] baud_rate Baud rate in bps (`300`..`2000000`).
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if out of range.
 */
esp_err_t tool_logic_set_uart_baud(uint32_t baud_rate);

/**
 * @brief Get the configured UART decoder baud rate in bps.
 *
 * @return UART baud rate in bps.
 */
uint32_t tool_logic_get_uart_baud(void);

/**
 * @brief Set the horizontal waveform zoom factor (`1x`, `2x`, `4x`, `8x`).
 *
 * @param[in] zoom Target `logic_zoom_t`.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if out of range.
 */
esp_err_t tool_logic_set_zoom(logic_zoom_t zoom);

/**
 * @brief Get the active horizontal waveform zoom factor.
 *
 * @return Current `logic_zoom_t`.
 */
logic_zoom_t tool_logic_get_zoom(void);

/**
 * @brief Set the horizontal waveform pan offset in samples.
 *
 * @param[in] offset_samples Sample offset from start of capture buffer.
 * @return `ESP_OK` on success.
 */
esp_err_t tool_logic_set_pan_offset(uint16_t offset_samples);

/**
 * @brief Get the current horizontal waveform pan offset in samples.
 *
 * @return Pan offset in samples.
 */
uint16_t tool_logic_get_pan_offset(void);

/**
 * @brief Pause or resume logic analyzer capture (`HOLD` / `RUN`).
 *
 * @param[in] running True to run live capture, false to freeze buffer and decoded table.
 * @return `ESP_OK` on success.
 */
esp_err_t tool_logic_set_running(bool running);

/**
 * @brief Return true if logic analyzer capture is running (`RUN`), false if paused (`HOLD`).
 *
 * @return True if running, false if paused.
 */
bool tool_logic_is_running(void);

/**
 * @brief Copy out the latest published `logic_frame_t` (viewport waveforms + decoded protocol table).
 *
 * @param[out] out_frame Destination structure.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if `out_frame` is `NULL`.
 */
esp_err_t tool_logic_get_latest_frame(logic_frame_t *out_frame);

/**
 * @brief Set the maximum wait timeout in microseconds for `LOGIC_CAP_MODE_RMT_EDGE`.
 *
 * Valid range: `LOGIC_EDGE_MIN_TIMEOUT_US` (`5000 us`) .. `LOGIC_EDGE_MAX_TIMEOUT_US` (`500000 us`).
 *
 * @param[in] timeout_us Edge capture timeout in microseconds.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if out of range.
 */
esp_err_t tool_logic_set_edge_timeout_us(uint32_t timeout_us);

/**
 * @brief Get the configured maximum wait timeout in microseconds for `LOGIC_CAP_MODE_RMT_EDGE`.
 *
 * @return Edge capture timeout in microseconds.
 */
uint32_t tool_logic_get_edge_timeout_us(void);

/**
 * @brief Return the static `analyser_tool_t` descriptor for the Logic Analyzer instrument.
 *
 * @return Pointer to `analyser_tool_t` descriptor.
 */
const analyser_tool_t *tool_logic_get_descriptor(void);

/**
 * @brief Register the Logic Analyzer tool (`id = "logic"`) and its self-test with `tool_core`.
 *
 * @return `ESP_OK` on success, or error code from `tool_registry_register()`.
 */
esp_err_t tool_logic_register(void);

#ifdef __cplusplus
}
#endif
