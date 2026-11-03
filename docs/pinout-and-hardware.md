# M5Stack Core2 Pinout & Hardware Reference (`M5CoreAnalyser`)

## 1. Hardware Platform Overview
- **SoC**: ESP32-D0WDQ6-V3 Dual-Core Xtensa LX6 @ 240 MHz
- **Memory**: 520 KB internal SRAM + **8 MB Quad-SPI PSRAM** (`CONFIG_SPIRAM=y`, 80 MHz) + **16 MB SPI Flash** (`CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y`, 80 MHz DIO)
- **PMIC**: X-Powers **AXP192** (`I2C_NUM_0` @ `0x34`)
- **Display**: 2.0" **ILI9342C** 320×240 IPS TFT (`SPI2_HOST` @ 40 MHz, 16-bit RGB565)
- **Touch**: FocalTech **FT6336U** capacitive touch controller (`I2C_NUM_0` @ `0x38`, IRQ on `GPIO39`)
- **Audio Amplifier**: **NS4168** Mono Class-D I²S speaker amplifier (`I2S_NUM_1`: `GPIO12` BCK, `GPIO0` WS, `GPIO2` DOUT, power-gated by AXP192 `GPIO2` / `SPK_EN`)

---

## 2. AXP192 PMIC Rail & GPIO Assignment (`0x34` on `I2C_NUM_0`)

| AXP192 Rail / Pin | Register(s) | Configured Value | M5Stack Core2 Function | Safety Note |
| :--- | :--- | :--- | :--- | :--- |
| **DCDC1** | `0x26`, `0x12[0]` | `3350 mV` (`0x6A`), ON | ESP32 `VDD3P3` main power | **CRITICAL**: Never disable `DCDC1` |
| **DCDC3** | `0x27`, `0x12[1]` | `2500–3100 mV` (`0x54` = `2.8 V`), ON | ILI9342C LCD Backlight | Controlled by `bsp_axp192_set_lcd_backlight()` |
| **LDO2** | `0x28[7:4]`, `0x12[2]` | `3300 mV` (`0xF0`), ON | LCD logic & TF-Card power | Must remain enabled while LCD is active |
| **LDO3** | `0x28[3:0]`, `0x12[3]` | `0 V`, OFF | Coin vibration motor | Kept OFF by default |
| **EXTEN** | `0x12[6]` | ON | 5 V boost converter for Grove/M5-Bus | Supplies 5 V expansion rail |
| **AXP GPIO0** | `0x90`, `0x91` | LDO mode (`0x02`), `3.3 V` (`0xF0`) | M5-Bus peripheral VDD | Powers bus logic |
| **AXP GPIO1** | `0x92`, `0x94[1]` | NMOS Open-Drain (`0x00`), bit 1 (`0x02`) | Green system LED (active-low: `0`=ON, `1`=OFF) | Controlled by `bsp_axp192_set_led()` |
| **AXP GPIO2** | `0x93`, `0x94[2]` | NMOS Open-Drain (`0x00`), bit 2 (`0x04`) | NS4168 Speaker Amp Enable (`SPK_EN`, active-high) | Gated OFF when muted/idle (`bsp_axp192_set_speaker_enable()`) |
| **AXP GPIO4** | `0x95`, `0x96[1]` | NMOS Open-Drain (`0x84`) | LCD & Touch Hardware Reset (`TFT_RST`) | Pulsed during `bsp_axp192_init()` |
| **VBAT ADC** | `0x78` (hi), `0x79` (lo) | 12-bit (`1.1 mV / LSB`) | LiPo Battery Voltage Telemetry | Converted via `bsp_axp192_raw12_to_mv()` |

---

## 3. Protected Internal Pins (`bsp_pin_is_protected(pin) == true`)

The following GPIOs are hard-wired to internal M5Stack Core2 peripherals and are **strictly rejected** (`ESP_ERR_NOT_ALLOWED`) by `bsp_pin_claim()`:

| GPIO | Signal | Connected Internal Hardware |
| :--- | :--- | :--- |
| `GPIO0` | `I2S_WS` / `LRCK` | NS4168 Speaker Amplifier Word Select & Boot Strap Pin |
| `GPIO1`, `GPIO3` | `U0TXD`, `U0RXD` | CP2104 / CH9102F USB-UART0 Console |
| `GPIO2` | `I2S_DOUT` | NS4168 Speaker Amplifier Serial Audio Data Out |
| `GPIO4` | `TF_CS` | MicroSD / TF-Card SPI Chip Select (pulled HIGH on boot) |
| `GPIO5` | `LCD_CS` | ILI9342C Display SPI Chip Select |
| `GPIO6`–`GPIO11`, `GPIO16`, `GPIO17` | `SPI_FLASH` / `PSRAM` | 16 MB SPI Flash & 8 MB Quad-SPI PSRAM |
| `GPIO12` | `I2S_BCK` | NS4168 Speaker Amplifier Bit Clock |
| `GPIO15` | `LCD_DC` | ILI9342C Display Data/Command Select |
| `GPIO18` | `SPI_SCLK` | SPI2 Clock (ILI9342C LCD & TF-Card) |
| `GPIO21`, `GPIO22` | `I2C0_SDA`, `I2C0_SCL` | Internal I²C0 Bus (`AXP192`, `FT6336U`, `BM8563`, `MPU6886`) |
| `GPIO23` | `SPI_MOSI` | SPI2 MOSI (ILI9342C LCD & TF-Card) |
| `GPIO34` | `PDM_MIC_DATA` | Onboard SPM1423 PDM Microphone (input-only, excluded) |
| `GPIO38` | `SPI_MISO` | SPI2 MISO (ILI9342C LCD & TF-Card, input-only) |
| `GPIO39` | `TOUCH_INT` | FT6336U Touch Controller Interrupt (input-only) |

---

## 4. Claimable Instrument Expansion Pins (`bsp_pin_is_protected(pin) == false`)

These 10 external pins on **Port A**, **Port B**, **Port C**, and the **M5-Bus** header can be claimed by foreground measurement tools and background signal generators:

| GPIO | Connector / Location | Direction | Analog / Digital Capabilities | Default Instrument Assignment |
| :--- | :--- | :--- | :--- | :--- |
| **`GPIO36`** | **Port B** (Black) & M5-Bus | **Input-Only** | `ADC1_CH0` (12-bit), RMT RX, PCNT, GPIO In | Default **Voltmeter**, **Oscilloscope**, **Frequency Meter**, **Logic Analyzer** Ch0 |
| **`GPIO35`** | **M5-Bus** Pin 5 | **Input-Only** | `ADC1_CH7` (12-bit), RMT RX, PCNT, GPIO In | Alternate **Voltmeter** / **Oscilloscope** Input, **Logic Analyzer** Ch1 |
| **`GPIO25`** | **M5-Bus** Pin 9 | **In / Out** | `DAC_CHAN_0` (8-bit DAC1), LEDC, MCPWM, RMT | Default **Analog Generator** DAC1 Output |
| **`GPIO26`** | **Port B** (Black) & M5-Bus | **In / Out** | `DAC_CHAN_1` (8-bit DAC2), LEDC, MCPWM, RMT, GPIO | Alternate **Analog Generator** DAC2 / **Wave Generator** / **Logic Analyzer** |
| **`GPIO32`** | **Port A** (Red, SDA) | **In / Out** | `ADC1_CH4`, Digital I/O, I²C, LEDC, MCPWM, RMT | **Logic Analyzer** / **Wave Generator** |
| **`GPIO33`** | **Port A** (Red, SCL) | **In / Out** | `ADC1_CH5`, Digital I/O, I²C, LEDC, MCPWM, RMT | **Logic Analyzer** / **Wave Generator** |
| **`GPIO13`** | **Port C** (Blue, RXD2) | **In / Out** | UART2 RX, SPI, LEDC, MCPWM, RMT, PCNT | **Logic Analyzer** / **Frequency Meter** / **Wave Generator** |
| **`GPIO14`** | **Port C** (Blue, TXD2) | **In / Out** | UART2 TX, SPI, LEDC, MCPWM, RMT | Default **Digital Wave Generator** Output |
| **`GPIO27`** | **M5-Bus** Pin 19 | **In / Out** | Digital I/O, LEDC, MCPWM, RMT | **Wave Generator** / **Logic Analyzer** |
| **`GPIO19`** | **M5-Bus** Pin 17 | **In / Out** | Digital I/O, LEDC, MCPWM, RMT | **Wave Generator** / **Logic Analyzer** |

---

## 5. Peripheral Contention Avoidance & Loopback Self-Testing

1. **Dedicated `I2S_NUM_1` for Speaker Audio**:
   - The NS4168 speaker amplifier uses `I2S_NUM_1` (`GPIO12`/`GPIO0`/`GPIO2`), leaving `ADC1` (`GPIO36`/`GPIO35`) and `DAC` (`GPIO25`/`GPIO26`) free from `I2S1` pin or DMA contention.
2. **Simultaneous Background Generation + Foreground Measurement**:
   - **Digital Loopback**: Run `tool_wavegen` in the background on `GPIO14` (Port C) or `GPIO26` (Port B) and jumper it to `GPIO36` (Port B) to inspect the waveform in `tool_scope`, `tool_freq`, or `tool_logic`.
   - **Analog Loopback**: Run `tool_analoggen` in the background on `GPIO25` (M5-Bus DAC1) or `GPIO26` (Port B DAC2) and jumper it to `GPIO36` (`ADC1_CH0`) to measure DC/AC-RMS in `tool_voltmeter` or visualize the analog waveform and hear it through the speaker in `tool_scope`.
