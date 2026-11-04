# M5CoreAnalyser — Multi-Instrument Electronic Analyzer Firmware for M5Stack Core2

**M5CoreAnalyser** is a modular, dual-core ESP-IDF v6.1 multi-instrument electronic analyzer firmware for the **M5Stack Core2** (**ESP32-D0WDQ6-V3 @ 240 MHz**, **16 MB Flash**, **8 MB Quad-SPI PSRAM**, **320×240 ILI9342C** IPS TFT display, **FT6336U** capacitive touchscreen, **AXP192 / AXP2101** PMIC, and **NS4168** `I2S1` speaker amplifier).

It provides an **LVGL 9 touch-friendly UI** to launch and operate **six electrical instruments** implemented as independent ESP-IDF components, **software Analog-to-Speaker audio passthrough**, **background generator concurrency** for single-board loopback testing, and **recording UI controls + backend hooks (`recorder_sink`)** prepared for USB and Bluetooth streaming.

---

## 1. Component Index (10 Self-Contained ESP-IDF Components)

Every subsystem and instrument lives in its own component under `components/<module>/` with a Doxygen-annotated public header (`include/<module>.h`), private test seams (`priv_include/<module>_priv.h`), side-by-side Unity test suite (`test/test_<module>.c`), and dedicated module documentation (`README.md`):

| # | Component | Tool ID / Role | Key Peripherals & Capabilities | Documentation & Tests |
| :-: | :--- | :--- | :--- | :--- |
| 1 | [`components/bsp_m5core2`](components/bsp_m5core2/README.md) | Board Support Package | AXP192 (Core2 v1.0) & AXP2101 (Core2 v1.1) PMIC (`I2C0` `0x34`), ILI9342C LCD (`SPI2`), FT6336U touch (`0x38`), shared calibrated `ADC1` (`GPIO36`/`GPIO35`), GPIO pin reservation (`bsp_pin_claim_ex`), and software DC-blocking `I2S1` speaker passthrough (`bsp_spk_passthrough`). | [`README.md`](components/bsp_m5core2/README.md) · [`test_bsp_m5core2.c`](components/bsp_m5core2/test/test_bsp_m5core2.c) |
| 2 | [`components/tool_core`](components/tool_core/README.md) | Plugin Registry & PSRAM Pipeline | `analyser_tool_t` lifecycle manager, foreground/background generator concurrency, Core 1 `tool_worker_t` helper, and tear-free 3-slot PSRAM buffer (`psram_double_buf_t`). | [`README.md`](components/tool_core/README.md) · [`test_tool_core.c`](components/tool_core/test/test_tool_core.c) |
| 3 | [`components/recorder_sink`](components/recorder_sink/README.md) | Capture Stream Sink | Thread-safe recording state machine (`REC_MODE_OFF`, `REC_MODE_USB_STUB`, `REC_MODE_BT_STUB`) and frame/sample/byte accounting (`recorder_sink_push`). | [`README.md`](components/recorder_sink/README.md) · [`test_recorder_sink.c`](components/recorder_sink/test/test_recorder_sink.c) |
| 4 | [`components/ui_shell`](components/ui_shell/README.md) | LVGL 9 Application Shell | Core 0 LVGL manager with persistent `320×28` status bar (`HOME`, Title, `BG GEN` pill, `SPK` toggle, `REC` mode badge, `VBAT %`) and `2×3` instrument launcher grid. | [`README.md`](components/ui_shell/README.md) · [`test_ui_shell.c`](components/ui_shell/test/test_ui_shell.c) |
| 5 | [`components/tool_voltmeter`](components/tool_voltmeter/README.md) | `"voltmeter"` (`TOOL_CATEGORY_ANALOG_IN`) | 0–3.3 V DC, AC-RMS, True-RMS, Vpp, Min/Max/Avg on `GPIO36`/`GPIO35` with outlier-rejecting median + PSRAM moving-average filter, NVS 2-point calibration, and speaker monitoring. | [`README.md`](components/tool_voltmeter/README.md) · [`test_tool_voltmeter.c`](components/tool_voltmeter/test/test_tool_voltmeter.c) |
| 6 | [`components/tool_scope`](components/tool_scope/README.md) | `"scope"` (`TOOL_CATEGORY_ANALOG_IN`) | 1-Ch digital oscilloscope on `GPIO36`/`GPIO35` with 4096-sample PSRAM capture, rising/falling/auto trigger with hysteresis, `100 us/div`–`100 ms/div` timebases, live metrics (`Vpp`, `Vavg`, `Vrms`, `Freq`, `Duty`), and speaker passthrough. | [`README.md`](components/tool_scope/README.md) · [`test_tool_scope.c`](components/tool_scope/test/test_tool_scope.c) |
| 7 | [`components/tool_freq`](components/tool_freq/README.md) | `"freq"` (`TOOL_CATEGORY_DIGITAL_IN`) | Low-frequency edge period measurement ($f = 1/T$) and high-frequency PCNT + GPTimer gate counting ($f = N/t_{\text{gate}}$) with `4.5–5.5 kHz` hysteresis crossover, duty cycle, pulse width, and RPM. | [`README.md`](components/tool_freq/README.md) · [`test_tool_freq.c`](components/tool_freq/test/test_tool_freq.c) |
| 8 | [`components/tool_logic`](components/tool_logic/README.md) | `"logic"` (`TOOL_CATEGORY_DIGITAL_IN`) | 4-Ch logic analyzer with `10–100 kHz` GPTimer auto-reload ISR sampled mode + RMT/edge timestamp mode into `64 KB` PSRAM and software **UART**, **I²C**, **SPI**, and **PWM** protocol decoders. | [`README.md`](components/tool_logic/README.md) · [`test_tool_logic.c`](components/tool_logic/test/test_tool_logic.c) |
| 9 | [`components/tool_wavegen`](components/tool_wavegen/README.md) | `"wavegen"` (`TOOL_CATEGORY_GENERATOR`) | Digital wave generator driving `GPIO14`/`13`/`26`/`32`/`33`/`27`/`19`/`25` via **LEDC** (`1 Hz–1 MHz` square/PWM), **MCPWM** (`10 Hz–250 kHz` precision PWM), and **RMT TX** (`50 Hz` servo & $N$-pulse bursts) with background execution. | [`README.md`](components/tool_wavegen/README.md) · [`test_tool_wavegen.c`](components/tool_wavegen/test/test_tool_wavegen.c) |
| 10 | [`components/tool_analoggen`](components/tool_analoggen/README.md) | `"analoggen"` (`TOOL_CATEGORY_GENERATOR`) | 8-bit DAC generator on `GPIO25` (`DAC1`) / `GPIO26` (`DAC2`) with 256-entry PSRAM/SRAM LUTs (**Sine**, **Triangle**, **Sawtooth**, **Staircase**, **DC**), `1–1000 Hz` task-driven `dac_oneshot` updates, background execution, and speaker monitoring. | [`README.md`](components/tool_analoggen/README.md) · [`test_tool_analoggen.c`](components/tool_analoggen/test/test_tool_analoggen.c) |

### System Documentation (`docs/`)
- [`docs/architecture.md`](docs/architecture.md) — Dual-core execution model (`PRO_CPU` UI vs `APP_CPU` DSP/acquisition), tear-free 3-slot PSRAM buffer pipeline, plugin lifecycle, and lock hierarchy.
- [`docs/pinout-and-hardware.md`](docs/pinout-and-hardware.md) — AXP192 PMIC rails, protected internal M5Core2 pins, claimable expansion pins, and single-board loopback wiring.
- [`docs/testing-and-git-workflow.md`](docs/testing-and-git-workflow.md) — Side-by-side Unity test architecture, nonblocking QEMU runner, and Conventional Commits workflow.

---

## 2. M5Stack Core2 Hardware Pinout Summary

### Claimable External Instrument Pins
| GPIO | M5Core2 Connector | Direction | Default / Primary Instrument Roles |
| :--- | :--- | :--- | :--- |
| **`GPIO36`** | **Port B** (Black) & M5-Bus | Input-Only | Default **Voltmeter**, **Oscilloscope**, **Frequency Meter**, and **Logic Analyzer (`CH0`)** input (`ADC1_CH0`) |
| **`GPIO35`** | **M5-Bus** Pin 5 | Input-Only | Alternate **Voltmeter** / **Oscilloscope** input (`ADC1_CH7`) and **Logic Analyzer (`CH1`)** |
| **`GPIO25`** | **M5-Bus** Pin 9 | In / Out | Default **Analog Generator (`DAC1`)** output; also supported by **Wave Generator** / **Logic** / **Freq** |
| **`GPIO26`** | **Port B** (Black) & M5-Bus | In / Out | Alternate **Analog Generator (`DAC2`)**, **Wave Generator**, **Logic Analyzer (`CH2`)**, **Frequency Meter** |
| **`GPIO14`** | **Port C** (Blue, TXD2) | In / Out | Default **Digital Wave Generator** output; also supported by **Logic Analyzer** / **Frequency Meter** |
| **`GPIO13`** | **Port C** (Blue, RXD2) | In / Out | **Logic Analyzer (`CH3`)**, **Frequency Meter**, **Wave Generator** |
| **`GPIO32`** | **Port A** (Red, SDA) | In / Out | **Logic Analyzer**, **Frequency Meter**, **Wave Generator** |
| **`GPIO33`** | **Port A** (Red, SCL) | In / Out | **Logic Analyzer**, **Frequency Meter**, **Wave Generator** |
| **`GPIO27`**, **`GPIO19`** | **M5-Bus** Pins 19, 17 | In / Out | **Wave Generator**, **Logic Analyzer**, **Frequency Meter** |

### Protected Internal System Pins (Rejected by `bsp_pin_claim` with `ESP_ERR_NOT_ALLOWED`)
- **Internal I²C0 (`AXP192` @ `0x34`, `FT6336U` @ `0x38`)**: `GPIO21` (`SDA`), `GPIO22` (`SCL`), `GPIO39` (`TOUCH_INT`)
- **SPI2 Display (`ILI9342C` 320×240) & TF-Card**: `GPIO18` (`SCLK`), `GPIO23` (`MOSI`), `GPIO38` (`MISO`), `GPIO5` (`LCD_CS`), `GPIO15` (`LCD_DC`), `GPIO4` (`TF_CS`)
- **NS4168 Speaker Amplifier (`I2S_NUM_1`)**: `GPIO12` (`BCK`), `GPIO0` (`WS`), `GPIO2` (`DOUT`)
- **USB-UART0 & SPI Flash/PSRAM**: `GPIO1`, `GPIO3`, `GPIO6`–`GPIO11`, `GPIO16`, `GPIO17`, `GPIO34`

---

## 3. Build, Flash & Test Instructions

### 3.1 Activate the ESP-IDF v6.1 Environment
```bash
_ORIG_PATH="$PATH"
set -a
eval "$(/home/sstix/.espressif/tools/activate_idf_v6.1.sh -e)" >/dev/null 2>&1
set +a
export PATH="$PATH:$_ORIG_PATH"
```

### 3.2 Build & Flash the Main Production Firmware
```bash
# Configure (first time or after sdkconfig.defaults changes) and build:
idf.py build
# Or incremental ninja build:
ninja -C build

# Flash and monitor on physical M5Stack Core2 hardware (e.g. /dev/ttyACM0 or /dev/ttyUSB0):
idf.py -p /dev/ttyACM0 flash monitor
```

Production configuration highlights (`sdkconfig.defaults`):
- Target: `esp32` @ **`240 MHz`** (`CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y`)
- External SPI PSRAM: **`8 MB` @ `80 MHz`** (`CONFIG_SPIRAM=y`, `CONFIG_SPIRAM_SPEED_80M=y`, `CONFIG_SPIRAM_USE_MALLOC=y`)
- SPI Flash: **`16 MB` @ `80 MHz`** (`CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y`)
- FreeRTOS Tick Rate: **`1000 Hz`** (`CONFIG_FREERTOS_HZ=1000`)

### 3.3 Build & Run the Full Unity Test Suite (`test_app/` on Physical M5Core2 & ESP32 QEMU)
All 10 component Unity test suites (**64 on-target test cases**) are compiled into `test_app/` and verified both on physical **M5Stack Core2** hardware (`/dev/ttyACM0`) and inside `qemu-system-xtensa -machine esp32` via the watchdog-protected runner script:

```bash
# Build test_app incrementally, merge flash image, and run all 64 Unity tests in QEMU:
./test_app/run_qemu_tests.sh --timeout 60 --test-timeout 20

# Re-run QEMU without rebuilding when test_app/build is already up to date:
./test_app/run_qemu_tests.sh --no-build --timeout 60 --test-timeout 20

# Run the host-side Python unit tests for the QEMU runner's timeout/crash/FAIL parser:
python3 -m unittest -v test_app/test_qemu_test_runner.py
```
