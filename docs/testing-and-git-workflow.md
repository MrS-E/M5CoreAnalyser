# Testing Architecture & Conventional Git Workflow

## 1. Side-by-Side Component Unity Test Architecture
Every ESP-IDF component in `components/<module>/` is developed alongside a dedicated Unity unit test component in `components/<module>/test/`:

```text
components/<module>/
├── CMakeLists.txt
├── README.md
├── include/<module>.h
├── priv_include/<module>_priv.h
├── <module>.c
└── test/
    ├── CMakeLists.txt
    └── test_<module>.c
```

### Design Principles for Testability
1. **Pure DSP & Formatting Seams**: Hardware-independent logic (DC-blocker audio filter, AXP192 register & battery math, PSRAM double-buffer swapping, recorder frame accounting, UI header badge formatting, protocol decoders, and waveform calculations) is exposed as deterministic functions callable without physical sensors attached.
2. **Private Test Seams (`priv_include/`)**: Reset functions and mock hardware operations tables (`bsp_m5core2_priv.h`, `recorder_sink_priv.h`, `tool_core_priv.h`) live in `priv_include/` rather than public headers so production callers cannot invoke test-only resets.
3. **Thread-Safe Copy-Out APIs**: Public getters copy state out under lock (`bsp_pin_get_owner_copy`, `bsp_spk_passthrough_copy_last_pcm`, `recorder_sink_get_stats`, `psram_double_buf_read_copy`, `ui_shell_get_header_state`) rather than returning unlocked pointers to mutable static buffers.

---

## 2. Building & Running Tests

### Environment Activation (ESP-IDF v6.1)
```bash
_ORIG_PATH="$PATH"
set -a
eval "$(/home/sstix/.espressif/tools/activate_idf_v6.1.sh -e)" >/dev/null 2>&1
set +a
export PATH="$PATH:$_ORIG_PATH"
```

### Main Firmware Build
```bash
ninja -C build
```

### Unity `test_app` Build & On-Target Execution in ESP32 QEMU
All component test suites registered in `test_app/CMakeLists.txt` (`TEST_COMPONENTS`) can be compiled and executed on the emulated dual-core Xtensa ESP32 machine via `qemu-system-xtensa`:

```bash
ninja -C test_app/build
cd test_app/build
esptool --chip esp32 merge-bin -o flash_image.bin --pad-to-size 4MB @flash_args
timeout 10s qemu-system-xtensa -nographic -machine esp32 -drive file=flash_image.bin,if=mtd,format=raw || true
rm -f flash_image.bin
```

---

## 3. Conventional Git Commit Discipline
Commits are made in small, atomic milestones immediately after completing each component implementation, fix, test suite, or documentation update:

- `feat(<module>): <what>` — New component or feature implementation
- `fix(<module>): <what>` — Bug fix or concurrency/safety hardening
- `test(<module>): <what>` — Unity test suite addition or expansion
- `docs(<module>): <what>` — Component `README.md`, Doxygen headers, or `docs/*.md` guides
- `chore(<scope>): <what>` — Build configuration, Kconfig, or project tooling updates
