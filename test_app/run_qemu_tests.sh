#!/usr/bin/env bash
# Reusable Unity QEMU test runner for M5CoreAnalyser test_app.
# Builds test_app (unless --no-build is given), merges the 4 MB ESP32 flash image,
# boots qemu-system-xtensa, terminates QEMU as soon as Unity completes (or on timeout),
# and parses/reports passed, failed, and skipped counts plus every failed test name & error.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
LOG_FILE="${BUILD_DIR}/qemu_unity_output.log"

DO_BUILD=1
QEMU_TIMEOUT_SEC=60

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-build)
            DO_BUILD=0
            shift
            ;;
        --timeout)
            QEMU_TIMEOUT_SEC="$2"
            shift 2
            ;;
        *)
            echo "Usage: $0 [--no-build] [--timeout <seconds>]" >&2
            exit 2
            ;;
    esac
done

# Activate ESP-IDF v6.1 environment if idf.py / esptool / qemu-system-xtensa are not on PATH
if ! command -v qemu-system-xtensa >/dev/null 2>&1 || ! command -v esptool >/dev/null 2>&1; then
    _ORIG_PATH="$PATH"
    set +u
    set -a
    # shellcheck disable=SC1091
    eval "$(/home/sstix/.espressif/tools/activate_idf_v6.1.sh -e)" >/dev/null 2>&1
    set +a
    set -u
    export PATH="$PATH:$_ORIG_PATH"
fi

START_TOTAL_MS=$(date +%s%3N)

if [[ "${DO_BUILD}" -eq 1 ]]; then
    BUILD_START_MS=$(date +%s%3N)
    echo "[run_qemu_tests] Building test_app via ninja -C ${BUILD_DIR} ..."
    ninja -C "${BUILD_DIR}"
    BUILD_END_MS=$(date +%s%3N)
    echo "[run_qemu_tests] Build finished in $((BUILD_END_MS - BUILD_START_MS)) ms"
fi

MERGE_START_MS=$(date +%s%3N)
(
    cd "${BUILD_DIR}"
    esptool --chip esp32 merge-bin -o flash_image.bin --pad-to-size 4MB @flash_args >/dev/null
)
MERGE_END_MS=$(date +%s%3N)
echo "[run_qemu_tests] Merged flash_image.bin in $((MERGE_END_MS - MERGE_START_MS)) ms"

rm -f "${LOG_FILE}"
touch "${LOG_FILE}"

QEMU_START_MS=$(date +%s%3N)
echo "[run_qemu_tests] Running ESP32 QEMU (timeout=${QEMU_TIMEOUT_SEC}s) ..."

# Run QEMU and stop immediately once Unity prints its completion banner
python3 - "${BUILD_DIR}/flash_image.bin" "${LOG_FILE}" "${QEMU_TIMEOUT_SEC}" <<'PYEOF'
import os
import re
import signal
import subprocess
import sys
import time

flash_img = sys.argv[1]
log_path = sys.argv[2]
timeout_s = float(sys.argv[3])
per_test_cap_s = 20.0

cmd = [
    "qemu-system-xtensa",
    "-nographic",
    "-machine", "esp32",
    "-drive", f"file={flash_img},if=mtd,format=raw",
]

proc = subprocess.Popen(
    cmd,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    bufsize=1,
)

start_time = time.monotonic()
last_case_time = start_time
completed_banner = False
timed_out = False
per_test_timed_out = False
crashed_line = ""

with open(log_path, "w", encoding="utf-8") as logf:
    try:
        while True:
            now = time.monotonic()
            if now - start_time > timeout_s:
                timed_out = True
                break
            line = proc.stdout.readline()
            if not line:
                if proc.poll() is not None:
                    break
                time.sleep(0.01)
                continue
            logf.write(line)
            logf.flush()
            if "Running " in line and "..." in line:
                last_case_time = time.monotonic()
            elif re.search(r":(PASS|FAIL|IGNORE)", line):
                case_elapsed = time.monotonic() - last_case_time
                sys.stdout.write(f"  {line.strip()} ({case_elapsed * 1000.0:.0f} ms)\n")
                sys.stdout.flush()
                if case_elapsed > per_test_cap_s:
                    per_test_timed_out = True
                last_case_time = time.monotonic()
            elif any(tok in line for tok in ("stack overflow in task", "Guru Meditation Error", "abort() was called")):
                crashed_line = line.strip()
                sys.stdout.write(f"  [CRASH] {crashed_line}\n")
                sys.stdout.flush()
                break
            if "=== Unity Tests Complete" in line:
                completed_banner = True
                break
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()

with open(log_path, "r", encoding="utf-8", errors="replace") as logf:
    content = logf.read()

summary_match = re.search(r"(\d+)\s+Tests\s+(\d+)\s+Failures\s+(\d+)\s+Ignored", content)
fail_lines = []
for m in re.finditer(r"^(.+?):(\d+):([^:]+):FAIL(?::(.*))?$", content, re.MULTILINE):
    filepath, lineno, testname, reason = m.groups()
    fail_lines.append((testname.strip(), f"{filepath}:{lineno}", (reason or "").strip()))

pass_count = len(re.findall(r"^.+?:\d+:[^:]+:PASS\s*$", content, re.MULTILINE))
ignore_count = len(re.findall(r"^.+?:\d+:[^:]+:IGNORE", content, re.MULTILINE))
fail_count = len(fail_lines)

if summary_match:
    total_tests = int(summary_match.group(1))
    reported_failures = int(summary_match.group(2))
    reported_ignored = int(summary_match.group(3))
    passed_tests = total_tests - reported_failures - reported_ignored
else:
    total_tests = pass_count + fail_count + ignore_count
    reported_failures = fail_count
    reported_ignored = ignore_count
    passed_tests = pass_count

print("\n============================================================")
print("UNITY QEMU TEST SUMMARY")
print("============================================================")
print(f"  Total   : {total_tests}")
print(f"  Passed  : {passed_tests}")
print(f"  Failed  : {reported_failures}")
print(f"  Skipped : {reported_ignored}")
print(f"  Complete: {completed_banner} (timeout={timed_out}, per_test_cap_exceeded={per_test_timed_out})")

if fail_lines:
    print("\nFAILED TESTS:")
    for name, loc, reason in fail_lines:
        print(f"  - {name} [{loc}]: {reason}")
print("============================================================")

if not completed_banner or timed_out or per_test_timed_out or reported_failures > 0 or total_tests == 0:
    sys.exit(1)
sys.exit(0)
PYEOF
STATUS=$?

QEMU_END_MS=$(date +%s%3N)
rm -f "${BUILD_DIR}/flash_image.bin"
echo "[run_qemu_tests] QEMU run finished in $((QEMU_END_MS - QEMU_START_MS)) ms (total $((QEMU_END_MS - START_TOTAL_MS)) ms)"
exit "${STATUS}"
