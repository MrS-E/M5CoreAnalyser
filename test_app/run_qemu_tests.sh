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
TEST_TIMEOUT_SEC=20

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
        --test-timeout)
            TEST_TIMEOUT_SEC="$2"
            shift 2
            ;;
        *)
            echo "Usage: $0 [--no-build] [--timeout <seconds>] [--test-timeout <seconds>]" >&2
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
RUN_PHASE="startup"
QEMU_START_MS=0

if ! [[ "${QEMU_TIMEOUT_SEC}" =~ ^[0-9]+$ ]] || [[ "${QEMU_TIMEOUT_SEC}" -le 0 ]] ||
   ! [[ "${TEST_TIMEOUT_SEC}" =~ ^[0-9]+$ ]] || [[ "${TEST_TIMEOUT_SEC}" -le 0 ]]; then
    echo "Timeout values must be positive whole seconds." >&2
    exit 2
fi

cleanup()
{
    local status=$?
    trap - EXIT
    local end_ms
    end_ms=$(date +%s%3N)
    rm -f "${BUILD_DIR}/flash_image.bin"
    if [[ "${status}" -ne 0 ]]; then
        echo "[run_qemu_tests] FAILED during ${RUN_PHASE} (exit=${status})"
        if [[ "${QEMU_START_MS}" -gt 0 && -f "${LOG_FILE}" ]]; then
            echo "[run_qemu_tests] Full QEMU log preserved at ${LOG_FILE}"
        fi
    fi
    if [[ "${QEMU_START_MS}" -gt 0 ]]; then
        echo "[run_qemu_tests] QEMU phase ${RUN_PHASE} finished in $((end_ms - QEMU_START_MS)) ms (total $((end_ms - START_TOTAL_MS)) ms)"
    else
        echo "[run_qemu_tests] Stopped during ${RUN_PHASE} (total $((end_ms - START_TOTAL_MS)) ms)"
    fi
    return "${status}"
}
trap cleanup EXIT

if [[ "${DO_BUILD}" -eq 1 ]]; then
    RUN_PHASE="build"
    BUILD_START_MS=$(date +%s%3N)
    echo "[run_qemu_tests] Building test_app via ninja -C ${BUILD_DIR} ..."
    ninja -C "${BUILD_DIR}"
    BUILD_END_MS=$(date +%s%3N)
    echo "[run_qemu_tests] Build finished in $((BUILD_END_MS - BUILD_START_MS)) ms"
fi

RUN_PHASE="flash merge"
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
RUN_PHASE="QEMU"
echo "[run_qemu_tests] Running ESP32 QEMU (global timeout=${QEMU_TIMEOUT_SEC}s, per-test timeout=${TEST_TIMEOUT_SEC}s) ..."
python3 "${SCRIPT_DIR}/qemu_test_runner.py" \
    --log "${LOG_FILE}" \
    --timeout "${QEMU_TIMEOUT_SEC}" \
    --test-timeout "${TEST_TIMEOUT_SEC}" \
    -- "qemu-system-xtensa" \
       -nographic \
       -machine esp32 \
       -drive "file=${BUILD_DIR}/flash_image.bin,if=mtd,format=raw"
