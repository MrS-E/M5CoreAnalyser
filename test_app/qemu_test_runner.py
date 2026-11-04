#!/usr/bin/env python3
"""Run QEMU with nonblocking deadlines and summarize ESP-IDF Unity output."""

from __future__ import annotations

import argparse
import os
import re
import selectors
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import TextIO


SUMMARY_RE = re.compile(r"(\d+)\s+Tests\s+(\d+)\s+Failures\s+(\d+)\s+Ignored")
RUNNING_RE = re.compile(r"^Running\s+(.+?)\.\.\.\s*$")
RESULT_RE = re.compile(r"^(.+?):(\d+):(.+?):(PASS|FAIL|IGNORE)(?::(.*))?\s*$")
CRASH_MARKERS = (
    "stack overflow in task",
    "guru meditation error",
    "abort() was called",
    "assert failed",
    "panic'ed",
    "double exception",
)


@dataclass
class Failure:
    name: str
    location: str
    reason: str


@dataclass
class MonitorResult:
    total: int = 0
    passed: int = 0
    failed: int = 0
    skipped: int = 0
    complete: bool = False
    global_timeout: bool = False
    test_timeout: bool = False
    crash_line: str = ""
    active_test: str = ""
    process_error: str = ""
    failures: list[Failure] = field(default_factory=list)
    elapsed_seconds: float = 0.0

    @property
    def exit_code(self) -> int:
        return int(
            not self.complete
            or self.global_timeout
            or self.test_timeout
            or bool(self.crash_line)
            or bool(self.process_error)
            or self.failed > 0
            or self.total == 0
        )


def _failure_name(result: MonitorResult, fallback: str, reason: str) -> None:
    name = result.active_test or fallback
    if not any(item.name == name and item.reason == reason for item in result.failures):
        result.failures.append(Failure(name, "runtime", reason))


def _print_summary(result: MonitorResult, output: TextIO) -> None:
    print("\n============================================================", file=output)
    print("UNITY QEMU TEST SUMMARY", file=output)
    print("============================================================", file=output)
    print(f"  Total   : {result.total}", file=output)
    print(f"  Passed  : {result.passed}", file=output)
    print(f"  Failed  : {result.failed}", file=output)
    print(f"  Skipped : {result.skipped}", file=output)
    print(
        f"  Complete: {result.complete} (global_timeout={result.global_timeout}, "
        f"per_test_timeout={result.test_timeout}, crash={bool(result.crash_line)})",
        file=output,
    )
    if result.failures:
        print("\nFAILED TESTS:", file=output)
        for failure in result.failures:
            print(f"  - {failure.name} [{failure.location}]: {failure.reason}", file=output)
    if result.process_error:
        print(f"  QEMU error: {result.process_error}", file=output)
    print("============================================================", file=output, flush=True)


def _terminate_and_reap(proc: subprocess.Popen[bytes]) -> None:
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


def monitor_qemu(
    command: list[str],
    log_path: str | Path,
    timeout_seconds: float,
    test_timeout_seconds: float = 20.0,
    output: TextIO = sys.stdout,
) -> MonitorResult:
    """Supervise QEMU without blocking on output and retain all observed output.

    Both deadlines are checked between selector polls, so a QEMU process that emits no
    bytes cannot prevent timeout handling. A timeout/crash is associated with the
    currently running Unity test when one has been announced.
    """
    result = MonitorResult()
    start = time.monotonic()
    active_started = start
    pending = bytearray()
    proc: subprocess.Popen[bytes] | None = None
    selector: selectors.BaseSelector | None = None
    log_file = Path(log_path)
    log_file.parent.mkdir(parents=True, exist_ok=True)

    def process_line(line: str) -> bool:
        nonlocal active_started
        with log_file.open("a", encoding="utf-8") as logf:
            logf.write(line + "\n")

        running_match = RUNNING_RE.match(line)
        if running_match:
            result.active_test = running_match.group(1).strip()
            active_started = time.monotonic()
            return False

        test_match = RESULT_RE.match(line)
        if test_match:
            filepath, line_number, test_name, status, reason = test_match.groups()
            duration_ms = (time.monotonic() - active_started) * 1000.0
            print(f"  {line} ({duration_ms:.0f} ms)", file=output, flush=True)
            if status == "FAIL":
                result.failures.append(
                    Failure(test_name.strip(), f"{filepath}:{line_number}", (reason or "").strip())
                )
            result.active_test = ""
            return False

        if "=== Unity Tests Complete" in line:
            result.complete = True
            return True

        lowered = line.lower()
        if any(marker in lowered for marker in CRASH_MARKERS):
            result.crash_line = line.strip()
            print(f"  [CRASH] {result.crash_line}", file=output, flush=True)
            _failure_name(result, "QEMU", f"QEMU crash: {result.crash_line}")
            return True

        return False

    try:
        log_file.write_text("", encoding="utf-8")
        try:
            proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0)
        except OSError as exc:
            result.process_error = f"could not start QEMU: {exc}"
            print(f"[qemu_test_runner] {result.process_error}", file=output, flush=True)
            result.total = 1
            result.failed = 1
            result.failures.append(Failure("QEMU", "startup", result.process_error))
            result.elapsed_seconds = time.monotonic() - start
            _print_summary(result, output)
            return result

        assert proc.stdout is not None
        fd = proc.stdout.fileno()
        os.set_blocking(fd, False)
        selector = selectors.DefaultSelector()
        selector.register(fd, selectors.EVENT_READ)

        global_deadline = start + timeout_seconds
        stop_monitoring = False
        eof = False
        while not stop_monitoring and not eof:
            now = time.monotonic()
            if now >= global_deadline:
                result.global_timeout = True
                reason = f"global QEMU timeout after {timeout_seconds:g}s"
                print(f"  [TIMEOUT] {reason}; active test: {result.active_test or '<startup>'}", file=output, flush=True)
                _failure_name(result, "QEMU", reason)
                break

            if result.active_test and now - active_started >= test_timeout_seconds:
                result.test_timeout = True
                reason = f"per-test timeout after {test_timeout_seconds:g}s"
                print(f"  [TIMEOUT] {reason}; active test: {result.active_test}", file=output, flush=True)
                _failure_name(result, result.active_test, reason)
                break

            deadlines = [global_deadline - now]
            if result.active_test:
                deadlines.append(test_timeout_seconds - (now - active_started))
            events = selector.select(max(0.0, min(0.1, *deadlines)))
            if events:
                try:
                    chunk = os.read(fd, 65536)
                except BlockingIOError:
                    chunk = None
                if chunk == b"":
                    eof = True
                elif chunk:
                    pending.extend(chunk)
                    while b"\n" in pending:
                        raw_line, _, remainder = pending.partition(b"\n")
                        pending = bytearray(remainder)
                        if process_line(raw_line.decode("utf-8", errors="replace").rstrip("\r")):
                            stop_monitoring = True
                            break

            if proc.poll() is not None and not events:
                # Drain any final bytes before treating the pipe as exhausted.
                try:
                    chunk = os.read(fd, 65536)
                except BlockingIOError:
                    chunk = None
                if chunk == b"":
                    eof = True
                elif chunk:
                    pending.extend(chunk)

        if pending:
            process_line(pending.decode("utf-8", errors="replace").rstrip("\r"))
            pending.clear()

        if not result.complete and not result.global_timeout and not result.test_timeout and not result.crash_line:
            return_code = proc.poll()
            if return_code is None:
                return_code = proc.wait(timeout=0.1)
            if return_code != 0:
                result.process_error = f"QEMU exited with status {return_code}"
            else:
                result.process_error = "QEMU exited before the Unity completion banner"
            if result.active_test:
                _failure_name(result, result.active_test, result.process_error)
            print(f"  [INCOMPLETE] {result.process_error}; active test: {result.active_test or '<none>'}",
                  file=output, flush=True)
    except subprocess.TimeoutExpired:
        result.process_error = "QEMU did not exit after output pipe closed"
        if result.active_test:
            _failure_name(result, result.active_test, result.process_error)
    except Exception as exc:
        result.process_error = f"runner error: {type(exc).__name__}: {exc}"
        if result.active_test:
            _failure_name(result, result.active_test, result.process_error)
        print(f"[qemu_test_runner] {result.process_error}", file=output, flush=True)
    finally:
        if selector is not None:
            selector.close()
        if proc is not None:
            _terminate_and_reap(proc)
            if proc.stdout is not None:
                proc.stdout.close()
        result.elapsed_seconds = time.monotonic() - start

    content = log_file.read_text(encoding="utf-8", errors="replace") if log_file.exists() else ""
    summary = SUMMARY_RE.search(content)
    pass_count = len(re.findall(r"^.+?:\d+:.+?:PASS\s*$", content, re.MULTILINE))
    ignore_count = len(re.findall(r"^.+?:\d+:.+?:IGNORE(?:\s|$)", content, re.MULTILINE))
    parsed_failures = len(re.findall(r"^.+?:\d+:.+?:FAIL(?::.*)?$", content, re.MULTILINE))
    if summary:
        result.total, summary_failures, result.skipped = map(int, summary.groups())
        result.failed = max(summary_failures, parsed_failures)
        result.passed = max(0, result.total - summary_failures - result.skipped)
    else:
        result.passed = pass_count
        result.skipped = ignore_count
        result.failed = max(parsed_failures, len(result.failures))
        result.total = result.passed + result.failed + result.skipped

    # A timeout/crash is a failed run even if Unity never emitted a test result line.
    if (result.global_timeout or result.test_timeout or result.crash_line or result.process_error) and result.failed == 0:
        result.failed = 1
        result.total = max(result.total, result.passed + result.skipped + result.failed)

    _print_summary(result, output)
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True, help="Full QEMU output log path")
    parser.add_argument("--timeout", type=float, default=60.0, help="Global QEMU timeout in seconds")
    parser.add_argument("--test-timeout", type=float, default=20.0, help="Per-Unity-case timeout in seconds")
    parser.add_argument("command", nargs=argparse.REMAINDER, help="QEMU command after --")
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command and args.command[0] == "--" else args.command
    if not command:
        parser.error("a QEMU command is required after --")
    if args.timeout <= 0 or args.test_timeout <= 0:
        parser.error("timeouts must be greater than zero")

    result = monitor_qemu(command, args.log, args.timeout, args.test_timeout)
    return result.exit_code


if __name__ == "__main__":
    raise SystemExit(main())