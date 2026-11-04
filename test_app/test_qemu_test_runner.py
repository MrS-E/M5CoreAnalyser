"""Synthetic stream tests for the QEMU Unity runner's timeout and failure parser."""

import io
import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from qemu_test_runner import monitor_qemu


class QemuRunnerTests(unittest.TestCase):
    def run_child(self, script: str, global_timeout: float = 1.0, case_timeout: float = 0.3):
        temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(temp_dir.cleanup)
        log_path = Path(temp_dir.name) / "qemu.log"
        output = io.StringIO()
        result = monitor_qemu(
            [sys.executable, "-u", "-c", script],
            log_path,
            global_timeout,
            case_timeout,
            output,
        )
        return result, output.getvalue(), log_path

    def test_silent_process_hits_global_timeout_and_retains_log(self):
        started = time.monotonic()
        result, output, log_path = self.run_child("import time; time.sleep(10)", global_timeout=0.25)

        self.assertTrue(result.global_timeout)
        self.assertEqual(1, result.failed)
        self.assertEqual(1, result.exit_code)
        self.assertLess(time.monotonic() - started, 2.0)
        self.assertIn("global QEMU timeout", output)
        self.assertTrue(log_path.exists())

    def test_silent_active_case_hits_per_test_timeout(self):
        script = "import time; print('Running synthetic hanging case...', flush=True); time.sleep(10)"
        result, output, log_path = self.run_child(script, global_timeout=2.0, case_timeout=0.25)

        self.assertTrue(result.test_timeout)
        self.assertEqual("synthetic hanging case", result.active_test)
        self.assertIn("active test: synthetic hanging case", output)
        self.assertIn("synthetic hanging case", log_path.read_text(encoding="utf-8"))
        self.assertEqual(1, result.exit_code)

    def test_crash_stream_reports_active_test_and_crash_line(self):
        script = (
            "print('Running synthetic crashing case...', flush=True); "
            "print('Guru Meditation Error: Core 0 panic', flush=True); "
            "import time; time.sleep(10)"
        )
        result, output, log_path = self.run_child(script)

        self.assertIn("Guru Meditation Error", result.crash_line)
        self.assertEqual("synthetic crashing case", result.active_test)
        self.assertIn("synthetic crashing case", output)
        self.assertIn("QEMU crash", output)
        self.assertIn("Guru Meditation Error", log_path.read_text(encoding="utf-8"))
        self.assertEqual(1, result.exit_code)

    def test_unity_fail_stream_parses_test_name_reason_and_counts(self):
        script = "\n".join(
            [
                "print('Running synthetic failing case...', flush=True)",
                "print('/project/test_sample.c:42:synthetic failing case:FAIL:expected 3 got 4', flush=True)",
                "print('-----------------------', flush=True)",
                "print('1 Tests 1 Failures 0 Ignored', flush=True)",
                "print('=== Unity Tests Complete (failures=1) ===', flush=True)",
            ]
        )
        result, output, log_path = self.run_child(script)

        self.assertTrue(result.complete)
        self.assertEqual((1, 0, 1, 0), (result.total, result.passed, result.failed, result.skipped))
        self.assertEqual(1, result.exit_code)
        self.assertIn("synthetic failing case", output)
        self.assertIn("expected 3 got 4", output)
        self.assertIn("/project/test_sample.c:42", output)
        self.assertIn("FAIL:expected 3 got 4", log_path.read_text(encoding="utf-8"))

    def test_missing_qemu_binary_still_prints_failure_summary(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output = io.StringIO()
            result = monitor_qemu(
                ["/path/that/does/not/exist/qemu"],
                Path(temp_dir) / "qemu.log",
                1.0,
                0.3,
                output,
            )

        self.assertIn("could not start QEMU", result.process_error)
        self.assertEqual((1, 0, 1, 0), (result.total, result.passed, result.failed, result.skipped))
        self.assertEqual(1, result.exit_code)
        self.assertIn("UNITY QEMU TEST SUMMARY", output.getvalue())


if __name__ == "__main__":
    unittest.main()