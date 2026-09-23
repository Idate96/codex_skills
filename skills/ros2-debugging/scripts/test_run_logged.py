"""Focused contracts for the command helper; no ROS graph required."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

SCRIPT = Path(__file__).with_name("run_logged.py")


class RunLoggedTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def run_command(self, name, command, timeout=5):
        out = self.root / name
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "--out",
                str(out),
                "--timeout-sec",
                str(timeout),
                "--",
                *command,
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
        return result, out

    def test_success_preserves_full_output_with_small_receipt(self):
        result, out = self.run_command(
            "success",
            [sys.executable, "-c", "print('measurement=12345\\n' * 10000, end='')"],
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        summary = json.loads(result.stdout)
        self.assertEqual(summary["status"], "exit_zero")
        self.assertLess(len(result.stdout), 1024)
        self.assertEqual(len((out / "output.log").read_text().splitlines()), 10000)
        receipt = json.loads((out / "result.json").read_text())
        self.assertEqual(receipt["child_returncode"], 0)
        self.assertEqual(summary["log_bytes"], (out / "output.log").stat().st_size)

    def test_failure_preserves_exit_and_bounded_error_tail(self):
        result, out = self.run_command(
            "failure",
            [
                sys.executable,
                "-u",
                "-c",
                "import sys; print('x'*20000); print('assertion failed', file=sys.stderr); sys.exit(7)",
            ],
        )
        self.assertEqual(result.returncode, 7)
        summary = json.loads(result.stdout)
        self.assertEqual(summary["status"], "failed")
        self.assertLess(len(result.stdout), 4500)
        self.assertIn("assertion failed", summary["failure_tail"])
        self.assertIn("assertion failed", (out / "output.log").read_text())
        self.assertGreater(summary["log_bytes"], 20000)

    def test_timeout_and_signal_exit_are_not_success(self):
        result, out = self.run_command(
            "timeout",
            [sys.executable, "-c", "import time; time.sleep(30)"],
            timeout=0.2,
        )
        self.assertEqual(result.returncode, 124)
        self.assertEqual(json.loads(result.stdout)["status"], "timed_out")
        self.assertLess(
            json.loads((out / "result.json").read_text())["child_returncode"], 0
        )
        result, _ = self.run_command(
            "signal",
            [
                sys.executable,
                "-c",
                "import os, signal; os.kill(os.getpid(), signal.SIGTERM)",
            ],
        )
        self.assertEqual(result.returncode, 128 + signal.SIGTERM)
        self.assertEqual(json.loads(result.stdout)["status"], "failed")
        # Terminating the wrapper must also stop its own still-running command.
        out = self.root / "interrupt"
        ready = self.root / "child.pid"
        child = (
            "import os,time,pathlib; pathlib.Path(%r).write_text(str(os.getpid())); time.sleep(30)"
            % str(ready)
        )
        wrapper = subprocess.Popen(
            [
                sys.executable,
                str(SCRIPT),
                "--out",
                str(out),
                "--timeout-sec",
                "5",
                "--",
                sys.executable,
                "-c",
                child,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            deadline = time.monotonic() + 3
            while not ready.exists() and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue(ready.exists(), "child never started")
            wrapper.send_signal(signal.SIGTERM)
            stdout, stderr = wrapper.communicate(timeout=4)
            self.assertEqual(wrapper.returncode, 143, stderr)
            self.assertEqual(json.loads(stdout)["status"], "interrupted")
            with self.assertRaises(ProcessLookupError):
                os.kill(int(ready.read_text()), 0)
        finally:
            if wrapper.poll() is None:
                wrapper.send_signal(signal.SIGTERM)
                wrapper.communicate(timeout=4)

    def test_launch_failure_and_existing_results_are_preserved(self):
        result, out = self.run_command("missing", [str(self.root / "missing-command")])
        self.assertEqual(result.returncode, 127)
        self.assertEqual(json.loads(result.stdout)["status"], "launch_failed")
        before = (out / "result.json").read_bytes()
        result, _ = self.run_command(
            "missing", [sys.executable, "-c", "print('replacement')"]
        )
        self.assertEqual(result.returncode, 2)
        self.assertEqual((out / "result.json").read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
