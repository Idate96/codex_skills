#!/usr/bin/env python3
"""Run one finite command; retain its output and print a compact terminal receipt."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def interrupt(signum, _frame):
    raise KeyboardInterrupt(signum)


def stop_group(process):
    """Stop only this invocation's group, including remaining descendants."""
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        pass
    finally:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, type=Path, help="New output directory")
    parser.add_argument("--timeout-sec", required=True, type=float)
    parser.add_argument(
        "command", nargs=argparse.REMAINDER, help="-- command arguments"
    )
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("provide a command after --")
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0:
        parser.error("--timeout-sec must be finite and positive")
    out = args.out.absolute()
    try:
        out.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        parser.error(f"output directory already exists: {out}")
    log_path = out / "output.log"
    receipt_path = out / "result.json"
    started = time.monotonic()
    process = None
    status, exit_code = "launch_failed", 127
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    with log_path.open("wb") as log:
        try:
            process = subprocess.Popen(
                command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
            )
            code = process.wait(timeout=args.timeout_sec)
            exit_code = code if code >= 0 else 128 - code
            status = "exit_zero" if code == 0 else "failed"
        except subprocess.TimeoutExpired:
            stop_group(process)
            status, exit_code = "timed_out", 124
        except KeyboardInterrupt as error:
            if process is not None:
                stop_group(process)
            signum = error.args[0] if error.args else signal.SIGINT
            status, exit_code = "interrupted", 128 + signum
        except OSError as error:
            if process is not None:
                stop_group(process)
            log.write(f"Could not run command: {error}\n".encode())

    summary = {
        "status": status,
        "exit_code": exit_code,
        "elapsed_s": round(time.monotonic() - started, 3),
        "log_bytes": log_path.stat().st_size,
        "log": str(log_path),
        "receipt": str(receipt_path),
    }
    receipt = dict(
        summary,
        argv=command,
        cwd=os.getcwd(),
        child_returncode=process.returncode if process is not None else None,
    )
    temporary = out / "result.json.tmp"
    temporary.write_text(json.dumps(receipt, indent=2) + "\n")
    temporary.replace(receipt_path)
    if exit_code:
        with log_path.open("rb") as log:
            log.seek(max(0, summary["log_bytes"] - 3000))
            tail = log.read().decode(errors="replace")
        summary["failure_tail"] = "\n".join(tail.splitlines()[-24:])
    print(json.dumps(summary, ensure_ascii=False))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
