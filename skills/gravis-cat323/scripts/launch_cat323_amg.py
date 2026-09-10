#!/usr/bin/env python3
"""AMG command for the prepared bundle; invoke through AMG's existing entrypoint."""

import argparse
import hashlib
import json
import os
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def running_amg(proc_root=Path("/proc")):
    matches = []
    for proc in proc_root.iterdir():
        if not proc.name.isdigit():
            continue
        try:
            args = proc.joinpath("cmdline").read_bytes().rstrip(b"\0").split(b"\0")
        except OSError:
            continue
        is_launch = any(args[i:i + 3] == [b"launch", b"amg", b"amg.launch.py"] for i in range(len(args)))
        is_republisher = bool(args) and Path(os.fsdecode(args[0])).name == "gridmap_selected_layers_republisher_node"
        if is_launch or is_republisher:
            matches.append(int(proc.name))
    return matches


def check_bundle(bundle, manifest):
    for path, expected in [
        (Path(manifest["native_launch_path"]), manifest["patched_launch_sha256"]),
        (Path(manifest["base_profile_path"]), manifest["base_profile_sha256"]),
        (bundle / "selected-map-fastdds.xml", manifest["profile_sha256"]),
    ]:
        if sha(path) != expected:
            raise RuntimeError(f"File differs from the reviewed bundle: {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Check mounted files and stopped-stack preflight without launching")
    parser.add_argument("launch_arguments", nargs="*", help="Existing AMG launch arguments, e.g. use_sim_time:=False")
    args = parser.parse_args()
    bundle = Path(__file__).resolve().parent
    manifest = json.loads((bundle / "manifest.json").read_text())
    check_bundle(bundle, manifest)
    pids = running_amg()
    if pids:
        raise RuntimeError(f"AMG or native republishers already running (PIDs {pids}); no second stack started")
    command = ["ros2", "launch", "amg", "amg.launch.py", *args.launch_arguments]
    profile = str(bundle / "selected-map-fastdds.xml")
    if args.check:
        print(json.dumps({"command": command, "profile": profile, "started": False}, indent=2))
        return
    environment = os.environ.copy()
    environment[manifest["profile_environment_variable"]] = profile
    os.execvpe(command[0], command, environment)


if __name__ == "__main__":
    main()
