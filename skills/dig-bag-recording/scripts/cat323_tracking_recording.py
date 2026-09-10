#!/usr/bin/env python3
"""Record explicit CAT323 tracking topics; never starts or controls the machine."""

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid

import yaml

JOINTS = ("Boom", "Dipper", "EndeffectorPitch")
TERMS = (
    "desiredJointVel", "measuredJointVel", "desiredCylinderVel", "measuredCylinderVel",
    "lutCommand", "pidCommand", "pidProportional", "pidIntegral", "pidDerivative",
)
PREFIX = "/mole/dig_ugep"
SPLITS = {
    "state": [
        "/mole/measurements", "/machine_measurements", "/machine_state", "/machine_status",
        "/joint_states", "/machine_lowlevel_controller/actuator_states",
        "/hal/machine_lowlevel_controller/actuator_states", "/tf", "/tf_static",
        "/robot_description",
        "/platform/state_estimator/visualization/estimated_cabin_odometry_in_world_frame",
    ],
    "commands": [
        "/mole/actuator_commands_ugep", "/joint_commands",
        "/machine_lowlevel_controller/raw_commands_in",
        "/machine_lowlevel_controller/raw_commands_out",
    ],
    "telemetry": [
        *(f"/velocityCtrl/{term}/{joint}" for term in TERMS for joint in JOINTS),
        *(f"{PREFIX}/{name}" for name in (
            "observations", "observations_raw", "policy_action_stamped",
            "commanded_joint_velocity", "pullup_distance_locked", "controller_status",
            "depth_shield", "scooped_soil_volume", "filled_soil_volume",
        )),
        "/mole/run_dig_ugep/_action/feedback", "/mole/run_dig_ugep/_action/status",
        "/mole/dig_ugep_controller/transition_event", "/parameter_events", "/rosout",
    ],
    "elevation_map": [
        "/excavation_mapping/grid_map", "/excavation_mapping/upstream_fusion_event",
    ],
}
REQUIRED_STATE = [
    "/mole/measurements", "/machine_measurements", "/machine_status", "/joint_states",
    "/tf", "/tf_static", "/excavation_mapping/grid_map",
]
REQUIRED_MOTION = [
    "/mole/actuator_commands_ugep", "/joint_commands",
    "/machine_lowlevel_controller/raw_commands_out", f"{PREFIX}/commanded_joint_velocity",
    *(f"/velocityCtrl/{term}/{joint}" for term in ("desiredJointVel", "measuredJointVel")
      for joint in JOINTS),
]


def write_json(path, data):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n")
    temporary.replace(path)


def process_identity(pid):
    """Linux start tick plus cmdline protects stop against stale PID reuse."""
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
        fields = stat[stat.rfind(")") + 2:].split()
        if fields[0] == "Z":
            return None
        return {
            "pid": pid, "start_ticks": int(fields[19]),
            "cmdline": Path(f"/proc/{pid}/cmdline").read_bytes().decode().split("\0")[:-1],
        }
    except (FileNotFoundError, ProcessLookupError):
        return None


def same_process(saved):
    return saved is not None and process_identity(saved["pid"]) == saved


def plan(run_dir):
    qos_path = run_dir / "qos-overrides.yaml"
    token = uuid.uuid4().hex[:8]
    return {
        split: [
            "ros2", "bag", "record", "--storage", "mcap", "--output",
            str(run_dir / "raw" / split), "--node-name", f"cat323_bag_{split}_{token}",
            "--disable-keyboard-controls", "--include-hidden-topics",
            "--include-unpublished-topics", "--polling-interval", "100",
            "--max-cache-size", "16777216", "--qos-profile-overrides-path", str(qos_path),
            "--topics", *topics,
        ] for split, topics in SPLITS.items()
    }


def qos_overrides():
    result = {
        topic: {"history": "keep_last", "depth": 100, "reliability": "best_effort",
                "durability": "volatile"}
        for topics in SPLITS.values() for topic in topics
    }
    for topic in ("/tf_static", "/robot_description", "/excavation_mapping/grid_map"):
        result[topic] = {"history": "keep_last", "depth": 10, "reliability": "reliable",
                         "durability": "transient_local"}
    return result


def verify(run_dir, allow_no_commands=False):
    counts = {}
    results = {}
    errors = []
    for split, expected_topics in SPLITS.items():
        bag_dir = run_dir / "raw" / split
        metadata_path = bag_dir / "metadata.yaml"
        result = {"path": str(bag_dir), "expected_topics": expected_topics}
        results[split] = result
        if not metadata_path.is_file() or not list(bag_dir.glob("*.mcap")):
            errors.append(f"{split}: missing finalized metadata.yaml or MCAP")
            continue
        metadata = yaml.safe_load(metadata_path.read_text())["rosbag2_bagfile_information"]
        result["message_count"] = metadata["message_count"]
        result["duration_s"] = metadata["duration"]["nanoseconds"] / 1e9
        result["counts"] = {
            entry["topic_metadata"]["name"]: entry["message_count"]
            for entry in metadata["topics_with_message_count"]
        }
        counts.update(result["counts"])
        result["absent_or_empty"] = [t for t in expected_topics if not result["counts"].get(t, 0)]
        info = subprocess.run(["ros2", "bag", "info", str(bag_dir)], text=True,
                              capture_output=True, timeout=15)
        (run_dir / f"bag-info-{split}.txt").write_text(info.stdout + info.stderr)
        result["readable"] = info.returncode == 0
        if not result["readable"]:
            errors.append(f"{split}: ros2 bag info failed")
    required = REQUIRED_STATE + ([] if allow_no_commands else REQUIRED_MOTION)
    for topic in required:
        if counts.get(topic, 0) == 0:
            errors.append(f"missing required samples: {topic}")
    process_state = run_dir / "processes.json"
    if process_state.exists():
        saved = json.loads(process_state.read_text())
        live = [identity for identity in saved.get("recorders", {}).values() if same_process(identity)]
        if live:
            errors.append("one or more owned recorders still running; bag is not finalized")
    report = {"complete": not errors, "motion_topics_required": not allow_no_commands,
              "splits": results, "errors": errors}
    write_json(run_dir / "verification.json", report)
    print(json.dumps({"complete": report["complete"], "errors": errors,
                      "report": str(run_dir / "verification.json")}, indent=2), flush=True)
    return 0 if report["complete"] else 1


def record(args):
    run_dir = args.run_dir.resolve()
    if shutil.disk_usage(run_dir.parent).free < args.min_free_gib * 1024**3:
        raise RuntimeError(f"Less than {args.min_free_gib:g} GiB free in output filesystem")
    run_dir.mkdir(exist_ok=False)
    (run_dir / "raw").mkdir()
    commands = plan(run_dir)
    write_json(run_dir / "recording-plan.json", {
        "machine": "gravis_cat323", "time": "wall_clock", "split_topics": SPLITS,
        "commands": commands, "created_unix_s": time.time(),
        "environment": {key: os.environ.get(key) for key in (
            "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE",
            "FASTDDS_DEFAULT_PROFILES_FILE", "ROS_DISCOVERY_SERVER", "AMENT_PREFIX_PATH",
        )},
    })
    (run_dir / "qos-overrides.yaml").write_text(yaml.safe_dump(qos_overrides(), sort_keys=False))
    # Graph inspection only; the only data subscriptions are the four allowlisted recorders.
    graph = subprocess.run(["ros2", "topic", "list", "-t", "--include-hidden-topics"],
                           capture_output=True, text=True, timeout=15)
    (run_dir / "topics-before.txt").write_text(graph.stdout + graph.stderr)
    shutdown = False

    def request_shutdown(_signal, _frame):
        nonlocal shutdown
        shutdown = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    children = {}
    handles = []
    started = time.monotonic()
    state = {"supervisor": process_identity(os.getpid()), "recorders": {},
             "run_dir": str(run_dir), "status": "starting"}
    write_json(run_dir / "processes.json", state)
    try:
        for split, command in commands.items():
            output = (run_dir / f"recorder-{split}.log").open("w")
            handles.append(output)
            children[split] = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                                stdin=subprocess.DEVNULL, start_new_session=True)
            state["recorders"][split] = process_identity(children[split].pid)
        write_json(run_dir / "processes.json", state)
        while not shutdown:
            if any(child.poll() is not None for child in children.values()):
                raise RuntimeError("A recorder exited early; inspect recorder logs")
            if state["status"] == "starting":
                ready = all("Listening for topics" in (run_dir / f"recorder-{split}.log").read_text()
                            for split in SPLITS)
                if ready:
                    state["status"] = "recording"
                    write_json(run_dir / "processes.json", state)
                    print(f"RECORDING {run_dir}; verify state subscriptions before motion", flush=True)
                elif time.monotonic() - started > 25:
                    raise RuntimeError("Recorder startup not confirmed in 25 s; inspect logs")
            time.sleep(0.2)
    finally:
        state["status"] = "finalizing"
        write_json(run_dir / "processes.json", state)
        for split, child in children.items():
            if child.poll() is None and same_process(state["recorders"][split]):
                os.killpg(child.pid, signal.SIGINT)
        deadline = time.monotonic() + 25
        while any(child.poll() is None for child in children.values()) and time.monotonic() < deadline:
            time.sleep(0.2)
        state["exit_codes"] = {split: child.poll() for split, child in children.items()}
        state["status"] = "finalized" if all(v is not None for v in state["exit_codes"].values()) else "finalization_timeout"
        write_json(run_dir / "processes.json", state)
        for output in handles:
            output.close()
        print(json.dumps({"status": state["status"], "run_dir": str(run_dir)}, indent=2), flush=True)
        if state["status"] != "finalized":
            print("Owned recorder still running. Inspect processes.json; no force kill was sent.", file=sys.stderr)
    return 0 if state["status"] == "finalized" else 1


def stop(args):
    run_dir = args.run_dir.resolve()
    state_path = run_dir / "processes.json"
    state = json.loads(state_path.read_text())
    if same_process(state.get("supervisor")):
        print(f"Recording {args.settle_seconds:g} s of post-motion state before recorder shutdown", flush=True)
        time.sleep(args.settle_seconds)
        if same_process(state["supervisor"]):
            os.kill(state["supervisor"]["pid"], signal.SIGINT)
        deadline = time.monotonic() + 30
        while same_process(state["supervisor"]) and time.monotonic() < deadline:
            time.sleep(0.2)
        if same_process(state["supervisor"]):
            raise RuntimeError("Recorder supervisor did not exit; inspect saved owned processes")
    else:
        print("Saved supervisor is absent; no signal sent", flush=True)
    return verify(run_dir, args.allow_no_commands)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    for mode in ("plan", "record", "stop", "verify"):
        command = sub.add_parser(mode)
        command.add_argument("--run-dir", required=True, type=Path)
        if mode == "record":
            command.add_argument("--min-free-gib", type=float, default=1.0)
        if mode == "stop":
            command.add_argument("--settle-seconds", type=float, default=8.0)
        if mode in ("stop", "verify"):
            command.add_argument("--allow-no-commands", action="store_true",
                                 help="Validate a recording made without a motion attempt")
    args = parser.parse_args()
    if args.mode == "plan":
        print(json.dumps(plan(args.run_dir.resolve()), indent=2))
        return 0
    if args.mode == "record":
        return record(args)
    if args.mode == "stop":
        if not 0 <= args.settle_seconds <= 60:
            parser.error("--settle-seconds must be between 0 and 60")
        return stop(args)
    return verify(args.run_dir.resolve(), args.allow_no_commands)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.TimeoutExpired, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
