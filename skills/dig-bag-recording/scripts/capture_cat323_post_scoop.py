#!/usr/bin/env python3
"""Explicitly capture the cleared cut and attach its full canonical GridMap to one scoop.

Invoke only after the operator clears the bucket from the cut. This command only
subscribes and writes a new snapshot; it never moves the machine or changes mapping.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import time
import traceback
import uuid


MAP_TOPIC = "/excavation_mapping/grid_map"
ATTEMPT_FILENAME = re.compile(r"scoop[-_](\d+)(?:[-_].+)?\.json")


def read_attempt(run_dir, attempt_file):
    path = (run_dir / attempt_file).resolve()
    relative_path = path.relative_to(run_dir)
    name = ATTEMPT_FILENAME.fullmatch(path.name)
    if name is None:
        raise ValueError("Attempt filename must identify its scoop, e.g. scoop-10.json")
    raw = path.read_bytes()
    receipt = json.loads(raw)
    goals = [event for event in receipt["events"] if event["event"] == "goal_response"]
    results = [event for event in receipt["events"] if event["event"] == "result"]
    if len(goals) != 1 or goals[0].get("accepted") is not True or len(results) != 1:
        raise ValueError("Receipt must contain exactly one accepted goal and one final result")
    goal, result = goals[0], results[0]
    if result["status"] not in (4, 5, 6) or not isinstance(result["success"], bool):
        raise ValueError("Action result must be terminal (succeeded, canceled, or aborted)")
    start, end = float(goal["wall_s"]), float(result["wall_s"])
    if not (math.isfinite(start) and math.isfinite(end) and 0 < start <= end):
        raise ValueError("Invalid or reversed action timestamps")
    return raw, {
        "recording_dir": str(run_dir),
        "attempt_file": str(relative_path),
        "attempt_id": path.stem,
        "scoop_number": int(name[1]),
        "goal_id": uuid.UUID(goal["goal_id"]).hex,
        "action_start_wall_s": start,
        "action_end_wall_s": end,
        "action_result": {key: result[key] for key in ("status", "success", "message")},
        "receipt_copy": "attempt.json",
    }


def source_stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def require_latest_attempt(run_dir, assignment):
    # Another scoop changes the cut even if its action aborts or has no result yet.
    # All runs in this recording root belong to the same CAT323 field loop.
    recording_root = run_dir.parent
    paths = set((run_dir / assignment["attempt_file"]).parent.glob("scoop*.json"))
    paths.update(recording_root.glob("*/provenance/scoop*.json"))
    for path in sorted(paths):
        if ATTEMPT_FILENAME.fullmatch(path.name) is None:
            continue
        receipt = json.loads(path.read_bytes())
        for event in receipt["events"]:
            if event["event"] != "goal_response" or event.get("accepted") is not True:
                continue
            if (uuid.UUID(event["goal_id"]).hex != assignment["goal_id"]
                    and float(event["wall_s"]) >= assignment["action_start_wall_s"]):
                raise ValueError(f"Another scoop has started ({path.relative_to(recording_root)}); "
                                 "cannot assign new terrain "
                                 f"to {assignment['attempt_id']}")


def fresh_source(stamp_ns, request_ns, received_ns):
    if stamp_ns > received_ns:
        raise ValueError("Map source stamp is in the future; check host clock alignment")
    return stamp_ns > request_ns


def describe_map(msg):
    if msg.header.frame_id != "map":
        raise ValueError(f"Expected canonical frame 'map', got {msg.header.frame_id!r}")
    geometry = (msg.info.resolution, msg.info.length_x, msg.info.length_y)
    if not all(math.isfinite(value) and value > 0 for value in geometry):
        raise ValueError("GridMap resolution and lengths must be positive and finite")
    layers = list(msg.layers)
    if len(layers) != len(set(layers)) or len(layers) != len(msg.data):
        raise ValueError("GridMap layer names and arrays are inconsistent")
    if not {"elevation", "desired_elevation"} <= set(layers):
        raise ValueError("GridMap lacks elevation or desired_elevation")
    shape = [round(length / msg.info.resolution) for length in geometry[1:]]
    if any(size < 1 for size in shape) or any(len(array.data) != math.prod(shape) for array in msg.data):
        raise ValueError("GridMap arrays do not match its geometry")
    finite = {
        layer: sum(math.isfinite(value) for value in msg.data[layers.index(layer)].data)
        for layer in ("elevation", "desired_elevation")
    }
    if not all(finite.values()):
        raise ValueError("GridMap has no finite elevation or desired_elevation cells")
    return {
        "source_stamp_ns": source_stamp_ns(msg),
        "frame_id": msg.header.frame_id,
        "resolution_m": msg.info.resolution,
        "length_x_m": msg.info.length_x,
        "length_y_m": msg.info.length_y,
        "layers": layers,
        "finite_cells": finite,
        "cells_per_layer": math.prod(shape),
    }


def save_and_verify_map(directory, msg, request_ns, received_ns):
    import rosbag2_py
    from grid_map_msgs.msg import GridMap
    from rclpy.serialization import deserialize_message, serialize_message

    if directory.exists():
        raise FileExistsError(f"Refusing to overwrite snapshot: {directory}")
    if not fresh_source(source_stamp_ns(msg), request_ns, received_ns):
        raise ValueError("Map source stamp must be newer than the explicit capture request")
    describe_map(msg)
    payload = serialize_message(msg)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(directory), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    try:
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=MAP_TOPIC, type="grid_map_msgs/msg/GridMap", serialization_format="cdr"
        ))
        writer.write(MAP_TOPIC, payload, received_ns)
    finally:
        writer.close()
    if not (directory / "metadata.yaml").is_file() or not list(directory.glob("*.mcap")):
        raise ValueError("Snapshot was not finalized as MCAP with metadata.yaml")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(directory), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    if not reader.has_next():
        raise ValueError("Saved snapshot has no message")
    topic, saved, stamp = reader.read_next()
    if topic != MAP_TOPIC or saved != payload or stamp != received_ns or reader.has_next():
        raise ValueError("Saved snapshot does not exactly match the single captured GridMap")
    return {**describe_map(deserialize_message(saved, GridMap)),
            "received_wall_ns": received_ns, "cdr_sha256": hashlib.sha256(saved).hexdigest()}


def capture(directory, assignment, timeout_s):
    import rclpy
    from grid_map_msgs.msg import GridMap
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init(args=[])
    node = rclpy.create_node("capture_cat323_post_scoop")
    selected = []
    request_ns = assignment["requested_wall_ns"]
    observation = assignment["map_observation"] = {"received_messages": 0}

    def on_map(msg):
        received_ns = time.time_ns()
        stamp_ns = source_stamp_ns(msg)
        observation.update(last_source_stamp_ns=stamp_ns, last_received_wall_ns=received_ns)
        observation["received_messages"] += 1
        if fresh_source(stamp_ns, request_ns, received_ns):
            selected.append((msg, received_ns))

    try:
        node.create_subscription(
            GridMap, MAP_TOPIC, on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        deadline = time.monotonic() + timeout_s
        while not selected:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"No source map newer than capture request within {timeout_s:g} s")
            rclpy.spin_once(node, timeout_sec=min(0.25, remaining))
        msg, received_ns = selected[0]
        assignment["map"] = save_and_verify_map(directory / "map", msg, request_ns, received_ns)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def write_assignment(directory, assignment):
    (directory / "assignment.json").write_text(json.dumps(assignment, indent=2, allow_nan=False) + "\n")
    result = assignment["action_result"]
    map_stamp = assignment.get("map", {}).get("source_stamp_ns", "unverified")
    (directory / "README.md").write_text(
        f"# Post-scoop surface: {assignment['attempt_id']}\n\n"
        f"Capture status: **{assignment['capture_status']}**.\n\n"
        f"Recording: `{assignment['recording_dir']}`  \n"
        f"Attempt receipt: `{assignment['attempt_file']}` (copy: [attempt.json](attempt.json))  \n"
        f"Scoop number: {assignment['scoop_number']}  \n"
        f"Goal UUID: `{assignment['goal_id']}`  \n"
        f"Action start/end (Unix seconds): {assignment['action_start_wall_s']} / "
        f"{assignment['action_end_wall_s']}  \n"
        f"Actual action result: status={result['status']}, success={result['success']}; {result['message']}\n\n"
        f"Capture request (Unix ns): {assignment['requested_wall_ns']}  \n"
        f"Saved map source stamp (Unix ns): {map_stamp}  \n"
        f"Operator note: {assignment['operator_note'] or '(none)'}\n\n"
        f"{assignment.get('error', '')}\n\n"
        "Invocation means the operator has cleared the cut. This does not certify visibility, "
        "map accuracy, new sensor integration, or action success.\n\n"
        "Keep this entire directory beside the postprocessed scoop matching the goal UUID. "
        "Attach `map/`, `assignment.json`, and this receipt together to the same scoop upload, "
        "even when POST capture lies outside the action's time cut. Do not use a failed capture "
        "as verified surface evidence. The original recording is unchanged.\n"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--attempt-file", type=Path, required=True,
                        help="Explicit scoop-*.json receipt, relative to run-dir or absolute within it")
    parser.add_argument("--note", default="", help="Optional operator note stored with this surface")
    parser.add_argument("--timeout-s", type=float, default=30.0, help="Fresh-map wait, default 30 s")
    args = parser.parse_args()
    if not math.isfinite(args.timeout_s) or not 0 < args.timeout_s <= 60:
        parser.error("--timeout-s must be positive and at most 60")
    run_dir = args.run_dir.expanduser().resolve()
    if not (run_dir / "raw").is_dir():
        parser.error("--run-dir must contain the original recording's raw/ directory")
    try:
        raw_receipt, assignment = read_attempt(run_dir, args.attempt_file.expanduser())
        require_latest_attempt(run_dir, assignment)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    request_ns = time.time_ns()
    if assignment["action_end_wall_s"] * 1e9 > request_ns:
        parser.error("The action result is later than the capture request; check host clocks")
    suffix = datetime.fromtimestamp(request_ns / 1e9, timezone.utc).strftime("%Y%m%dT%H%M%S")
    directory = run_dir / "post_scoop_surfaces" / f"{assignment['attempt_id']}-{suffix}-{request_ns % 1_000_000_000:09d}Z"
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "attempt.json").write_bytes(raw_receipt)
    assignment.update(
        schema_version=1, capture_status="pending", requested_wall_ns=request_ns,
        requested_source_stamp_after_ns=request_ns, map_topic=MAP_TOPIC, map_bag="map",
        operator_note=args.note,
    )
    write_assignment(directory, assignment)
    print(f"Capturing {assignment['attempt_id']} / goal {assignment['goal_id']} into {directory}", flush=True)
    try:
        capture(directory, assignment, args.timeout_s)
        require_latest_attempt(run_dir, assignment)
        assignment["capture_status"] = "captured"
    except (Exception, KeyboardInterrupt) as exc:
        assignment.update(capture_status="failed", error=f"{type(exc).__name__}: {exc}")
        (directory / "diagnostic.txt").write_text(traceback.format_exc())
        print(assignment["error"], file=sys.stderr)
    assignment["finished_wall_ns"] = time.time_ns()
    write_assignment(directory, assignment)
    print(json.dumps({"status": assignment["capture_status"], "snapshot_dir": str(directory)}))
    return 0 if assignment["capture_status"] == "captured" else 1


if __name__ == "__main__":
    raise SystemExit(main())
