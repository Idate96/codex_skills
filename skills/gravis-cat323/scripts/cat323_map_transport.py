#!/usr/bin/env python3
"""Prepare the optional AMG test overlay, inspect it, or restore today's temporary publisher."""

import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time


SCRIPTS = Path(__file__).resolve().parent
WORKSPACE = SCRIPTS.parents[3]


def require_idle():
    """Use a direct lifecycle service and discovered native graph before disrupting map input."""
    profile = WORKSPACE / "evidence/dds-observer-lan-only.xml"
    if not profile.is_file():
        raise RuntimeError("Run restore from gravis_ugep with the sourced CAT323 ROS environment")
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(profile)
    try:
        import rclpy
        from lifecycle_msgs.srv import GetState
    except ImportError as error:
        raise RuntimeError("Source evidence/cat323-env.bash inside gravis_ugep before restore") from error
    rclpy.init()
    node = rclpy.create_node("cat323_map_transport_preflight")
    try:
        client = node.create_client(GetState, "/mole/dig_ugep_controller/get_state")
        if not client.wait_for_service(timeout_sec=6):
            raise RuntimeError("UGEP lifecycle service unavailable; no restore requested")
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=4)
        if not future.done() or future.result() is None or future.result().current_state.id != 2:
            raise RuntimeError("UGEP must be INACTIVE before restoring its map input")
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.get_subscriptions_info_by_topic("/joint_commands"):
            raise RuntimeError("Native command graph not discovered; cannot establish idle state")
        for topic in ("/joint_commands", "/mole/actuator_commands_ugep"):
            if node.get_publishers_info_by_topic(topic):
                raise RuntimeError(f"Command publisher exists on {topic}; no restore requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def remote(host, container, command):
    code = (SCRIPTS / "cat323_map_transport_remote.py").read_text()
    remote_command = shlex.join(["docker", "exec", "-i", container, "python3", "-", command])
    result = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", host, remote_command], input=code, text=True, capture_output=True, timeout=35, check=True)
    return json.loads(result.stdout)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="orin-a")
    parser.add_argument("--container", default="amg")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("status", help="Read-only publisher/profile/probe inspection")
    prepare = commands.add_parser("prepare", help="Create a local test bundle; no remote writes or process changes")
    prepare.add_argument("--output", type=Path, default=WORKSPACE / "amg-map-transport-test")
    maps = commands.add_parser("check-maps", help="Check canonical map freshness/advancement without adding native map traffic")
    maps.add_argument("--seconds", type=int, default=30)
    commands.add_parser("restore", help="Require inactive UGEP and no command publishers, then request temporary supervisor rollback")
    args = parser.parse_args()
    if args.command == "prepare":
        command = [sys.executable, str(SCRIPTS / "prepare_cat323_map_transport.py"), "--host", args.host, "--container", args.container, "--output", str(args.output)]
        subprocess.run(command, check=True)
    elif args.command == "check-maps":
        result = subprocess.run([sys.executable, str(SCRIPTS / "check_cat323_maps.py"), "--seconds", str(args.seconds)])
        raise SystemExit(result.returncode)
    else:
        if args.command == "restore":
            require_idle()
        print(json.dumps(remote(args.host, args.container, args.command), indent=2))


if __name__ == "__main__":
    main()
