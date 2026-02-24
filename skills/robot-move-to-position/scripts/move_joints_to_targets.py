#!/usr/bin/env python3
"""Move Mole joints to target positions using velocity commands with feedback."""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from mole_msgs.msg import MoleActuatorCommand, MoleActuatorCommands


DEFAULT_COMMAND_TOPIC = "/mole/actuator_commands"
DEFAULT_STATE_TOPIC = "/mole/joint_states"
DEFAULT_JOINTS = ["J_TURN", "J_BOOM", "J_STICK", "J_TELE", "J_EE_PITCH"]


@dataclass(frozen=True)
class Target:
    joint: str
    position: float


class JointMover(Node):
    def __init__(
        self,
        *,
        command_topic: str,
        state_topic: str,
        all_joints: List[str],
    ) -> None:
        super().__init__("move_joints_to_targets")
        self._all_joints = list(all_joints)
        self._positions: Dict[str, float] = {}
        self._pub = self.create_publisher(MoleActuatorCommands, command_topic, 10)
        self._sub = self.create_subscription(JointState, state_topic, self._on_joint_state, 10)

    def _on_joint_state(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            return
        for name, pos in zip(msg.name, msg.position):
            self._positions[str(name)] = float(pos)

    def get_pos(self, joint: str) -> Optional[float]:
        return self._positions.get(joint)

    def publish_velocity(self, joint_to_vel: Dict[str, float]) -> None:
        msg = MoleActuatorCommands()
        msg.header.stamp = self.get_clock().now().to_msg()
        actuators: List[MoleActuatorCommand] = []
        for joint in self._all_joints:
            cmd = MoleActuatorCommand()
            cmd.joint_name = joint
            cmd.mode = MoleActuatorCommand.MODE_JOINTVELOCITY
            cmd.position = 0.0
            cmd.velocity = float(joint_to_vel.get(joint, 0.0))
            cmd.effort = 0.0
            cmd.current = 0.0
            actuators.append(cmd)
        msg.actuators = actuators
        self._pub.publish(msg)


def _parse_target(spec: str) -> Target:
    text = str(spec).strip()
    if "=" not in text:
        raise ValueError(f"Invalid --target '{text}'. Use JOINT=POSITION.")
    joint, value = text.split("=", 1)
    joint = joint.strip()
    if not joint:
        raise ValueError(f"Invalid --target '{text}': missing joint name.")
    try:
        position = float(value.strip())
    except ValueError as exc:
        raise ValueError(f"Invalid --target '{text}': bad numeric position.") from exc
    if not math.isfinite(position):
        raise ValueError(f"Invalid --target '{text}': position must be finite.")
    return Target(joint=joint, position=position)


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Move Mole joints to target positions using feedback.")
    p.add_argument(
        "--target",
        action="append",
        required=True,
        help="Target in JOINT=POSITION form. Repeat for multiple joints.",
    )
    p.add_argument("--command-topic", default=DEFAULT_COMMAND_TOPIC)
    p.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    p.add_argument(
        "--all-joints",
        default=",".join(DEFAULT_JOINTS),
        help="Comma-separated joints to include in each command message.",
    )
    p.add_argument(
        "--wrap-joints",
        default="J_TURN",
        help=(
            "Comma-separated joints treated as continuous revolute joints. "
            "For these joints, position error is wrapped to [-pi, pi] so targets "
            "like 0.0 use the nearest 2*pi equivalent."
        ),
    )
    p.add_argument("--kp", type=float, default=1.0, help="Proportional gain from position error to velocity.")
    p.add_argument("--max-vel", type=float, default=0.35, help="Max absolute velocity command.")
    p.add_argument("--min-vel", type=float, default=0.06, help="Min non-zero absolute velocity command.")
    p.add_argument("--tol", type=float, default=0.02, help="Per-joint target tolerance.")
    p.add_argument("--hold-sec", type=float, default=0.5, help="Stay in tolerance this long before success.")
    p.add_argument("--timeout-sec", type=float, default=90.0, help="Abort if not reached by this time.")
    p.add_argument("--rate", type=float, default=20.0, help="Command publish rate (Hz).")
    return p


def _zero(node: JointMover, num: int = 5, dt: float = 0.05) -> None:
    for _ in range(num):
        try:
            node.publish_velocity({})
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception:
            # Best-effort zeroing during shutdown; avoid noisy tracebacks on invalid context.
            break
        time.sleep(dt)


def _parse_joint_set(spec: str) -> set[str]:
    return {x.strip() for x in str(spec).split(",") if x.strip()}


def main() -> None:
    args = _build_parser().parse_args()

    if args.rate <= 0.0:
        raise RuntimeError("--rate must be > 0")
    if args.max_vel <= 0.0:
        raise RuntimeError("--max-vel must be > 0")
    if args.min_vel < 0.0:
        raise RuntimeError("--min-vel must be >= 0")
    if args.tol <= 0.0:
        raise RuntimeError("--tol must be > 0")
    if args.timeout_sec <= 0.0:
        raise RuntimeError("--timeout-sec must be > 0")
    if args.hold_sec < 0.0:
        raise RuntimeError("--hold-sec must be >= 0")

    targets: Dict[str, float] = {}
    for spec in args.target:
        t = _parse_target(spec)
        targets[t.joint] = t.position

    all_joints = [x.strip() for x in str(args.all_joints).split(",") if x.strip()]
    if not all_joints:
        raise RuntimeError("--all-joints resolved to an empty list")
    for j in targets:
        if j not in all_joints:
            all_joints.append(j)
    wrap_joints = _parse_joint_set(args.wrap_joints)

    stop_requested = False

    def _sig_handler(_sig, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    rclpy.init()
    node = JointMover(
        command_topic=args.command_topic,
        state_topic=args.state_topic,
        all_joints=all_joints,
    )
    if wrap_joints:
        node.get_logger().info(f"Using wrapped-error mode for joints: {sorted(wrap_joints)}")

    rc = 0
    try:
        start = time.monotonic()
        next_log = start
        in_tol_since: Optional[float] = None
        dt = 1.0 / float(args.rate)

        while rclpy.ok() and not stop_requested:
            now = time.monotonic()
            if (now - start) > float(args.timeout_sec):
                rc = 1
                node.get_logger().error("Timed out before reaching targets.")
                break

            rclpy.spin_once(node, timeout_sec=0.03)

            missing = [j for j in targets if node.get_pos(j) is None]
            if missing:
                if now >= next_log:
                    node.get_logger().info(f"Waiting for joint states for: {missing}")
                    next_log = now + 0.5
                time.sleep(0.03)
                continue

            joint_to_vel: Dict[str, float] = {}
            all_in_tol = True
            status_parts = []
            for joint, target in targets.items():
                pos = float(node.get_pos(joint))
                raw_err = float(target - pos)
                if joint in wrap_joints:
                    # Choose shortest angular distance to target for continuous joints.
                    err = math.atan2(math.sin(raw_err), math.cos(raw_err))
                else:
                    err = raw_err
                abs_err = abs(err)
                if abs_err > float(args.tol):
                    all_in_tol = False
                vel = float(args.kp) * err
                if abs_err <= float(args.tol):
                    vel = 0.0
                else:
                    vel = max(min(vel, float(args.max_vel)), -float(args.max_vel))
                    if abs(vel) < float(args.min_vel):
                        vel = float(args.min_vel) if vel >= 0.0 else -float(args.min_vel)
                joint_to_vel[joint] = vel
                status_parts.append(f"{joint}: q={pos:.3f} tgt={target:.3f} e={err:.3f} v={vel:.3f}")

            node.publish_velocity(joint_to_vel)

            if now >= next_log:
                node.get_logger().info(" | ".join(status_parts))
                next_log = now + 0.5

            if all_in_tol:
                if in_tol_since is None:
                    in_tol_since = now
                if (now - in_tol_since) >= float(args.hold_sec):
                    node.get_logger().info("Targets reached.")
                    break
            else:
                in_tol_since = None

            time.sleep(dt)

    finally:
        _zero(node)
        node.destroy_node()
        rclpy.shutdown()

    if stop_requested and rc == 0:
        rc = 130
    sys.exit(rc)


if __name__ == "__main__":
    main()
