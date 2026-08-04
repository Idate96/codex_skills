#!/usr/bin/env python3
"""Move bounded M445 arm joints with guarded velocity feedback."""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from typing import Callable, Dict, Mapping, Optional, Sequence

import rclpy
from machine_msgs.msg import MachineMeasurements, MachineStatus
from mole_msgs.msg import MoleActuatorCommand, MoleActuatorCommands, MoleMeasurements
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


COMMAND_TOPIC = "/mole/actuator_commands"
EXPECTED_COMMAND_SUBSCRIBER = "mole_pid_joint_controller"
MEASUREMENTS_TOPIC = "/mole/measurements"
MACHINE_STATUS_TOPIC = "/machine_status"
RAW_MEASUREMENTS_TOPIC = "/machine_measurements"
ARM_JOINTS = ("J_TURN", "J_BOOM", "J_STICK", "J_TELE", "J_EE_PITCH", "J_EE_ROLL")
RAW_JOINT_ALIASES = {"J_DIPPER": "J_STICK"}
INTERLOCKS = (
    "is_hydraulilock_unlocked",
    "is_autonomous_operation_unlocked",
    "is_using_gravis_commands",
)

# Match the maintained soft bounds in mole_sysid.robot_tuning.M445_ARM_SAFETY.
M445_SOFT_LIMITS: Mapping[str, tuple[float, float]] = {
    "J_TURN": (-3.14, 3.14),
    "J_BOOM": (-1.33, -0.38),
    "J_STICK": (0.67, 2.72),
    "J_TELE": (0.08, 1.524),
    "J_EE_PITCH": (-0.58, 2.218),
    "J_EE_ROLL": (-0.596, 0.505),
}
M445_MAX_VELOCITIES: Mapping[str, float] = {
    "J_TURN": 0.86,
    "J_BOOM": 0.45,
    "J_STICK": 0.62,
    "J_TELE": 0.64,
    "J_EE_PITCH": 1.30,
    "J_EE_ROLL": 0.25,
}

GRAPH_SETTLE_SEC = 1.0
SUBSCRIBER_STABLE_SEC = 0.5
OWNERSHIP_SETTLE_SEC = 0.5
LIMIT_GUARD_HORIZON_SEC = 0.2
ZERO_ATTEMPTS = 10
ZERO_PERIOD_SEC = 0.05


class JointMover(Node):
    def __init__(self) -> None:
        super().__init__("move_joints_to_targets")
        self.positions: Dict[str, float] = {}
        self.measurements_recv_time: Optional[float] = None
        self.raw_status: Optional[int] = None
        self.raw_sensor_status: Dict[str, tuple[Optional[int], Optional[int]]] = {}
        self.raw_measurements_recv_time: Optional[float] = None
        self.interlocks: Dict[str, bool] = {}
        self.status_recv_time: Optional[float] = None
        self.command_claimed = False

        feedback_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            MoleMeasurements,
            MEASUREMENTS_TOPIC,
            self._on_measurements,
            feedback_qos,
        )
        self.create_subscription(
            MachineMeasurements,
            RAW_MEASUREMENTS_TOPIC,
            self._on_raw_measurements,
            feedback_qos,
        )
        self.create_subscription(
            MachineStatus,
            MACHINE_STATUS_TOPIC,
            self._on_status,
            feedback_qos,
        )
        # Matching the command writers makes competing publishers visible to a
        # Discovery Server CLIENT before this node claims the command topic.
        self.command_probe = self.create_subscription(
            MoleActuatorCommands,
            COMMAND_TOPIC,
            lambda _msg: None,
            feedback_qos,
        )
        # Create a disarmed writer so a Discovery Server CLIENT can also see
        # the expected downstream PID subscription. Never publish until every
        # preflight passes and command_claimed becomes true.
        self.command_pub = self.create_publisher(
            MoleActuatorCommands,
            COMMAND_TOPIC,
            feedback_qos,
        )

    def _on_measurements(self, msg: MoleMeasurements) -> None:
        # Replace the complete sample: never mix fresh joints with cached ones.
        self.positions = {
            RAW_JOINT_ALIASES.get(str(actuator.joint_name), str(actuator.joint_name)): float(actuator.position)
            for actuator in msg.actuators
        }
        self.measurements_recv_time = time.monotonic()

    def _on_raw_measurements(self, msg: MachineMeasurements) -> None:
        self.raw_status = int(msg.status)
        self.raw_sensor_status = {}
        for index, raw_name in enumerate(msg.joints.name):
            joint = RAW_JOINT_ALIASES.get(str(raw_name), str(raw_name))
            position_status = (
                int(msg.position_sensor_status[index])
                if index < len(msg.position_sensor_status)
                else None
            )
            velocity_status = (
                int(msg.velocity_sensor_status[index])
                if index < len(msg.velocity_sensor_status)
                else None
            )
            self.raw_sensor_status[joint] = (position_status, velocity_status)
        self.raw_measurements_recv_time = time.monotonic()

    def _on_status(self, msg: MachineStatus) -> None:
        self.interlocks = {name: bool(getattr(msg, name)) for name in INTERLOCKS}
        self.status_recv_time = time.monotonic()

    def safety_error(
        self,
        required_joints: Sequence[str],
        max_age_sec: float,
    ) -> Optional[str]:
        now = time.monotonic()
        if self.measurements_recv_time is None:
            return f"no {MEASUREMENTS_TOPIC} received"
        age = now - self.measurements_recv_time
        if age > max_age_sec:
            return f"{MEASUREMENTS_TOPIC} receive age {age:.3f}s > {max_age_sec:.3f}s"
        missing = [joint for joint in required_joints if joint not in self.positions]
        if missing:
            return f"latest {MEASUREMENTS_TOPIC} is missing {missing}"
        invalid = [
            joint
            for joint in required_joints
            if not math.isfinite(self.positions[joint])
        ]
        if invalid:
            return f"latest {MEASUREMENTS_TOPIC} has non-finite positions for {invalid}"

        if self.raw_measurements_recv_time is None:
            return f"no {RAW_MEASUREMENTS_TOPIC} received"
        age = now - self.raw_measurements_recv_time
        if age > max_age_sec:
            return (
                f"{RAW_MEASUREMENTS_TOPIC} receive age {age:.3f}s > {max_age_sec:.3f}s"
            )
        if self.raw_status != MachineMeasurements.OPERATIONAL:
            return (
                f"{RAW_MEASUREMENTS_TOPIC} status is {self.raw_status}, "
                f"expected OPERATIONAL={MachineMeasurements.OPERATIONAL}"
            )
        missing_raw = [
            joint for joint in required_joints if joint not in self.raw_sensor_status
        ]
        if missing_raw:
            return f"latest {RAW_MEASUREMENTS_TOPIC} is missing {missing_raw}"
        invalid_raw = [
            f"{joint}(position={self.raw_sensor_status[joint][0]}, "
            f"velocity={self.raw_sensor_status[joint][1]})"
            for joint in required_joints
            if self.raw_sensor_status[joint]
            != (MachineMeasurements.CURRENT_VALID, MachineMeasurements.CURRENT_VALID)
        ]
        if invalid_raw:
            return (
                f"{RAW_MEASUREMENTS_TOPIC} target sensor status is not CURRENT_VALID: "
                f"{invalid_raw}"
            )

        if self.status_recv_time is None:
            return f"no {MACHINE_STATUS_TOPIC} received"
        age = now - self.status_recv_time
        if age > max_age_sec:
            return f"{MACHINE_STATUS_TOPIC} receive age {age:.3f}s > {max_age_sec:.3f}s"
        locked = [name for name in INTERLOCKS if not self.interlocks.get(name, False)]
        if locked:
            return f"{MACHINE_STATUS_TOPIC} interlocks are false: {locked}"
        return None

    def publisher_count(self) -> int:
        return int(self.count_publishers(COMMAND_TOPIC))

    def expected_subscriber_count(self) -> int:
        return sum(
            info.node_name == EXPECTED_COMMAND_SUBSCRIBER
            for info in self.get_subscriptions_info_by_topic(COMMAND_TOPIC)
        )

    def require_command_subscriber(self) -> None:
        count = self.expected_subscriber_count()
        if count != 1:
            raise RuntimeError(
                f"Expected exactly one {EXPECTED_COMMAND_SUBSCRIBER} subscriber on "
                f"{COMMAND_TOPIC}, found {count}"
            )

    def claim_command_topic(self) -> None:
        count = self.publisher_count()
        if count != 1:
            raise RuntimeError(
                f"Refusing motion: found {count} publishers on {COMMAND_TOPIC}; "
                "expected this disarmed script only"
            )
        self.command_claimed = True

    def require_exclusive_ownership(self) -> None:
        count = self.publisher_count()
        if count != 1:
            raise RuntimeError(
                f"Lost exclusive command ownership: found {count} publishers "
                f"on {COMMAND_TOPIC}, expected this script only"
            )

    def publish_velocity(self, joint_to_velocity: Mapping[str, float]) -> None:
        if not self.command_claimed:
            raise RuntimeError("command topic has not been claimed")
        msg = MoleActuatorCommands()
        msg.header.stamp = self.get_clock().now().to_msg()
        for joint in ARM_JOINTS:
            actuator = MoleActuatorCommand()
            actuator.joint_name = joint
            actuator.mode = MoleActuatorCommand.MODE_JOINTVELOCITY
            actuator.position = 0.0
            actuator.velocity = float(joint_to_velocity.get(joint, 0.0))
            actuator.effort = 0.0
            actuator.current = 0.0
            msg.actuators.append(actuator)
        self.command_pub.publish(msg)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Move bounded M445 arm joints using guarded velocity feedback."
    )
    parser.add_argument("--confirm-hardware", action="store_true")
    parser.add_argument("--confirm-safe-start", action="store_true")
    parser.add_argument(
        "--target",
        action="append",
        required=True,
        help="M445 target as JOINT=POSITION; repeat for multiple joints.",
    )
    parser.add_argument("--kp", type=float, default=1.0)
    parser.add_argument("--max-vel", type=float, default=0.35)
    parser.add_argument("--min-vel", type=float, default=0.06)
    parser.add_argument("--tol", type=float, default=0.02)
    parser.add_argument("--hold-sec", type=float, default=0.5)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--max-feedback-age-sec", type=float, default=0.5)
    parser.add_argument("--preflight-timeout-sec", type=float, default=10.0)
    return parser


def _validate_args(args: argparse.Namespace) -> Dict[str, float]:
    if not args.confirm_hardware or not args.confirm_safe_start:
        raise ValueError("--confirm-hardware and --confirm-safe-start are required")
    numeric = {
        "kp": args.kp,
        "max-vel": args.max_vel,
        "min-vel": args.min_vel,
        "tol": args.tol,
        "hold-sec": args.hold_sec,
        "timeout-sec": args.timeout_sec,
        "rate": args.rate,
        "max-feedback-age-sec": args.max_feedback_age_sec,
        "preflight-timeout-sec": args.preflight_timeout_sec,
    }
    for name, value in numeric.items():
        if not math.isfinite(float(value)):
            raise ValueError(f"--{name} must be finite")

    if args.kp <= 0 or args.max_vel <= 0 or args.tol <= 0:
        raise ValueError("--kp, --max-vel, and --tol must be > 0")
    if args.min_vel < 0 or args.min_vel > args.max_vel:
        raise ValueError("--min-vel must be between 0 and --max-vel")
    if args.hold_sec < 0:
        raise ValueError("--hold-sec must be >= 0")
    if args.timeout_sec <= 0 or args.rate <= 0:
        raise ValueError("--timeout-sec and --rate must be > 0")
    if args.max_feedback_age_sec <= 0 or args.preflight_timeout_sec <= 0:
        raise ValueError("feedback age and preflight timeout must be > 0")

    targets: Dict[str, float] = {}
    for spec in args.target:
        text = str(spec).strip()
        if "=" not in text:
            raise ValueError(f"invalid --target '{text}'; use JOINT=POSITION")
        joint, raw_position = (part.strip() for part in text.split("=", 1))
        if joint not in M445_SOFT_LIMITS:
            raise ValueError(
                f"unsupported joint '{joint}'; allowed joints are {list(ARM_JOINTS)}"
            )
        if joint in targets:
            raise ValueError(f"duplicate --target for {joint}")
        try:
            position = float(raw_position)
        except ValueError as exc:
            raise ValueError(f"invalid position in --target '{text}'") from exc
        if not math.isfinite(position):
            raise ValueError(f"target {joint} must be finite")
        lower, upper = M445_SOFT_LIMITS[joint]
        if not lower <= position <= upper:
            raise ValueError(
                f"target {joint}={position:.6g} is outside M445 soft limits "
                f"[{lower:.6g}, {upper:.6g}]"
            )
        targets[joint] = position
    smallest_velocity_cap = min(M445_MAX_VELOCITIES[joint] for joint in targets)
    if args.min_vel > smallest_velocity_cap:
        raise ValueError(
            f"--min-vel exceeds the smallest target-joint safety cap "
            f"({smallest_velocity_cap:.3g})"
        )
    return targets


def _check_stop(stop_requested: Callable[[], bool], phase: str) -> None:
    if stop_requested():
        raise InterruptedError(f"stop requested during {phase}")


def _preflight(
    node: JointMover,
    targets: Mapping[str, float],
    args: argparse.Namespace,
    stop_requested: Callable[[], bool],
) -> None:
    joints = tuple(targets)
    deadline = time.monotonic() + float(args.preflight_timeout_sec)
    next_log = 0.0
    last_error = "feedback not checked"
    while time.monotonic() < deadline:
        _check_stop(stop_requested, "feedback preflight")
        rclpy.spin_once(node, timeout_sec=0.05)
        last_error = node.safety_error(joints, float(args.max_feedback_age_sec)) or ""
        if not last_error:
            break
        if time.monotonic() >= next_log:
            node.get_logger().info(f"Preflight waiting: {last_error}")
            next_log = time.monotonic() + 1.0
    else:
        raise RuntimeError(f"Preflight timed out: {last_error}")

    # Require a stable downstream controller while DDS discovery settles.
    deadline = time.monotonic() + GRAPH_SETTLE_SEC
    subscriber_since: Optional[float] = None
    while time.monotonic() < deadline:
        _check_stop(stop_requested, "command graph preflight")
        rclpy.spin_once(node, timeout_sec=0.05)
        error = node.safety_error(joints, float(args.max_feedback_age_sec))
        if error:
            raise RuntimeError(f"Safety preflight failed: {error}")
        if node.publisher_count() != 1:
            raise RuntimeError(
                f"Refusing motion: another publisher owns {COMMAND_TOPIC}"
            )
        if node.expected_subscriber_count() == 1:
            subscriber_since = subscriber_since or time.monotonic()
            if time.monotonic() - subscriber_since >= SUBSCRIBER_STABLE_SEC:
                break
        else:
            subscriber_since = None
    else:
        raise RuntimeError(
            f"No {EXPECTED_COMMAND_SUBSCRIBER} subscriber remained matched on {COMMAND_TOPIC} for "
            f"{SUBSCRIBER_STABLE_SEC:.1f}s"
        )

    node.claim_command_topic()
    deadline = time.monotonic() + OWNERSHIP_SETTLE_SEC
    while time.monotonic() < deadline:
        _check_stop(stop_requested, "command ownership claim")
        rclpy.spin_once(node, timeout_sec=0.02)
        error = node.safety_error(joints, float(args.max_feedback_age_sec))
        if error:
            raise RuntimeError(f"Safety preflight failed: {error}")
        if node.publisher_count() > 1:
            raise RuntimeError(
                f"Refusing motion: another publisher appeared on {COMMAND_TOPIC}"
            )
        node.require_command_subscriber()
    node.require_exclusive_ownership()
    node.get_logger().info(f"Exclusive command ownership confirmed on {COMMAND_TOPIC}")


def _position_error(joint: str, target: float, position: float) -> float:
    error = target - position
    if joint == "J_TURN":
        return math.atan2(math.sin(error), math.cos(error))
    return error


def _velocity_command(
    joint: str,
    position_error: float,
    args: argparse.Namespace,
) -> float:
    if abs(position_error) <= float(args.tol):
        return 0.0
    max_velocity = min(float(args.max_vel), M445_MAX_VELOCITIES[joint])
    velocity = max(
        -max_velocity,
        min(max_velocity, float(args.kp) * position_error),
    )
    if abs(velocity) < float(args.min_vel):
        velocity = math.copysign(float(args.min_vel), velocity)
    return velocity


def _guard_outward_soft_limit(
    joint: str,
    position: float,
    velocity: float,
    horizon_sec: float,
) -> None:
    if joint == "J_TURN" or velocity == 0.0:
        return
    lower, upper = M445_SOFT_LIMITS[joint]
    projected = position + velocity * horizon_sec
    if (velocity < 0.0 and projected < lower) or (velocity > 0.0 and projected > upper):
        raise RuntimeError(
            f"Refusing outward {joint} velocity {velocity:.3f}: position "
            f"{position:.3f} projects to {projected:.3f}, outside soft limits "
            f"[{lower:.3f}, {upper:.3f}]"
        )


def _run(
    node: JointMover,
    targets: Mapping[str, float],
    args: argparse.Namespace,
    stop_requested: Callable[[], bool],
) -> None:
    start = time.monotonic()
    next_tick = start
    next_log = start
    in_tolerance_since: Optional[float] = None
    period = 1.0 / float(args.rate)
    guard_horizon = max(LIMIT_GUARD_HORIZON_SEC, 2.0 * period)

    while rclpy.ok():
        _check_stop(stop_requested, "motion")
        if time.monotonic() - start > float(args.timeout_sec):
            raise RuntimeError("Timed out before reaching targets")

        rclpy.spin_once(node, timeout_sec=min(0.03, period))
        error = node.safety_error(tuple(targets), float(args.max_feedback_age_sec))
        if error:
            raise RuntimeError(f"Safety check failed: {error}")
        node.require_exclusive_ownership()
        node.require_command_subscriber()

        velocities: Dict[str, float] = {}
        status = []
        all_in_tolerance = True
        for joint, target in targets.items():
            position = node.positions[joint]
            position_error = _position_error(joint, target, position)
            velocity = _velocity_command(joint, position_error, args)
            if velocity != 0.0:
                all_in_tolerance = False
            _guard_outward_soft_limit(joint, position, velocity, guard_horizon)
            velocities[joint] = velocity
            status.append(
                f"{joint}: q={position:.3f} tgt={target:.3f} "
                f"e={position_error:.3f} v={velocity:.3f}"
            )

        node.publish_velocity(velocities)
        now = time.monotonic()
        if now >= next_log:
            node.get_logger().info(" | ".join(status))
            next_log = now + 0.5

        if all_in_tolerance:
            in_tolerance_since = in_tolerance_since or now
            if now - in_tolerance_since >= float(args.hold_sec):
                node.get_logger().info("Targets reached")
                return
        else:
            in_tolerance_since = None

        next_tick += period
        if next_tick > time.monotonic():
            time.sleep(next_tick - time.monotonic())
        else:
            next_tick = time.monotonic()
    raise RuntimeError("ROS context stopped before targets were reached")


def _zero(node: JointMover) -> bool:
    if not node.command_claimed:
        return True
    successful = 0
    for attempt in range(ZERO_ATTEMPTS):
        try:
            node.require_command_subscriber()
            node.publish_velocity({})
            successful += 1
        except Exception as exc:
            node.get_logger().error(
                f"Zero attempt {attempt + 1}/{ZERO_ATTEMPTS} failed: {exc}"
            )
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception:
            pass
        if attempt + 1 < ZERO_ATTEMPTS:
            time.sleep(ZERO_PERIOD_SEC)
    if successful != ZERO_ATTEMPTS:
        node.get_logger().error(
            f"Only {successful}/{ZERO_ATTEMPTS} zero commands were published"
        )
    return successful == ZERO_ATTEMPTS


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        targets = _validate_args(args)
    except ValueError as exc:
        parser.error(str(exc))

    stop = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop
        stop = True

    old_handlers = {
        signal.SIGINT: signal.getsignal(signal.SIGINT),
        signal.SIGTERM: signal.getsignal(signal.SIGTERM),
    }
    rclpy.init(args=[])
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node: Optional[JointMover] = None
    result = 1
    try:
        node = JointMover()
        node.get_logger().info(f"Validated soft-limited targets: {targets}")
        _preflight(node, targets, args, lambda: stop)
        _run(node, targets, args, lambda: stop)
        result = 0
    except (InterruptedError, KeyboardInterrupt):
        if node:
            node.get_logger().warning("Interrupted; zeroing arm velocities")
        result = 130
    except Exception as exc:
        if node:
            node.get_logger().error(str(exc))
        else:
            print(f"move_joints_to_targets: {exc}", file=sys.stderr)
    finally:
        if node:
            if not _zero(node) and result == 0:
                result = 1
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
