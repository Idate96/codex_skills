#!/usr/bin/env python3
"""Republish the latest PointCloud2 message at a fixed rate."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_topic")
    parser.add_argument("output_topic")
    parser.add_argument("--rate-hz", type=float, default=1.0)
    parser.add_argument("--reliability", choices=("reliable", "best_effort"), default="reliable")
    return parser.parse_args()


def make_qos(reliability_name: str, depth: int) -> QoSProfile:
    reliability = (
        QoSReliabilityPolicy.RELIABLE
        if reliability_name == "reliable"
        else QoSReliabilityPolicy.BEST_EFFORT
    )
    return QoSProfile(depth=depth, reliability=reliability, history=QoSHistoryPolicy.KEEP_LAST)


class Throttle(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("colored_pointcloud_throttle")
        if args.rate_hz <= 0.0:
            raise ValueError("--rate-hz must be positive")
        self.latest: PointCloud2 | None = None
        self.received = 0
        self.published = 0
        self.create_subscription(PointCloud2, args.input_topic, self._callback, make_qos(args.reliability, 5))
        self.publisher = self.create_publisher(PointCloud2, args.output_topic, make_qos(args.reliability, 1))
        self.create_timer(1.0 / args.rate_hz, self._publish_latest)
        self.get_logger().info(
            f"republishing latest cloud from {args.input_topic} to {args.output_topic} at {args.rate_hz:g} Hz"
        )

    def _callback(self, msg: PointCloud2) -> None:
        self.latest = msg
        self.received += 1

    def _publish_latest(self) -> None:
        if self.latest is None:
            return
        self.publisher.publish(self.latest)
        self.published += 1
        if self.published % 10 == 1:
            self.get_logger().info(
                f"received={self.received} published={self.published} frame={self.latest.header.frame_id}"
            )


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = Throttle(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
