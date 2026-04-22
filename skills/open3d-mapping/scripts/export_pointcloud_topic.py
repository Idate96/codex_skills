#!/usr/bin/env python3
"""Export one ROS 2 PointCloud2 topic message to PLY/PCD with RGB preservation."""

from __future__ import annotations

import argparse
import struct
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topic", help="PointCloud2 topic to export, e.g. /dense_map")
    parser.add_argument("output_dir", help="Directory to write output files into")
    parser.add_argument("--basename", default="cloud", help="Output basename without extension")
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--formats", default="ply,pcd", help="Comma-separated output formats: ply,pcd")
    parser.add_argument("--reliability", choices=("reliable", "best_effort"), default="reliable")
    parser.add_argument("--durability", choices=("volatile", "transient_local"), default="volatile")
    return parser.parse_args()


def make_qos(args: argparse.Namespace) -> QoSProfile:
    reliability = (
        QoSReliabilityPolicy.RELIABLE
        if args.reliability == "reliable"
        else QoSReliabilityPolicy.BEST_EFFORT
    )
    durability = (
        QoSDurabilityPolicy.TRANSIENT_LOCAL
        if args.durability == "transient_local"
        else QoSDurabilityPolicy.VOLATILE
    )
    return QoSProfile(
        depth=1,
        reliability=reliability,
        durability=durability,
        history=QoSHistoryPolicy.KEEP_LAST,
    )


class Grabber(Node):
    def __init__(self, topic: str, qos: QoSProfile) -> None:
        super().__init__("pointcloud_topic_exporter")
        self.msg: PointCloud2 | None = None
        self.create_subscription(PointCloud2, topic, self._callback, qos)

    def _callback(self, msg: PointCloud2) -> None:
        self.msg = msg


def packed_rgb_to_colors(values: np.ndarray) -> np.ndarray:
    if np.issubdtype(values.dtype, np.floating):
        rgb_u32 = values.astype(np.float32, copy=False).view(np.uint32)
    else:
        rgb_u32 = values.astype(np.uint32, copy=False)
    return np.column_stack(
        [
            ((rgb_u32 >> 16) & 255),
            ((rgb_u32 >> 8) & 255),
            (rgb_u32 & 255),
        ]
    ).astype(np.float64) / 255.0


def records_to_arrays(records: object, fields: list[str]) -> tuple[np.ndarray, np.ndarray | None]:
    if isinstance(records, np.ndarray):
        if records.dtype.names:
            xyz = np.column_stack([records["x"], records["y"], records["z"]]).astype(np.float64)
            colors = None
            if "rgb" in records.dtype.names:
                colors = packed_rgb_to_colors(records["rgb"])
            elif "rgba" in records.dtype.names:
                colors = packed_rgb_to_colors(records["rgba"])
            elif {"r", "g", "b"}.issubset(records.dtype.names):
                colors = np.column_stack([records["r"], records["g"], records["b"]]).astype(np.float64) / 255.0
            return xyz, colors
        return records[:, :3].astype(np.float64), None

    rows = list(records)
    index = {name: i for i, name in enumerate(fields)}
    xyz = np.array([[row[index["x"]], row[index["y"]], row[index["z"]]] for row in rows], dtype=np.float64)
    colors = None
    if "rgb" in index or "rgba" in index:
        field = "rgb" if "rgb" in index else "rgba"
        vals = np.array([row[index[field]] for row in rows])
        colors = packed_rgb_to_colors(vals)
    elif {"r", "g", "b"}.issubset(index):
        colors = np.array([[row[index["r"]], row[index["g"]], row[index["b"]]] for row in rows], dtype=np.float64) / 255.0
    return xyz, colors


def cloud_from_msg(msg: PointCloud2) -> o3d.geometry.PointCloud:
    fields = [field.name for field in msg.fields]
    required = {"x", "y", "z"}
    missing = required - set(fields)
    if missing:
        raise RuntimeError(f"PointCloud2 is missing required fields: {sorted(missing)}")

    records = pc2.read_points(msg, field_names=fields, skip_nans=True)
    xyz, colors = records_to_arrays(records, fields)
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    if colors is not None and len(colors) == len(xyz):
        cloud.colors = o3d.utility.Vector3dVector(colors)
    return cloud


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Grabber(args.topic, make_qos(args))
    deadline = time.monotonic() + args.timeout_sec
    while time.monotonic() < deadline and node.msg is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    msg = node.msg
    node.destroy_node()
    rclpy.shutdown()

    if msg is None:
        raise SystemExit(f"No PointCloud2 message received on {args.topic} within {args.timeout_sec:.1f}s")

    cloud = cloud_from_msg(msg)
    formats = [fmt.strip().lower() for fmt in args.formats.split(",") if fmt.strip()]
    if not formats:
        raise SystemExit("No output formats requested")

    for fmt in formats:
        if fmt not in {"ply", "pcd"}:
            raise SystemExit(f"Unsupported format: {fmt}")
        path = output_dir / f"{args.basename}.{fmt}"
        o3d.io.write_point_cloud(str(path), cloud, write_ascii=False, compressed=False)
        print(f"wrote={path}")

    bounds = cloud.get_axis_aligned_bounding_box() if cloud.points else None
    print(f"topic={args.topic}")
    print(f"frame={msg.header.frame_id}")
    print(f"points={len(cloud.points)}")
    print(f"has_colors={cloud.has_colors()}")
    print(f"fields={[field.name for field in msg.fields]}")
    if bounds is not None:
        print(f"aabb_min={bounds.min_bound.tolist()}")
        print(f"aabb_max={bounds.max_bound.tolist()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
