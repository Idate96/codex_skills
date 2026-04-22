#!/usr/bin/env python3
"""Accumulate a live PointCloud2 topic into a cropped voxel map."""

from __future__ import annotations

import argparse
import math
import struct
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformException, TransformListener


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topic", help="PointCloud2 topic to accumulate")
    parser.add_argument("output_dir", help="Directory to write map artifacts into")
    parser.add_argument("--basename", default="accumulated_map")
    parser.add_argument("--target-frame", default="map")
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--min-radius", type=float, default=2.0)
    parser.add_argument("--max-radius", type=float, default=10.0)
    parser.add_argument("--voxel-size", type=float, default=0.01)
    parser.add_argument("--reliability", choices=("best_effort", "reliable"), default="best_effort")
    parser.add_argument("--tf-timeout-sec", type=float, default=0.5)
    parser.add_argument("--source-frame", default="", help="Known cloud frame used for optional TF warm-up")
    parser.add_argument("--tf-warmup-sec", type=float, default=0.0)
    parser.add_argument("--progress-every-sec", type=float, default=5.0)
    parser.add_argument("--formats", default="ply,pcd")
    return parser.parse_args()


def make_qos(reliability_name: str) -> QoSProfile:
    reliability = (
        QoSReliabilityPolicy.RELIABLE
        if reliability_name == "reliable"
        else QoSReliabilityPolicy.BEST_EFFORT
    )
    return QoSProfile(depth=1, reliability=reliability, history=QoSHistoryPolicy.KEEP_LAST)


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


def pointcloud_msg_to_arrays(msg: PointCloud2) -> tuple[np.ndarray, np.ndarray | None]:
    fields = [field.name for field in msg.fields]
    missing = {"x", "y", "z"} - set(fields)
    if missing:
        raise RuntimeError(f"PointCloud2 is missing required fields: {sorted(missing)}")
    records = pc2.read_points(msg, field_names=fields, skip_nans=True)
    return records_to_arrays(records, fields)


def transform_matrix(transform_msg) -> np.ndarray:
    t = transform_msg.transform.translation
    q = transform_msg.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise RuntimeError("Received zero-norm TF quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    matrix = np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy), t.x],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx), t.y],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy), t.z],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return matrix


class VoxelAccumulator(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("pointcloud_voxel_accumulator")
        self.args = args
        self.tf_buffer = Buffer()
        tf_qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        tf_static_qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=True, qos=tf_qos, static_qos=tf_static_qos
        )
        self.subscription = self.create_subscription(PointCloud2, args.topic, self.callback, make_qos(args.reliability))
        self.is_accumulating = args.tf_warmup_sec <= 0.0
        self.voxels: dict[tuple[int, int, int], list[np.ndarray | int]] = {}
        self.frames = 0
        self.frames_used = 0
        self.frames_tf_failed = 0
        self.raw_points = 0
        self.cropped_points = 0
        self.has_color = False
        self.start = time.monotonic()
        self.next_progress = self.start + args.progress_every_sec

    def begin_accumulation(self) -> None:
        self.is_accumulating = True
        self.start = time.monotonic()
        self.next_progress = self.start + self.args.progress_every_sec

    def callback(self, msg: PointCloud2) -> None:
        if not self.is_accumulating:
            return
        self.frames += 1
        try:
            xyz_sensor, colors = pointcloud_msg_to_arrays(msg)
        except Exception as exc:
            self.get_logger().warn(f"Skipping malformed cloud: {exc}")
            return

        self.raw_points += len(xyz_sensor)
        if len(xyz_sensor) == 0:
            return

        radii = np.linalg.norm(xyz_sensor, axis=1)
        mask = (radii >= self.args.min_radius) & (radii <= self.args.max_radius)
        xyz_sensor = xyz_sensor[mask]
        if colors is not None:
            colors = colors[mask]
        if len(xyz_sensor) == 0:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=Duration(seconds=self.args.tf_timeout_sec),
            )
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.args.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.args.tf_timeout_sec),
                )
            except TransformException as exc:
                self.frames_tf_failed += 1
                self.get_logger().warn(
                    f"TF lookup failed {self.args.target_frame} <- {msg.header.frame_id}: {exc}"
                )
                return

        matrix = transform_matrix(tf)
        xyz_map = xyz_sensor @ matrix[:3, :3].T + matrix[:3, 3]
        self.cropped_points += len(xyz_map)
        self.frames_used += 1
        self.has_color = self.has_color or colors is not None

        keys = np.floor(xyz_map / self.args.voxel_size).astype(np.int64)
        for idx, key_arr in enumerate(keys):
            key = (int(key_arr[0]), int(key_arr[1]), int(key_arr[2]))
            if key not in self.voxels:
                self.voxels[key] = [np.zeros(3, dtype=np.float64), np.zeros(3, dtype=np.float64), 0]
            voxel = self.voxels[key]
            voxel[0] += xyz_map[idx]
            if colors is not None:
                voxel[1] += colors[idx]
            else:
                voxel[1] += np.array([0.5, 0.5, 0.5], dtype=np.float64)
            voxel[2] += 1

        now = time.monotonic()
        if now >= self.next_progress:
            self.print_progress()
            self.next_progress = now + self.args.progress_every_sec

    def print_progress(self) -> None:
        elapsed = time.monotonic() - self.start
        self.get_logger().info(
            "elapsed=%.1fs frames=%d used=%d tf_failed=%d raw=%d cropped=%d voxels=%d"
            % (
                elapsed,
                self.frames,
                self.frames_used,
                self.frames_tf_failed,
                self.raw_points,
                self.cropped_points,
                len(self.voxels),
            )
        )

    def to_cloud(self) -> o3d.geometry.PointCloud:
        points = []
        colors = []
        for point_sum, color_sum, count in self.voxels.values():
            if count <= 0:
                continue
            points.append(point_sum / count)
            colors.append(color_sum / count)
        cloud = o3d.geometry.PointCloud()
        if not points:
            cloud.points = o3d.utility.Vector3dVector(np.empty((0, 3), dtype=np.float64))
            cloud.colors = o3d.utility.Vector3dVector(np.empty((0, 3), dtype=np.float64))
            return cloud
        cloud.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=np.float64))
        cloud.colors = o3d.utility.Vector3dVector(np.clip(np.asarray(colors, dtype=np.float64), 0.0, 1.0))
        return cloud


def warm_tf_buffer(node: VoxelAccumulator, source_frame: str, warmup_sec: float) -> None:
    deadline = time.monotonic() + warmup_sec
    ready = False
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            node.tf_buffer.lookup_transform(
                node.args.target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            ready = True
            break
        except TransformException:
            continue

    if ready:
        node.get_logger().info(
            f"TF warm-up ready for {node.args.target_frame} <- {source_frame}"
        )
    else:
        node.get_logger().warn(
            f"TF warm-up did not resolve {node.args.target_frame} <- {source_frame}; accumulating anyway"
        )
    node.begin_accumulation()


def main() -> int:
    args = parse_args()
    if args.min_radius < 0.0 or args.max_radius <= args.min_radius:
        raise SystemExit("--max-radius must be greater than --min-radius and both must be non-negative")
    if args.voxel_size <= 0.0:
        raise SystemExit("--voxel-size must be positive")
    if args.tf_warmup_sec > 0.0 and not args.source_frame:
        raise SystemExit("--tf-warmup-sec requires --source-frame")

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = VoxelAccumulator(args)
    if args.tf_warmup_sec > 0.0:
        warm_tf_buffer(node, args.source_frame, args.tf_warmup_sec)
    deadline = math.inf if args.duration_sec <= 0.0 else time.monotonic() + args.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.print_progress()
        cloud = node.to_cloud()
        for fmt in [item.strip().lower() for item in args.formats.split(",") if item.strip()]:
            if fmt not in {"ply", "pcd"}:
                raise SystemExit(f"Unsupported format: {fmt}")
            path = output_dir / f"{args.basename}.{fmt}"
            o3d.io.write_point_cloud(str(path), cloud, write_ascii=False, compressed=False)
            print(f"wrote={path}")
        bounds = cloud.get_axis_aligned_bounding_box() if cloud.points else None
        print(f"topic={args.topic}")
        print(f"target_frame={args.target_frame}")
        print(f"frames={node.frames}")
        print(f"frames_used={node.frames_used}")
        print(f"frames_tf_failed={node.frames_tf_failed}")
        print(f"raw_points={node.raw_points}")
        print(f"cropped_points={node.cropped_points}")
        print(f"voxel_points={len(cloud.points)}")
        print(f"voxel_size={args.voxel_size}")
        print(f"radius_min={args.min_radius}")
        print(f"radius_max={args.max_radius}")
        print(f"has_colors={cloud.has_colors()}")
        if bounds is not None:
            print(f"aabb_min={bounds.min_bound.tolist()}")
            print(f"aabb_max={bounds.max_bound.tolist()}")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
