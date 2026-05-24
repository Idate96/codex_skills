#!/usr/bin/env python3
"""Render residual volume from a saved workspace-planner Terra checkpoint."""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
except ModuleNotFoundError:
    go = None
    make_subplots = None

from workspace_planner.live_map_case_bundle import snapshot_from_gridmap


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render counted/core and full-zone residual maps from a saved Terra checkpoint GridMap."
    )
    parser.add_argument("checkpoint", help="Checkpoint directory or checkpoint.yaml path.")
    parser.add_argument("--output-dir", help="Output directory. Defaults next to the checkpoint render.")
    parser.add_argument("--stack-log", help="Optional stack.log with WorkspacePlanner target/progress lines to overlay.")
    parser.add_argument("--topic", default=None, help="GridMap topic in the checkpoint bag. Defaults to metadata/grid_map.")
    parser.add_argument("--storage-id", default=None, help="rosbag2 storage id. Defaults to metadata/mcap.")
    parser.add_argument("--elevation-layer", default="elevation")
    parser.add_argument("--desired-layer", default="desired_elevation")
    parser.add_argument("--core-layer", default="dig_zone_core")
    parser.add_argument("--full-layer", default="dig_zone")
    parser.add_argument("--dug-layer", default="dug_zone")
    parser.add_argument("--zone-threshold", type=float, default=0.5)
    parser.add_argument("--max-depth-m", type=float, default=0.30, help="Colorbar upper bound for residual depth.")
    parser.add_argument("--xlim", nargs=2, type=float, metavar=("MIN", "MAX"))
    parser.add_argument("--ylim", nargs=2, type=float, metavar=("MIN", "MAX"))
    return parser.parse_args()


def checkpoint_dir(path: Path) -> Path:
    if path.name == "checkpoint.yaml":
        return path.parent
    return path


def read_metadata(checkpoint: Path) -> dict[str, Any]:
    metadata_path = checkpoint_dir(checkpoint) / "checkpoint.yaml"
    if not metadata_path.exists():
        raise RuntimeError(f"missing checkpoint metadata: {metadata_path}")
    loaded = yaml.safe_load(metadata_path.read_text())
    if not isinstance(loaded, dict):
        raise RuntimeError(f"invalid checkpoint metadata: {metadata_path}")
    return loaded


def resolve_bag_dir(checkpoint: Path, metadata: dict[str, Any]) -> Path:
    root = checkpoint_dir(checkpoint)
    uri = metadata.get("excavation_map_uri") or "excavation_map"
    bag = Path(str(uri))
    if not bag.is_absolute():
        bag = root / bag
    if not bag.exists():
        raise RuntimeError(f"missing excavation map bag directory: {bag}")
    return bag


def read_gridmap(checkpoint: Path, topic: str | None, storage_id: str | None):
    metadata = read_metadata(checkpoint)
    bag = resolve_bag_dir(checkpoint, metadata)
    topic_name = topic or str(metadata.get("excavation_map_topic") or "grid_map")
    storage = storage_id or str(metadata.get("excavation_map_storage_id") or "mcap")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic_name not in topic_types:
        raise RuntimeError(f"topic {topic_name!r} not in {bag}; available={sorted(topic_types)}")
    msg_type = get_message(topic_types[topic_name])

    latest = None
    while reader.has_next():
        read_topic, data, _stamp = reader.read_next()
        if read_topic == topic_name:
            latest = data
    if latest is None:
        raise RuntimeError(f"no messages on {topic_name!r} in {bag}")
    return snapshot_from_gridmap(deserialize_message(latest, msg_type)), metadata, bag


def parse_targets(stack_log: Path | None) -> list[dict[str, Any]]:
    if stack_log is None:
        return []
    if not stack_log.exists():
        raise RuntimeError(f"missing stack log: {stack_log}")
    target_re = re.compile(
        r"WorkspacePlanner target: attempt=(?P<attempt>\d+).*?target_r_m=(?P<r>[-0-9.]+) "
        r"target_theta_deg=(?P<theta>[-0-9.]+) target_xy=\((?P<x>[-0-9.]+),(?P<y>[-0-9.]+)\)"
    )
    progress_re = re.compile(
        r"WorkspacePlanner scoop_progress: attempt=(?P<attempt>\d+).*?local_removed_m3=(?P<local>[-0-9.]+) "
        r"global_removed_m3=(?P<global>[-0-9.]+) remaining_after_m3=(?P<remaining>[-0-9.]+).*?"
        r"weak_progress=(?P<weak>True|False)"
    )
    rows: dict[int, dict[str, Any]] = {}
    for line in stack_log.read_text(errors="ignore").splitlines():
        target_match = target_re.search(line)
        if target_match:
            attempt = int(target_match.group("attempt"))
            rows[attempt] = {
                "attempt": attempt,
                "x": float(target_match.group("x")),
                "y": float(target_match.group("y")),
                "r": float(target_match.group("r")),
                "theta": float(target_match.group("theta")),
                "local_removed": math.nan,
                "global_removed": math.nan,
                "remaining": math.nan,
                "weak": False,
            }
            continue
        progress_match = progress_re.search(line)
        if progress_match:
            attempt = int(progress_match.group("attempt"))
            if attempt in rows:
                rows[attempt].update(
                    local_removed=float(progress_match.group("local")),
                    global_removed=float(progress_match.group("global")),
                    remaining=float(progress_match.group("remaining")),
                    weak=progress_match.group("weak") == "True",
                )
    return [rows[key] for key in sorted(rows)]


def finite_zone(layer: np.ndarray, threshold: float) -> np.ndarray:
    return np.isfinite(layer) & (layer > float(threshold))


def auto_limits(mask: np.ndarray, xs: np.ndarray, ys: np.ndarray, padding_m: float) -> tuple[tuple[float, float], tuple[float, float]]:
    rows, cols = np.nonzero(mask)
    if rows.size == 0:
        return (float(xs[0]), float(xs[-1])), (float(ys[0]), float(ys[-1]))
    x_min = float(xs[int(np.min(cols))]) - padding_m
    x_max = float(xs[int(np.max(cols))]) + padding_m
    y_min = float(ys[int(np.min(rows))]) - padding_m
    y_max = float(ys[int(np.max(rows))]) + padding_m
    return (x_min, x_max), (y_min, y_max)


def write_targets_csv(path: Path, targets: list[dict[str, Any]]) -> None:
    if not targets:
        return
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "attempt",
                "x",
                "y",
                "r",
                "theta",
                "local_removed",
                "global_removed",
                "remaining",
                "weak",
            ],
        )
        writer.writeheader()
        for row in targets:
            writer.writerow(row)


def add_target_overlay_matplotlib(ax, targets: list[dict[str, Any]]) -> None:
    if not targets:
        return
    colors = ["tab:red" if row["weak"] else "tab:blue" for row in targets]
    ax.scatter(
        [row["x"] for row in targets],
        [row["y"] for row in targets],
        c=colors,
        s=28,
        edgecolors="white",
        linewidths=0.5,
        zorder=5,
    )
    for row in targets:
        ax.text(row["x"] + 0.04, row["y"] + 0.04, str(row["attempt"]), color="white", fontsize=7, zorder=6)


def add_target_overlay_plotly(fig, targets: list[dict[str, Any]], row: int, col: int, showlegend: bool) -> None:
    if not targets:
        return
    fig.add_trace(
        go.Scatter(
            x=[target["x"] for target in targets],
            y=[target["y"] for target in targets],
            mode="markers+text",
            text=[str(target["attempt"]) for target in targets],
            textposition="top center",
            marker={
                "size": 8,
                "color": ["red" if target["weak"] else "deepskyblue" for target in targets],
                "line": {"color": "white", "width": 1},
            },
            customdata=[
                [
                    target["attempt"],
                    target["local_removed"],
                    target["global_removed"],
                    target["remaining"],
                    target["weak"],
                ]
                for target in targets
            ],
            hovertemplate=(
                "attempt=%{customdata[0]}<br>x=%{x:.2f}<br>y=%{y:.2f}"
                "<br>local=%{customdata[1]:.4f} m3<br>global=%{customdata[2]:.4f} m3"
                "<br>remaining=%{customdata[3]:.4f} m3<br>weak=%{customdata[4]}<extra></extra>"
            ),
            name="targets",
            showlegend=showlegend,
        ),
        row=row,
        col=col,
    )


def main() -> int:
    args = parse_args()
    checkpoint = Path(args.checkpoint).expanduser().resolve()
    snapshot, metadata, bag_dir = read_gridmap(checkpoint, args.topic, args.storage_id)
    layers = snapshot.layers
    required = [args.elevation_layer, args.desired_layer, args.core_layer, args.full_layer]
    missing = [layer for layer in required if layer not in layers]
    if missing:
        raise RuntimeError(f"checkpoint GridMap missing required layers: {missing}; available={sorted(layers)}")

    elevation = layers[args.elevation_layer]
    desired = layers[args.desired_layer]
    finite = np.isfinite(elevation) & np.isfinite(desired)
    residual_m = np.maximum(np.where(finite, elevation - desired, 0.0), 0.0).astype(np.float32)
    core_mask = finite_zone(layers[args.core_layer], args.zone_threshold) & finite
    full_mask = finite_zone(layers[args.full_layer], args.zone_threshold) & finite
    boundary_mask = full_mask & ~core_mask
    dug_mask = finite_zone(layers[args.dug_layer], args.zone_threshold) if args.dug_layer in layers else np.zeros_like(core_mask)

    resolution = float(snapshot.resolution_m)
    area_m2 = resolution * resolution
    counted_core_m3 = float(np.sum(residual_m[core_mask]) * area_m2)
    not_dug_core_m3 = float(np.sum(residual_m[core_mask & ~dug_mask]) * area_m2)
    boundary_m3 = float(np.sum(residual_m[boundary_mask]) * area_m2)
    full_m3 = float(np.sum(residual_m[full_mask]) * area_m2)

    rows, cols = elevation.shape
    xs = snapshot.origin_xy_m[0] + np.arange(cols, dtype=np.float64) * resolution
    ys = snapshot.origin_xy_m[1] + np.arange(rows, dtype=np.float64) * resolution
    x_grid, y_grid = np.meshgrid(xs, ys)
    counted = np.where(core_mask, residual_m, np.nan)
    full_residual = np.where(full_mask, residual_m, np.nan)

    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else checkpoint_dir(checkpoint) / "residual_render"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    targets = parse_targets(Path(args.stack_log).expanduser().resolve() if args.stack_log else None)
    write_targets_csv(output_dir / "targets.csv", targets)

    xlim, ylim = auto_limits(full_mask | core_mask, xs, ys, padding_m=0.5)
    if args.xlim:
        xlim = (float(args.xlim[0]), float(args.xlim[1]))
    if args.ylim:
        ylim = (float(args.ylim[0]), float(args.ylim[1]))

    extent = [xs[0] - 0.5 * resolution, xs[-1] + 0.5 * resolution, ys[0] - 0.5 * resolution, ys[-1] + 0.5 * resolution]
    fig, axes = plt.subplots(1, 2, figsize=(15, 7), constrained_layout=True)
    for ax, data, title in (
        (axes[0], counted, "Counted residual inside core"),
        (axes[1], full_residual, "Physical residual inside full dig zone"),
    ):
        image = ax.imshow(data, origin="lower", extent=extent, cmap="magma", vmin=0.0, vmax=args.max_depth_m)
        ax.contour(x_grid, y_grid, core_mask.astype(float), levels=[0.5], colors="cyan", linewidths=1.2)
        ax.contour(x_grid, y_grid, full_mask.astype(float), levels=[0.5], colors="white", linewidths=0.8, linestyles="--")
        add_target_overlay_matplotlib(ax, targets)
        ax.set_aspect("equal")
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_xlabel("world x [m]")
        ax.set_ylabel("world y [m]")
        ax.set_title(title)
        colorbar = fig.colorbar(image, ax=ax, shrink=0.82)
        colorbar.set_label("residual depth [m]")
    fig.suptitle(
        f"{checkpoint_dir(checkpoint).name}: core residual {counted_core_m3:.3f} m3, "
        f"full-zone residual {full_m3:.3f} m3",
        fontsize=13,
    )
    png_path = output_dir / "residual.png"
    fig.savefig(png_path, dpi=180)
    plt.close(fig)

    html_path = None
    if go is not None and make_subplots is not None:
        html = make_subplots(
            rows=1,
            cols=2,
            subplot_titles=(f"Counted core residual: {counted_core_m3:.3f} m3", f"Full-zone residual: {full_m3:.3f} m3"),
            horizontal_spacing=0.08,
        )
        for col, data in ((1, counted), (2, full_residual)):
            html.add_trace(
                go.Heatmap(
                    x=xs,
                    y=ys,
                    z=data,
                    colorscale="Magma",
                    zmin=0.0,
                    zmax=args.max_depth_m,
                    colorbar={"title": "m"} if col == 2 else None,
                    hovertemplate="x=%{x:.2f}<br>y=%{y:.2f}<br>residual=%{z:.3f} m<extra></extra>",
                ),
                row=1,
                col=col,
            )
            add_target_overlay_plotly(html, targets, row=1, col=col, showlegend=(col == 1))
        html.update_xaxes(range=list(xlim), scaleanchor="y", scaleratio=1)
        html.update_yaxes(range=list(ylim))
        html.update_layout(
            title="Workspace planner residual render. Blue markers are normal actions; red markers are weak-progress actions.",
            width=1350,
            height=650,
        )
        html_path = output_dir / "residual.html"
        html.write_html(html_path, include_plotlyjs="cdn")

    summary_path = output_dir / "summary.md"
    summary_path.write_text(
        "# Workspace Planner Residual Render\n\n"
        f"- Checkpoint: `{checkpoint_dir(checkpoint)}`\n"
        f"- GridMap bag: `{bag_dir}`\n"
        f"- Frame: `{snapshot.map_frame}`\n"
        f"- Resolution: `{resolution:.3f} m`\n"
        f"- Counted physical residual in core: `{counted_core_m3:.4f} m3`\n"
        f"- Residual in core after excluding `dug_zone`: `{not_dug_core_m3:.4f} m3`\n"
        f"- Physical residual in excluded boundary: `{boundary_m3:.4f} m3`\n"
        f"- Physical residual in full dig zone: `{full_m3:.4f} m3`\n"
        f"- PNG: `{png_path}`\n"
        + (f"- HTML: `{html_path}`\n" if html_path else "")
        + (f"- Targets CSV: `{output_dir / 'targets.csv'}`\n" if targets else ""),
    )

    print(f"PNG: {png_path}")
    if html_path:
        print(f"HTML: {html_path}")
    print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
