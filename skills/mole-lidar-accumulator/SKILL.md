---
name: mole-lidar-accumulator
description: Run Moleworks ROS 2 pointcloud accumulation and pointcloud export with the `mole_lidar_accumulator` package. Use when asked to start/stop Mole pointcloud accumulation, enable accumulation before elevation mapping, export a short georeferenced LiDAR cloud in map frame, filter LiDAR points by sensor range such as 2-15 m, save PLY/PCD point clouds for CloudCompare, run a 10-second pointcloud capture, or run these workflows on the Perseverance robot remote.
---

# Mole Lidar Accumulator

## Overview

Use the ROS package `mole_lidar_accumulator` for Mole Livox pointcloud accumulation and point-based cloud export. Prefer this package for sparse filtered LiDAR accumulation; use Open3D only when the user explicitly asks for dense/colorized Open3D maps, submaps, or camera-colored output.

## Environment

Use the already selected robot workspace and Jazzy environment in every shell:

```bash
WS="${MOLE_ROS_WS:-$HOME/ros2_ws}"
[[ -f "$WS/install/setup.bash" ]] || WS="$HOME/moleworks/ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
command -v ros2
```

When the user says `perseverance`, `perservance`, or misspells it as `perserverance`, use the SSH alias:

```bash
ssh perseverance
```

On `perseverance`, if `command -v ros2` fails after sourcing, use the Moleworks ROS environment/container from `/home/lorenzo/ros2_ws/src/moleworks_ros/docker` before launching ROS nodes. Do not try to run accumulator launches from a shell where `ros2` is unavailable.

Do not stop unrelated robot nodes. Only stop `mole_lidar_accumulator` launch processes that you started.

## Preflight

Check the package and live inputs:

```bash
ros2 pkg executables mole_lidar_accumulator
ros2 topic list | rg '/mole/livox_lidar_publisher/lidar_front_left_filtered|/tf|/tf_static|/mole/state'
timeout 6 ros2 topic hz /mole/livox_lidar_publisher/lidar_front_left_filtered
```

Check TF before exporting in `map`:

```bash
timeout 15 ros2 run tf2_ros tf2_echo map livox_front_left 2>&1 | head -50
```

Check TF before the short-window accumulator when the target frame is `odom`:

```bash
timeout 15 ros2 run tf2_ros tf2_echo odom livox_front_left 2>&1 | head -50
```

If TF lookup fails, do not fake a frame; fix/start the estimator or choose the frame the user requested.

## Short-Window Accumulation

Use this when the user wants the accumulated topic for perception/elevation mapping:

```bash
ros2 launch mole_lidar_accumulator accumulate.launch.py robot_namespace:=mole
```

Default behavior from `config/accumulate.yaml`:

- input: `/mole/livox_lidar_publisher/lidar_front_left_filtered`
- output: `/mole/livox_lidar_publisher/lidar_front_left_accum`
- target frame: `odom`
- window: `0.5` seconds
- publish rate: `10` Hz

Verify:

```bash
ros2 topic echo /mole/livox_lidar_publisher/lidar_front_left_accum --once --field header
timeout 6 ros2 topic hz /mole/livox_lidar_publisher/lidar_front_left_accum
```

For perception bringup/survey launch, enable the existing integration instead of starting a duplicate accumulator. The survey launch requires robot self filtering when accumulation is enabled:

```bash
enable_pointcloud_accumulation:=true enable_robot_self_filter:=true
```

## Pointcloud Export

Use the exporter when the user wants a saved cloud artifact. The exporter appends transformed XYZ points in `export_frame` and writes PLY/PCD plus a YAML sidecar. It does not voxelize, register, mesh, or color points.

For moving-machine dense survey captures, keep `export_frame:=map`, `use_latest_tf:=false`, and a nonzero TF timeout such as `tf_timeout_s:=2.0`. This uses timestamped TF for each cloud instead of stamping all points with the latest pose. Use `min_range_m` and `max_range_m` to crop points in the source LiDAR frame before transforming them into `map`.

Default behavior from `config/export.yaml`:

- input: `/mole/livox_lidar_publisher/lidar_front_left_filtered`
- export frame: `map`
- output directory: `~/mole_lidar_exports`
- format: `ply`
- source-frame range filter: disabled by default with `min_range_m: 0.0`, `max_range_m: 0.0`
- save service: `/mole/save_georeferenced_cloud`
- clear service: `/mole/clear_georeferenced_cloud`
- auto-export on shutdown: `true`

### 10-Second Map Export

For bounded map-frame captures, prefer starting the exporter, waiting for the service, sleeping for the requested duration, calling the save service, then stopping the process. Do not rely on `timeout --signal=INT` shutdown export for large ASCII clouds; SIGINT can interrupt the final write.

```bash
set -euo pipefail
OUT="$HOME/mcap/mole_lidar_map_export_$(date -u +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
ros2 run mole_lidar_accumulator export_node.py \
  --ros-args \
  -r __ns:=/mole \
  -p input_topic:=/mole/livox_lidar_publisher/lidar_front_left_filtered \
  -p export_frame:=map \
  -p output_dir:="$OUT" \
  -p output_basename:=mole_lidar_map_2to15m_10s \
  -p output_format:=ply \
  -p min_range_m:=2.0 \
  -p max_range_m:=15.0 \
  -p tf_cache_s:=60.0 \
  -p tf_timeout_s:=2.0 \
  -p use_latest_tf:=false \
  -p auto_export_on_shutdown:=false \
  >"$OUT/exporter.log" 2>&1 &
PID=$!
cleanup_exporter() {
  kill -INT "$PID" >/dev/null 2>&1 || true
  wait "$PID" >/dev/null 2>&1 || true
}
trap cleanup_exporter EXIT
save_ready=false
for _ in $(seq 1 40); do
  if ros2 service type /mole/save_georeferenced_cloud >/dev/null 2>&1; then
    save_ready=true
    break
  fi
  sleep 0.25
done
if [[ "$save_ready" != true ]]; then
  echo "save service did not appear" >&2
  exit 1
fi
sleep 10
timeout 240 ros2 service call /mole/save_georeferenced_cloud std_srvs/srv/Trigger "{}"
cleanup_exporter
trap - EXIT
ls -lh "$OUT"
```

Expected artifacts:

- `mole_lidar_map_2to15m_10s_<timestamp>.ply`
- `mole_lidar_map_2to15m_10s_<timestamp>.yaml`

Read the YAML sidecar for `point_count`, accepted/skipped clouds, bounds, source frames, `export_frame`, GNSS reference metadata, and filters. For this workflow, verify `export_frame: map`, `use_latest_tf: false`, `min_range_m: 2.0`, `max_range_m: 15.0`, and `range_filter_frame: source_cloud_frame_before_transform`. If `point_count` is zero or no files were written, inspect `exporter.log` for missing TF or empty input clouds.

### Export From Running Node

If the exporter is already running and should remain running:

```bash
ros2 service call /mole/save_georeferenced_cloud std_srvs/srv/Trigger {}
```

Clear only when the user asks for a fresh buffer:

```bash
ros2 service call /mole/clear_georeferenced_cloud std_srvs/srv/Trigger {}
```

## Cleanup

Find only accumulator/exporter processes:

```bash
pgrep -af 'mole_lidar_accumulator|lidar_accumulator|lidar_pointcloud_exporter' || true
```

Stop only the launch/process you started. Do not stop the estimator, sensor drivers, perception bringup, elevation mapping, excavation mapping, or Foxglove bridge unless explicitly requested.
