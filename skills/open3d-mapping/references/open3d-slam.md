# Open3D SLAM Diagnostics

Use Open3D only for registration, odometry, submap, or loop-closure
diagnostics. For a dense colored artifact use `survey_scene.launch.py`: Open3D
buffering and dense-map space carving can drop scans or punch holes.

## Setup

Resolve the workspace and create an isolated run directory:

```bash
WS="$HOME/ros2_ws"; [[ -f "$WS/install/setup.bash" ]] || WS="$HOME/moleworks/ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
OUT="$HOME/mcap/open3d_slam_$(date -u +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
SKILL="${CODEX_HOME:-$HOME/.codex}/skills/open3d-mapping"
```

Input is `/mole/colored_point_cloud` (start it with `survey_scene.launch.py
launch_colorizer:=true` or the `mole_lidar_backprojection` launch with
`keep_uncolored_points:=true`) or the filtered front LiDAR.

Copy the installed Mole MID360 profile into the run directory; never edit the
installed/source YAML. It enables dense 1 cm mapping, disables loop closure and
spinning-lidar undistortion, and uses enlarged Livox buffers:

```bash
cp "$(ros2 pkg prefix --share open3d_slam_ros)/param/param_mole_livox_mid360.yaml" \
  "$OUT/param_mole_livox_mid360.yaml"
```

Launch beside the estimator, which keeps owning TF:

```bash
ros2 launch open3d_slam_ros mapping.launch.py \
  cloud_topic:=/mole/colored_point_cloud \
  parameter_folder_path:="$OUT" \
  parameter_filename:=param_mole_livox_mid360.yaml \
  map_saving_folder:="$OUT" \
  external_pose_frame:=map \
  external_pose_lookup_timeout_sec:=0.5 \
  publish_tf:=false
```

Expected topics include `/assembled_map`, `/dense_map`, `/submaps`,
`/scan2scan_odometry`, and `/scan2map_odometry`. If `/dense_map` is empty,
re-check `is_build_dense_map` in the copied config. The repo composition
`mole_mapping survey.launch.py enable_open3d_slam:=true` is the alternative
standalone bringup; its services are `/open3d_slam/save_map` and
`/open3d_slam/save_submaps` (see the `mole_mapping` README).

## Hole Or Throughput Diagnosis

Confirm the copied profile still has `odometry_buffer_size: 200`, the relevant
point-cloud/mapping buffers at `120`, dense voxel size `0.01`, and loop closure
disabled. For a controlled diagnostic, throttle the input:

```bash
python3 "$SKILL/scripts/throttle_pointcloud_topic.py" \
  /mole/colored_point_cloud /mole/colored_point_cloud_1hz \
  --rate-hz 1.0 --reliability reliable
```

Compare vertex counts in exported PLY files before changing the copied profile,
and against a `survey_scene` snapshot of the same interval.

## Save And Export

```bash
ros2 service call /save_map open3d_slam_msgs/srv/SaveMap '{}'
ros2 service call /save_submaps open3d_slam_msgs/srv/SaveSubmaps '{}'
python3 "$SKILL/scripts/export_pointcloud_topic.py" \
  /dense_map "$OUT" --basename dense_map \
  --durability transient_local --reliability reliable --formats ply,pcd
```

Record input topics, config changes, point counts, static/moving mode, and
cleanup PIDs in `RUN_NOTES.md`. Stop only the Open3D launch and throttle you
started.
