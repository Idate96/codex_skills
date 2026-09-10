# Open3D Manual Capture And SLAM Diagnostics

## Manual Colorized Accumulation

Resolve the workspace and create an isolated run directory:

```bash
WS="$HOME/ros2_ws"; [[ -f "$WS/install/setup.bash" ]] || WS="$HOME/moleworks/ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
OUT="/home/lorenzo/mcap/colored_site_map_1cm_$(date -u +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
```

Start the colorizer only when `/mole/colored_point_cloud` has no publisher:

```bash
ros2 launch mole_lidar_backprojection mole_lidar_backprojection.launch.py \
  use_sim_time:=false robot_namespace:=mole mode:=color \
  camera_topic:=/camMainView/image_raw \
  camera_info_topic:=/camMainView/camera_info \
  lidar_topic:=livox_lidar_publisher/lidar_front_left \
  keep_uncolored_points:=true
```

Expected colored cloud: frame `livox_front_left`, usually near 10 Hz, with XYZ plus packed RGB. Keep uncolored points so LiDAR points outside the camera projection receive fallback gray instead of being dropped.

Accumulate in `map`:

```bash
python3 /home/lorenzo/codex_skills/skills/open3d-mapping/scripts/accumulate_pointcloud_topic.py \
  /mole/colored_point_cloud "$OUT" \
  --basename colored_accum_1cm \
  --target-frame map --source-frame livox_front_left \
  --tf-warmup-sec 8 --duration-sec 45 \
  --min-radius 2.0 --max-radius 10.0 \
  --voxel-size 0.01 --reliability reliable --formats ply,pcd
```

Use radius limits for map extent. Use `--duration-sec 0` for manual stop.

## Open3D SLAM Setup

Copy the stock config into the run directory; never edit the installed/source YAML. Enable dense mapping and disable loop closures for a controlled static diagnostic:

```yaml
mapping:
  is_build_dense_map: true
  is_attempt_loop_closures: false
  dense_map_builder:
    map_voxel_size: 0.01
    scan_cropping:
      cropping_radius_max: 40.0
      cropping_radius_min: 0.0
saving:
  save_dense_submaps: true
```

Launch with the estimator owning TF:

```bash
ros2 launch open3d_slam_ros mapping.launch.py \
  cloud_topic:=/mole/colored_point_cloud \
  parameter_folder_path:="$OUT" \
  parameter_filename:=param_livox_dense_static.yaml \
  map_saving_folder:="$OUT" \
  external_pose_frame:=map \
  external_pose_lookup_timeout_sec:=0.5 \
  publish_tf:=false
```

Expected topics include `/assembled_map`, `/dense_map`, `/submaps`, `/scan2scan_odometry`, and `/scan2map_odometry`. If `/dense_map` is empty, re-check `is_build_dense_map` in the copied config.

## Hole Or Throughput Diagnosis

Open3D can overwrite old measurements when scan-processing or mapping buffers are too small, and dense-map space carving can remove visible voxels. For a controlled diagnostic, throttle the input and enlarge copied-config buffers:

```bash
python3 /home/lorenzo/codex_skills/skills/open3d-mapping/scripts/throttle_pointcloud_topic.py \
  /mole/colored_point_cloud /mole/colored_point_cloud_1hz \
  --rate-hz 1.0 --reliability reliable
```

```yaml
odometry:
  odometry_buffer_size: 200
  scan_processing:
    point_cloud_buffer_size: 120
mapping:
  ignore_minimum_refinement_fitness: true
  mapping_buffer_size: 120
  map_builder:
    space_carving:
      carve_space_every_n_scans: 1
  dense_map_builder:
    map_voxel_size: 0.01
    space_carving:
      carve_space_every_n_scans: 1
```

Compare vertex counts in exported PLY files. For the densest static inspection artifact, prefer the accumulator result when Open3D drops scans or carves holes.

## Save And Export

```bash
ros2 service call /save_map open3d_slam_msgs/srv/SaveMap '{}'
ros2 service call /save_submaps open3d_slam_msgs/srv/SaveSubmaps '{}'
python3 /home/lorenzo/codex_skills/skills/open3d-mapping/scripts/export_pointcloud_topic.py \
  /dense_map "$OUT" --basename dense_map \
  --durability transient_local --reliability reliable --formats ply,pcd
```

Record the input topics, config changes, point counts, static/moving mode, and cleanup PIDs in `RUN_NOTES.md`.
