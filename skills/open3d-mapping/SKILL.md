---
name: open3d-mapping
description: Capture, generate, or sync dense 1 cm camera-colored Moleworks LiDAR maps and Open3D/Foxglove point-cloud artifacts.
---

# Open3D Mapping

## Overview

Use this for live robot-side dense mapping with camera color projected onto LiDAR.

Default density is always `0.01 m` (`1 cm`). Adjust the map extent with min/max radius first; only change voxel size if the user explicitly asks.

For the common static 1 cm colored map, run the wrapper first. It performs quick topic checks, creates the run directory, prepares the copied dense config, starts/stops the colorizer, warms TF before accumulation, writes `RUN_NOTES.md`, and can sync the result:

```bash
/home/lorenzo/.codex/skills/open3d-mapping/scripts/record_colored_map_fast.sh
```

If the user asks to sync to Perseverance:

```bash
/home/lorenzo/.codex/skills/open3d-mapping/scripts/record_colored_map_fast.sh --sync-perseverance
```

If the user wants to decide when to stop, run an open-ended capture and interrupt it later:

```bash
/home/lorenzo/.codex/skills/open3d-mapping/scripts/record_colored_map_fast.sh --duration-sec 0
```

Preferred paths:

1. Fast static/high-density map: run `scripts/record_colored_map_fast.sh`. This is the default.
2. Manual static/high-density map: start `mole_lidar_backprojection`, then run `scripts/accumulate_pointcloud_topic.py` against `/mole/colored_point_cloud`. This behaves like Foxglove accumulation: no ICP rejection, no space carving.
3. Open3D SLAM map/submaps: use `open3d_slam_ros` only when you need Open3D submaps, odometry topics, or loop-closure-style outputs. Do not use it as the default dense inspection artifact; its dense mapper can space-carve holes and process far fewer points than the accumulator.
4. Save PLY/PCD artifacts, sync them when requested, and stop only temporary mapping/colorization nodes.

## Manual Preflight

The fast wrapper performs the quick checks automatically. Use this section when debugging or running the manual path.

Source the robot workspace first:

```bash
source /opt/ros/jazzy/setup.bash
source /home/lorenzo/ros2_ws/install/setup.bash
```

Required topics:

- `/mole/livox_lidar_publisher/lidar_front_left`
- `/camMainView/image_raw`
- `/camMainView/camera_info`
- `/tf` and `/tf_static`

Required packages:

```bash
ros2 pkg executables mole_lidar_backprojection
ros2 pkg executables open3d_slam_ros
```

Check TF with long timeouts:

```bash
timeout 15 bash -lc 'ros2 run tf2_ros tf2_echo Main livox_front_left 2>&1' | head -50
timeout 15 bash -lc 'ros2 run tf2_ros tf2_echo map livox_front_left 2>&1' | head -50
```

Expected:

- `Main -> livox_front_left` exists, usually static at time `0.0`.
- `map -> livox_front_left` exists from the estimator.

If `/mole/colored_point_cloud` already exists, inspect it first and avoid starting a duplicate backprojection node.

## Manual Run Directory

Create a run directory under `/home/lorenzo/mcap`:

```bash
OUT=/home/lorenzo/mcap/colored_site_map_1cm_$(date -u +%Y%m%d_%H%M%S)
mkdir -p "$OUT"
cp /home/lorenzo/ros2_ws/src/open3d_slam/ros/open3d_slam_ros/param/param_robosense_rs16.yaml \
  "$OUT/param_livox_dense_static.yaml"
```

Edit only the copied config. Do not modify the installed/source Open3D parameter files.

Set these values in `$OUT/param_livox_dense_static.yaml`:

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

The stock Robosense/Livox config has dense mapping disabled; if this is not changed, `/dense_map` will be empty.

## Manual Backprojection

Launch the temporary colorizer:

```bash
ros2 launch mole_lidar_backprojection mole_lidar_backprojection.launch.py \
  use_sim_time:=false \
  robot_namespace:=mole \
  mode:=color \
  camera_topic:=/camMainView/image_raw \
  camera_info_topic:=/camMainView/camera_info \
  lidar_topic:=livox_lidar_publisher/lidar_front_left \
  keep_uncolored_points:=true
```

Verify `/mole/colored_point_cloud`:

```bash
ros2 topic info /mole/colored_point_cloud --verbose
ros2 topic echo /mole/colored_point_cloud --once --field header
ros2 topic echo /mole/colored_point_cloud --once --field point_step
```

Expected:

- frame: `livox_front_left`
- `point_step` is typically `16`, consistent with XYZ plus packed RGB
- typical static-site rate is near 10 Hz
- point count should stay close to the input LiDAR cloud; points that do not project into the camera image get fallback gray RGB instead of being dropped

Avoid `ros2 topic echo --field fields` on this large topic during fast runs; it can time out without adding useful signal.

## Static 1 cm Accumulator

Prefer this for static tests and dense site snapshots. It accumulates in `map`, crops in the sensor frame, and exports PLY/PCD directly:

```bash
python3 /home/lorenzo/.codex/skills/open3d-mapping/scripts/accumulate_pointcloud_topic.py \
  /mole/colored_point_cloud \
  "$OUT" \
  --basename colored_accum_1cm \
  --target-frame map \
  --source-frame livox_front_left \
  --tf-warmup-sec 8 \
  --duration-sec 45 \
  --min-radius 2.0 \
  --max-radius 10.0 \
  --voxel-size 0.01 \
  --reliability reliable \
  --formats ply,pcd
```

Use `--min-radius` and `--max-radius` for map size. Keep `--voxel-size 0.01` unless the user explicitly changes density.
Use `--duration-sec 0` when the user wants to stop the run manually; interrupting the wrapper still lets the accumulator write artifacts.

## Start Open3D

Run Open3D against the colored cloud in the estimator `map` frame:

```bash
ros2 launch open3d_slam_ros mapping.launch.py \
  cloud_topic:=/mole/colored_point_cloud \
  parameter_folder_path:=$OUT \
  parameter_filename:=param_livox_dense_static.yaml \
  map_saving_folder:=$OUT \
  external_pose_frame:=map \
  external_pose_lookup_timeout_sec:=0.5 \
  publish_tf:=false
```

Keep `publish_tf:=false` when the estimator is running. Open3D should not own a second localization TF tree.

Use this path for SLAM diagnostics, submaps, and odometry topics. If the user wants the densest colored site map or complains about holes, use or sync the accumulator output instead. In one real run, Open3D `/dense_map` exported `286704` points while the accumulator from the same blind-spot pass exported `3698162` 1 cm points.

Known Open3D SLAM dense-map failure modes on Mole:

- The ROS input subscription uses depth `100`, but Open3D then skips the first 5 clouds and uses internal ring buffers. With the stock config, `mapping_buffer_size`, `odometry.scan_processing.point_cloud_buffer_size`, and scan-processing buffers are `1`; old measurements are silently overwritten when workers lag.
- The dense-map worker consumes `registeredCloudBuffer_`, whose size comes from `odometry.scan_processing.point_cloud_buffer_size`. At 10 Hz input with stock carving enabled, dense insertion has been observed at only `0.45-1.4 Hz`, so most frames never reach `/dense_map`.
- Dense-map carving is called for every registered dense insertion with `isPerformCarving=true`. The stock `carve_space_every_n_scans: 10` removes voxels along LiDAR rays and can create visible holes.
- Setting `carve_space_every_n_scans: 1` effectively disables carving in the current implementation because the code only carves when `nScansInserted % carve_space_every_n_scans == 1`.

For a controlled Open3D SLAM debug run, throttle the colored cloud and disable carving:

```bash
python3 /home/lorenzo/.codex/skills/open3d-mapping/scripts/throttle_pointcloud_topic.py \
  /mole/colored_point_cloud \
  /mole/colored_point_cloud_1hz \
  --rate-hz 1.0 \
  --reliability reliable
```

Patch the copied config for debugging:

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
    scan_cropping:
      cropping_radius_max: 40.0
      cropping_radius_min: 0.0
    space_carving:
      carve_space_every_n_scans: 1
```

Then launch Open3D on `/mole/colored_point_cloud_1hz`. In a real debug run, disabling carving and throttling to `1 Hz` improved dense insertion from roughly `725-2199 ms/cloud` to `5-36 ms/cloud`, and `/dense_map` increased from `286704` points to `642137` points despite the lower input rate.

Check outputs:

```bash
ros2 topic list | rg 'assembled_map|dense_map|submaps|scan2'
```

Expected map topics:

- `/assembled_map`
- `/dense_map`
- `/submaps`
- `/scan2scan_odometry`
- `/scan2map_odometry`

If `/dense_map` has zero points, re-check that dense mapping is enabled in the copied config and relaunch Open3D.

## Save Artifacts

After enough scanning time, call the Open3D save services:

```bash
ros2 service call /save_map open3d_slam_msgs/srv/SaveMap '{}'
ros2 service call /save_submaps open3d_slam_msgs/srv/SaveSubmaps '{}'
```

Colored Open3D service outputs are usually written as `.ply`, even though the service implementation names `map.pcd` internally.

Export the latched dense topic in `map` with the bundled script:

```bash
python3 /home/lorenzo/.codex/skills/open3d-mapping/scripts/export_pointcloud_topic.py \
  /dense_map \
  "$OUT" \
  --basename dense_map \
  --durability transient_local \
  --reliability reliable \
  --formats ply,pcd
```

Expected artifacts:

- `dense_map.ply`: dense colored map in `map`
- `dense_map.pcd`: same dense map in PCD
- `map.ply`: assembled Open3D map in `map`
- `submap_0.ply`: Open3D submap in `map`
- `denseSubmap_0.ply`: dense submap saved at shutdown, usually in Open3D internal submap coordinates
- `param_livox_dense_static.yaml`: exact config used

Add a short `RUN_NOTES.md` with inputs, config changes, point counts, and whether the run was static or moving.

## Sync

If the user asks to sync the generated map, prefer the wrapper flag:

```bash
/home/lorenzo/.codex/skills/open3d-mapping/scripts/record_colored_map_fast.sh --sync-perseverance
```

If the map already exists locally, sync manually with `rsync` and verify checksums:

```bash
RUN=/home/lorenzo/mcap/colored_site_map_1cm_YYYYMMDD_HHMMSS
ssh -o BatchMode=yes perseverance "mkdir -p /home/lorenzo/mcap/$(basename "$RUN")"
rsync -avh "$RUN/" "perseverance:/home/lorenzo/mcap/$(basename "$RUN")/"
sha256sum "$RUN"/*
ssh -o BatchMode=yes perseverance "sha256sum /home/lorenzo/mcap/$(basename "$RUN")/*"
```

Normalize user typos `perserverance` and `perservance` to the configured SSH host `perseverance`.

## Cleanup

Stop only the temporary nodes you started:

- `open3d_slam_ros` mapping launch
- `mole_lidar_backprojection` color launch
- `pointcloud_voxel_accumulator`, if still running

Verify cleanup:

```bash
pgrep -af 'open3d|mole_lidar_backprojection|mapping_node|accumulate_pointcloud_topic|record_colored_map_fast' || true
ros2 topic info /mole/colored_point_cloud --verbose 2>/dev/null || true
ros2 service list | rg '^/(save_map|save_submaps)$|open3d' || true
```

Do not stop the estimator, elevation mapping, excavation mapping, sensor drivers, or Foxglove unless the user explicitly asks.

## Notes

- For a real site map, move or pan the sensor rig through the area; a static run only maps the visible frustum.
- For static/high-density maps, prefer the 1 cm accumulator over Open3D SLAM. Open3D can drop scans through registration gates and carve voxels; the accumulator keeps the Foxglove-style point density.
- If an Open3D SLAM dense map has holes, first compare point counts with `grep -a -m1 'element vertex' *.ply`. Usually the fix is to use the latest `colored_accum_1cm.{ply,pcd}` artifact, not to trust the SLAM dense output.
- Use `keep_uncolored_points:=true` for dense mapping. Otherwise color backprojection drops any LiDAR point outside the camera projection, which can make Livox non-repeating scans much sparser than Foxglove accumulation.
- Use the raw LiDAR topic for colorization. The filtered topic is fine for LiDAR-only Open3D tests, but color backprojection expects the live LiDAR cloud and camera sync.
- If colorization publishes very few points, check `Main -> livox_front_left`, camera intrinsics, and image/LiDAR timestamps.
- TF lookup warnings at startup are common while buffers warm up. Use `--source-frame livox_front_left --tf-warmup-sec 8` with the accumulator to avoid losing the first useful frames. Persistent future/past extrapolation warnings mean the estimator TF stream or sensor timestamps need attention.
