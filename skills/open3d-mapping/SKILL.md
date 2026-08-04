---
name: open3d-mapping
description: "Capture, generate, debug, or sync dense 1 cm camera-colored Moleworks LiDAR maps. Use for static accumulations, Open3D SLAM diagnostics, PLY/PCD export, and Foxglove-ready artifacts."
---

# Open3D Mapping

Default to the static camera-colored accumulator at `0.01 m` voxel size. Adjust capture duration and min/max radius before changing density; change voxel size only when the user explicitly asks.

## Route

1. **Dense static site map:** use `record_colored_map_fast.sh` (default).
2. **Manual static capture:** run the colorizer and `accumulate_pointcloud_topic.py` when debugging or customizing topics/frames.
3. **Open3D SLAM:** use only for submaps, odometry, registration, or loop-closure diagnostics. It is not the default dense artifact because lagging buffers and space carving can drop points or create holes.
4. **Existing map:** export or sync it without restarting mapping nodes.

Read [references/manual-and-slam.md](references/manual-and-slam.md) only for manual capture, Open3D SLAM, or hole/throughput diagnosis.

## Fast Capture

```bash
SKILL="${CODEX_HOME:-$HOME/.codex}/skills/open3d-mapping"

# Default 45-second static capture
"$SKILL/scripts/record_colored_map_fast.sh"

# Let the user stop capture manually
"$SKILL/scripts/record_colored_map_fast.sh" --duration-sec 0

# Capture and sync after successful artifact generation
"$SKILL/scripts/record_colored_map_fast.sh" --sync-perseverance
```

The wrapper auto-detects `~/ros2_ws` or `~/moleworks/ros2_ws`; set `MOLEWORKS_ROS_WS` to override. It checks required topics, reuses an existing `/mole/colored_point_cloud` publisher instead of starting a duplicate, copies the installed Mole MID360 Open3D profile, warms TF, accumulates in `map`, writes PLY/PCD plus run notes, and stops only the temporary colorizer it created.

## Required Preflight

Require these before capture:

- `/mole/livox_lidar_publisher/lidar_front_left`
- `/camMainView/image_raw`
- `/camMainView/camera_info`
- `/tf` and `/tf_static`
- `Main -> livox_front_left` and `map -> livox_front_left` with 10–15 second TF timeouts
- sufficient disk space under the selected output root

Keep `publish_tf:=false` when running Open3D beside the estimator. Open3D must not own a second localization TF tree.

## Artifacts And Verification

The fast path writes a timestamped directory under `/home/lorenzo/mcap` by default. Require:

- `colored_accum_1cm.ply`
- `colored_accum_1cm.pcd`
- `RUN_NOTES.md`
- topic/header/backprojection logs

Verify the files are non-empty and record point counts. A static capture maps only the visible frustum; move or pan the sensor rig for site coverage.

For manual Open3D export, use the bundled `export_pointcloud_topic.py`; for input throttling during SLAM diagnosis, use `throttle_pointcloud_topic.py`.

## Sync And Cleanup

Normalize `perserverance` or `perservance` to the configured SSH host `perseverance`. Verify local and remote checksums after manual `rsync`.

Stop only nodes started by this workflow: the temporary Open3D launch, colorizer, throttle, or accumulator. Do not stop the estimator, sensor drivers, elevation/excavation mapping, or Foxglove unless the user explicitly asks.
