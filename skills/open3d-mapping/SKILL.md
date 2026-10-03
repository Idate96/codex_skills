---
name: open3d-mapping
description: "Run Moleworks site surveys with the `mole_mapping` launches: a datum-anchored elevation/excavation map saved to `mole_maps`, or a dense 1 cm camera-colored 3D scene exported as PLY/PCD. Use for site surveys, elevation maps, colored point clouds, offline scene-to-GridMap conversion, and Open3D SLAM diagnostics."
---

# Site Survey And Open3D Mapping

The survey owner is `moleworks_ros/perception/mole_mapping` (`README.md`,
`launch/survey_excavation.launch.py`, `launch/survey_scene.launch.py`,
`scripts/capture_scene.py`; added in `e221b15b1`). Read the README before
changing arguments; this skill is the agent runbook around it.

## Route

1. **Elevation / excavation map** (terrain GridMap for Terra or a saved site
   surface): `survey_excavation.launch.py`, then `save_map.launch.py`.
2. **Colored 3D scene** (XYZ/RGB at 1 cm in `map`, including walls and
   overhangs): `survey_scene.launch.py`, then `/mole/save_scene`.
3. Both can run side by side; scene capture is independent of the GridMap.
4. **Terra already owns elevation/excavation mapping:** do not start
   `survey_excavation`; save Terra's running map with `save_map.launch.py`.
5. **Open3D SLAM** only for registration, odometry, submap, or loop-closure
   diagnostics: read [references/open3d-slam.md](references/open3d-slam.md).
6. Sparse uncolored LiDAR export: `mole-lidar-accumulator`.

## Prerequisites

Every survey needs the fixed site datum. Run the estimator with the site
reference overlay and pass the same YAML to the survey; otherwise `map` is
wherever the estimator initialized and saved maps will not align later. The
`robot-startup` "Site survey" section has the bringup.

```bash
: "${ROBOT_WS:?Set ROBOT_WS to the verified main workspace}"
SITE_REFERENCE_YAML="$ROBOT_WS/install/mole_estimator/share/mole_estimator/config/mole_estimator_reference_<site>.yaml"
ros2 param get /mole/mole_estimator_node gnss_params.useGnssReference   # must be True
```

- `low_level` running; estimator with that overlay and `/mole/state`
  top-level `status: 1` (`STATUS_OK`).
- `perception` with LiDAR and self filter but no elevation mapping (wrapper
  `--no-elevation-mapping`, i.e. `enable_elevation_mapping:=false`). The
  wrapper still starts perception's own `/mole/excavation_mapping`; the
  survey map is the global `/excavation_mapping`, which `save_map` targets.
- Colored scene only: camera. The default robot-startup stack does **not**
  publish `/camMainView/*`. Start it in its own window:

```bash
tmux new-window -d -t ros -n camera "cd '$ROBOT_WS' && source install/setup.bash && \
  ros2 launch mole_perception_bringup camera.launch.py use_sim_time:=false robot_namespace:=mole; exec bash"
timeout 5 ros2 topic echo /camMainView/camera_info --once --field header
timeout 5 ros2 topic echo /mole/livox_lidar_publisher/lidar_front_left_filtered \
  --once --field header --qos-reliability best_effort
```

## Excavation-Map Survey

```bash
tmux new-window -d -t ros -n survey_map "cd '$ROBOT_WS' && source install/setup.bash && \
  ros2 launch mole_mapping survey_excavation.launch.py robot_namespace:=mole \
    map_reference_config_file:='$SITE_REFERENCE_YAML'; exec bash"
```

It waits up to 60 s for three `STATUS_OK` states (and shuts down otherwise),
then starts elevation and excavation mapping in empty-map survey mode on the
filtered front LiDAR. It starts no filter or accumulator; pass
`pointcloud_topic:=` to use an existing accumulated cloud. Move the cabin/boom
or drive for coverage: only measured `elevation` cells patch the terrain, and
unknown cells stay gaps even when a visualization layer inpaints them.

Save from the workspace root:

```bash
cd "$ROBOT_WS"
ros2 launch mole_mapping save_map.launch.py map_name:=<name> \
  artifact_stage:=surface robot_namespace:=mole use_nvblox:=false
```

This writes `src/mole_maps/maps/<name>/` (git-lfs tracked). `surface` keeps
terrain only; `design` keeps design and progress layers. Check the launch
result and the new files before stopping the survey window. Pass
`maps_root:=<survey dir>/maps` to keep scratch surveys out of `mole_maps`; a save
takes about 1 s, so a 2-minute save loop is cheap insurance during a long drive.
Do not record `/excavation_mapping/grid_map`: about 29 MB per message at 5 Hz.

Post-process offline (vegetation spikes, then enclosed holes), check the preview
PNG, and compare in Foxglove:

```bash
ros2 run mole_excavation_mapping postprocess_survey_map.py \
  <maps>/<name>/<name>_surface <maps>/<name>_post/<name>_post_surface \
  --keep-box X_MIN Y_MIN X_MAX Y_MAX   # walls/containers to keep, map frame
ros2 run mole_excavation_mapping publish_grid_map_artifacts.py \
  /survey/elevation_measured=<maps>/<name>/<name>_surface \
  /survey/elevation_post=<maps>/<name>_post/<name>_post_surface
```

If the survey's elevation map lags (TF "extrapolation into the past", map stamps
several seconds old), profile `elevation_mapping_node` with `sudo py-spy`; on
2026-10-03 full-map inpainting saturated it (fixed in elevation_mapping_cupy
`cf74ed6`).

## Colored Scene Survey

```bash
SURVEY_DIR="$HOME/mcap/site_rgb_$(date -u +%Y%m%d_%H%M%S)"
tmux new-window -d -t ros -n survey_scene "cd '$ROBOT_WS' && source install/setup.bash && \
  ros2 launch mole_mapping survey_scene.launch.py output_dir:='$SURVEY_DIR' \
    launch_colorizer:=true map_reference_config_file:='$SITE_REFERENCE_YAML'; exec bash"
```

Drop `launch_colorizer:=true` if `/mole/colored_point_cloud` already has a
publisher; never run two colorizers on one output. The colorizer keeps
out-of-camera points gray. Defaults: `voxel_size:=0.01`, no range crop
(`min_range_m`/`max_range_m`, sensor frame), and per-cloud stamped TF into
`map` (missing TF skips the cloud; no latest-pose fallback).

Snapshot while capture continues:

```bash
ros2 service call /mole/save_scene std_srvs/srv/Trigger '{}'
```

Each call writes a new timestamped directory under `SURVEY_DIR` with
`scene.ply`, `scene.pcd` (binary, uncompressed), `scene.yaml` (frame, voxel,
accepted/skipped clouds, point count, bounds), and `site_reference.yaml` (copy
of the datum). An empty capture fails the service. Require `success: true`,
non-empty files, and plausible skipped-cloud counts. Save explicitly before
Ctrl-C on a large capture.

One stationary scan sees only its visible surfaces: capture overlapping
viewpoints over the approach, base stations, dig/dump areas, and ground hidden
by the chassis or tool. To keep raw evidence, start the README's
`rosbag_record.launch.py` command into `$SURVEY_DIR/evidence` before capturing.

Optional offline GridMap from a scene (no robot needed):

```bash
ros2 run mole_excavation_mapping pcd_to_grid_map.py "$SCENE_DIR/scene.pcd" \
  --output "$SCENE_DIR/scene_grid_map" --frame-id map --resolution 0.1 \
  --no-interpolate --nearest-fill-distance 0.0 --no-force
```

It bins heights; it is not a ground classifier. Crop vegetation, walls, and
machinery first, and keep the 3D cloud for meshing.

The old skill-local static accumulator (`record_colored_map_fast.sh`) is
superseded by `survey_scene.launch.py`. Only for a workspace that predates
`e221b15b1`, restore `scripts/` from codex_skills commit `e6b84b5`.

## Sync And Cleanup

Normalize `perserverance` or `perservance` to the SSH host `perseverance`.
Verify local and remote checksums after `rsync`.

Save before stopping. Stop only windows this workflow created (`survey_map`,
`survey_scene`, `camera` if started here, Open3D). Do not stop the estimator,
sensor drivers, perception, Terra, or Foxglove unless the user asks.
