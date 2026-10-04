---
name: dig-bag-recording
description: "Record canonical split MCAP runs for Mole DIG/Newton controllers, including sensors, state, commands, LiDAR, compressed camera, excavation maps, actions, and Dig3D observations."
---

# DIG Bag Recording

This skill owns recording and finalized-artifact verification. It does not launch a controller (`dig-controllers`), start the base stack (`robot-startup`), replay data (`dig-bag-replay`), or upload it (`kleinkram-upload`).

## Quick start

```bash
SKILL_DIR="${CODEX_HOME:-$HOME/.codex}/skills/dig-bag-recording"
"$SKILL_DIR/scripts/dig_split_recording_tmux.sh" \
  --scenario dig_newton \
  --controller newton \
  --use-sim-time true \
  --attach
```

The helper calls the repository-owned `mole_bag_tools rosbag_record.launch.py` and creates one run root with `raw/sensors`, `raw/state`, `raw/commands`, `raw/lidar`, `raw/camera`, and `raw/elevation_map`. Dig3D also enables `raw/dig3d_special_obs`. The launch writes `recording_manifest.json`, `README.txt`, and `derived/`.

Pass `--controller` explicitly when the scenario tag is ambiguous. Recognized values are `dig3d`, `newton`, `dig`, `dig-ee`, and `ugep`; this selects the namespaced action transport to retain. Otherwise the helper makes a conservative inference from common scenario names. UGEP additionally records its namespaced observation topics and defaults command capture to its isolated `actuator_commands_ugep_dryrun` output; override `--actuator-commands-topic` only when the controller was deliberately launched with a different output.

## Runtime contract

- Use the published `rslheap/moleworks_ros:latest` image by default; retain an immutable `sha-<merge-sha>` tag when exact provenance matters.
- On a discovery-server robot shell, use normal plain `ros2 ...` inspection commands. The canonical launcher assigns the observer DDS profile to rosbag children while leaving normal processes on the runtime client profile.
- Do not replace the launcher with raw remote `ros2 bag record` or hand-copied DDS exports. If enabled splits stay empty, first verify the deployed image/container and start a fresh recorder window.
- Current robot-local state, command, LiDAR, and action topics derive from `--robot-namespace` (default `mole`). The primary DIG map is `/excavation_mapping/grid_map`; the canonical recorder also retains `/<robot_namespace>/elevation_map_filter` and the supported excavation-map aliases.
- Camera recording uses compressed image topics, preferring `/hal/grpc_image_client/Main/image_raw/compressed` while also retaining `/camMainView/image_raw/compressed` under the default camera configuration.
- For ordinary field-test video, prefer the lightweight MainView gRPC camera. Launch only `mole_perception_bringup camera.launch.py` in a separate tmux window when the existing perception owner omitted the camera; do not restart perception and lose an authored excavation target. The ZED sidecar is for requests that need registered depth or a native colored point cloud and requires its own host-managed container.
- When the operator explicitly asks for video, a configured camera split is not evidence that video exists. Before the action, receive at least one real `sensor_msgs/msg/CompressedImage` sample from `/hal/grpc_image_client/Main/image_raw/compressed`. After finalization, require `ros2 bag info <run>/raw/camera` to report a positive message count. Stop and report the gap if either check fails.

## Workflow

1. Confirm hardware versus simulation and set `--use-sim-time` accordingly.
2. Check required topics and free space under the output filesystem.
3. If video is requested, verify a real compressed camera frame before starting the helper. MainView is the default field-video source; use ZED only when its depth/cloud capability was requested.
4. Start the helper before the DIG action.
5. Stop with `Ctrl-C` in the left `record` pane and wait for every rosbag process to finalize.
6. Verify each enabled split has `metadata.yaml`, at least one `.mcap`, and readable `ros2 bag info` output. For requested video, the camera split must contain at least one message.

Read-only preflight example:

```bash
df -h ~/mcap/dig
timeout 10 ros2 topic echo /mole/state --once
timeout 10 ros2 topic echo /mole/actuator_commands --once
timeout 10 ros2 topic echo /excavation_mapping/grid_map --once
timeout 10 ros2 topic echo /hal/grpc_image_client/Main/image_raw/compressed \
  sensor_msgs/msg/CompressedImage --once --field header --qos-reliability best_effort
```

Defaults are tmux `ros:record`, output root `~/mcap/dig`, the first built workspace among `~/ros2_ws` and `~/moleworks/ros2_ws`, namespace `mole`, and hardware time (`false`). The helper refuses any busy pane by default; `--restart-window` replaces only the managed window and requires explicit restart authority.

## Verification

```bash
find ~/mcap/dig -maxdepth 4 -type f -name metadata.yaml | sort
ros2 bag info /path/to/run/raw/state
```

## Post-run scoop extraction

Use the repository-owned analyzer to turn one finalized campaign run into replayable scoop bags:

```bash
: "${RUN_ROOT:?Set RUN_ROOT to the finalized run directory}"
ros2 run mole_bag_tools analyze_dig_trenching_session \
  --run-root "$RUN_ROOT" \
  --write-splits \
  --split-excavation-map-snapshots-only
```

The default map topic is `/excavation_mapping/grid_map`. The analyzer resolves
`--command-topic` across the canonical raw splits. For isolated Dig3D tests where
other controllers keep `/mole/actuator_commands` continuously alive, add:

```bash
--command-topic /dig_3d/policy_action_stamped
```

This writes `derived/scoop_bags/scoop_XXX`. Run `ros2 bag info` on every child
bag before discarding the source. For command-versus-motion, bottom tracking,
current activity, and joint jitter diagnostics, run:

```bash
ros2 run mole_bag_tools analyze_dig_policy_motion \
  --input-root "$RUN_ROOT" \
  --force
```

If the source run must be discarded, first relocate the verified `derived/`
artifacts outside that run directory, preserve its manifest, and move the source
to recoverable trash. Never delete the only copy before all split metadata and
MCAP files pass `ros2 bag info`.

For OCS2 paper-evidence runs, use the OCS2 experiment skill and the repository's explicit `record_ocs2:=true` plus `validate_ocs2_recording` contract; this generic DIG profile intentionally does not claim OCS2 provenance completeness.

## Terra field runs on rslpc (2026-10-04, worked)

**Relays.** Start two relays first, each in its own permanent tmux window. The full excavation map is about 9 MB at
5 Hz, so it gets a 0.5 Hz copy, and the camera gets a 2 Hz compressed copy:

```bash
cd ~/ros2_ws && source install/setup.bash
T=src/moleworks_ros/mole_utils/mole_bag_tools/scripts/throttle_topic.py
tmux new-window -d -t ros -n map_throttle "cd ~/ros2_ws && source install/setup.bash && python3 $T /excavation_mapping/grid_map /excavation_mapping/grid_map_1hz grid_map_msgs/msg/GridMap 0.5; exec bash"
tmux new-window -d -t ros -n cam_throttle "cd ~/ros2_ws && source install/setup.bash && python3 $T /hal/grpc_image_client/Main/image_raw/compressed /hal/grpc_image_client/Main/image_raw/compressed_2hz sensor_msgs/msg/CompressedImage 2.0; exec bash"
```

**Recorder** (about 27–36 GB/h). The state split also carries `/mole/terra/run_events`:

```bash
B=terra_<run>_$(date -u +%Y%m%d_%H%M%S); mkdir -p ~/mcap/$B/derived
tmux new-window -d -t ros -n record_run "cd ~/ros2_ws && source install/setup.bash && ros2 launch mole_bag_tools rosbag_record.launch.py bag_path:=$HOME/mcap/$B append_timestamp:=false record_sensors:=true record_state:=true record_commands:=true record_lidar:=true lidar_topic:=livox_lidar_publisher/lidar_front_left_filtered record_camera:=true camera_image_topic:=/hal/grpc_image_client/Main/image_raw/compressed_2hz record_elevation_map:=false record_controller_observations:=true record_campaign_markers:=false extra_state_topics:=/excavation_mapping/grid_map_1hz,/mole/workspace_planner/debug/status,/mole/workspace_planner/debug/metrics,/mole/workspace_planner/debug/grid_map,/mole/planner_debug_markers,/terra/current_dig_zone_marker,/terra/current_dump_zone_marker,/terra/workspace_target_markers,/terra/plan_waypoint_markers,/camMainView/camera_info,/mole/plan; exec bash"
```

**Finalizing a run**
1. Ctrl-C the record window and check every split has `metadata.yaml`.
2. Copy the Terra run-event JSONL (`~/.local/state/moleworks/terra_runs/`) and the planner_watch log into
   `derived/`, plus the session's Terra checkpoints (`~/.local/state/moleworks/terra/checkpoints/<map>/<session>`).
   Those checkpoints hold the pre-run map and one map per scoop, which is what a simulation re-run needs.
3. Write `RUN_NOTES.md` in the run root.

**Idle trimming**
- Tool: `~/terra_field_tools/scratch_20261004/trim/trim_static_bag.py`.
- Active means a joint moves faster than 0.02 rad/s, a command exceeds 0.01, or the executor is running a step.
  It pads 15 s around activity, drops idle gaps longer than 60 s (keeping 10 s at each end), and re-writes latched
  topics at each window start.
- It writes `<run>_trimmed/` with `TRIM_NOTES.md`. Verify per-topic counts before touching the original.
- Delete originals only with explicit operator approval and once a verified copy exists elsewhere.

**Moving bags to lhs** (`192.168.19.88`, nearly full): `rsync -a`, then `rsync -c --dry-run --itemize-changes`
must list nothing before the local copy may go.
