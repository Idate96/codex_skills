---
name: dig-bag-replay
description: Replay split or single-scoop DIG bags with TF, mapping, and Foxglove in the `moleworks_ros` container.
---

# Dig Bag Replay

Use this for offline DIG bag review in `moleworks_ros`.

There are two entrypoints:
- full split-run replay: local self-filter + local elevation mapping + local excavation mapping
- fast single-scoop review: replay one monolithic segment bag exactly as recorded

Assumption:
- one active replay session at a time; each helper clears the other replay session before launch

Do not use it for the live robot stack:
- for the real robot, use `robot-startup`
- for Newton sim startup, use `newton-sim-ros-startup`

## Quick Start

Full split-run replay:

```bash
~/.codex/skills/dig-bag-replay/scripts/dig_bag_replay_tmux.sh \
  --bag-root /home/lorenzo/mcap/dig3d_2026-03-26/dig3d_real_run_2026-03-26_21-02-16 \
  --attach
```

Single-scoop segment replay from a manifest row:

```bash
~/.codex/skills/dig-bag-replay/scripts/dig_segment_replay_tmux.sh \
  --manifest /home/lorenzo/mcap/dig3d_2026-03-26/split_single_scoops_obs_2026-03-28/single_scoop_manifest_2026-03-28.csv \
  --segment-name trenching_single_2026-03-26_21-44-29_s01_strong_scoop \
  --attach
```

Each helper starts a fresh tmux session with:

- `replay`: launches the replay command in `moleworks_ros`
- `monitor`: sourced ROS shell in the same container

Foxglove bridge is exposed on `ws://localhost:8766`.

Before launch, the helper stops prior replay-style processes in that container:
- `dig_bag_replay.launch.py`
- `foxglove_bridge`
- `self_filter`
- `elevation_mapping_node`
- `mole_excavation_mapping_node`
- `ros2 bag play`

## Defaults

- container: `moleworks_ros`
- container workspace: `/workspace/moleworks/ros2_ws`
- tmux session: `bag_replay`
- segment tmux session: `segment_replay`
- playback rate: `0.2`
- end effector: `shovel`
- excavation preload: `package://mole_maps/maps/hong0326_no_holes/hong0326_no_holes_surface`

## Kleinkram Downloads

Downloaded Kleinkram missions produced by the upload skill are usually flattened into one mission directory. `dig_bag_replay_tmux.sh` expects one split run root with `sensors/`, `state/`, `commands/`, `lidar/`, and optional `elevation_map/`, so reconstruct first:

```bash
python3 /home/lorenzo/codex_skills/skills/kleinkram-upload/scripts/reconstruct_split_bags.py \
  /path/to/downloaded_project/mission_name/project_name/mission_name
```

Then point `--bag-root` at one reconstructed run:

```bash
~/.codex/skills/dig-bag-replay/scripts/dig_bag_replay_tmux.sh \
  --bag-root /path/to/downloaded_project/reconstructed_runs/run_name \
  --attach
```

## Workflow

Use full split-run replay when:
- you want to validate self-filter, elevation mapping, or excavation mapping behavior
- the run exists as `sensors/`, `state/`, `commands/`, `lidar/`, optional `elevation_map/`

Full split-run workflow:
1. Run `dig_bag_replay_tmux.sh` with `--bag-root`.
2. Open Foxglove on `ws://localhost:8766`.
3. Review replayed `/tf`, `/tf_static`, `/mole/state`, `/mole/actuator_commands`, raw lidar.
4. Review `/mole/robot_description` if you want the URDF model in Foxglove.
5. Review live `/mole/self_filter`, `/mole/elevation_map_filter`, `/mole/excavation_mapping/grid_map`.
6. Use the `monitor` window for checks.

Use fast single-scoop segment replay when:
- you want to review accept/reject quality for one split scoop from the manifest
- the segment exists only as one monolithic `.mcap`
- you do not need to regenerate mapping live

Single-scoop workflow:
1. Pick a `segment_name` from `single_scoop_manifest_*.csv` or `PLAYBACK_PRIORITY_*.md`.
2. Run `dig_segment_replay_tmux.sh` with `--manifest` and `--segment-name`.
3. Open Foxglove on `ws://localhost:8766`.
4. Review the recorded topics already inside the segment bag:
   - `/tf`, `/tf_static`
   - live `/mole/robot_description` for the URDF model
   - `/mole/state`, `/mole/actuator_commands`, `/dig_3d/observations`
   - `/mole/livox_lidar_publisher/lidar_front_left`
   - recorded `/mole/elevation_map_filter`
   - recorded `/mole/excavation_mapping/grid_map`
   - camera topics when present
5. Use full split-run replay later if you need to rerun mapping with changed code.

## Useful Commands

Check the loaded excavation preload:

```bash
ros2 param get /mole/excavation_mapping design_bag_path
```

Check that elevation mapping uses the filtered cloud:

```bash
ros2 topic info -v /mole/livox_lidar_publisher/lidar_front_left_filtered
ros2 node info /mole/elevation_mapping_node
```

Override the excavation preload:

```bash
~/.codex/skills/dig-bag-replay/scripts/dig_bag_replay_tmux.sh \
  --bag-root /path/to/run \
  --design-bag-path package://mole_maps/maps/hong0326_holes/hong0326_holes_surface \
  --attach
```

## Resource

- Split-run helper: `scripts/dig_bag_replay_tmux.sh`
- Segment helper: `scripts/dig_segment_replay_tmux.sh`
