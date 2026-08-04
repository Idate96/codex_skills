---
name: dig-bag-replay
description: "Replay canonical split or single-scoop DIG bags exactly as recorded in a dedicated moleworks_ros container, with robot description and Foxglove visualization."
---

# DIG Bag Replay

This skill owns offline playback and visualization. It does not start a live robot or simulation stack, launch controllers, or record bags. Use `dig-bag-recording` for capture and `kleinkram-upload` for download reconstruction.

The repository no longer contains the former `mole_bringup/dig_bag_replay.launch.py`. Therefore this skill does not claim to regenerate self-filter, elevation mapping, or excavation mapping. Both helpers replay recorded topics as-is with `/clock`, a robot-description publisher, and Foxglove. Use a package-owned mapping launch and a task-specific replay plan when changed-code mapping regeneration is actually required.

## Safety contract

- Use one container dedicated to offline replay.
- Both helpers require `--confirm-offline-container`.
- Never pass that confirmation for a container connected to the live robot or hosting another needed ROS stack.
- The helpers replace their managed replay tmux sessions but avoid broad `pkill` cleanup inside the container.
- Commands are not replayed by default. `--include-commands` is explicit and is only appropriate in the isolated offline graph.

## Canonical split-run replay

Pass either the run root containing `raw/` or the `raw/` directory itself:

```bash
SKILL_DIR="${CODEX_HOME:-$HOME/.codex}/skills/dig-bag-replay"
"$SKILL_DIR/scripts/dig_bag_replay_tmux.sh" \
  --bag-root ~/mcap/dig/dig3d_real_20260804_120000 \
  --confirm-offline-container \
  --attach
```

The default selection is `sensors,state,lidar,elevation_map,dig3d_special_obs,camera`; absent optional splits are skipped. Select an exact set with `--splits`, for example:

```bash
SKILL_DIR="${CODEX_HOME:-$HOME/.codex}/skills/dig-bag-replay"
"$SKILL_DIR/scripts/dig_bag_replay_tmux.sh" \
  --bag-root /path/to/run \
  --splits sensors,state,ocs2 \
  --confirm-offline-container \
  --attach
```

If the run has a different path inside the container, pass the corresponding run or raw root through `--container-bag-root`.

## Single-scoop replay

Replay a monolithic bag directly:

```bash
SKILL_DIR="${CODEX_HOME:-$HOME/.codex}/skills/dig-bag-replay"
"$SKILL_DIR/scripts/dig_segment_replay_tmux.sh" \
  --segment-dir /path/to/segment_bag \
  --confirm-offline-container \
  --attach
```

Or resolve it from a split manifest:

```bash
SKILL_DIR="${CODEX_HOME:-$HOME/.codex}/skills/dig-bag-replay"
"$SKILL_DIR/scripts/dig_segment_replay_tmux.sh" \
  --manifest /path/to/single_scoop_manifest.csv \
  --segment-name scoop_001 \
  --confirm-offline-container \
  --attach
```

Use `--container-segment-dir` when host and container paths differ.

Both helpers expose Foxglove on `ws://localhost:8766` by default and create `replay` plus `monitor` tmux windows. The default playback rate is `0.2`.

## What to inspect

- `/tf`, `/tf_static`, `/mole/robot_description`
- `/mole/state`, `/mole/joint_states`, `/mole/actuator_commands`
- `/dig_3d/observations` and other controller diagnostics when recorded
- `/mole/livox_lidar_publisher/lidar_front_left`
- `/mole/elevation_map_filter` and `/excavation_mapping/grid_map`
- compressed camera topics when present

The canonical recording layout stores TF in `raw/sensors`; include that split whenever model or frame visualization matters.

## Flattened Kleinkram downloads

Flattened missions contain `upload_name_map.yaml` instead of a replayable split tree. Reconstruct them first through the `kleinkram-upload` skill. Its reconstruction helper produces legacy top-level split roots (`sensors/`, `state/`, and so on), which the split replay helper accepts in addition to the current `raw/` layout.
