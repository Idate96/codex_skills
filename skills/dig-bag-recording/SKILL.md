---
name: dig-bag-recording
description: Start fast split rosbag recording for Mole dig/newton runs using the canonical `rosbag_record.launch.py` workflow. Use when you want separate bags for sensors, state, commands, lidar, camera (compressed image topics), elevation_map, and Dig3D special observations during digging experiments.
---

# Dig Bag Recording

## Quick Start

```bash
~/.codex/skills/dig-bag-recording/scripts/dig_split_recording_tmux.sh \
  --scenario dig_newton \
  --attach
```

This creates a single run directory:

- `raw/sensors/`
- `raw/state/`
- `raw/commands/`
- `raw/lidar/`
- `raw/camera/` (compressed image topics)
- `raw/elevation_map/`
- `raw/dig3d_special_obs/` for `dig_3d*` scenarios

## Workflow

1. Start recording with the helper script.
2. Run digging action(s).
3. Stop the recorder with `Ctrl-C` in the left `record` tmux pane.
4. Verify bag folders exist under the printed run directory.

## Defaults

- tmux session/window: `ros:record`
- workspace: `~/ros2_ws`
- output root: `~/mcap/dig`
- elevation topic: `/mole/elevation_map_filter`
- compressed camera topic preference:
  - `/hal/grpc_image_client/Main/image_raw/compressed`
  - fallback also recorded by estimator utility: `/camMainView/image_raw/compressed`

## Useful Commands

Check recorder processes:

```bash
tmux list-panes -t ros:record -F '#{pane_index} #{pane_pid} #{pane_current_command}'
```

List generated bags:

```bash
find ~/rosbags/dig -maxdepth 3 -type f -name metadata.yaml | sort
```

## Resource

- Script: `scripts/dig_split_recording_tmux.sh`
