---
name: dig-bag-recording
description: "Record split Mole DIG/Newton rosbags for sensors, state, commands, LiDAR, camera, maps, and Dig3D observations. Use when capturing a DIG run or preparing replayable run artifacts."
---

# Dig Bag Recording

## Quick Start

```bash
/home/lorenzo/codex_skills/skills/dig-bag-recording/scripts/dig_split_recording_tmux.sh \
  --scenario dig_newton \
  --use-sim-time true \
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

1. Before recording, confirm the intended real/sim stack and `use_sim_time` choice, check that the
   required topics are live, and inspect free space under the output filesystem.
2. Start recording with the helper script.
3. Run digging action(s).
4. Stop the recorder with `Ctrl-C` in the left `record` tmux pane and wait for rosbag finalization.
5. Verify each expected bag has `metadata.yaml`, at least one `.mcap`, and a readable `ros2 bag info`.

Useful preflight (adapt the required topics to the scenario):

```bash
df -h ~/mcap/dig
timeout 10 ros2 topic list
timeout 10 ros2 topic echo /mole/state --once
timeout 10 ros2 topic echo /mole/actuator_commands --once
```

## Defaults

- tmux session/window: `ros:record`
- workspace: first built workspace among `~/ros2_ws` and `~/moleworks/ros2_ws` (or `--ws`)
- output root: `~/mcap/dig`
- time: `false` by default for real hardware; pass `--use-sim-time true` for Newton
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
find ~/mcap/dig -maxdepth 3 -type f -name metadata.yaml | sort
```

Validate one finalized split:

```bash
find /path/to/run/raw/state -maxdepth 1 -type f -name '*.mcap' -print -quit
ros2 bag info /path/to/run/raw/state
```

## Resource

- Script: `scripts/dig_split_recording_tmux.sh`
