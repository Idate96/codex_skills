---
name: robot-startup
description: Set up the standard Moleworks ROS 2 tmux session for on-machine work with 4 windows in order (low_level, perception, estimator, foxglove). Use when asked to create/recreate the tmux window layout and start the stack quickly (dig controllers are started separately).
---

# Robot Startup

Create a tmux session that matches the standard on-machine window layout (4 windows in order, pinned names) and starts the usual launch commands (no hardcoded window indexes).

This assumes DDS/discovery is already configured in the container shell environment.

## Execute (Fast)

If the user asks to "execute" (or uses phrasing like "execute fast"), just run the startup script immediately and return (do not do extra probing like `tmux ls`, pane log capture, etc. unless asked for debugging):

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh
```

Add flags only if the user asked for them:

- attach: `--attach`
- clean slate: `--restart`
- legacy 3-window layout (skip estimator): `--no-estimator`

## Quick Start

Run the setup script:

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh --attach
```

Disable the estimator window (legacy 3-window layout):

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh --no-estimator --attach
```

Optionally override the estimator config:

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh \
  --estimator-config ~/ros2_ws/src/moleworks_ros/mole_estimator/config/mole_estimator_robot_heading_corrected.yaml \
  --attach
```

Use a non-default session/workspace:

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh \
  --session ros \
  --ws ~/ros2_ws \
  --endeffector-type shovel_w_teeth \
  --attach
```

If you want a clean slate (kills the existing session):

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh --restart --attach
```

## What It Creates

tmux session (default: `ros`) with 4 windows in order by name:

- `low_level`: `ros2 launch mole_low_level_bringup bringup.launch.py use_sim_time:=false on_machine:=true activate_trajectory_controller:=false`
- `perception`: `ros2 launch mole_perception_bringup bringup.launch.py use_sim_time:=false enable_lidar:=true enable_robot_self_filter:=true enable_elevation_mapping:=true map_name:=none`
- `estimator`: `ros2 launch mole_estimator mole_estimator.launch.py ...`
- `foxglove`: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml` (default port `8765`)

The script prompts for `endeffector_type` when run interactively (defaults to `shovel` if omitted) and passes it into low_level, perception, and estimator launches.

Window names are locked (automatic rename disabled), and the order is enforced each run.

By default, the script does not touch windows that already have a non-shell process running; use `--restart` to recreate everything.

To start dig controllers, use the separate `dig-controllers` skill.

## Resources

- Script: `scripts/robot_startup_tmux.sh`
