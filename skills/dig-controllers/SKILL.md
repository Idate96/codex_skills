---
name: dig-controllers
description: "Start Moleworks dig controllers (dig_3d, dig_newton, dig_ee, etc.) in a split tmux window: one pane runs the controller launch, the other pane runs lifecycle helpers and shows the `ros2 action send_goal` command. Use when the base stack is already running and you want to add or restart only a dig controller."
---

# Dig Controllers

## Quick Start

If the user wants base stack plus dig in one step, prefer the `robot-startup` skill with `--dig-controller ...`. Use this skill when the base stack is already up and only the dig controller should be started or replaced.

```bash
~/.codex/skills/dig-controllers/scripts/dig_controllers_tmux.sh --controller dig3d --attach
```

Newton controller:

```bash
~/.codex/skills/dig-controllers/scripts/dig_controllers_tmux.sh --controller newton --attach
```

## What It Does

Creates (or reuses) a tmux session (default: `ros`) and a window (default: `dig`) split into 2 panes:

- Left pane: launches the controller (`ros2 launch ...`)
- Right pane: waits for the controller node, configures + activates it, and prints the `ros2 action send_goal` command (does not auto-run)

## Supported Controllers

- `dig3d` → `mole_highlevel_controller_cpp/dig_3d_controller_cpp.launch.py` (`/dig_3d_controller`, action `/run_dig_3d`)
- `newton` → `mole_highlevel_controller_cpp/dig_newton_controller.launch.py` (`/dig_newton_controller`, action `/run_dig_newton`)
- `dig` → `mole_highlevel_controller_cpp/dig_controller_cpp.launch.py` (`/dig_controller`, action `/run_dig`)
- `dig-ee` → `mole_highlevel_controller_cpp/dig_ee_controller_cpp.launch.py` (`/dig_ee_controller`, action `/run_dig_ee`)

## Resources

- Script: `scripts/dig_controllers_tmux.sh`
