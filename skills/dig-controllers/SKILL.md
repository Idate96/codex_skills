---
name: dig-controllers
description: Start or restart a Mole DIG controller in tmux when the base stack is already running.
---

# Dig Controllers

## Quick Start

If the user wants base stack plus dig in one step, prefer the `robot-startup` skill with `--dig-controller ...`. Use this skill when the base stack is already up and only the dig controller should be started or replaced.

If the user expects the controller to follow a target surface, excavation mapping must already be running. Dig controllers now read both `elevation` and `desired_elevation` from `/excavation_mapping/grid_map`; they should not depend on elevation mapping for the target surface.

```bash
~/.codex/skills/dig-controllers/scripts/dig_controllers_tmux.sh --controller dig3d --attach
```

`dig3d` now defaults to the deployment preset and packaged shovel fillbridge policy.

Newton controller:

```bash
~/.codex/skills/dig-controllers/scripts/dig_controllers_tmux.sh --controller newton --attach
```

If the user needs a runtime target surface before starting the controller, apply it once through excavation mapping first:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 service call /excavation_mapping/apply_runtime_profile mole_excavation_mapping/srv/ApplyRuntimeProfile \
  "{authoring_frame_id: BASE, profile_type: trench, start_x_m: 8.0, end_x_m: 4.0, center_y_m: 0.0, half_width_m: 1.0, depth_m: 0.0, angle_deg: 20.0, entry_angle_deg: 20.0, exit_angle_deg: 20.0, trench_length_m: 2.0, trench_depth_m: -1.5}"
```

For a 60 degree wedge in front of the robot, from radius 4 m to 8 m:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 service call /excavation_mapping/apply_runtime_profile mole_excavation_mapping/srv/ApplyRuntimeProfile \
  "{authoring_frame_id: BASE, profile_type: polar_sector, depth_m: -1.5, radius_min_m: 4.0, radius_max_m: 8.0, theta_min_deg: -30.0, theta_max_deg: 30.0}"
```

This runtime apply is one-shot per excavation-mapping lifetime. If the user needs a different target later, restart excavation mapping or the perception stack and apply again.

## What It Does

Creates (or reuses) a tmux session (default: `ros`) and a window (default: `dig`) split into 2 panes:

- Left pane: launches the controller (`ros2 launch ...`)
- Right pane: waits for the controller node, configures + activates it, and prints the `ros2 action send_goal` command (does not auto-run)

## Supported Controllers

- `dig3d` → `mole_highlevel_controller_cpp/dig_3d_controller_cpp.launch.py` (`/mole/dig_3d_controller` by default, action `/run_dig_3d`)
- `newton` → `mole_highlevel_controller_cpp/dig_newton_controller.launch.py` (`/dig_newton_controller`, action `/run_dig_newton`)
- `dig` → `mole_highlevel_controller_cpp/dig_controller_cpp.launch.py` (`/dig_controller`, action `/run_dig`)
- `dig-ee` → `mole_highlevel_controller_cpp/dig_ee_controller_cpp.launch.py` (`/dig_ee_controller`, action `/run_dig_ee`)

## Resources

- Script: `scripts/dig_controllers_tmux.sh`
