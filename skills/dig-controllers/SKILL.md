---
name: dig-controllers
description: "Start or restart a Mole DIG controller in tmux when the base stack is already running. Use for Dig3D or Newton controller-only bringup and recovery."
---

# DIG Controllers

Use this skill when the base stack is already running. For base stack plus controller in one operation, use `robot-startup --dig-controller ...`.

## Preflight

1. Identify the intended runtime: real robot (`use_sim_time:=false`) or Newton simulation (`use_sim_time:=true`). Do not infer it from a ROS graph alone.
2. Verify the active workspace, ROS domain, `/clock` behavior, controller launch file, and required map/state topics.
3. If the controller follows a target surface, require excavation mapping with both `elevation` and `desired_elevation` on `/excavation_mapping/grid_map`.
4. Check command-publisher exclusivity before activation. On hardware, also require an operator and the robot interlocks.
5. `--restart-window` kills the managed tmux window; use it only for an authorized restart.

The helper configures and activates the controller by default. Use `--no-activate` for inspection-only bringup. `--run-action` sends motion and is always explicit opt-in.

## Commands

Real-robot Dig3D:

```bash
/home/lorenzo/codex_skills/skills/dig-controllers/scripts/dig_controllers_tmux.sh \
  --controller dig3d --use-sim-time false --attach
```

Newton simulation:

```bash
/home/lorenzo/codex_skills/skills/dig-controllers/scripts/dig_controllers_tmux.sh \
  --controller newton --use-sim-time true --attach
```

Inspection-only launch:

```bash
/home/lorenzo/codex_skills/skills/dig-controllers/scripts/dig_controllers_tmux.sh \
  --controller dig3d --use-sim-time false --no-activate --attach
```

Supported controller keys are `dig3d`, `newton`, `dig`, and `dig-ee`. The window has a launch pane and a lifecycle/action-helper pane; it prints an action command but does not run it unless `--run-action` is passed.

Runtime target application is one-shot per excavation-mapping lifetime. If a different target is required, restart excavation mapping and reapply it through `/excavation_mapping/apply_runtime_profile`; do not route `desired_elevation` through elevation mapping.
