---
name: robot-move-check
description: "Run one guarded Mole/Menzi M445 joint-velocity step through the PID controller. Use for short real-robot direction checks with an operator present, bounded motion, recording, and automatic zero-command cleanup."
---

# Robot Move Check

Use the maintained `mole_sysid_pid_step` runner for an explicitly requested signed joint velocity and
duration. It owns the velocity-step operation, safety gates, bag, analysis, and two-second zero flush.
Do not recreate it with `ros2 topic pub`. Use `$robot-move-to-position` for an explicit position target,
`$open-loop-lut-recollection` for current commands, and an OCS2 skill for OCS2 motion.

## Required Preflight

Before running a nonzero step:

1. Require an operator, clear workspace, working emergency stop, and explicit joint, signed velocity,
   and duration. Do not infer a maneuver from an earlier run.
2. Keep `mole_pid_joint_controller` running as the sole `/mole/current_commands` publisher. Stop any
   OCS2, DIG, trajectory, teleop, or other `/mole/actuator_commands` publisher.
3. Inspect the requested joint position. For bounded joints, use `$robot-move-to-position` first when
   more travel is needed in the requested direction.
4. Confirm the active overlay resolves `mole_sysid_pid_step`.

The runner then rechecks processed and raw sensor validity, feedback freshness, hydraulic/autonomy/
Gravis interlocks, command ownership, and the expected PID subscriber throughout the step. It applies
the maintained M445 joint-specific position, velocity, and predictive stopping limits.

## Run One Step

```bash
WS="${MOLE_ROS_WS:-$HOME/ros2_ws}"
[[ -f "$WS/install/setup.bash" ]] || WS="$HOME/moleworks/ros2_ws"
source "$WS/install/setup.bash"

OUT_ROOT="$HOME/mcap/pid_direction_checks"
mkdir -p "$OUT_ROOT"
ros2 run mole_sysid mole_sysid_pid_step \
  --confirm-hardware --confirm-safe-start \
  --joint J_BOOM --direction pos --abs-velocity 0.10 \
  --step-s 1.0 --output-root "$OUT_ROOT"
```

Choose `pos` for positive joint velocity and `neg` for negative joint velocity. The runner refuses
velocities above the joint-specific cap; use a conservative value below the cap for direction checks.

Read the printed `TRIAL_RESULT` YAML and its bag/analysis paths. A nonzero exit, aborted trial, missing
bag, or incomplete zero flush is not a successful check. If motion direction is unexpected, use the
operator stop path and do not retry with the opposite sign until the command/measurement mapping is
understood.
