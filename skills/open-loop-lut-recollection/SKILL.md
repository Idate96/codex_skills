---
name: open-loop-lut-recollection
description: "Collect, inspect, and merge one-at-a-time M445 open-loop current-to-cylinder-velocity LUT points on the robot. Use for LUT regeneration, breakaway recollection, monotonicity repair, or bounded-joint current calibration."
---

# Open-Loop LUT Recollection

Actuate only after Lorenzo explicitly authorizes robot motion and an operator confirms clearance and
the emergency stop. Stop `mole_pid_joint_controller`; the guarded runner refuses a competing command
publisher, stale `/mole/measurements`, invalid raw position/velocity sensor status, stale
`/machine_status`, locked interlocks, a soft-limit approach, or an excessive
joint-specific current.

## One-point loop

This current-command LUT path covers `J_TURN`, `J_BOOM`, `J_STICK`, `J_TELE`,
`J_EE_PITCH`, and `J_EE_ROLL`. The M445 Gravis bridge maps roll to TiltrotTilt
axis 25. Its measured roll hard stops are approximately `[-0.646, +0.555] rad`;
keep steady windows within the configured soft margins. Positive current decreases
roll position. The analyzer must use the runtime `m445_differential_roll` geometry to
convert measured joint velocity to the equivalent cylinder-velocity LUT domain; do
not substitute raw joint angular velocity or hand-written two-cylinder math.

Do not collect `J_EE_YAW` on the current machine state. Axis 26 visibly actuates yaw,
but Gravis reported a frozen position and `NOT_SUPPORTED` velocity on 2026-07-31.
Restore or remap yaw feedback before recollection, and never accept a zero-speed yaw
analysis as a LUT point.

1. `J_TURN` is continuous and may start from any angle. Preposition bounded joints with
   `$robot-move-to-position` so there is enough travel in the commanded direction for the final
   steady-state window to remain away from a soft limit. There is no midpoint requirement.
2. Run exactly one amplitude:

```bash
SKILL_DIR="${CODEX_HOME:-$HOME/.codex}/skills/open-loop-lut-recollection"
"$SKILL_DIR/scripts/run_open_loop_lut_step.sh" \
  --confirm-hardware --confirm-safe-start J_TELE pos 0.25 4.0
```

3. Read the printed `TRIAL_RESULT` YAML and its `analysis/lut_result.yaml`,
   `lut_points.csv`, `segments.csv`, and plots.
4. If aborted, do not bypass the guard. Reposition or repair the reported precondition. If analysis
   has no clean steady segment, repeat once with more directional travel and a longer plateau before
   increasing current.
5. Add points progressively. Densify the first-moving/breakaway region; stop when a point hits a
   limit, folds back, or cannot sustain a steady window.

Set `LUT_BAG_ROOT` to choose the output root. Set `MOLEWORKS_ROS_SETUP` to an exact `setup.bash`
when using a custom install overlay; otherwise set `ROS_WS`/`MOLE_ROS_WS` or let the wrapper discover
the normal workspace. The wrapper contains no motion logic; it only sources the selected overlay and
calls `mole_sysid_lut_collect`, which owns graph checks, motion, recording, zero flush, and analysis.

## Build a candidate

After reviewing at least two moving analyses for one direction, merge that entire branch once:

```bash
ros2 run mole_sysid mole_sysid_build_lut_candidate \
  --base-lut "$(ros2 pkg prefix --share mole_pid_joint_controller)/params/luts_m445.yaml" \
  --output-lut /tmp/luts_m445_candidate.yaml \
  --joint J_TELE --direction pos \
  /data/analysis_point_1 /data/analysis_point_2
```

The builder never overwrites the active LUT. It rejects mixed provenance, fallback cylinder math,
no-motion points, duplicate inverse velocities, and folded branches. Review the candidate YAML and
its `.points.csv` before any deployment. Collect and merge the other direction separately.
The checked-in `J_BOOM` negative branch already has a small fold: rebuild its negative branch first,
then use that candidate as the base when rebuilding the positive branch.

Canonical package details live in
`<active-workspace>/src/moleworks_ros/low_level/mole_sysid/OPEN_LOOP_LUT_TUNING.md`.
