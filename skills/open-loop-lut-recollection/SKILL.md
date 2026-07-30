---
name: open-loop-lut-recollection
description: "Collect, inspect, and merge one-at-a-time M445 open-loop current-to-cylinder-velocity LUT points on the robot. Use for LUT regeneration, breakaway recollection, monotonicity repair, or bounded-joint current calibration."
---

# Open-Loop LUT Recollection

Actuate only after Lorenzo explicitly authorizes robot motion and an operator confirms clearance and
the emergency stop. Stop `mole_pid_joint_controller`; the guarded runner refuses a competing command
publisher, stale `/mole/measurements`, invalid raw position/velocity sensor status, stale
`/machine_status`, locked interlocks, an unsafe start side, a soft-limit approach, or an excessive
joint-specific current.

## One-point loop

1. Preposition with `$robot-move-to-position`. For `J_TURN` and `J_BOOM`, positive current starts
   in the upper half because it decreases joint position. For `J_STICK`, `J_TELE`, and
   `J_EE_PITCH`, positive starts in the lower half. Negative uses the opposite side. Do not sit on
   a hard stop.
2. Run exactly one amplitude:

```bash
/home/lorenzo/codex_skills/skills/open-loop-lut-recollection/scripts/run_open_loop_lut_step.sh \
  --confirm-hardware --confirm-safe-start J_TELE pos 0.25 4.0
```

3. Read the printed `TRIAL_RESULT` YAML and its `analysis/lut_result.yaml`,
   `lut_points.csv`, `segments.csv`, and plots.
4. If aborted, do not bypass the guard. Reposition or repair the reported precondition. If analysis
   has no clean steady segment, repeat once from the correct side with a longer plateau before
   increasing current.
5. Add points progressively. Densify the first-moving/breakaway region; stop when a point hits a
   limit, folds back, or cannot sustain a steady window.

Set `LUT_BAG_ROOT` to reuse an existing output root. The wrapper contains no motion logic; it only
sources the built workspace and calls `mole_sysid_lut_collect`.

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
`$ROS_WS/src/moleworks_ros/low_level/mole_sysid/OPEN_LOOP_LUT_TUNING.md`.
