---
name: open-loop-lut-recollection
description: Recollect or repair bounded-joint open-loop current LUT points on the robot, especially `J_TELE` and `J_EE_PITCH`.
---

# Open-Loop LUT Recollection

## Overview

Use this for robot-side `MODE_CURRENT` LUT recollection on bounded joints.

Default rules:
- one bag per amplitude,
- stop `mole_pid_joint_controller`,
- positive current starts near the lower joint limit,
- negative current starts near the upper joint limit,
- dense extra points near breakaway,
- exclude edge amplitudes that cannot produce a clean steady window.

## Quick Start

1. In tmux, source:
   - `source /opt/ros/jazzy/setup.bash`
   - `source ~/ros2_ws/install/setup.bash`
2. Stop `mole_pid_joint_controller`.
3. Set engine RPM separately.
4. Preposition manually with direct current pulses.
5. Run one amplitude:

```bash
/home/lorenzo/.codex/skills/open-loop-lut-recollection/scripts/run_open_loop_lut_step.sh \
  J_TELE pos 0.25 4.0 1.0 1.0
```

## Collection Rules

### Bounded-joint start rule

- positive current: start near the lower limit
- negative current: start near the upper limit

Do not start directly on the hard stop. Back off slightly first.

### Dense low-speed rule

If there is a jump from "no motion" to "moving", fill the gap with extra points.

Examples:
- `J_TELE`: fill around `-0.20 .. -0.25` and `+0.17 .. +0.25`
- `J_EE_PITCH`: fill around `+/-0.10 .. +/-0.20`

### Plateau rule

If analysis fails with no valid steady segment:
- repeat once from the correct start side,
- try a longer plateau before increasing current,
- use `4.0 s` for higher-current bounded-joint points by default.

### Edge-point rule

Drop amplitudes that cannot produce a clean steady window away from joint stops.

Symptoms:
- large current but near-zero median velocity,
- strong non-monotonic jump relative to neighbors,
- repeated recollection still collapses near zero.

## Wrapper

The wrapper records one MCAP bag, runs the current step, stops recording
cleanly, and runs `mole_sysid_tune_lut`.

```bash
/home/lorenzo/.codex/skills/open-loop-lut-recollection/scripts/run_open_loop_lut_step.sh \
  <joint> <pos|neg> <abs_current> <step_phase_s> [settle_s] [steady_window_s]
```

Defaults:
- bag root: `/home/lorenzo/rosbags_from_diego_20260304`
- baseline: `1.0 s`
- final hold: `1.0 s`
- no return leg
- `--max-abs-current 1.0`
- fastwrite MCAP

## Outputs

Per amplitude:
- bag: `<bag_root>/<tag>`
- analysis: `<bag_root>/analysis_<tag>_lut`

Read:
- `lut_points.csv`
- `segments.csv`
- `plots.pdf`

## References

- Repo note:
  - `/home/lorenzo/ros2_ws/src/moleworks_ros/low_level/mole_sysid/OPEN_LOOP_LUT_TUNING.md`
