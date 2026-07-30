---
name: mole-pid-tuning
description: "Run, inspect, and iteratively tune one-at-a-time M445 hydraulic joint velocity PID steps on the robot. Use for PID gain inspection, directional gain tuning, tracking-bias diagnosis, overshoot, settling, or validating a new LUT."
---

# Mole PID Tuning

Actuate only after Lorenzo explicitly authorizes robot motion and an operator confirms clearance and
the emergency stop. Keep exactly one `mole_pid_joint_controller` running. The guarded runner refuses
stale measurements/status, invalid raw position/velocity sensor status, locked interlocks, a
competing velocity or current publisher, an unsafe start side, a soft-limit approach, or an
excessive joint-specific velocity.

## Inspect first

Use live parameters as the truth:

```bash
ros2 param dump /mole_pid_joint_controller
```

The deployed persistent gain source is
`$ROS_WS/src/moleworks_ros/low_level/mole_low_level_bringup/config/pid_gains_m445.yaml`;
`mole_pid_joint_controller/params/pid_m445.yaml` is only the controller package default.

## One-step loop

1. Preposition with `$robot-move-to-position`: a positive step starts in the lower half of travel; a
   negative step starts in the upper half.
2. Run one low or medium velocity:

```bash
/home/lorenzo/codex_skills/skills/mole-pid-tuning/scripts/run_pid_step.sh \
  --confirm-hardware --confirm-safe-start J_BOOM neg 0.10 4.0
```

3. Read the printed `TRIAL_RESULT`. It records the exact live PID/limit snapshot and resolved LUT path. Inspect
   `step_response_summary.md`, `step_response_phase_metrics.csv`, the overview, and the phase plot.
4. Change one cause at a time while idle, then repeat the same step:

```bash
ros2 param set /mole_pid_joint_controller J_BOOM.kp_neg <reviewed_new_value>
```

5. Interpret direction-normalized metrics:
   - slow rise with little overshoot: cautiously increase `|kp|`;
   - persistent negative `steady_bias_cmd_dir`: cautiously increase `|ki|`;
   - overshoot or ringing: reduce `|kp|`/`|ki|`, or cautiously increase `|kd|`;
   - compare `tracking_mae`, `tracking_rmse`, `rise_time_10_90_s`,
     `overshoot_fraction`, and `settling_time_10pct_s`.

Tune positive and negative gains separately. Use the same start region, RPM, command, and duration
when comparing gains. Test low speed first, then one medium-speed case; do not run an automatic
sweep. Preserve each joint's existing gain sign convention; adjust magnitude from the measured
baseline rather than copying a generic numeric value. Persist only an accepted live snapshot into
the canonical gain file, rebuild/restart, and repeat one confirmation step.
For LUT comparisons, use a new candidate filename, restart the controller with that path, confirm
the live `lut_path`, and do not overwrite the selected file during the comparison.

Set `PID_BAG_ROOT` to reuse an existing output root. The wrapper contains no control logic; it only
sources the built workspace and calls `mole_sysid_pid_step`.
