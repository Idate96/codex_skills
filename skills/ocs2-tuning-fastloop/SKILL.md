---
name: ocs2-tuning-fastloop
description: Run fast, repeatable Mole M4 OCS2 tuning loops with clean tmux bringup, benchmark CSV capture, single-case matrix execution, and segment-level analysis/reporting. Use when users ask to start or restart MPC tuning, iterate one case at a time (forward/backward), compare parameter sets, or verify that updated velocity/acceleration limits are actually active in diagnostics.
---

# OCS2 Tuning Fast Loop

Run this skill for practical OCS2 tuning work on Mole M4 when fast iteration speed and reproducible artifacts matter more than manual command entry.

## Quick Start

1. Start a clean tuning session and benchmark recorder:
`bash scripts/start_clean_tuning_session.sh`

2. Run one matrix case (for example radial pitch30 loop):
`bash scripts/run_single_case_matrix.sh --matrix ~/mpc_tuning/configs/tuning_matrix_s1_radial_pitch30_loop.yaml`

3. Snapshot current effective limits from diagnostics + controller params:
`python3 scripts/report_runtime_limits.py`

## Workflow

1. Start session with `scripts/start_clean_tuning_session.sh`.
2. Verify MPC state in `tmux` window `node_check`:
`source ~/ros2_ws/install/setup.bash && ros2 lifecycle get /mole/mole_arm_mpc_controller`
3. Execute one case with `scripts/run_single_case_matrix.sh`.
4. Read output summary from `~/mpc_tuning/current/summaries`.
5. Read segment analysis JSON/plot files in `~/mpc_tuning/current/artifacts`.
6. Apply at most one parameter update and rerun the same case.
7. Move to next case only after current case is qualitatively and quantitatively acceptable.

## Decision Rules

- Treat hard-gate failures as blockers before tracking optimization.
- Prioritize `z` oscillation and coupling when operator feedback says behavior is unstable, even if settle time passes.
- Use per-joint ratios to avoid false confidence:
`diag_cmd_vel_ratio_*`, `diag_cmd_accel_ratio_*`.
- Watch for repeated `1.00` acceleration ratios on the same joint (frequent on `J_BOOM` during aggressive radial moves).

## Velocity And Acceleration Limits

When tuning after limit updates, verify both sources each session:

- Controller runtime params:
`command.max_velocity`, `command.max_accel`, `command.accel_scale`
- Diagnostic limits in benchmark CSV:
`diag_cmd_vel_limit_*`, `diag_cmd_accel_limit_*`

Run:
`python3 scripts/report_runtime_limits.py`

Fail fast on mismatches. Do not proceed with tuning if param limits and diagnostic limits disagree.

## References

- Load `references/segment-report-template.md` when producing per-run summaries and update proposals.
