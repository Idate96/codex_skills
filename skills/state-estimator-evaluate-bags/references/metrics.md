# Estimator Evaluation Metrics

- The summary table marks strict-threshold failures as `FAIL(xxx)`.
- `J_TURN lag abs-med [ms]` is effective turn-joint lag; keep it small because `J_TURN` is actively controlled.
- `pose-twist map v_xy RMS` includes `map <-> odom` corrections and can be inflated by global corrections.
- `pose-twist odom v_xy RMS` is the preferred local consistency check between `odom -> base` and twist.
- Backward-aligned (`*_bwd`) metrics match velocities derived from a backward finite difference.
- The analyzer drops initial `STATUS_OK` transients with its `ok_warmup_s` setting.

For velocity quality, inspect `output_twist.source` in `mole_estimator/config/mole_estimator.yaml`:

- `graph`: derive BASE twist from Graph-MSF IMU twist plus transforms.
- `pose_diff_odom`: finite-difference `odom -> BASE` for locally consistent `v_xy`.

If `v_xy` is noisy, evaluate a small `output_twist.pose_diff_lpf_tau_s` (normally no more than `0.10 s`) under the same pinned bag and metrics contract.
