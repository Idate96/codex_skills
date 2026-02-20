# Segment Report Template

Use this template when reporting a tuning run to keep decisions consistent across iterations.

## 1. Run Context

- Case name:
- CSV path:
- Summary JSON path:
- Segment IDs (forward/backward):
- Runtime updates applied before run:

## 2. Hard-Gate Status

- SAFE_STOP:
- Constraint margin availability:
- Policy timing health (`policy_age`, `policy_time_to_end`):
- Command saturation (`cmd_sat_max_ratio`, `cmd_accel_max_ratio`):

## 3. Tracking Metrics

- Settle time all axes (fwd/back):
- Axis settle (`r`, `theta`, `z`, `pitch`) where available:
- Final error (`r`, `theta`, `z`, `pitch`, `pos_3d`):
- Cost-aligned errors:
- `err_tangential_m`
- `err_ori_deg`

## 4. Oscillation And Coupling

- `z` zero crossings:
- `z` peak absolute error:
- Coupling max ratio and worst axis:
- Qualitative operator feedback:

## 5. Joint Limit And Effort Clues

- Worst velocity ratio joint/value:
- Worst acceleration ratio joint/value:
- If any repeated acceleration ratio near `1.0`, identify joint and direction.
- Runtime limit consistency check output from `scripts/report_runtime_limits.py`.

## 6. Decision

- Pass/fail for this case:
- Primary blocker:
- Single next update (param + before/after):
- Why this update addresses the blocker:
