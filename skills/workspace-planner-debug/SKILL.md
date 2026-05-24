---
name: workspace-planner-debug
description: Debug Moleworks workspace planner Newton/ROS runs with per-action planner GridMaps, predicted-vs-executed scoop analysis, Terra checkpoint replay of failed or high-discrepancy scoops, and live-run inspection artifacts.
---

# Workspace Planner Debug

Use this when the task is about the workspace planner behavior in ROS/Newton: too many actions, remaining slivers, poor scooping, policy/planner mismatch, action-debug artifacts, residual volume, checkpoint replay, or Foxglove visualization of chosen strips.

Use `newton-sim-ros-startup` first when the stack itself must be started or restarted. This skill owns the planner-specific debug loop after the Newton ROS runtime is available.

## Core Rule

Every planner action must be traceable across three artifacts:

- planner prediction: `/mole/workspace_planner/debug/status` plus `/mole/workspace_planner/debug/grid_map`
- execution feedback: the next planner compute's `local_removed_m3`, `global_removed_m3`, `remaining_before_m3`, `remaining_after_m3`, or the terminal status for the same attempt
- replay point: the Terra checkpoint saved before moving to dig for that scoop

Do not compare `global_removed_m3` from attempt `N+1` with the new action selected at attempt `N+1`; that feedback belongs to action `N`.

## Standard Run Layout

Put run artifacts under:

```bash
RUN_DIR=/workspace/moleworks/ros2_ws/failure_state/$(date -u +%Y%m%d_%H%M%S)_real_ros_workspace_planner_debug
mkdir -p "$RUN_DIR"
```

Start the recorder before launching or resuming Terra:

```bash
export ROS_DOMAIN_ID=24
source /workspace/moleworks/ros2_ws/install/setup.bash
ros2 run workspace_planner workspace-planner-capture-action-debug \
  --output-dir "$RUN_DIR/action_debug" \
  --grid-topic /mole/workspace_planner/debug/grid_map \
  --status-topic /mole/workspace_planner/debug/status \
  2>&1 | tee "$RUN_DIR/action_debug.log"
```

The planner server must publish these debug layers:

```text
elevation, planner_score, planner_cost, selected_strip, selected_target,
remaining_error_m, active_work, feasible_candidate, search_candidate,
root_candidate, candidate_reject_reason, strip_gain_m3,
predicted_completion_delta_m3, bucket_fill_ratio, pull_length_m
```

Check them with:

```bash
ros2 topic echo /mole/workspace_planner/debug/grid_map grid_map_msgs/msg/GridMap --once --field layers
```

If `predicted_completion_delta_m3`, `bucket_fill_ratio`, or `pull_length_m` are all NaN in a captured `.npz`, treat that as a debug-tool bug before drawing planner conclusions.

## Failure Bundle

Before restarting after a failure or timeout, preserve:

```bash
tmux capture-pane -pt newton_24:newton -S -2000 > "$RUN_DIR/tmux_newton_after_failure.log"
tmux capture-pane -pt newton_24:stack -S -6000 > "$RUN_DIR/tmux_stack_after_failure.log"
tmux capture-pane -pt newton_24:action_debug -S -1000 > "$RUN_DIR/tmux_action_debug_after_failure.log"
ps -eo pid,ppid,etime,cmd > "$RUN_DIR/processes_after_failure.txt"
```

Also save the excavation map if the service exists:

```bash
ros2 service call /mole/excavation_mapping/save_map mole_excavation_mapping/srv/SaveGridMap \
  "{uri: $RUN_DIR/excavation_map_after_failure, topic: grid_map, storage_id: mcap, overwrite: false, include_layers: []}" \
  > "$RUN_DIR/save_map_after_failure_response.txt" 2>&1
```

## Checkpoint Matching

`Saved checkpoint: .../<pair>_<completed_loop>` before `move_to_dig` is the exact pre-action replay state for the next scoop.

For action `N`, use the checkpoint whose completed loop index is `N-1`; for the first scoop this is usually:

```text
.../single_workspace/<session>/1_0/checkpoint.yaml
```

For failed or high-discrepancy scoops, copy or record the checkpoint path in the run notes and replay from it rather than restarting from the beginning.

Replay in Newton sim:

```bash
export ROS_DOMAIN_ID=24
source /workspace/moleworks/ros2_ws/install/setup.bash
python3 /workspace/moleworks/ros2_ws/src/moleworks_ros/scripts/resume_from_checkpoint.py \
  <checkpoint-or-checkpoint.yaml> \
  --on-machine false
```

Then resume Terra through the normal executor path, keeping planner/policy parameters identical unless intentionally running an A/B change.

## Inspection

After a run has `stack.log`, `snapshot_layers.npz` or a saved map bundle, and optional `action_debug/`, run:

```bash
ros2 run workspace_planner workspace-planner-inspect-live-run \
  --bench-dir "$RUN_DIR" \
  --output-dir "$RUN_DIR/inspection"
```

Use:

- `inspection/scoops.csv` for selected action, unique `record_key`, checkpoint path, timeout/failure, policy termination, and execution feedback.
- `inspection/action_debug.csv` for per-action grid summary: `action_key`, selected strip residual, best/selected completion delta, candidate/reject counts, selected active fraction.
- `inspection/residual_candidate_diagnostics.csv` to distinguish planner-not-selecting from policy-not-scooping.
- `inspection/summary.md` for the compact run narrative.

Checkpoint replay can reuse numeric attempts. Treat `record_key` in `scoops.csv`, and `action_key` in
`action_debug.csv`, as the unique action id (`1`, `1#2`, ...). Verify the failed row points to the
matching checkpoint suffix such as `1_0(2)`.

## Interpretation

High discrepancy means the planner predicted a useful action but execution did not reduce global residual. First inspect:

- whether the action reached the dig policy at all; `move_to_dig_timeout` means the planner selected a target that the move-leg phase did not reach, so policy-volume comparison is not valid yet
- termination: timeout, curl blocker, partial, close, back-clearance, fill
- selected strip geometry: `selected_active_fraction`, `pull_length_m`, `bucket_fill_ratio`
- contact model: `contact_volume`, `contact_distance`, `contact_search`
- rejection pattern around residual hot spots: `candidate_reject_reason`
- checkpoint availability for the exact failed/high-discrepancy action

Planner-not-selecting means residual hot spots exist but candidates covering them are absent, rejected, or below the selector floor. Use the debug grid layers plus residual-candidate diagnostics before tuning the policy model.

Policy-not-scooping means a high-delta candidate was selected, but execution feedback is low, timed out, or terminates in a regime the policy is not trained for. Replay from the matched checkpoint and compare only one changed parameter at a time.
