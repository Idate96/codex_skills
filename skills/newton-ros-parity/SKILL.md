---
name: newton-ros-parity
description: "Validate or replay Moleworks Newton observations through `moleworks_ros`. Use when bringing up the ROS parity stack, checking `/clock` and TF, debugging controller-facing terrain topics, replaying Dig3D-style comparisons, or cleaning up stale ROS/Newton processes after parity work."
---

# Newton ROS Parity

Use this skill for Newton <-> `moleworks_ros` parity and replay workflows. Pair it with `ros2-debugging` for TF/topic primitives.

## Source Of Truth

Start from the active branch docs and scripts:

- `docs/ros/WORKTREE_SETUP.md`
- `docs/ros/DOCKER_RUNBOOK.md`
- `docs/ROS_PARITY_LOG.md`
- `scripts/ros/AGENTS.md`
- `scripts/ros/tests/run_compare_smoke_checks.sh`
- `scripts/debug/compare_bucket_kinematics_to_pinocchio.py`

Open narrower diagnostics only when needed:

- `scripts/ros/diagnostics/check_base_pose_match.py`
- `scripts/ros/diagnostics/dump_synced_ros_snapshot.py`
- `scripts/ros/diagnostics/dump_newton_height_grid.py`
- `scripts/ros/diagnostics/dump_gridmap_sampling.py`
- `scripts/ros/diagnostics/dump_controller_grid_bounds.py`

## Bringup Preference

If parity uses the running `moleworks_ros` container:

1. Attach interactively to the container.
2. Create and manage a fresh tmux session inside that shell.
3. Launch components in order instead of blasting everything at once.
4. Inspect pane output between steps with `tmux capture-pane`.

This avoids fragile nested host quoting and makes cleanup tractable.

## Health Checks Before Dig3D / Compare

Use long TF timeouts. Minimum checks:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/moleworks/ros2_ws/install/setup.bash
timeout 15 bash -lc "ros2 topic echo /clock --once"
timeout 15 bash -lc "ros2 topic echo /excavation_mapping/grid_map --once"
timeout 15 bash -lc "ros2 run tf2_ros tf2_echo map CABIN 2>&1" | head -40
```

Do not start the compare path until `/clock`, the controller-facing terrain topic, and the required TF chain are live.

## Terrain Publisher Rule

- If direct analytical terrain publishing is enabled, keep the standard perception path off unless the user explicitly wants both.
- Do not leave multiple publishers on the controller-facing terrain topic by accident.

## Cleanup Is Mandatory

After parity, Dig3D replay, or TorchScript comparison:

1. Kill the tmux session that launched the stack.
2. Scan for orphaned `ros2 launch`, controller, state-publisher, and helper processes.
3. Kill the exact PIDs that remain.
4. Re-check process lists and host/container resource usage.

Useful checks:

```bash
ps -eo pid,cmd | rg 'ros2 launch|dig_3d_controller|robot_state_publisher|mole_joint_state_publisher|publish_flat_excavation_map|compare'
docker stats --no-stream --format '{{.Name}}\t{{.MemUsage}}\t{{.CPUPerc}}' <container>
nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader,nounits
```

Do not leave a stale parity stack running while starting a new one.

## Kinematics Contract Notes

When debugging bucket or EE velocity parity:

- `State.body_q` is the body-frame or body-origin pose.
- `State.body_qd` is a COM-referenced twist in world coordinates.
- Public `eval_jacobian()` behaves like a world-origin screw Jacobian in world coordinates.
- Therefore `eval_jacobian(...) @ joint_qd` is not generally equal to `State.body_qd`.
- If parity depends on tool-point or COM velocity, transport the screw twist to the correct point or compare against an independent implementation plus finite-difference checks.

Do not interpret a Jacobian mismatch as proof of FK failure without checking the contract first.

## Context Hygiene

- Prefer repo scripts and diagnostics over ad-hoc ROS archaeology.
- Keep live CLI output bounded with timeouts and `tmux capture-pane`.
- If the task turns into prolonged pane/log/process inspection, also use `moleworks-subagent-orchestrator`.
