---
name: newton-sim-ros-startup
description: "Start or restart single-container Newton and Moleworks ROS stacks. Use for bridge, Nav2, perception, Dig3D, Terra, checkpoint recovery, and single-container cleanup."
---

# Newton Sim ROS Startup

Use this for the single-container Newton workflow in `moleworks_ros`. For the split IsaacLab/Terra and ROS-container path, use `sim-startup` instead.

## Procedure Reference

The safety contract and routing rules below are mandatory. Then read only the task-relevant sections of [references/full-runbook.md](references/full-runbook.md): shell/preflight, tmux layout, the selected launch windows, sanity checks, Dig deployment, failure capture/resume, or teardown. Do not load the entire long runbook for a narrow operation.

Use the narrower companion skill when applicable:

- `newton-nav-stack-test`: standardized post-bringup Nav2 validation
- `terra-foundation-execution`: packaged foundation plans
- `workspace-planner-debug`: per-action planner artifacts and checkpoint replay
- `ros2-debugging`: generic TF, topic, node, service, and DDS checks

## Non-Negotiables

- Assume the current shell is already inside the target runtime container when `/.dockerenv` and `/workspace/moleworks/ros2_ws/install/setup.bash` exist.
- Stay in the container-local shell and tmux control plane. Use `docker_attach.sh` only from a confirmed host shell; do not use `docker exec` for normal interactive bringup.
- Keep the container's default Fast DDS setup. Use `ROS_DOMAIN_ID=24` unless the user requests another valid domain (`<=232`).
- Source `/workspace/moleworks/ros2_ws/install/setup.bash` as the canonical ROS overlay. Build only from `/workspace/moleworks/ros2_ws`.
- Default to headless Newton. Enable a GUI only when the user requests visual inspection.
- Use one foreground, long-lived process per tmux pane. Do not use `nohup` in shared bringup panes.
- Before reusing a pane, stop the old process and verify its process tree is gone. Check RAM, VRAM, and stale ROS/Newton processes before a restart.
- A Newton restart invalidates the ROS side through the sim-time jump. Restart the full downstream ROS stack, not only the failed controller pane.
- Keep Newton terrain and ROS excavation mapping seeded from the same artifact. Apply runtime profiles before the Hong no-holes pose reset.
- Never send Dig3D or Nav2 goals before clock, TF, required topics, and publisher ownership are healthy.
- Capture the failure-state bundle and latest pre-action Terra checkpoint before teardown unless the user explicitly opts out.
- Resume Newton checkpoints with `--on-machine false`; never use the real-machine mode in simulation.

## Workflow

1. Identify the shell and runtime.
   - Check `/.dockerenv`, the canonical ROS overlay, the Newton worktree, Docker availability, RAM, VRAM, and stale processes.
   - If already inside the container, do not detach and reattach.
2. Validate the overlay.
   - Source the canonical setup.
   - Confirm `ros2 pkg prefix` for `mole_msgs`, `mole_ocs2_arm_controller`, and `terra_planner` resolves under the expected install prefix.
3. Choose exactly one layout from the full runbook.
   - bridge-only diagnostics
   - fast Newton + Ackermann + Nav2
   - split perception/Dig3D
   - split Terra development
   - integrated Terra single-workspace
4. Launch in dependency order.
   - Newton first, then model/TF, perception or planning, controllers/executor, and finally Foxglove.
   - For Dig3D, seed terrain on both sides and apply the target before launching or sending the action.
5. Verify before commanding motion.
   - `/clock` and `/mole/state` are live.
   - required TF resolves with a 10–15 second buffer timeout.
   - `/mole/actuator_commands` and `/mole/cmd_vel_smoothed` have the expected publishers and no stale extras.
   - no duplicate node names or deleted hold-controller nodes are present.
6. On failure, preserve state before restarting.
   - Save the latest retry checkpoint, current diagnostic map/pose snapshot, pane logs, processes, ROS graph, parameters, and controller status.
   - If Newton was restarted or the clock jumped, rebuild the full downstream stack.
7. Re-check RAM, VRAM, processes, topics, and TF after teardown or restart.

## Fast Routing

- Nav2 smoke or lateral-shift regression: use the fast Nav2 layout, disable the LiDAR publisher unless needed, then run `newton-nav-stack-test`.
- Terrain or Dig3D parity: use the split perception layout and the dual map-load contract.
- Terra controller/BT iteration: use split `planner`, `ocs2`, `dig`, and `executor` panes so only the failing owner is restarted.
- End-to-end Terra validation: use integrated `single_workspace.launch.py`; do not add hold-controller nodes or a second split Dig3D pane.
- A failed owner other than Newton: capture evidence, then restart the smallest owning pane first.
- Newton failure or restart: capture evidence, then restart every ROS-side pane against the new clock.

## Detailed Procedures

The full runbook contains:

- shell identity, attach fallback, overlay repair, and stale-process cleanup
- all tmux layouts and exact launch commands
- fast Nav2 build and validation loop
- Newton, robot, state publisher, perception, STL target, planner, OCS2, Dig3D, executor, Nav2, and Foxglove launches
- Hong no-holes map and pose-reset ordering
- sanity checks and expected publisher ownership
- Terra failure bundle capture and checkpoint resume
- teardown and post-cleanup verification
