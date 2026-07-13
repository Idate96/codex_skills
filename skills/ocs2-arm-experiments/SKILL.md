---
name: ocs2-arm-experiments
description: Run Mole OCS2 arm experiments on hardware with safe bringup, cylindrical goals, diagnostics, rosbagging, and publisher-conflict checks.
---

# OCS2 Arm Experiments

Use this for real-actuation Mole OCS2 arm experiments: lifecycle bringup, cylindrical end-effector goals, diagnostic recording, staged validation, and recovery.

## Required Reference

Read [references/full-runbook.md](references/full-runbook.md) before launching the controller or commanding motion. It contains the exact machine task profiles, lifecycle commands, goal senders, diagnostic artifacts, recovery commands, and tuning interpretation rules.

Use `ocs2-tuning-fastloop` when the task is specifically repeatable one-case tuning and limit verification.

## Safety Contract

- Before motion, verify `/machine_status` reports the required hydraulic, autonomy, and Gravis-command readiness.
- `/mole/actuator_commands` must have exactly one intended command publisher. Stop competing DIG, scheduler, Foxglove, or stale controller publishers first.
- A subagent may manage launch, configure/activate, first-policy, and publisher checks only when the user explicitly allows delegation. It must not publish motion goals or actuator commands.
- Do not switch from `real_collisions` to a blind/no-collision profile unless the user or main agent explicitly authorizes it.
- Use TF or `mole_m4_print_ee_cyl.py` as the current end-effector source. Do not derive hold targets from `/mole/ocs2/observation.state`.
- Do not call a stationary arm a true hold until the current target matches the live EE pose and a fresh benchmark segment confirms it.
- For every commanded segment, capture a goal log, pre/post snapshots, and continuous benchmark diagnostics. Do not run unrecorded motion segments.
- For dump/open-close maneuvers, judge direction and endpoint using the calibrated `bucket_angle`; raw contact-frame Euler pitch is secondary.
- Keep the runtime workspace, task file, and reviewed source provenance aligned. Record the exact active task file.
- Stop goal sources before deactivation or recovery so commands can return to zero.

## Workflow

1. Confirm runtime provenance.
   - Check the active workspace setup and `ros2 pkg prefix` for `mole_msgs` and `mole_ocs2_arm_controller`.
   - Record `task.info_path` and copy the exact live task/model sources into any review bundle.
2. Establish a run directory and start the continuous benchmark logger.
3. Check machine readiness and publisher exclusivity.
4. Launch OCS2 in the shared container-local tmux session.
   - Prefer the current machine startup helper.
   - Use manual launch only for a required custom handover or diagnostic.
5. Configure the lifecycle controller.
   - Wait for bootstrap completion and the first fresh policy.
   - Activate only after the policy and publisher checks pass.
6. Verify diagnostics.
   - lifecycle state
   - `safe_stop_active` and reason
   - policy age/horizon
   - command-chain diagnostics
   - effective realtime scheduling and any requested CPU affinity
7. Read the live cylindrical EE pose, optionally run the predicted dry check, then send one conservative semantic goal with the direct sender.
8. Capture the post snapshot and analyze the segment artifacts before the next motion.
9. Increase motion difficulty in stages: hold, tiny single-axis steps, combined small steps, then longer sweeps.
10. If commands remain non-zero but the arm does not move, stop the goal source, force zero commands through deactivation, wait, and re-check machine communication before blaming MPC.

## Motion Rules

- Preferred sender: `ros2 run mole_ocs2_arm_controller mole_m4_send_cyl_goal.py`.
- `--pitch-deg` is the calibrated semantic bucket angle on the current direct sender.
- Use absolute goals from a freshly read pose for large-theta tests.
- For large theta, validate progressively (`15 -> 30 -> 45` degrees); attempt `90` only after predicted and executed paths are sane.
- Do not override command velocity or acceleration limits casually; tune response with the documented acceleration scale and verify runtime diagnostic limits.
- Record a separate benchmark run per scheduler-driven maneuver leg when clean command-versus-measured plots matter.

## Recovery Routing

- Missing collision GridMap/SDF: report the blocker; do not silently disable collision constraints.
- Stale or expired policy, repeated `currentTime` errors, or climbing policy age: restart the full OCS2 launch pane instead of cycling lifecycle transitions repeatedly.
- Breakaway safe-stop with otherwise fresh policy: follow the documented deactivate/reactivate recovery, then republish only a small goal.
- Newton was restarted: restart the full ROS-side stack; do not restart only OCS2.
- Missing tmux pane: recreate it before sending commands.
- Controller active, commands non-zero, no motion: stop scheduler/goal source, force zero commands for several seconds, then verify machine communication and publisher ownership.

## Detailed Procedures

The full runbook contains:

- delegation boundaries and compact worker status
- per-segment logging, scheduler-leg analysis, and artifact sync
- runtime provenance and true-hold checks
- hardware readiness and competing-publisher cleanup
- fast and manual launches, lifecycle bootstrap, realtime/CPU checks
- semantic goal publishing, Foxglove/action alternatives, rosbag recording
- staged validation, large-theta policy, diagnostics, and safe-stop recovery
