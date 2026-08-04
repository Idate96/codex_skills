---
name: ocs2-arm-experiments
description: "Run the Mole OCS2 arm on the real robot: one-command bringup, cylindrical moves, dump/move-leg actions, recording, and fast recovery."
---

# OCS2 Arm on Hardware

Use this only for the real Mole/Menzi arm. Do not start dummy MRT, Newton, a separate ROS domain, or simulation unless the user explicitly asks for simulation.

## Operating Model

- The low-level robot stack, estimator, mapping, hydraulics, and autonomy are external prerequisites.
- One OCS2 launch owns MPC, the arm command publisher, target bridge, and dump/move action nodes.
- A normal robot tmux shell already has the correct DDS environment. Run ROS commands directly; do not paste discovery-profile exports before every command.
- `use_sim_time` is always `false` on the robot.

Route full-stack bringup, hydraulic unlock, and base-stack recovery to `$robot-startup`. Route engine
speed changes to `$set-engine-rpm`. This skill owns only OCS2 launch, lifecycle handover, OCS2 goals,
OCS2 actions, recording, and OCS2-specific recovery. Use `$robot-move-check` or
`$robot-move-to-position` only when OCS2 is stopped and direct low-level motion was explicitly requested.

## One Session Check

Before the first motion of a session, confirm the operator authorized the maneuver, `/machine_status`
is fresh with hydraulic/autonomy/Gravis-command interlocks unlocked, and
`/mole/actuator_commands` has no competing publisher. Treat startup as non-motion authorization.
Repeat this preflight when the ROS graph, controller launch, or machine state changes.

An `auto_handover:=true` launch is actuation-capable because it activates a measured-pose hold. Do
not run it for a read-only diagnosis or treat permission to inspect/start the base stack as permission
to activate OCS2.

Keep one DDS-aware bag running for the session; a raw `ros2 bag record` in the runtime client can discover zero publishers. Sender output is the goal log.

```bash
ros2 launch mole_bag_tools rosbag_record.launch.py \
  bag_path:=/home/lorenzo/ocs2_benchmarks/ocs2_arm/<run_name> append_timestamp:=false \
  record_sensors:=false record_state:=false record_commands:=false \
  record_lidar:=false record_camera:=false record_elevation_map:=false \
  record_ocs2:=true record_dig3d_special_obs:=false \
  capture_ocs2_provenance:=true require_ocs2_provenance:=true
```

## Start OCS2

Run this in the robot stack's `ocs2` tmux pane:

```bash
ros2 launch mole_ocs2_arm_controller ocs2_arm.launch.py \
  use_sim_time:=false taskProfile:=real_collisions \
  launch_target_bridge:=true launch_dump_leg:=true launch_move_leg:=true \
  auto_handover:=true command_chain_enable:=true
```

Use the built overlay selected by the robot stack. After launch, record the output of
`ros2 pkg prefix mole_ocs2_arm_controller` and the live `task.info_path`; do not review or tune a
different checkout's task file.

Expected startup behavior:

- The controller bootstraps from the measured pose.
- The MPC computes one initial SDF from `/excavation_mapping/grid_map`.
- `auto_handover` configures, waits for a fresh policy, activates, and holds.
- `/mole/actuator_commands` has one publisher and the hold converges to zero command.

If startup fails, read the launch error. Do not disable `real_collisions` to make it start.

Before the first goal, verify the lifecycle state, one intended actuator-command publisher, a fresh
policy, and `safe_stop_active=0` on `/mole/ocs2/arm_controller/diagnostics`. The direct goal helper
does not replace those checks.

## Cylindrical Move

Read the pose when an exact endpoint matters:

```bash
ros2 run mole_ocs2_arm_controller mole_m4_print_ee_cyl.py --rate 1
```

For a relative move:

```bash
ros2 run mole_ocs2_arm_controller mole_m4_send_cyl_goal.py \
  --call-reset --dtheta-deg 45
```

The sender defaults to an automatic smooth transition, waits only for the axes explicitly changed by a relative goal, and latches the measured pose as a hold after success. `--call-reset` is required because the current `real_collisions` profile enables the grading-surface path; raw target updates are rejected.

Use `--dr-m`, `--dz-m`, and `--dpitch-deg` only for axes that should move. Use absolute `--r`, `--theta-deg`, `--z`, and `--pitch-deg` when the endpoint is specified in the cylindrical world frame.

A clean 45-degree turn validates the path for 90-degree reversals in the same unchanged session. Do not force 15/30-degree repetitions after that.

## Dump Leg

The dump action recomputes the SDF once at action start, closes the bucket for transport, moves to the target, opens the bucket, restores baseline MPC weights, and finishes in a hold.

```bash
ros2 run mole_ocs2_arm_controller mole_m4_send_dump_leg_goal.py \
  --dtheta-deg 90 --dz-m 1.2
```

Relative mode waits for the `BASE <- ENDEFFECTOR_CONTACT` transform. For an exact known target, use absolute `--r`, `--theta-deg`, and `--z`.

## Move Leg

The move action is trench-aware and recomputes the SDF once at action start:

```bash
ros2 run mole_ocs2_arm_controller mole_m4_send_move_leg_goal.py \
  --dtheta-deg 20 --dr-m 0.0 --dz-m 0.3 --dpitch-deg 0.0
```

Use an operator-selected map target for a real trench move. Do not invent a trench coordinate.

## Fast Recovery

- Goal helper refuses a raw grading target: rerun it with `--call-reset`.
- Relative helper waits for TF and times out: use the measured absolute pose/target; do not restart DDS repeatedly.
- Policy becomes stale or controller safe-stops: stop the goal source and restart the full OCS2 launch.
- Commands remain nonzero after a failed action: cancel the action or stop its client, then publish/latch the measured current pose or deactivate the controller.
- Missing map/SDF: fix mapping; do not switch to a no-collision profile silently.
- Base stack, estimator, or machine interlocks are unhealthy: leave OCS2 stopped and route recovery to `$robot-startup` or `$robot-ros`.

Use `$ocs2-tuning-fastloop` only for repeated gain/constraint tuning experiments. It is not the normal robot-motion workflow.
