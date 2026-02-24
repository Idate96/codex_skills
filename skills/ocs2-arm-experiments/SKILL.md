---
name: ocs2-arm-experiments
description: Start/restart Mole OCS2 arm controller experiments with real actuation commands in tmux, reset+activate the lifecycle controller, send cylindrical (r, theta, z, pitch) end-effector goals (Foxglove or CLI), record rosbag, and run staged validation phases while monitoring diagnostics and preventing competing /mole/actuator_commands publishers (e.g. dig_controller).
---

# Ocs2 Arm Experiments

## Workflow

### 0. Safety / Preconditions (Hardware)

Do these checks before publishing to `/mole/actuator_commands`:

```bash
ros2 topic echo --once /machine_status | rg -n "is_hydraulilock_unlocked|is_autonomous_operation_unlocked|is_using_gravis_commands"
ros2 topic info /mole/actuator_commands -v
```

Ensure:
- `is_hydraulilock_unlocked=true`, `is_autonomous_operation_unlocked=true` only when you are ready to move.
- `/mole/actuator_commands` has **exactly one** command publisher (the OCS2 arm controller).
- Do **not** infer conflicts from tmux window state alone: `ros:foxglove` may be running while not publishing actuator commands.

If `dig_controller` is publishing actuator commands, stop it first:

```bash
ros2 lifecycle get /dig_controller
ros2 service call /dig_controller/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 6}}"
```

If `foxglove_bridge` appears as a publisher on `/mole/actuator_commands`, clear stale Foxglove publish sessions before activating OCS2:

```bash
ros2 topic info /mole/actuator_commands -v
# if you see "Node name: foxglove_bridge" under Publisher:
tmux send-keys -t ros:foxglove.1 C-c
tmux send-keys -t ros:foxglove.1 "cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml" C-m
sleep 2
ros2 topic info /mole/actuator_commands -v
```

### 1. tmux Layout (ros session)

Use/attach to the existing `ros` tmux session and create an `ocs2` window with 2 panes:
- left pane: launch OCS2
- right pane: rosbag + helper commands

Common tmux commands:

```bash
tmux ls
tmux list-windows -t ros
tmux new-window -t ros -n ocs2
tmux split-window -t ros:ocs2 -h
```

Use the same ROS environment for launch + activate + goal publish (same tmux session/pane family) to avoid DDS/domain mismatches.

### 2. Launch (Real Actuation)

Launch OCS2 arm with real actuator commands:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mole_ocs2_arm_controller ocs2_arm.launch.py \
  use_sim_time:=false \
  actuator_commands_topic:=/mole/actuator_commands \
  launch_policy_visualizer:=true \
  launch_target_bridge:=false \
  launch_dump_scheduler:=false \
  use_augmented_lagrangian:=false
```

Use `launch_target_bridge:=true` only when you explicitly want the action/GUI goal path.

Kickstart policy generation and activate controller lifecycle:

```bash
python3 src/moleworks_ros/mole_utils/scripts/publish_debug_mpc_target_hold.py
ros2 service call /mole/mole_arm_mpc_controller/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
```

If activation fails with "no MPC policy received yet", prime transient-local policy once, then activate again:

```bash
ros2 topic echo --once /mole/ocs2/policy --qos-durability transient_local
ros2 service call /mole/mole_arm_mpc_controller/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
```

Sanity checks:

```bash
timeout 5 ros2 topic hz /mole/ocs2/policy
ros2 topic echo --once /mole/ocs2/arm_controller/diagnostics
ros2 topic echo --once /mole/ocs2/arm_controller/diagnostics | rg -n "lifecycle_state|safe_stop_active|safe_stop_reason|policy_age_sec|cmd_vel.J_TURN"
```

### 3. Goal Publish (Preferred: Bridge-Free Deterministic)

Use direct target publication + reset (no bridge/action dependency) via policy step checker:
- publishes `/mole/ocs2/target`
- calls `/mole/mobile_manipulator_mpc_reset`
- validates predicted policy horizon in the same run

1) Read current cylindrical pose:

```bash
python3 src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_print_ee_cyl.py --timeout 10
```

2) Send deterministic absolute target and check policy/DDP behavior:

```bash
python3 src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_policy_turn_step_check.py \
  --goal-r-m 5.41 \
  --goal-theta-deg 30.0 \
  --goal-z-m 0.10 \
  --goal-pitch-deg 33.8 \
  --target-mode ramp \
  --goal-duration-sec 1.0 \
  --json
```

For a relative `+30 deg` step, keep `r/z/pitch` from the current printout and add `30` to current theta.

Quick verification:

```bash
python3 src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_print_ee_cyl.py --timeout 10
ros2 topic echo --once /mole/ocs2/arm_controller/diagnostics | rg -n "safe_stop_active|cmd_vel.J_TURN|cmd_sat_max_ratio"
```

Optional (action path, requires bridge):

Enable bridge in launch:

```bash
# set in launch command
launch_target_bridge:=true
```

CLI action goal:

```bash
python3 src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_send_cyl_goal.py \
  --dtheta-deg 30 \
  --call-reset \
  --quiet-feedback
```

Optional (Foxglove action client):

Use the OCS2 arm debug layout:
- `src/moleworks_ros/mole_bringup/foxglove/ocs2_arm_debug.json`

Send an action goal in Foxglove (with bridge enabled):
- action: `/mole/ocs2/send_cyl_goal`
- type: `mole_highlevel_msgs/action/SendCylindricalGoal`
- goal fields: `r_m`, `theta_deg`, `z_m`, `pitch_deg`, `call_reset`

Prefer publishing goals from the same `ros` tmux window used for OCS2.

Print current EE state in the same format (use as a baseline):

```bash
python3 src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_print_ee_cyl.py --timeout 10
```

### 4. Command Scaling (Keep Defaults)

```bash
# Do not override command.max_velocity or command.max_accel.
# Keep defaults and tune response only via command.accel_scale in (0, 1].
ros2 param set /mole/mole_arm_mpc_controller command.accel_scale 1.0
```

Then kickstart and activate:

```bash
python3 src/moleworks_ros/mole_utils/scripts/publish_debug_mpc_target_hold.py
ros2 service call /mole/mole_arm_mpc_controller/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
ros2 topic info /mole/actuator_commands -v
```

### 5. Recording (rosbag)

Record the minimum useful set:

```bash
mkdir -p ~/bags/ocs2_arm
ros2 bag record -o ~/bags/ocs2_arm/$(date +%Y%m%d_%H%M%S)_ocs2_arm \
  --topics \
  /mole/joint_states /tf /tf_static /machine_status \
  /mole/ocs2/arm_controller/diagnostics \
  /mole/ocs2/target /mole/ocs2/observation /mole/ocs2/policy \
  /mole/actuator_commands
```

### 6. Validation Phases (Recommended Order)

- Phase 0: HOLD at current pose for 30-60s, verify `safe_stop_active=0` and low saturation ratios.
- Phase 1: Single-axis tiny steps (return to baseline each time): `dr=0.02 m`, `dtheta=1 deg`, `dz=0.01 m`, `dpitch=1 deg`.
- Phase 2: Combined tiny steps (still conservative).
- Phase 3: Smooth sweeps (theta or pitch) and disturbance recovery.
- Only later: enable terrain/self-collision constraints (`use_augmented_lagrangian:=true`) and then integrate excavation mapping.

Monitor during any motion:

```bash
ros2 topic echo --once /mole/ocs2/arm_controller/diagnostics
```

### 7. Troubleshooting: Planned Trajectory Visible But Robot Does Not Move

Most common cause: controller is active but latched in `SAFE_STOP` (often from `breakaway` timeout).

If CLI goal send fails with action-server timeout, verify bridge is enabled:

```bash
ros2 action list | rg -n "/mole/ocs2/send_cyl_goal"
# If missing, restart launch with:
#   launch_target_bridge:=true
```

Inspect:

```bash
ros2 topic echo --once /mole/ocs2/arm_controller/diagnostics | rg -n "lifecycle_state|safe_stop_active|safe_stop_reason|cmd_vel.J_TURN|policy_age_sec|policy_time_to_end_sec"
```

If `safe_stop_active=1` with a breakaway timeout reason:

```bash
python3 src/moleworks_ros/mole_utils/scripts/publish_debug_mpc_target_hold.py
ros2 service call /mole/mole_arm_mpc_controller/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
```

Then republish a small goal and confirm commands are non-zero:

```bash
python3 src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_send_cyl_goal.py --dtheta-deg 10 --call-reset
ros2 topic echo --once /mole/ocs2/arm_controller/diagnostics | rg -n "safe_stop_active|cmd_vel.J_TURN"
ros2 topic echo --once /mole/actuator_commands
```

Bench-debug only (not final on-robot tuning):
- You can relax or disable breakaway to validate control-path plumbing, then re-enable it.
- Launch-time knobs: `breakaway_enable`, `breakaway.t_fail_pos[0]`, `breakaway.t_fail_neg[0]`.
