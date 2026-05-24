---
name: newton-nav-stack-test
description: Validate the Newton + ROS Nav2 driving stack in a clean tmux session after bringup. Use when the user wants a repeatable navigation check in Newton sim, including health checks for the bridge/model/drive path and the lateral-shift golden test.
---

# Newton Nav Stack Test

Use this after `newton-sim-ros-startup` has brought up a clean `newton_sim` session on one ROS domain.

## Use This Skill For

- Standardizing Newton Nav2 validation after a restart
- Verifying the drive path before blaming the planner
- Running the lateral-shift "parking maneuver" golden test
- Checking that Foxglove has the robot, path, and drive diagnostics

## Assumptions

- Canonical in-container workspace: `/workspace/moleworks/ros2_ws`
- Default ROS domain: `24`
- Newton is already running in tmux
- Foxglove is already connected to the matching bridge

## 1) Health Checks First

Before sending a goal, verify the sim stack is complete:

```bash
export ROS_DOMAIN_ID=24
source /opt/ros/jazzy/setup.bash
source /workspace/moleworks/ros2_ws/install/local_setup.bash

timeout 10 bash -lc 'ros2 topic hz /clock'
timeout 15 bash -lc 'ros2 run tf2_ros tf2_echo map BASE_GRAV 2>&1' | head -20
ros2 topic list | rg '^/mole/(state|measurements|robot_description|joint_states|plan|cmd_vel_smoothed|actuator_commands)$'
ros2 node list | sort | uniq -d
ros2 topic info /mole/cmd_vel_smoothed -v
ros2 topic info /mole/actuator_commands -v
```

Expected result:

- `/clock` is live
- `map -> BASE_GRAV` resolves
- `/mole/state`, `/mole/measurements`, `/mole/robot_description`, `/mole/joint_states` exist
- no duplicate node names
- `/mole/cmd_vel_smoothed` has one publisher
- `/mole/actuator_commands` has one `ackermann_drive_controller` publisher and no stale extra publishers

If the CLI graph looks incomplete, also inspect the Foxglove bridge log pane. Treat the stack as alive only when Foxglove advertises the same topics.

## 2) Foxglove Checks

Use Foxglove to confirm:

- the robot model is visible
- `/mole/plan` is visible
- `/mole/lookahead_point`, `/mole/curvature_lookahead_point`, and `/mole/lookahead_collision_arc` are visible
- the red/ICR debug markers are not flipping before a goal is sent

If the robot is missing in Foxglove, the likely cause is missing `robot_description` or `joint_states`, not the bridge itself.

## 3) Golden Test

Run the standardized lateral-shift golden test:

```bash
export ROS_DOMAIN_ID=24
source /opt/ros/jazzy/setup.bash
source /workspace/moleworks/ros2_ws/install/local_setup.bash
python3 /workspace/moleworks/ros2_ws/src/moleworks_ros/mole_bringup/scripts/nav2_lateral_shift_golden.py \
  --robot-ns mole \
  --lateral-m 1.0 \
  --timeout-sec 180.0
```

This sends a goal offset in `+y` in the robot base frame so the excavator must execute a parking-style maneuver instead of a trivial straight segment.

## 4) What To Watch During The Test

Watch these topics together:

```bash
ros2 topic echo /mole/plan --once
ros2 topic info /mole/cmd_vel_smoothed -v
ros2 topic info /mole/actuator_commands -v
```

In Foxglove, watch:

- `/mole/plan`
- `/mole/plan_smoothed`
- `/mole/cmd_vel_nav`
- `/mole/cmd_vel_smoothed`
- `/mole/actuator_commands`

## 5) Failure Triage

If the robot does not move:

- If Newton exits, inspect the `newton` pane first. Do not debug Nav2 before the bridge is stable.
- If `/mole/plan` exists but `/mole/actuator_commands` does not, debug Ackermann/controller bringup.
- If `/mole/cmd_vel_smoothed` changes sign rapidly while the path barely changes, debug planner churn or path commitment.
- If `/mole/actuator_commands` is active but the chassis barely moves, debug wheel contact or command-mode mismatch below Nav2.

## 6) Pass Criteria

The test passes when:

- Newton stays alive for the full run
- the robot is visible in Foxglove
- the lateral-shift goal produces a valid path
- `/mole/cmd_vel_smoothed` and `/mole/actuator_commands` remain single-instance
- the excavator completes the maneuver without planner flip-flopping or bridge resets
