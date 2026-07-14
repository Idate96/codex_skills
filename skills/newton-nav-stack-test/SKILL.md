---
name: newton-nav-stack-test
description: "Validate Newton Nav2 bringup, drive-path health, Foxglove visibility, and the side-target tracking benchmark. Use for post-bringup navigation acceptance and failure triage."
---

# Newton Nav Stack Test

Use this after `newton-sim-ros-startup` has brought up a clean `newton_sim` session on one ROS domain.

## Use This Skill For

- Standardizing Newton Nav2 validation after a restart
- Verifying the drive path before blaming the planner
- Running the side-target parking/cusp tracking benchmark
- Checking that Foxglove has the robot, path, and drive diagnostics

## Assumptions

- Canonical in-container workspace: `/workspace/moleworks/ros2_ws`
- `ROS_DOMAIN_ID` is the domain of the intended Newton simulation (commonly `24`)
- Newton is already running in tmux
- Foxglove is already connected to the matching bridge

This workflow sends navigation commands. Before the benchmark, prove that the selected domain is the
intended Newton simulation: confirm a live `/clock`, the expected Newton tmux/container, and no bridge
from that domain to physical-machine command hardware. If identity is ambiguous, stop before motion.

## 1) Health Checks First

Before sending a goal, verify the sim stack is complete:

```bash
: "${ROS_DOMAIN_ID:?Set ROS_DOMAIN_ID to the intended Newton simulation domain}"
source /opt/ros/jazzy/setup.bash
source /workspace/moleworks/ros2_ws/install/local_setup.bash

timeout 10 bash -lc 'ros2 topic hz /clock'
timeout 15 bash -lc 'ros2 run tf2_ros tf2_echo map BASE_GRAV 2>&1' | head -20
timeout 10 ros2 topic info /clock -v
timeout 10 ros2 topic list | rg '^/mole/(state|measurements|robot_description|joint_states|plan|cmd_vel_smoothed|actuator_commands)$'
timeout 10 ros2 node list | sort | uniq -d
timeout 10 ros2 topic info /mole/cmd_vel_smoothed -v
timeout 10 ros2 topic info /mole/actuator_commands -v
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

## 3) Side-Target Benchmark

Run the standardized lateral and cusp tracking benchmark without randomized targets:

```bash
: "${ROS_DOMAIN_ID:?Set ROS_DOMAIN_ID to the verified Newton simulation domain}"
source /opt/ros/jazzy/setup.bash
source /workspace/moleworks/ros2_ws/install/local_setup.bash
python3 /workspace/moleworks/ros2_ws/src/moleworks_ros/mole_bringup/scripts/nav2_side_target_benchmark.py \
  --robot-ns mole \
  --lateral-m 1.0 \
  --random-targets 0 \
  --goal-timeout-sec 180.0
```

This runs left/right lateral and forward/reverse cusp targets so the excavator must execute parking-style maneuvers rather than only straight segments.

## 4) What To Watch During The Test

Watch these topics together:

```bash
timeout 10 ros2 topic echo /mole/plan --once
timeout 10 ros2 topic info /mole/cmd_vel_smoothed -v
timeout 10 ros2 topic info /mole/actuator_commands -v
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
- the side and cusp targets produce valid paths
- `/mole/cmd_vel_smoothed` and `/mole/actuator_commands` remain single-instance
- the excavator completes the maneuver without planner flip-flopping or bridge resets
