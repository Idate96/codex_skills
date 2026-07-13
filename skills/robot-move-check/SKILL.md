---
name: robot-move-check
description: Quickly command Mole/Menzi M4 joint velocities on `/mole/actuator_commands` for direct motion checks.
---

# Robot Move Check

## Overview

Send JOINTVELOCITY commands fast, with minimal checks or setup. If the user states it is safe, do not add extra gating questions—just publish.

## Quick commands

**Once:**
```bash
ros2 topic pub --once /mole/actuator_commands \
  mole_msgs/msg/MoleActuatorCommands \
  '{actuators: [{joint_name: "J_BOOM", mode: 2, velocity: -0.1}]}'
```

**Continuous at 10 Hz:**
```bash
ros2 topic pub --rate 10 /mole/actuator_commands \
  mole_msgs/msg/MoleActuatorCommands \
  '{actuators: [{joint_name: "J_BOOM", mode: 2, velocity: -0.1}]}'
```

**tmux (fast, on-robot):**

```bash
tmux new-window -t ros2 -n boom_cmd
tmux send-keys -t "ros2:boom_cmd" "source ~/ros2_ws/install/setup.bash" Enter
tmux send-keys -t "ros2:boom_cmd" "ros2 topic pub --rate 10 /mole/actuator_commands mole_msgs/msg/MoleActuatorCommands '{actuators: [{joint_name: \\\"J_BOOM\\\", mode: 2, velocity: -0.1}]}'" Enter
```

**Stop:**
- Ctrl+C in the tmux window, or

```bash
ros2 topic pub --once /mole/actuator_commands \
  mole_msgs/msg/MoleActuatorCommands \
  '{actuators: [{joint_name: "J_BOOM", mode: 2, velocity: 0.0}]}'
```
