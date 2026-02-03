---
name: ros2-debugging
description: ROS2 debugging with proper TF buffer timeouts. Use when checking transforms, topics, or debugging ROS2 systems.
---

# ROS2 Debugging Best Practices

## Checking TF Transforms

**CRITICAL: Always use long timeouts (10-15 seconds minimum).**

Tools like `tf2_echo` need 1-2 seconds to initialize their TF buffer before they can query transforms.

```bash
# CORRECT: Long timeout allows TF buffer to initialize
timeout 15 bash -c 'ros2 run tf2_ros tf2_echo map BASE 2>&1' | head -20

# WRONG: Short timeout often fails before transform data appears
timeout 3 ros2 run tf2_ros tf2_echo map BASE  # Will likely fail
```

### Why This Matters

- `tf2_echo` needs 1-2 seconds to initialize its TF buffer
- During initialization, it prints "frame does not exist" errors (this is normal)
- Short timeouts (3-5s) often trigger before real transform data appears
- Piping to `head` can cause premature termination without proper delays

## Checking ROS Topics

```bash
# List all topics
ros2 topic list

# Check topic frequency (let it run for a few seconds)
timeout 10 ros2 topic hz /your_topic

# Echo topic data with timeout
timeout 10 ros2 topic echo /your_topic --once

# Check topic info
ros2 topic info /your_topic --verbose
```

## Common ROS2 Debugging Commands

```bash
# Check node status
ros2 node list
ros2 node info /your_node

# Service debugging
ros2 service list
ros2 service call /service_name std_srvs/srv/Empty "{}"

# Parameter inspection
ros2 param list /your_node
ros2 param get /your_node parameter_name

# View TF tree
ros2 run tf2_tools view_frames
```

## tmux Integration

When running ROS scripts, open a new permanent tmux window:

```bash
# If in tmux session, create new window (counting starts from 1)
tmux new-window -n ros2

# Run your ROS command
ros2 launch your_package your_launch.py

# Always capture pane output to verify
tmux capture-pane -p
```
