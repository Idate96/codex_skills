---
name: ros2-debugging
description: ROS2 debugging with proper TF buffer timeouts. Use when checking transforms, topics, nodes, services, or tmux-managed ROS2 systems.
---

# ROS2 Debugging Best Practices

Keep this skill generic. Do not turn it into a project-specific launch runbook.

## Checking TF Transforms

**CRITICAL: Always use long timeouts (10-15 seconds minimum).**

Tools like `tf2_echo` need 1-2 seconds to initialize their TF buffer before they can query transforms.

```bash
# CORRECT: Long timeout allows TF buffer to initialize
timeout 15 bash -lc 'ros2 run tf2_ros tf2_echo map BASE 2>&1' | head -20

# WRONG: Short timeout often fails before transform data appears
timeout 3 ros2 run tf2_ros tf2_echo map BASE
```

Why this matters:

- `tf2_echo` needs time to initialize its TF buffer.
- Early "frame does not exist" output is often transient and normal.
- Short timeouts often fail before real transform data appears.
- Piping to `head` is fine if the timeout is long enough.

## Checking ROS Topics

On crowded robot PCs, a fresh-shell `ros2 topic list` can look empty even when the
system is running. Treat that as a weak signal. Prefer direct checks for the
specific topic/service/node, longer timeouts, TF evidence, tmux process/logs, and
publisher ownership before restarting anything.

```bash
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
ros2 topic list
timeout 10 ros2 topic hz /your_topic
timeout 10 ros2 topic echo /your_topic --once
ros2 topic info /your_topic --verbose
```

If you are running multiple ROS stacks on different domains:

- verify `ROS_DOMAIN_ID` in the active shell before trusting any graph output
- keep one tmux session per domain when possible
- encode the domain in the session name so captures and restarts are unambiguous
- do not conclude "the topic is gone" until you have ruled out a domain mismatch

## Common ROS2 Debugging Commands

```bash
ros2 node list
ros2 node info /your_node

ros2 service list
ros2 service call /service_name std_srvs/srv/Empty "{}"

ros2 param list /your_node
ros2 param get /your_node parameter_name

ros2 run tf2_tools view_frames
```

## tmux Integration

When running ROS commands in tmux, keep the pane alive long enough to inspect failures:

```bash
tmux new-window -n ros2
tmux send-keys -t ros2 "bash -lc 'ros2 launch your_package your_launch.py; exec bash -i'" C-m
sleep 0.5
tmux capture-pane -p -S -200 -t ros2
```

For multi-domain debugging, make the tmux target itself identify the domain, for example:

```bash
tmux new-session -d -s ros123_debug
tmux send-keys -t ros123_debug "bash -lc 'export ROS_DOMAIN_ID=123; ros2 topic list; exec bash -i'" C-m
```

## Moleworks / Newton Handoff

- If the task is Moleworks Newton parity through `moleworks_ros`, also use `newton-ros-parity`.
- Keep this skill focused on portable ROS2 debugging primitives: TF timing, topic checks, node/service inspection, and tmux capture.
