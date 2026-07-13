---
name: mole-graph-msf-container
description: Start or attach to the Mole Graph-MSF container in tmux, optionally launching `mole_estimator`.
---

# Mole Graph-MSF Container

## Overview
Start the Graph-MSF container from the host and keep it running in tmux, then (if requested) launch the `mole_estimator` inside the container.

## Quick Start (tmux)
1) Run the helper script:
```
/home/lorenzo/.codex/skills/mole-graph-msf-container/scripts/start_graph_msf_container.sh [session] [window]
```
2) Attach to the session:
```
tmux attach -t <session>
```
3) Verify the container is running:
```
docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
```

## Start Without tmux (direct shell)
Use this when a one-off interactive container is fine.
```
bash -lc "source ~/.bashrc; moleworks_ros_graph_msf"
```

## Launch the State Estimator (inside container, optional)
Use when the user asks to run the estimator from the Graph-MSF container.
```
source /opt/ros/jazzy/setup.bash
source /home/lorenzo/ros2_ws/install/setup.bash
ros2 launch mole_estimator mole_estimator.launch.py \
  config:=/home/lorenzo/ros2_ws/src/moleworks_ros/mole_estimator/config/mole_estimator_robot.yaml
```

## Health Checks
```
ros2 topic list | egrep "graph_msf|/mole/state"
ros2 topic echo --once /mole/state
ros2 topic echo --once --qos-reliability best_effort /tf
```

## Stop
- Exit the container shell, or run:
```
docker stop <container_name>
```
- If tmux was used:
```
tmux kill-session -t <session>
```
