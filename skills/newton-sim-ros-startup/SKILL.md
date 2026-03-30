---
name: newton-sim-ros-startup
description: Start or restart the Moleworks ROS2 stack using the Newton simulator inside the default moleworks_ros:latest Docker container. Use when you need a clean tmux layout for Newton bridge, robot/TF/RViz, perception (elevation + excavation mapping), and optional Foxglove bridge, all with use_sim_time:=true.
---

# Newton Sim ROS Startup

## Overview

Bring up a Newton-based sim run (publishes `/clock` + `/mole/state` + perception topics) plus the minimal Mole ROS stack for visualization and mapping.

Everything runs inside the same `moleworks_ros:latest` container.

Notes:
- Some images only have `root` inside the container. If `docker exec -u lorenzo ...` fails, rerun the same command without `-u lorenzo`.
- For Dig3D testing, `robot.launch.py` alone may leave `map -> CABIN` disconnected. In that case also launch `mole_joint_state_publisher/mole_state_publisher.launch.py`.
- On multi-monitor setups, the Newton GUI can appear on a different screen even when `gui:=true` works. If the user says they do not see it, inspect the host X tree:
```bash
xwininfo -root -tree 2>/dev/null | rg 'Newton Viewer|RViz'
```
  On this machine, the window has appeared at coordinates like `+5156,+38`, i.e. far to the right on a 3-screen layout.

## Workflow

### 0) Preflight

1. Find the running container:
```bash
docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
```

2. Pick a `ROS_DOMAIN_ID` (example `24`) and keep it consistent.

3. If you want Newton GUI, ensure X11 is available:
```bash
docker exec <container> bash -lc 'echo DISPLAY=$DISPLAY'
```

### 1) Launch/Attach The Container

Recommended workflow uses the Mole docker scripts (host side):
```bash
cd /home/lorenzo/moleworks/ros2_ws/src/moleworks_ros/docker
./docker_launch.sh moleworks_ros:latest mole.Dockerfile \
  --name moleworks-ros-newton \
  --detach
./docker_attach.sh --name moleworks-ros-newton --user lorenzo
tmux a -t newton_sim
```

If your local `moleworks_ros:latest` image predates the Newton merge into `mole.Dockerfile`, rebuild once with:
```bash
./docker_launch.sh moleworks_ros:latest mole.Dockerfile --no-cache --name moleworks-ros-newton --detach
```

If you need to restart from scratch:
```bash
docker rm -f moleworks-ros-newton || true
```

### 2) Create tmux Session (Inside The Container)

Create a persistent tmux session with 5 windows: `newton`, `robot`, `state_pub`, `perception`, `foxglove`.
```bash
docker exec -u lorenzo moleworks-ros-newton bash -lc '
  tmux kill-session -t newton_sim 2>/dev/null || true
  tmux new-session -d -s newton_sim -n newton
  tmux new-window -t newton_sim -n robot
  tmux new-window -t newton_sim -n state_pub
  tmux new-window -t newton_sim -n perception
  tmux new-window -t newton_sim -n foxglove
  tmux list-windows -t newton_sim
'
```

### 3) Start Newton Bridge (Window: newton)

Run the Newton ROS bridge via launch (sim time must be true):
```bash
docker exec -u lorenzo moleworks-ros-newton bash -lc '
  tmux send-keys -t newton_sim:newton "source /opt/ros/jazzy/setup.bash; source /workspace/moleworks/ros2_ws/install/setup.bash; ros2 launch mole_bringup newton_bridge.launch.py use_sim_time:=true gui:=true task:=m445_excavation_w_cabin_analytic publish_tf:=false" C-m
'
```

Notes:
- Keep `publish_tf:=false` when also running `robot.launch.py` (avoid TF conflicts).

### 4) Start Robot + TF + RViz (Window: robot)

Robot window is minimal: TF + robot_state_publisher + RViz (no perception).
```bash
docker exec -u lorenzo moleworks-ros-newton bash -lc '
  tmux send-keys -t newton_sim:robot "source /opt/ros/jazzy/setup.bash; source /workspace/moleworks/ros2_ws/install/setup.bash; ros2 launch mole_bringup robot.launch.py use_sim_time:=true on_machine:=false launch_low_level:=false launch_perception:=false launch_rviz:=true launch_foxglove:=false elevation_map_frame_mode:=map" C-m
'
```

### 5) Start Perception + Mapping (Window: perception)

Start elevation mapping + excavation mapping (global/map mode):
```bash
docker exec -u lorenzo moleworks-ros-newton bash -lc '
  tmux send-keys -t newton_sim:perception "source /opt/ros/jazzy/setup.bash; source /workspace/moleworks/ros2_ws/install/setup.bash; ros2 launch mole_perception_bringup bringup.launch.py use_sim_time:=true enable_camera:=false enable_lidar:=false enable_elevation_mapping:=true enable_robot_self_filter:=true enable_excavation_mapping:=true map_name:=excavation_site use_local_mapping:=false elevation_map_frame_mode:=map" C-m
'
```

### 6) Start Joint State Bridge When TF Is Split (Window: state_pub)

If `tf2_echo map CABIN` reports disconnected trees, start the state publisher stack:
```bash
docker exec -u lorenzo moleworks-ros-newton bash -lc '
  tmux send-keys -t newton_sim:state_pub "source /opt/ros/jazzy/setup.bash; source /workspace/moleworks/ros2_ws/install/setup.bash; ros2 launch mole_joint_state_publisher mole_state_publisher.launch.py use_sim_time:=true" C-m
'
```

### 7) Start Foxglove Bridge (Window: foxglove)

```bash
docker exec -u lorenzo moleworks-ros-newton bash -lc '
  tmux send-keys -t newton_sim:foxglove "source /opt/ros/jazzy/setup.bash; source /workspace/moleworks/ros2_ws/install/setup.bash; ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 use_sim_time:=true" C-m
'
```

### 8) Sanity Checks

Run with long timeouts (TF buffer needs time to warm up):
```bash
docker exec -u lorenzo moleworks-ros-newton bash -lc '
  timeout 15 bash -lc "source /opt/ros/jazzy/setup.bash; source /workspace/moleworks/ros2_ws/install/setup.bash; ros2 topic echo /clock --once 2>&1" | head -20
  timeout 15 bash -lc "source /opt/ros/jazzy/setup.bash; source /workspace/moleworks/ros2_ws/install/setup.bash; ros2 topic echo /mole/state --once 2>&1" | head -20
  timeout 15 bash -lc "source /opt/ros/jazzy/setup.bash; source /workspace/moleworks/ros2_ws/install/setup.bash; ros2 run tf2_ros tf2_echo map BASE 2>&1" | head -20
  timeout 15 bash -lc "source /opt/ros/jazzy/setup.bash; source /workspace/moleworks/ros2_ws/install/setup.bash; ros2 run tf2_ros tf2_echo map CABIN 2>&1" | head -20
  timeout 15 bash -lc "source /opt/ros/jazzy/setup.bash; source /workspace/moleworks/ros2_ws/install/setup.bash; ros2 run tf2_ros tf2_echo odom BASE 2>&1" | head -20
'
```

If the ROS topics are alive but the Newton GUI still seems "missing", confirm whether the X server has mapped `Newton Viewer` on another monitor:
```bash
xwininfo -root -tree 2>/dev/null | rg 'Newton Viewer|RViz'
```

### 9) Dig3D Bringup Note

`dig_3d_controller_cpp.launch.py` now requires an explicit `run_action:=false` unless you want it to auto-send a goal.

Example:
```bash
docker exec -u lorenzo moleworks-ros-newton bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /workspace/moleworks/ros2_ws/install/setup.bash
  ros2 launch mole_highlevel_controller_cpp dig_3d_controller_cpp.launch.py \
    use_sim_time:=true activate_controller:=true run_action:=false
'
```

### 10) If Something Is Missing

If `colcon build` complains about duplicate packages, always add `--base-paths src`.

If `newton_bridge.launch.py` is not found:
```bash
docker exec -u lorenzo moleworks-ros-newton bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /workspace/moleworks/ros2_ws
  colcon build --base-paths src --packages-select mole_bringup --symlink-install
'
```

If `mole_bringup` build fails due to missing `mole_sam3_segmentation` in the install (due to perception exec_dep chain), build it first:
```bash
docker exec -u lorenzo moleworks-ros-newton bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /workspace/moleworks/ros2_ws
  colcon build --base-paths src --packages-select mole_sam3_segmentation --symlink-install
'
```

If elevation mapping logs a shape/broadcast error when loading the `excavation_site` map, ensure you are running the updated `mole_perception_bringup` from `ros2_ws` (it selects a large-map core param file for that map name).

## 11) Teardown

When you finish a Newton ROS parity/debug session, do not assume killing the
tmux session is enough. Some `ros2 launch` trees can outlive the panes.

Preferred teardown inside the container:

```bash
tmux kill-session -t newton_sim 2>/dev/null || true

ps -eo pid,cmd | rg 'newton_bridge.launch.py|standalone_dig_newton_env.py|dig_3d_controller_cpp.launch.py|robot.launch.py|mole_state_publisher.launch.py|foxglove|compare_dig3d_live_obs.py|ros2 action send_goal'

kill <pid1> <pid2> ... 2>/dev/null || true
sleep 2
kill -9 <pid1> <pid2> ... 2>/dev/null || true
```

Host-side resource check:

```bash
docker stats --no-stream --format '{{.Name}}\t{{.MemUsage}}\t{{.CPUPerc}}' moleworks-ros-newton
nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader,nounits
```

Hard rule:

- If `tmux ls` is empty but `ros2 launch` / Newton / Dig3D processes are still
  alive, the stack is not cleaned up yet.
```
