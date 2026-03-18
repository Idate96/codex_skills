---
name: sim-startup
description: Start or restart the Moleworks IsaacLab simulation and ROS2 stack in a clean, repeatable tmux layout (sim, robot, perception). Use when you need a fresh sim + TF/robot visualization + perception (including excavation mapping) in RViz.
---

# Sim Startup

## Overview
Start IsaacLab sim and the ROS stack with three tmux windows: `sim`, `robot`, `perception`. The robot window runs only the minimal robot/TF/RViz (no perception). The perception window runs `mole_perception_bringup` plus excavation mapping.

## Workflow

### 0) Preflight
- Identify the ROS container and Isaac container:
  - ROS container is the one running the `moleworks_ros` image.
  - Isaac container is the one running the `isaac-lab-moleworks_ext-dev` image.
- Always use the same `ROS_DOMAIN_ID` on both sides (example: `24`).
- Use Cyclone DDS if already configured:
  - `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  - `export CYCLONEDDS_URI=file:///home/lorenzo/moleworks/ros2_ws/src/moleworks_ros/.ros/cyclonedds.xml`
- **Check for a running sim before starting a new one**:
  - On the host, run `nvidia-smi` and look for processes like:
    - `.../isaac_sim/kit/python/bin/python3`
  - If you see one or more, stop the sim **from the ROS container**:
    - `mole_sim_ctl stop`
  - Re-run `nvidia-smi` and confirm those Isaac Sim processes are gone before starting a new sim.

### 1) Clean tmux organization
In your current tmux session, create three windows: `sim`, `robot`, `perception`.
- **Never rename the user’s current window.** If you need new windows, create them instead.
- If any of those windows already exist, **ask** before killing them; otherwise create new ones with the same names.
- Always capture pane output after each launch command.
- When launching via `tmux send-keys`, run commands as `bash -lc '...; exec bash -i'` so the window stays inspectable even if the command exits.

### 2) Start sim (window: `sim`)
Run inside the ROS container:
```bash
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/lorenzo/moleworks/ros2_ws/src/moleworks_ros/.ros/cyclonedds.xml
source /opt/ros/jazzy/setup.bash
source /home/lorenzo/moleworks/ros2_ws/install/setup.bash
mole_sim_ctl terra
```
Capture the pane output immediately after launch.

### 3) Start robot (window: `robot`)
Robot/TF/RViz only (no perception):
```bash
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/lorenzo/moleworks/ros2_ws/src/moleworks_ros/.ros/cyclonedds.xml
source /opt/ros/jazzy/setup.bash
source /home/lorenzo/moleworks/ros2_ws/install/setup.bash
ros2 launch mole_bringup robot.launch.py \
  use_sim_time:=true \
  on_machine:=false \
  launch_perception:=false \
  launch_rviz:=true \
  elevation_map_frame_mode:=map
```
If `elevation_map_frame_mode` is not accepted on your branch, drop that argument.

### 4) Start perception + excavation mapping (window: `perception`)
Run the perception bringup (it now includes excavation mapping).
- For sim, keep drivers off (no real lidar/camera drivers).
- Use a map name if you want a design map, or `map_name:=none` to initialize from live elevation.
```bash
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/lorenzo/moleworks/ros2_ws/src/moleworks_ros/.ros/cyclonedds.xml
source /opt/ros/jazzy/setup.bash
source /home/lorenzo/moleworks/ros2_ws/install/setup.bash
ros2 launch mole_perception_bringup bringup.launch.py \
  use_sim_time:=true \
  enable_camera:=false \
  enable_lidar:=false \
  enable_elevation_mapping:=true \
  enable_robot_self_filter:=true \
  enable_excavation_mapping:=true \
  map_name:=excavation_site \
  use_local_mapping:=false
```

If you use `map_name:=none`, the excavation mapping launch will **auto-wait** for `map -> CABIN_ANCHOR` (default 30s)
when the desired-elevation override uses `profile` + `CABIN_ANCHOR`. You can override this behavior with:
```bash
ros2 launch mole_perception_bringup bringup.launch.py \
  use_sim_time:=true \
  enable_camera:=false \
  enable_lidar:=false \
  enable_elevation_mapping:=true \
  enable_robot_self_filter:=true \
  enable_excavation_mapping:=true \
  map_name:=none \
  use_local_mapping:=false \
  wait_for_cabin_anchor_tf:=auto \
  cabin_anchor_wait_timeout_s:=30.0
```
Capture the pane output after the command starts.

### 5) (Optional) Start Foxglove bridge (separate window: `foxglove`)
Keep Foxglove in its own tmux window (do **not** piggyback on `robot`).
- If `foxglove` already exists, **ask** before killing it; otherwise create it.
- Always capture the pane output after starting.

```bash
source /home/lorenzo/.bashrc
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

### 6) Quick sanity checks (optional)
Recommended: just inspect the three tmux windows (`sim`, `robot`, `perception`) and confirm nothing obviously failed (tracebacks, exceptions, nodes exiting, repeated `[ERROR]`).

If something looks wrong, run one or two of these (keep long timeouts for TF buffer warmup):
- `ros2 topic echo /clock --once`
- `ros2 topic echo /mole/state --once --qos-reliability best_effort`
- `timeout 15 bash -c 'ros2 run tf2_ros tf2_echo map BASE 2>&1'`
- `timeout 15 bash -c 'ros2 run tf2_ros tf2_echo map CABIN_ANCHOR 2>&1'`
  - `tf2_echo` may print “frame does not exist” for a few seconds while buffers warm up.

## Notes
- Keep `sim`, `robot`, `perception` as the only three windows so output is easy to inspect.
- For real hardware runs, set `on_machine:=true` and enable sensors in the perception launch.
- TF is on `/tf` and `/tf_static` (not `/mole/tf`).
- `CABIN_ANCHOR` is only required for the desired-elevation `profile` override; the dig controller uses `CABIN`/`BASE`.
- Expected noise: the sim side runs ROS 2 Humble while the ROS container is Jazzy, so CycloneDDS commonly prints lots of warnings on both sides (for example: “Failed to parse type hash ... from USER_DATA (null)” and various `serdata.cpp`/`rcutils` error-state warnings). Treat these as normal unless topics/TF are actually missing.
