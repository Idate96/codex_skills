---
name: sim-startup
description: Start or restart the Moleworks IsaacLab simulation and ROS2 stack in a clean, repeatable tmux layout (sim, robot, perception). Use when you need a fresh sim + TF/robot visualization + perception (including excavation mapping) in RViz.
---

# Sim Startup

## Overview
Start IsaacLab sim and the ROS stack with three tmux windows: `sim`, `robot`, `perception`. The robot window runs only the minimal robot/TF/RViz (no perception). The perception window runs `mole_perception_bringup` plus excavation mapping.

This skill is for the IsaacLab/Terra path, not the single-container Newton tmux workflow.
If everything runs inside one `moleworks_ros` container, use `newton-sim-ros-startup` and keep the default Fast DDS setup.
For this mixed-container path, `ROS_DOMAIN_ID=24` is the default local choice unless the user overrides it.

## Workflow

### 0) Preflight
- Identify the ROS container and Isaac container:
  - ROS container is the one running the `moleworks_ros` image.
  - Isaac container is the one running the `isaac-lab-moleworks_ext-dev` image.
- Always use the same `ROS_DOMAIN_ID` on both sides (example: `24`).
- DDS rule:
  - Default to the normal container DDS setup.
  - Only add Cyclone overrides in this mixed-container skill if the stack already depends on them or you have a concrete DDS failure to fix.
  - Do not copy those Cyclone exports into the single-container Newton workflow.
- **Check for a running sim before starting a new one**:
  - First inspect host memory and GPU headroom:
    - `free -h`
    - `nvidia-smi --query-gpu=index,name,memory.used,memory.total,utilization.gpu --format=csv,noheader,nounits`
  - Also inspect leftover ROS/Newton/sim processes before restarting:
    - `ps -eo pid,ppid,%mem,%cpu,etime,cmd | rg 'isaac|kit/python|standalone_dig_newton_env|newton_bridge.launch.py|robot.launch.py|mole_state_publisher|mole_perception_bringup|excavation_mapping|dig_3d_controller|ros2 launch'`
  - On the host, run `nvidia-smi` and look for processes like:
    - `.../isaac_sim/kit/python/bin/python3`
  - Map every suspicious GPU PID back to its command before killing anything:
    - `ps -fp <pid>`
  - Only stop the actual Isaac Sim / Isaac Lab kit process for this stack.
    Do **not** kill unrelated CUDA users such as tests, other Python jobs, or remote desktop helpers.
  - If you see one or more Isaac kit processes, stop the sim **from the ROS container**:
    - `mole_sim_ctl stop`
  - Re-run `nvidia-smi` and confirm the Isaac kit PID is gone **and** VRAM drops before starting a new sim.
  - Important: killing the outer `docker exec`/`timeout` is not enough. Isaac can leave an inner kit child alive, so always verify with `nvidia-smi` after cleanup.

### 1) Clean tmux organization
In your current tmux session, create three windows: `sim`, `robot`, `perception`.
- **Never rename the user’s current window.** If you need new windows, create them instead.
- If any of those windows already exist, **ask** before killing them; otherwise create new ones with the same names.
- Always capture pane output after each launch command.
- When launching via `tmux send-keys`, run commands as `bash -lc '...; exec bash -i'` so the window stays inspectable even if the command exits.
- Set `ROS_DOMAIN_ID=24` once per window before the launch command instead of resourcing or re-exporting between every follow-up command.

### 2) Start sim (window: `sim`)
Run inside the ROS container:
```bash
export ROS_DOMAIN_ID=24
source /opt/ros/jazzy/setup.bash
source /home/lorenzo/moleworks/ros2_ws/install/setup.bash
mole_sim_ctl terra
```
Capture the pane output immediately after launch.

### 3) Start robot (window: `robot`)
Robot/TF/RViz only (no perception):
```bash
export ROS_DOMAIN_ID=24
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

### 5) Controller rollout order (important)
For sim digging rollouts, keep this order:
1. Start sim.
2. Start robot/TF.
3. Start perception/excavation mapping.
4. Only after those are healthy, launch and activate the controller.
5. Only after the controller is active, send the action goal.

Do not send the goal before robot/perception are live, or you risk controller startup succeeding against stale or missing topics/TF.

### 6) (Optional) Start Foxglove bridge (separate window: `foxglove`)
Keep Foxglove in its own tmux window (do **not** piggyback on `robot`).
- If `foxglove` already exists, **ask** before killing it; otherwise create it.
- Always capture the pane output after starting.

```bash
source /home/lorenzo/.bashrc
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

### 7) Quick sanity checks (optional)
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
- If you explicitly opt into Cyclone for this mixed-container stack, expect some Humble/Jazzy DDS warning noise. Treat it as transport noise unless topics/TF are actually missing.
