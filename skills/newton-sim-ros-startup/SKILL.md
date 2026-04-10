---
name: newton-sim-ros-startup
description: Start or restart the Moleworks ROS2 stack using the Newton simulator inside the default moleworks_ros:latest Docker container. Use when you need a clean tmux layout for Newton bridge, robot/TF/RViz, perception (elevation + excavation mapping), optional Foxglove bridge, or an isolated bridge-only validation stack on a specific ROS domain, all with use_sim_time:=true.
---

# Newton Sim ROS Startup

## Use This Skill For

Use this only for the single-container Newton workflow inside `moleworks_ros:latest`.

If the stack is split across Isaac/Terra and ROS containers, use `sim-startup` or `moleworks-terra-stack` instead.

## Non-Negotiables

- Attach to the running `moleworks_ros` container with `docker_attach.sh` and work from one tmux session inside it.
- Once inside the container, prefer container-local tmux as the shared control plane. Do not hide shared stack bringup behind a separate host-side tmux session that attaches into the container.
- Do not use `docker exec` for normal interactive bringup.
- Keep the container default Fast DDS setup.
- Do not export `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` or `CYCLONEDDS_URI` for this workflow.
- Use `ROS_DOMAIN_ID=24` unless the user explicitly asks for a different domain.
- If the user asks for an isolated side stack, use one tmux session per ROS domain instead of mixing domains in one session.
- On this Fast DDS setup, keep `ROS_DOMAIN_ID <= 232`. Domain `333` is invalid here.
- Newton runs with `gui:=true` here.
- If the user asks to load a map, load the same terrain artifact on both sides:
  - Newton soil via the existing `--elevation-map` support in `standalone_fee_terra_newton_env.py`
  - ROS excavation mapping via `design_bag_path`
- The terrain seed order is strict: restart Newton sim with the requested map, or load terrain on the Newton side at runtime, before launching robot/perception/dig.
- Do not try to reseed Newton terrain after the rest of the ROS stack is already live. If the Newton soil seed is wrong, restart the split stack in the right order instead of patching it mid-run.
- Source ROS once per tmux window, not before every single command.
- Before every restart, check RAM, VRAM, and stale ROS/Newton processes. Do not stack a new sim on top of leftovers.
- For Nav2 or any sim driving workflow, enable the arm hold controller by default so the arm does not sag while the base moves.
- Do not substitute ad-hoc viewer scripts for `standalone_fee_terra_newton_env.py` or `standalone_dig_newton_env.py` when the user expects ROS bridge, pointcloud, or perception parity.

## 0) Host Preflight

Find the running container and check resources:

```bash
docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
free -h
nvidia-smi --query-gpu=index,name,memory.used,memory.total,utilization.gpu --format=csv,noheader,nounits
docker top moleworks_ros -eo pid,ppid,%mem,%cpu,etime,cmd 2>/dev/null | rg 'standalone_dig_newton_env|newton_bridge|robot.launch.py|mole_state_publisher|mole_perception_bringup|excavation_mapping|dig_3d_controller|rviz|ros2 launch|pytest|colcon' || true
```

If the container is not running yet:

```bash
cd /home/lorenzo/moleworks/ros2_ws/src/moleworks_ros/docker
./docker_launch.sh moleworks_ros:latest mole.Dockerfile --name moleworks_ros --detach
```

Attach the normal way:

```bash
cd /home/lorenzo/moleworks/ros2_ws/src/moleworks_ros/docker
./docker_attach.sh --name moleworks_ros --user lorenzo
```

If `lorenzo` is not present in that container, retry with `--user root`.

## 1) Container Preflight

Inside the attached shell:

```bash
export ROS_DOMAIN_ID=24
source /opt/ros/jazzy/setup.bash
source /workspace/moleworks/ros2_ws/install/setup.bash
export MW_ROS_WS=/workspace/moleworks/ros2_ws
export MW_LOCAL_SURFACE_PKG=package://mole_maps/maps/hong0326_no_holes/hong0326_no_holes_surface
export MW_LOCAL_SURFACE_MCAP=/workspace/moleworks/ros2_ws/src/moleworks_maps/maps/hong0326_no_holes/hong0326_no_holes_surface/hong0326_no_holes_surface_0.mcap
```

Overlay guard for this container:

```bash
export ROS_DOMAIN_ID=24
source /opt/ros/jazzy/setup.bash
source /workspace/moleworks/ros2_ws/install/setup.bash
export MW_ROS_WS=/workspace/moleworks/ros2_ws
readlink -f "$MW_ROS_WS/install/setup.bash"
ros2 pkg prefix mole_msgs
ros2 pkg prefix mole_ocs2_arm_controller
```

Expected result:
- `readlink -f` resolves to `/workspace/moleworks/ros2_ws/install/setup.bash`
- `ros2 pkg prefix mole_msgs` resolves under `/workspace/moleworks/ros2_ws/install`
- `ros2 pkg prefix mole_ocs2_arm_controller` resolves under `/workspace/moleworks/ros2_ws/install`

If a fresh attached shell prints `Sourcing ROS2 workspace at /home/lorenzo/ros2_ws`, do **not** trust that
as the live overlay for this workflow. That is a shell convenience default, not the validated Newton/Terra
overlay. Immediately re-source `/workspace/moleworks/ros2_ws/install/setup.bash` and re-run the checks above
before any `ros2 launch`, `ros2 run`, or service call. If `ros2 pkg prefix mole_ocs2_arm_controller` fails
or points somewhere else, stop and fix the overlay first.

Run Newton branch commands from the active Newton worktree root. Example:

```bash
cd /workspace/moleworks/.worktrees/moleworks_newton_pre_pr113_redesign
export MW_NEWTON_ROOT=$PWD
```

For Newton standalone ROS bridge scripts from the worktree, also activate the worktree venv after sourcing the ROS overlay:

```bash
source /workspace/moleworks/ros2_ws/install/setup.bash
source "$MW_NEWTON_ROOT/.venv/bin/activate"
```

If the user explicitly asks for a different ROS domain, replace `24` consistently in every pane and in every
`nohup env ROS_DOMAIN_ID=...` launch. Do not mix domains inside one tmux session.

Recommended naming for isolated runs:

- full stack: `newton_sim_<domain>`
- bridge-only stack: `ros<domain>_bridge`

If Foxglove is also split across domains, assign a unique port per stack and keep that port in the notebook or
final handoff. Do not reuse `8765` by default if another session is already using it.

This avoids missing ROS Python packages like `mole_msgs` when launching `standalone_*_newton_env.py`.
It also makes the local sibling `newton-actuators` source visible when the worktree
uses that package through `tool.uv.sources`.

If this session is also using a local planner/workspace overlay, source that in the tmux window before
`single_workspace.launch.py`. In the current `dev/terra` workflow on domain `100`, that overlay is:

```bash
source /tmp/sw100_wspl_install/setup.bash
```

If you skip this, `apply_workspace.py` can import an older `workspace_planner_msgs` package and fail
before Terra starts, typically with `ImportError: cannot import name 'ComputeWorkspaceNextAction'`.

Only if DDS looks contaminated from an older shell, inspect once:

```bash
env | rg '^(RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_DISCOVERY_SERVER|FASTRTPS_DEFAULT_PROFILES_FILE)='
```

For this skill, the expected local-mode result is no `RMW_IMPLEMENTATION`, no `CYCLONEDDS_URI`, and no stale robot discovery-server env. If `ROS_DISCOVERY_SERVER` or `FASTRTPS_DEFAULT_PROFILES_FILE` is set, reset the shell to local mode:

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER
ros2 daemon stop && ros2 daemon start
```

Confirm X11 is present before launch:

```bash
echo "DISPLAY=$DISPLAY"
```

If GUI apps fail to appear, also verify the host Xauth file exists:

```bash
ls -la /tmp/.docker.xauth
```

Clean up stale session state:

```bash
tmux kill-session -t newton_sim 2>/dev/null || true
pkill -f 'standalone_fee_terra_newton_env|standalone_dig_newton_env|native_fee_terra_default_viewer.py|newton_bridge.launch.py|robot.launch.py|mole_state_publisher|mole_perception_bringup|dig_3d_controller_cpp.launch.py|compare_dig3d_live_obs.py|foxglove_bridge' || true
sleep 2
pkill -9 -f 'standalone_fee_terra_newton_env|standalone_dig_newton_env|native_fee_terra_default_viewer.py|newton_bridge.launch.py|robot.launch.py|mole_state_publisher|mole_perception_bringup|dig_3d_controller_cpp.launch.py|compare_dig3d_live_obs.py|foxglove_bridge' || true
```

Also check for orphan attach-shell parents that can keep background `nohup` launches alive even after tmux is gone:

```bash
docker top moleworks_ros -eo pid,ppid,%mem,%cpu,etime,cmd 2>/dev/null | \
  rg 'docker_attach|standalone_fee_terra_newton_env|standalone_dig_newton_env|robot.launch.py|mole_state_publisher.launch.py'
```

If you still see old standalone Newton or launch processes under an old shell parent, kill those exact processes before relaunching. Otherwise you can end up with duplicate Newton viewers or duplicate bridge publishers even though the obvious tmux session is gone.

Also check for extra attached container shells. Hidden `docker_attach.sh` shells can keep
background Newton sims alive even after the visible tmux session is gone:

```bash
ps -ef | rg 'bash --rcfile /home/bash.bashrc|standalone_fee_terra_newton_env|standalone_dig_newton_env' || true
```

If more than one attached shell is still alive, kill the stale shell or its background Newton job
before relaunching. Otherwise you can end up with duplicate `moleworks_newton_ros_bridge`
publishers on `/clock` and `/mole/state` without noticing immediately.

If low-level passthrough looks wrong after restart, check `/mole/actuator_commands`
before blaming Newton or the bridge:

```bash
ros2 topic info /mole/actuator_commands -v
```

For the minimal Newton stack, unexpected publishers such as `dig_3d_controller`,
`wheel_hold_controller`, or a stale planner/executor mean the domain is still contaminated.
Kill that old launch tree first and only then re-test passthrough.

If the `ros2` CLI graph looks empty or inconsistent while the processes are clearly alive,
restart the daemon before trusting node/topic/service introspection:

```bash
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true
```

For low-level controller validation in this workflow, do not run multiple heavy CUDA
pytest lanes in parallel. Sequential runs give stable turn-tracking results.

## 2) Create The tmux Layout

Inside the container:

```bash
tmux new-session -d -s newton_sim -n newton
tmux new-window -t newton_sim -n robot
tmux new-window -t newton_sim -n state_pub
tmux new-window -t newton_sim -n perception
tmux new-window -t newton_sim -n dig
tmux new-window -t newton_sim -n single_ws
tmux new-window -t newton_sim -n debug
tmux new-window -t newton_sim -n foxglove
tmux list-windows -t newton_sim
```

For bridge-only validation on a separate domain, use the smaller layout instead:

```bash
tmux new-session -d -s ros123_bridge -n newton
tmux new-window -t ros123_bridge -n debug
tmux new-window -t ros123_bridge -n foxglove
tmux new-window -t ros123_bridge -n robot
tmux list-windows -t ros123_bridge
```

Use this layout when the task is low-level command validation, bridge-only diagnostics, or Foxglove inspection
below `robot.launch.py` / Nav2.

For the integrated Terra single-workspace workflow, use the `single_ws` window instead of the separate
`robot`, `state_pub`, `perception`, and `dig` windows.

Recommended shell prologue for each window:

```bash
export ROS_DOMAIN_ID=24
source /opt/ros/jazzy/setup.bash
source /workspace/moleworks/ros2_ws/install/setup.bash
export MW_ROS_WS=/workspace/moleworks/ros2_ws
readlink -f "$MW_ROS_WS/install/setup.bash"
```

If you start a window with a one-shot `tmux new-window '...'` command instead of opening
an interactive shell first, inline the same `source` sequence inside that quoted command.
Do not assume the standalone Newton script can import ROS Python packages otherwise.

## 3) Launch Windows

Use detached jobs so the panes stay usable. Write logs to `/tmp`.

### newton

For the local `fee_terra` single-workspace workflow, default to the dedicated wrapper and preload the
Hong no-holes surface into Newton itself. This is the Newton-side half of the dual map-load contract:

```bash
cd "$MW_NEWTON_ROOT"
nohup env ROS_DOMAIN_ID=24 \
  python scripts/ros/standalone_fee_terra_newton_env.py \
  --elevation-map "$MW_LOCAL_SURFACE_MCAP" \
  --elevation-layer elevation \
  --max-depth-layer desired_elevation \
  --gui \
  > /tmp/newton.log 2>&1 &
tail -f /tmp/newton.log
```

If the command is launched directly from `tmux new-window '...'`, write it like this instead:

```bash
cd "$MW_NEWTON_ROOT"
nohup bash -lc '
  export ROS_DOMAIN_ID=24
  source /opt/ros/jazzy/setup.bash
  source /workspace/moleworks/ros2_ws/install/setup.bash
  source "$MW_NEWTON_ROOT/.venv/bin/activate"
  export MW_LOCAL_SURFACE_MCAP=/workspace/moleworks/ros2_ws/src/moleworks_maps/maps/hong0326_no_holes/hong0326_no_holes_surface/hong0326_no_holes_surface_0.mcap
  exec python scripts/ros/standalone_fee_terra_newton_env.py \
    --elevation-map "$MW_LOCAL_SURFACE_MCAP" \
    --elevation-layer elevation \
    --max-depth-layer desired_elevation \
    --gui
' > /tmp/newton.log 2>&1 &
tail -f /tmp/newton.log
```

That exact failure mode shows up as `ModuleNotFoundError: No module named 'mole_msgs'`.

For non-`fee_terra` tasks, fall back to `standalone_dig_newton_env.py` and pass the requested
`--elevation-map` explicitly whenever the user asks for a preloaded terrain.

If the branch is using the ROS launch wrapper instead, use:

```bash
nohup env ROS_DOMAIN_ID=24 \
  ros2 launch mole_bringup newton_bridge.launch.py \
  use_sim_time:=true \
  gui:=true \
  publish_tf:=false \
  > /tmp/newton.log 2>&1 &
tail -f /tmp/newton.log
```

### robot

```bash
nohup env ROS_DOMAIN_ID=24 \
  ros2 launch mole_bringup robot.launch.py \
  use_sim_time:=true \
  on_machine:=false \
  launch_low_level:=false \
  launch_perception:=false \
  launch_rviz:=true \
  launch_foxglove:=false \
  launch_arm_hold_controller:=true \
  activate_arm_hold_controller:=true \
  launch_turn_hold_controller:=true \
  activate_turn_hold_controller:=false \
  elevation_map_frame_mode:=map \
  > /tmp/robot.log 2>&1 &
tail -f /tmp/robot.log
```

For this skill, treat arm hold as part of the default sim-nav safety posture. Do not launch a Nav2 driving stack in Newton sim with the arm unheld unless the user explicitly asks for that.

For the bridge-only layout, use `robot` as the model/TF sidecar instead of `robot.launch.py`:

```bash
nohup env ROS_DOMAIN_ID=123 \
  ros2 launch mole_joint_state_publisher mole_state_publisher.launch.py \
  use_sim_time:=true \
  namespace:=mole \
  publish_frequency:=25.0 \
  > /tmp/robot.log 2>&1 &
tail -f /tmp/robot.log
```

This is the minimal sidecar that makes Foxglove show the excavator model on a bridge-only domain. It provides:

- `/mole/joint_states`
- `/mole/robot_description`
- `/tf`
- `/tf_static`

Without it, Foxglove can connect successfully but still show no robot geometry.

### state_pub

Use this when `map -> CABIN` is disconnected or when sim TF needs the Mole state publisher path:

```bash
nohup env ROS_DOMAIN_ID=24 \
  ros2 launch mole_joint_state_publisher mole_state_publisher.launch.py \
  use_sim_time:=true \
  > /tmp/state_pub.log 2>&1 &
tail -f /tmp/state_pub.log
```

### perception

For Dig3D parity in Newton sim, use local excavation mapping and keep robot self-filter off unless the user explicitly wants a different setting:

```bash
nohup env ROS_DOMAIN_ID=24 \
  ros2 launch mole_perception_bringup bringup.launch.py \
  use_sim_time:=true \
  mapping_profile:=local \
  enable_camera:=false \
  enable_lidar:=false \
  enable_elevation_mapping:=true \
  enable_excavation_mapping:=true \
  enable_robot_self_filter:=false \
  excavation_mapping_upstream_layer:=smooth \
  > /tmp/perception.log 2>&1 &
tail -f /tmp/perception.log
```

### single_ws

For the full local Terra single-workspace workflow, prefer one integrated launch over the split
`robot` + `state_pub` + `perception` + `dig` bringup. This launch already manages wheel hold,
turn hold, OCS2 arm MPC, Dig3D, workspace planning, and the Terra executor.

Use the Hong no-holes local seed on both sides:
- Newton: `--elevation-map "$MW_LOCAL_SURFACE_MCAP"`
- ROS: `design_bag_path:=$MW_LOCAL_SURFACE_PKG`

Do not start `single_workspace.launch.py` against a Newton viewer that was not launched with the same
surface artifact. That creates exactly the perception mismatch where lidar sees one terrain and ROS
excavation mapping is seeded from another.

```bash
nohup env ROS_DOMAIN_ID=24 \
  bash -lc 'source /opt/ros/jazzy/setup.bash && \
  source /workspace/moleworks/ros2_ws/install/setup.bash && \
  source /tmp/sw100_wspl_install/setup.bash && \
  ros2 launch mole_bringup single_workspace.launch.py \
  use_sim_time:=true \
  on_machine:=false \
  launch_rviz:=false \
  enable_robot_self_filter:=false \
  excavation_mapping_upstream_layer:=smooth \
  use_dump_pitch_schedule:=false \
  ocs2_task_profile:=real_no_collisions \
  design_map_name:=none \
  design_bag_path:=$MW_LOCAL_SURFACE_PKG \
  workspace_config_path:=/workspace/moleworks/ros2_ws/src/moleworks_ros/high_level_planning/workspace_planner/config/test_workspace_dump_point_45.yaml' \
  > /tmp/single_ws.log 2>&1 &
tail -f /tmp/single_ws.log
```

### dig

Bring up Dig3D only after Newton, robot, state publisher, and perception are healthy:

```bash
nohup env ROS_DOMAIN_ID=24 \
  ros2 launch mole_highlevel_controller_cpp dig_3d_controller_cpp.launch.py \
  use_sim_time:=true \
  config:=no_aoa \
  mode:=weightedobs_rate0050_s203_1750 \
  activate_controller:=true \
  run_action:=false \
  > /tmp/dig3d.log 2>&1 &
tail -f /tmp/dig3d.log
```

### foxglove

Optional:

```bash
nohup env ROS_DOMAIN_ID=24 \
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:=8765 \
  use_sim_time:=true \
  > /tmp/foxglove.log 2>&1 &
tail -f /tmp/foxglove.log
```

For isolated side stacks, prefer a non-default port and state it explicitly, for example:

```bash
nohup env ROS_DOMAIN_ID=123 \
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:=8766 \
  address:=0.0.0.0 \
  use_sim_time:=true \
  > /tmp/foxglove.log 2>&1 &
tail -f /tmp/foxglove.log
```

If Foxglove connects but the excavator is not visible, check for these topics first:

```bash
ros2 topic list | rg '^/mole/(joint_states|robot_description)$'
ros2 topic info /mole/robot_description -v
```

That failure mode is usually missing model publishers, not a bridge failure.

## 4) Sanity Checks

Use long timeouts:

```bash
timeout 15 bash -lc 'ros2 topic echo /clock --once'
timeout 15 bash -lc 'ros2 topic echo /mole/state --once'
timeout 15 bash -lc 'ros2 run tf2_ros tf2_echo map BASE'
timeout 15 bash -lc 'ros2 run tf2_ros tf2_echo map CABIN'
```

If the Newton GUI seems missing, inspect the host X tree:

```bash
xwininfo -root -tree 2>/dev/null | rg 'Newton Viewer|RViz'
```

For sim-nav bringup, also confirm the hold controllers ended up where you expect:

```bash
ros2 lifecycle get /mole/arm_hold_controller
ros2 lifecycle get /mole/ackermann_drive_controller
ros2 topic info /mole/actuator_commands -v
```

Expected state after startup:
- `/mole/arm_hold_controller`: `active [3]`
- `/mole/ackermann_drive_controller`: usually `inactive [2]` until you intentionally enable driving

For bridge-only stacks, the expected minimum graph is:

- `/clock`
- `/mole/actuator_commands`
- `/mole/joint_states`
- `/mole/measurements`
- `/mole/robot_description`
- `/mole/state`
- `/odom`
- `/tf`
- `/tf_static`

If the arm is visibly sinking even though arm hold was launched, do not trust the current hold target. Re-capture it from the reset posture:

```bash
ros2 lifecycle set /mole/arm_hold_controller deactivate
ros2 service call /mole/reset_robot std_srvs/srv/Trigger '{}'
ros2 lifecycle set /mole/arm_hold_controller activate
```

Use `/reset` only when you want the full Newton environment reset. Use `/mole/reset_robot` when you want to respawn the robot without changing terrain.

## 5) Dig Deployment Order

For Dig3D parity runs, keep the rollout order strict:

1. Start Newton with the requested surface via `--elevation-map`.
2. Start `robot.launch.py`.
3. Start `mole_state_publisher` if TF is split.
4. If the Newton terrain seed is wrong or stale, stop here and restart Newton with the correct `--elevation-map` before continuing.
5. Start local perception and excavation mapping with the same surface via `design_bag_path`.
6. Apply the runtime profile to both:
   - `/mole/excavation_mapping/apply_runtime_profile`
   - `/mole/newton/apply_runtime_profile`
7. Only then launch Dig3D and send `/run_dig_3d`.

Do not send the goal before the runtime profile is applied on both sides.
Do not treat “map loaded” as complete unless both the Newton soil and ROS excavation mapping were seeded from the same artifact.
Do not mirror a runtime profile into Newton on top of a stale split stack. Fix the Newton terrain seed first, then relaunch the downstream ROS stack.

For `single_workspace.launch.py`, the executor owns that sequencing internally after Newton is up and
the runtime workspace apply succeeds, so do not also launch the split `dig` window on top of it.

## 6) Teardown

Inside the container:

```bash
tmux kill-session -t newton_sim 2>/dev/null || true
pkill -f 'standalone_fee_terra_newton_env|standalone_dig_newton_env|native_fee_terra_default_viewer.py|newton_bridge.launch.py|robot.launch.py|mole_state_publisher|mole_perception_bringup|dig_3d_controller_cpp.launch.py|compare_dig3d_live_obs.py|foxglove_bridge' || true
sleep 2
pkill -9 -f 'standalone_fee_terra_newton_env|standalone_dig_newton_env|native_fee_terra_default_viewer.py|newton_bridge.launch.py|robot.launch.py|mole_state_publisher|mole_perception_bringup|dig_3d_controller_cpp.launch.py|compare_dig3d_live_obs.py|foxglove_bridge' || true
```

Re-check resources before the next launch:

```bash
free -h
nvidia-smi --query-gpu=index,name,memory.used,memory.total,utilization.gpu --format=csv,noheader,nounits
ps -eo pid,ppid,%mem,%cpu,etime,cmd | rg 'standalone_dig_newton_env|newton_bridge|robot.launch.py|mole_state_publisher|mole_perception_bringup|excavation_mapping|dig_3d_controller|rviz|ros2 launch'
```
