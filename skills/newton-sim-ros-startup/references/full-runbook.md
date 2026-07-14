---
name: newton-sim-ros-startup
description: Start or restart single-container Newton and Moleworks ROS stacks for bridge, Nav2, perception, Dig3D, Terra, and checkpoint recovery.
---

# Newton Sim ROS Startup — Full Runbook

## Contents

- Use and non-negotiables
- Shell identity and attach fallback
- Runtime shell preflight
- tmux layouts and fast Nav2 loop
- Launch windows: Newton, robot, state publisher, perception, planner, OCS2, single workspace, Dig3D, Foxglove, Ackermann, Nav2, benchmark
- Sanity checks and Dig deployment order
- Failure bundle capture and checkpoint resume
- Teardown

Read only the sections selected by `SKILL.md`; the core safety contract remains in `SKILL.md`.

## Use This Skill For

Use this only for the single-container Newton workflow inside `moleworks_ros:latest`.

If the stack is split across Isaac/Terra and ROS containers, use `sim-startup` or `moleworks-terra-stack` instead.

For standardized post-bringup Nav2 validation in Newton, use the fast Nav2 validation layout in this skill.

For packaged flat-foundation Terra execution (`flat_foundation`, `flat_foundation_depth_0p5`, full multi-waypoint
foundation plans, or post-dump stall measurement), use `terra-foundation-execution` after this startup preflight. Keep
this skill focused on runtime bringup, process hygiene, failure capture, and checkpoint resume mechanics.

For workspace-planner behavior debugging, per-action planner GridMaps, predicted-vs-executed scoop analysis, or replaying
failed/high-discrepancy scoops from Terra checkpoints, also load the `workspace-planner-debug` skill. This skill handles
the Newton runtime and restart discipline; `workspace-planner-debug` handles the planner-specific artifact and replay loop.

## Non-Negotiables

- Assume the current shell is already inside the target `moleworks_ros` runtime unless the user explicitly says otherwise or direct checks prove you are on the host.
- If you are already inside the container, stay in that shell and use container-local tmux as the shared control plane. Do not detach and re-attach through Docker just to normalize the workflow.
- Only use `docker_attach.sh` when the user explicitly wants a host-to-container attach flow or when direct checks prove you are on the host shell.
- Do not use `docker exec` for normal interactive bringup.
- If the Docker CLI is unavailable in the current shell but `/workspace/moleworks/ros2_ws` and the Newton worktree are already mounted locally, treat the current shell as the active runtime shell and continue with the documented fallback-style preflight below instead of stalling on container discovery.
- Keep the container default Fast DDS setup.
- Do not export `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` or `CYCLONEDDS_URI` for this workflow.
- Use `ROS_DOMAIN_ID=24` unless the user explicitly asks for a different domain.
- If the user asks for an isolated side stack, use one tmux session per ROS domain instead of mixing domains in one session.
- On this Fast DDS setup, keep `ROS_DOMAIN_ID <= 232`. Domain `333` is invalid here.
- Use `/workspace/moleworks/ros2_ws` as the canonical in-container ROS workspace path.
- Source `/workspace/moleworks/ros2_ws/install/setup.bash` as the single ROS entrypoint. Do not stack `/opt/ros/jazzy/setup.bash` plus `install/local_setup.bash` in this workflow.
- Build ROS packages from `/workspace/moleworks/ros2_ws`, not from `/workspace/moleworks/ros2_ws/src/moleworks_ros`.
  A nested `src/moleworks_ros/install` can pass local tests but will not be used by the normal Newton/Terra stage panes.
- Before any `colcon build`, make sure `cmake` resolves to the system binary, not Lorenzo's stale user wrapper.
  If `colcon` fails with `/home/lorenzo/.local/bin/cmake` and `ModuleNotFoundError: No module named 'cmake'`,
  rerun with `/usr/bin` ahead of `~/.local/bin` on `PATH`; this is an environment issue, not a package failure.
- Before rebuilding, check `ros2 pkg prefix terra_planner` against the expected runtime prefix and then rebuild only the
  smallest affected package set. For pure Python, launch, YAML, or symlink-installed script changes, skip the rebuild
  and restart the affected tmux pane.
- Export `MW_EXPECTED_ROS_PREFIX=/workspace/moleworks/ros2_ws/install` in tmux panes that launch Terra stages. Stage
  launch files use this as a preflight guard against stale or nested ROS overlays.
- If `install/setup.bash` warns about missing Nav2 package prefixes, the workspace install is stale. Remove the stale install prefixes and re-source `install/setup.bash` before debugging controller behavior.
- Default to headless Newton for stack bringup and automated single-workspace testing. Only add `--gui` or `gui:=true` when the user explicitly wants visual inspection.
- If the user asks to load a map, load the same terrain artifact on both sides:
  - Newton soil via the existing `--elevation-map` support in `standalone_fee_terra_newton_env.py`
  - ROS excavation mapping via `design_bag_path`
- Premade STL target geometry is a ROS-side post-perception load into excavation mapping. Bring up
  `mole_perception_bringup` first, then use `mesh_to_excavation_grid_map.py preview` and `apply`.
  For trench authoring, use `--authoring-frame CABIN_CONTROL --mesh-anchor-x max --mesh-x <distance>`;
  do not apply trench geometry in `BASE`.
- For the Hong no-holes `fee_terra` workflow, use the default floating-base reset pose
  `x=-0.001212`, `y=0.299973`, `yaw=180 deg` unless the user explicitly asks for a different pose.
  Do not reset this map to `yaw=0 deg` for arm/planner tests: that places the excavator arm over the
  local obstacle side of the Hong no-holes surface.
- The pose-reset order matters on Hong no-holes: if you are also applying a Newton runtime profile, apply the
  runtime profile first and only then call `/mole/reset_robot_pose`, because the runtime-profile reset can
  overwrite the floating-base orientation.
- The terrain seed order is strict: restart Newton sim with the requested map, or load terrain on the Newton side at runtime, before launching robot/perception/dig.
- Do not try to reseed Newton terrain after the rest of the ROS stack is already live. If the Newton soil seed is wrong, restart the split stack in the right order instead of patching it mid-run.
- If you stop or restart the Newton sim process after ROS-side nodes are already running, treat the whole ROS stack as contaminated by a sim-time jump. Do not try to recover by restarting only `ocs2`, `dig`, or other helper panes in place.
- After a Newton restart in the split stack, restart the full ROS-side stack against the new clock: `robot`, `state_pub` if present, `perception`, `planner`, `ocs2`, `dig`, `executor`, and `foxglove` if it is part of the run.
- Source ROS once per tmux window, not before every single command.
- In tmux, run one long-lived stack process in the foreground per pane. Do not use `nohup` for shared bringup panes.
- Before reusing a pane, stop the current process with `Ctrl-C` and verify the old process tree is gone.
- Before every restart, check RAM, VRAM, and stale ROS/Newton processes. Do not stack a new sim on top of leftovers.
- For Nav2 or any split sim driving workflow, do not add deleted arm/wheel/turn hold-controller nodes back into the stack. The current Newton stale-command semantics should keep arm, turn, and steering stable without them.
- For the integrated `single_workspace` workflow, there should be no hold-controller nodes. The BT owns the arm-MPC/dig handoff directly.
- For Terra BT/controller development, prefer a split dev layout with separate `planner`, `ocs2`, `dig`, and `executor`
  windows. Keep the integrated `single_workspace.launch.py` path for end-to-end validation, not the main debug loop.
- In the split Terra dev loop, apply workspace geometry once up front with `apply_workspace.py`, then restart only the
  failing owner. If the BT fails but the rest of the stack is healthy, first try `/mole/terra_executor/restart`.
- Before tearing down or restarting after a failure, capture a failure-state bundle unless the user explicitly says to
  skip it. The bundle should copy the latest pre-action Terra checkpoint for retry, plus tmux panes, process state,
  ROS graph/state snapshots, and a diagnostic current excavation-map snapshot when `/excavation_mapping/save_map` is
  available.
- For Newton simulation retries, restore from a saved Terra checkpoint with
  `src/moleworks_ros/scripts/resume_from_checkpoint.py ... --on_machine false` after the replacement stack is up. Do not
  use the real-machine mode in Newton sim.
- For workspace-planner debugging, treat Terra checkpoints as the canonical way to replay exactly the failed scoop or a
  high predicted-vs-executed discrepancy scoop. Every `Saved checkpoint: .../<pair>_<completed_loop>` log line before
  `move_to_dig` is the pre-action state for the next scoop. Preserve the matching checkpoint path in the failure bundle
  and action-debug notes. If action `N` times out, fails, or has poor execution feedback, resume from the checkpoint whose
  completed loop index is `N-1` (for the first scoop this is usually `.../1_0/checkpoint.yaml`). After starting a clean
  Newton + ROS stack and the action-debug recorder, run:
  `python3 /workspace/moleworks/ros2_ws/src/moleworks_ros/scripts/resume_from_checkpoint.py <checkpoint> --on-machine false`.
  Then trigger Terra through `/mole/terra_executor/resume` or the existing executor workflow, keeping the same planner and
  policy parameters unless the test is intentionally comparing a changed parameter.
- When diagnosing high discrepancy, join planner action `N` to execution feedback reported on the next planner compute
  (`attempt_count=N+1`) or on a terminal status with `attempt_count=N`. Do not compare the previous action's
  `global_removed_m3` against the newly selected action's predicted volume without this attempt shift.
- For Terra controller-debug runs, start a live `/controller_status` recorder with `--full-length` before the action
  starts. Failure-state capture after the BT has stopped is useful for replay, but it cannot recover per-tick
  termination booleans if the status topic has gone idle.
- Exception: if the failing owner is `newton` or if the Newton process was restarted for any reason, do not try to restart
  only the downstream ROS panes. Restart the full ROS-side stack as well, because OCS2 helper nodes and TF consumers can
  retain old sim-time assumptions and degrade into `TF_OLD_DATA`, stale plans, or expired-policy SAFE_STOP.
- Before sending Nav2 or split-stack goals, verify `/mole/actuator_commands` and `/mole/cmd_vel_smoothed` do not have duplicate publishers. In integrated `single_workspace`, seeing both `mole_arm_mpc_controller` and `dig_3d_controller` as `/mole/actuator_commands` publisher endpoints is normal; extra hold/drive publishers are the problem.
- In the current local Newton `fee_terra` workflow, TF is exposed on the global `/tf` and `/tf_static`
  topics even when `robot_namespace:=mole` is set. Do not assume `/mole/tf` and `/mole/tf_static`
  exist in this stack. Any ad-hoc `TransformListener`, `tf2_echo`, or probe node should use the default
  global TF topics unless you have explicitly verified a namespaced TF transport on the branch you are running.
- Do not substitute ad-hoc viewer scripts for `standalone_fee_terra_newton_env.py` or `standalone_dig_newton_env.py` when the user expects ROS bridge, pointcloud, or perception parity.

## 0) Shell Identity And Attach Fallback

First determine whether the current shell is already the target runtime shell. Prefer this before any Docker CLI step:

```bash
test -f /.dockerenv && echo IN_DOCKERENV || echo NO_DOCKERENV
test -f /workspace/moleworks/ros2_ws/install/setup.bash && echo HAS_ROS_WS || echo NO_ROS_WS
test -d /home/lorenzo/moleworks/moleworks_newton && echo HAS_NEWTON_WORKTREE || echo NO_NEWTON_WORKTREE
which docker || true
```

If `/.dockerenv` exists and `/workspace/moleworks/ros2_ws/install/setup.bash` is present, assume you are already in the correct container and skip Docker discovery/attach.

Only if those checks fail, or if the user explicitly says you are starting from the host shell, inspect the running container and host resources:

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

Attach the normal way only in that host-shell case:

```bash
cd /home/lorenzo/moleworks/ros2_ws/src/moleworks_ros/docker
./docker_attach.sh --name moleworks_ros --user lorenzo
```

If `lorenzo` is not present in that container, retry with `--user root`.

If `docker` is not available in the current shell, but the machine already has the mounted ROS workspace and Newton worktree, treat the current shell as the active runtime shell and continue immediately:

```bash
test -d /workspace/moleworks/ros2_ws
test -d /home/lorenzo/moleworks/moleworks_newton
source /workspace/moleworks/ros2_ws/install/setup.bash
source /home/lorenzo/moleworks/moleworks_newton/.venv/bin/activate
```

For this fallback-style path, keep using the same tmux layout, `ROS_DOMAIN_ID`, and Foxglove ports described below. The only difference is the shell you launch them from.

## 1) Runtime Shell Preflight

In the active runtime shell (already in-container by default, or after an explicit attach/fallback decision):

```bash
export ROS_DOMAIN_ID=24
source /workspace/moleworks/ros2_ws/install/setup.bash
export MW_ROS_WS=/workspace/moleworks/ros2_ws
export MW_EXPECTED_ROS_PREFIX=/workspace/moleworks/ros2_ws/install
export MW_LOCAL_SURFACE_PKG=package://mole_maps/maps/hong0326_no_holes/hong0326_no_holes_surface
export MW_LOCAL_SURFACE_MCAP=/workspace/moleworks/ros2_ws/src/moleworks_maps/maps/hong0326_no_holes/hong0326_no_holes_surface/hong0326_no_holes_surface_0.mcap
export MW_LOCAL_WORKSPACE_CONFIG=/workspace/moleworks/ros2_ws/install/workspace_planner/share/workspace_planner/config/canonical_dig_dump_workspace.yaml
export MW_LOCAL_SURFACE_DEFAULT_X_M=-0.001212
export MW_LOCAL_SURFACE_DEFAULT_Y_M=0.299973
export MW_LOCAL_SURFACE_DEFAULT_YAW_DEG=180.0
```

Overlay guard for this runtime shell:

```bash
export ROS_DOMAIN_ID=24
source /workspace/moleworks/ros2_ws/install/setup.bash
export MW_ROS_WS=/workspace/moleworks/ros2_ws
export MW_EXPECTED_ROS_PREFIX=/workspace/moleworks/ros2_ws/install
export MW_LOCAL_WORKSPACE_CONFIG=/workspace/moleworks/ros2_ws/install/workspace_planner/share/workspace_planner/config/canonical_dig_dump_workspace.yaml
export MW_LOCAL_SURFACE_DEFAULT_X_M=-0.001212
export MW_LOCAL_SURFACE_DEFAULT_Y_M=0.299973
export MW_LOCAL_SURFACE_DEFAULT_YAW_DEG=180.0
readlink -f "$MW_ROS_WS/install/setup.bash"
ros2 pkg prefix mole_msgs
ros2 pkg prefix mole_ocs2_arm_controller
ros2 pkg prefix terra_planner
test "$(ros2 pkg prefix terra_planner)" = "$MW_EXPECTED_ROS_PREFIX/terra_planner"
```

Expected result:
- `readlink -f` resolves to `/workspace/moleworks/ros2_ws/install/setup.bash`
- `ros2 pkg prefix mole_msgs` resolves under `/workspace/moleworks/ros2_ws/install`
- `ros2 pkg prefix mole_ocs2_arm_controller` resolves under `/workspace/moleworks/ros2_ws/install`
- `ros2 pkg prefix terra_planner` resolves to `$MW_EXPECTED_ROS_PREFIX/terra_planner`

Packaged flat-foundation Terra plan fixtures and end-to-end execution checks now live in
`terra-foundation-execution`. Use that skill after this runtime preflight when the task is to run
`flat_foundation_depth_0p5` or another packaged foundation plan.

If a fresh attached shell prints `Sourcing ROS2 workspace at /home/lorenzo/ros2_ws`, do **not** trust that
as the live overlay for this workflow. That is a shell convenience default, not the validated Newton/Terra
overlay. Immediately re-source `/workspace/moleworks/ros2_ws/install/setup.bash` and re-run the checks above
before any `ros2 launch`, `ros2 run`, or service call. If `ros2 pkg prefix mole_ocs2_arm_controller` fails
or points somewhere else, stop and fix the overlay first.

Run Newton branch commands from the local `moleworks_newton` checkout. Keep the sibling
`newton` checkout present at `~/moleworks/newton`, because `moleworks_newton` resolves
editable `tool.uv.sources` from `../newton`. Example:

```bash
cd /home/lorenzo/moleworks/moleworks_newton
export MW_NEWTON_ROOT=$PWD
test -d /home/lorenzo/moleworks/newton
```

For Newton standalone ROS bridge scripts from the worktree, also activate the worktree venv after sourcing the ROS overlay:

```bash
source /workspace/moleworks/ros2_ws/install/setup.bash
source "$MW_NEWTON_ROOT/.venv/bin/activate"
```

If Newton reports `ModuleNotFoundError: No module named 'mcap'` or similar MCAP support failures,
repair the worktree venv before retrying:

```bash
sudo chown -R lorenzo:lorenzo "$MW_NEWTON_ROOT/.venv" 2>/dev/null || true
rm -rf "$MW_NEWTON_ROOT/.venv"
(cd "$MW_NEWTON_ROOT" && uv sync)
```

If the user explicitly asks for a different ROS domain, replace `24` consistently in every pane and in every
launch command. Do not mix domains inside one tmux session.

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

Quick TF health check for the namespaced Moleworks stack:

```bash
timeout 15 bash -lc 'export ROS_DOMAIN_ID=24; source /workspace/moleworks/ros2_ws/install/setup.bash; ros2 run tf2_ros tf2_echo map BASE_GRAV --ros-args -r __ns:=/mole 2>&1' | head -20
```

If you launch your own Python probe, default to the global TF topics for this workflow. Only force a
`namespace="mole"` TF subscription after you have verified that the running branch actually publishes
`/mole/tf` and `/mole/tf_static`.

If GUI apps fail to appear, also verify the host Xauth file exists:

```bash
ls -la /tmp/.docker.xauth
```

Clean up stale session state:

```bash
tmux kill-session -t newton_sim 2>/dev/null || true
self=$$
mw_stack_patterns=(
  'standalone_fee_terra_newton_env.py'
  'standalone_dig_newton_env.py'
  'native_fee_terra_default_viewer.py'
  'newton_bridge.launch.py'
  'robot.launch.py'
  'mole_state_publisher.launch.py'
  'mole_perception_bringup'
  'dig_3d_controller_cpp.launch.py'
  'compare_dig3d_live_obs.py'
  'foxglove_bridge'
  'ackermann_drive_controller_node'
  'controller_server'
  'planner_server'
  'behavior_server'
  'bt_navigator'
  'velocity_smoother'
  'collision_monitor'
  'waypoint_follower'
  'opennav_docking'
  'lifecycle_manager_navigation'
  'odom_nav2_adapter'
  'dynamic_footprint_publisher'
  'robot_state_publisher/robot_state_publisher'
  'mole_joint_state_publisher_node'
  'elevation_mapping_node.py'
)
for pat in "${mw_stack_patterns[@]}"; do
  for pid in $(pgrep -f "$pat" || true); do
    [ "$pid" = "$self" ] && continue
    kill "$pid" 2>/dev/null || true
  done
done
sleep 2
for pat in "${mw_stack_patterns[@]}"; do
  for pid in $(pgrep -f "$pat" || true); do
    [ "$pid" = "$self" ] && continue
    kill -9 "$pid" 2>/dev/null || true
  done
done
```

Do not use bare `pkill -f` against a large regex inside a shared shell unless you are sure it will not match
the shell that is running the cleanup command.

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

For the minimal Newton stack, unexpected publishers such as `dig_3d_controller` or a stale planner/executor mean the domain is still contaminated.
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

For fast Newton + Nav2 validation, prefer this narrower layout instead of the full perception/dig layout:

```bash
tmux new-session -d -s newton_sim -n newton
tmux new-window -t newton_sim -n robot
tmux new-window -t newton_sim -n state_pub
tmux new-window -t newton_sim -n ackermann
tmux new-window -t newton_sim -n nav2
tmux new-window -t newton_sim -n foxglove
tmux new-window -t newton_sim -n benchmark
tmux new-window -t newton_sim -n debug
tmux list-windows -t newton_sim
```

Use this profile when the goal is:

- Foxglove visibility
- Ackermann drive validation
- Nav2 path following
- the side-target tracking benchmark

For the integrated Terra single-workspace workflow, use the `single_ws` window instead of the separate
`robot`, `state_pub`, `perception`, and `dig` windows.

For the integrated Terra single-workspace workflow, prefer the minimal layout instead of the full split stack:

```bash
tmux new-session -d -s newton_${ROS_DOMAIN_ID} -n newton
tmux new-window -t newton_${ROS_DOMAIN_ID} -n stack
tmux new-window -t newton_${ROS_DOMAIN_ID} -n debug
tmux new-window -t newton_${ROS_DOMAIN_ID} -n foxglove
tmux list-windows -t newton_${ROS_DOMAIN_ID}
```

Use the default tmux socket. Do not hide this run behind a custom socket unless the user explicitly asks for isolation.

For Terra BT/controller development, prefer the split layout instead of the integrated `single_ws` window:

```bash
tmux new-session -d -s newton_sim -n newton
tmux new-window -t newton_sim -n robot
tmux new-window -t newton_sim -n perception
tmux new-window -t newton_sim -n planner
tmux new-window -t newton_sim -n ocs2
tmux new-window -t newton_sim -n dig
tmux new-window -t newton_sim -n executor
tmux new-window -t newton_sim -n foxglove
tmux new-window -t newton_sim -n debug
tmux list-windows -t newton_sim
```

Use this profile when you want to:

- keep Newton/perception/controller state alive while iterating on the BT
- restart only `workspace_planner`, OCS2, Dig3D, or the Terra executor
- use `/mole/terra_executor/restart` as the first recovery step after BT failure

Recommended shell prologue for each window:

```bash
export ROS_DOMAIN_ID=24
source /workspace/moleworks/ros2_ws/install/setup.bash
export MW_ROS_WS=/workspace/moleworks/ros2_ws
readlink -f "$MW_ROS_WS/install/setup.bash"
```

If you start a window with a one-shot `tmux new-window '...'` command instead of opening
an interactive shell first, inline the same `source` sequence inside that quoted command.
Do not assume the standalone Newton script can import ROS Python packages otherwise.
For long shared-session relaunches, prefer the two-step pattern:
- `tmux new-window -t <session> -n <name>`
- then `tmux send-keys -t <session>:<name> ...`

This is slower to type but more reliable than a single huge quoted `tmux new-window '...'`
command when you are recovering any long launch pane under time pressure.

General tmux recovery rules for this skill:
- Before `tmux send-keys -t <session>:<name> ...`, verify the target window exists with `tmux list-windows -t <session>`.
- If `tmux capture-pane` or `tmux send-keys` reports `can't find window`, stop and recreate the window first. Do not keep sending commands to a dead pane.
- When a relaunch matters more than terseness, optimize for inspectability rather than command golf: create the window, send one command per line, then inspect the pane.
- Prefer killing and recreating one bad launch pane over trying to salvage a half-started pane with mixed old and new commands in its scrollback.

### Fast Nav2 Loop

When the task is only Newton + Ackermann + Nav2 validation, stay on the narrow `newton + robot + state_pub + ackermann + nav2 + foxglove + debug` layout. Do not launch perception, Dig3D, or `single_workspace` unless the user explicitly needs them.

If only Nav2 / Ackermann packages changed, rebuild the smallest useful set first:

```bash
which -a cmake
export PATH=/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
hash -r
cmake --version
colcon build --symlink-install --packages-select \
  mole_msgs \
  mole_highlevel_msgs \
  mole_nav2_utils \
  mole_nav2_bringup \
  mole_highlevel_controller_cpp
```

If only Python or YAML files changed under `mole_bringup`, `mole_nav2_bringup`, or the Newton bridge, skip the rebuild and restart only the affected tmux windows. Avoid broad `--packages-up-to` rebuilds for this loop; unrelated packages such as `ocs2_robotic_assets` can fail and waste the iteration.

For fast headless Nav2 bringup, append `--disable-lidar-publisher` to `standalone_fee_terra_newton_env.py` unless the user explicitly needs the soil point cloud. The point cloud is not needed for base-nav smoke tests and this avoids the known headless crash:

```text
RuntimeError: Soil point cloud sampled no points inside imported terrain bounds.
```

Use this validation order:

1. `timeout 15 ros2 topic echo /mole/state --once`
2. `ros2 action list | rg "/mole/(navigate_to_pose|follow_path)"`
3. `ros2 topic info /mole/actuator_commands -v`
4. `ros2 topic info /mole/cmd_vel_smoothed -v`
5. straight goal smoke test
6. mild curved goal smoke test
7. `nav2_side_target_benchmark.py`

If a curved or lateral run leaves Newton in repeated `numerical_failure` resets, restart only that `newton_sim_<domain>` session before the next case. Do not keep sending goals into a reset-churning bridge.

## 3) Launch Windows

Run the main stack in the foreground in each tmux pane. One pane should own one long-lived process.
Do not use `nohup` inside these shared bringup panes. That is how duplicate launch trees survive pane restarts
and later fight each other on `/tf`, `/clock`, `/mole/actuator_commands`, and Nav2 topics.

### newton

For the local `fee_terra` single-workspace workflow, default to the dedicated wrapper and preload the
Hong no-holes surface into Newton itself. This is the Newton-side half of the dual map-load contract:

```bash
cd "$MW_NEWTON_ROOT"
python scripts/ros/standalone_fee_terra_newton_env.py \
  --elevation-map "$MW_LOCAL_SURFACE_MCAP" \
  --elevation-layer elevation \
  --max-depth-layer desired_elevation

# For fast headless Nav2 validation, add --disable-lidar-publisher.
```

Add `--gui` only when the user explicitly wants to inspect Newton visually.

If the command is launched directly from `tmux new-window '...'`, write it like this instead:

```bash
tmux new-window -t newton_sim -n newton "bash -lc '
  export ROS_DOMAIN_ID=24
  source /workspace/moleworks/ros2_ws/install/setup.bash
  source "$MW_NEWTON_ROOT/.venv/bin/activate"
  export MW_LOCAL_SURFACE_MCAP=/workspace/moleworks/ros2_ws/src/moleworks_maps/maps/hong0326_no_holes/hong0326_no_holes_surface/hong0326_no_holes_surface_0.mcap
  cd \"$MW_NEWTON_ROOT\"
  exec python scripts/ros/standalone_fee_terra_newton_env.py \
    --elevation-map "$MW_LOCAL_SURFACE_MCAP" \
    --elevation-layer elevation \
    --max-depth-layer desired_elevation
'"
```

That exact failure mode shows up as `ModuleNotFoundError: No module named 'mole_msgs'`.

For non-`fee_terra` tasks, fall back to `standalone_dig_newton_env.py` and pass the requested
`--elevation-map` explicitly whenever the user asks for a preloaded terrain.

If the branch is using the ROS launch wrapper instead, use:

```bash
ros2 launch mole_bringup newton_bridge.launch.py \
  use_sim_time:=true \
  gui:=true \
  publish_tf:=false
```

### robot

```bash
ros2 launch mole_bringup robot.launch.py \
  use_sim_time:=true \
  on_machine:=false \
  launch_low_level:=false \
  launch_perception:=false \
  launch_rviz:=false \
  launch_foxglove:=false \
  robot_namespace:=mole
```

For this skill, rely on the native Newton stale-command freeze semantics for uncommanded arm, turn, and steering joints. Do not add deleted helper nodes back into the sim-nav stack.

For the bridge-only layout, use `robot` as the model/TF sidecar instead of `robot.launch.py`:

```bash
ros2 launch mole_joint_state_publisher mole_state_publisher.launch.py \
  use_sim_time:=true \
  namespace:=mole \
  publish_frequency:=25.0
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
ros2 launch mole_joint_state_publisher mole_state_publisher.launch.py \
  use_sim_time:=true
```

### perception

For Dig3D parity in Newton sim, use the same local perception contract as the machine-facing stack:

- `mapping_profile:=local`
- `enable_robot_self_filter:=true`
- let `mole_perception_bringup` keep its default local runtime choices instead of overriding them by hand
- this means elevation mapping consumes the canonical filtered cloud when the self-filter is healthy
- excavation mapping consumes the default local upstream layer `inpaint` from `elevation_map_filter`, not `smooth`

```bash
ros2 launch mole_perception_bringup bringup.launch.py \
  use_sim_time:=true \
  on_machine:=false \
  mapping_profile:=local \
  enable_camera:=false \
  enable_lidar:=false \
  enable_elevation_mapping:=true \
  enable_excavation_mapping:=true \
  enable_robot_self_filter:=true \
  design_bag_path:=$MW_LOCAL_SURFACE_MCAP
```

If the current image has a broken `mole_pointcloud_filter` runtime (for example a Fast-CDR symbol lookup error),
do not switch excavation mapping to `smooth` as a workaround. Keep `mapping_profile:=local`, keep the same
`design_bag_path`, and use a temporary fallback with:

```bash
enable_robot_self_filter:=false
```

That preserves the correct local mapping contract and the despiked/inpainted excavation-map handoff, even if the
filtered-cloud producer is temporarily unavailable in this image.

#### Premade STL Target Geometry

Use this after the `perception` window is healthy when the user asks to load a premade trench,
beam, or other STL target into excavation mapping. The mesh load is local-mode ROS target geometry;
it does not replace the Newton soil surface. Keep Newton seeded with the same base surface map via
`--elevation-map`, then apply the STL target to `/excavation_mapping/grid_map`.

First preview the placement. For cabin-control trench/beam authoring, `CABIN_CONTROL +X` at
the live cabin yaw is the intended trench major axis:

```bash
GEOM="$(ros2 pkg prefix mole_excavation_mapping)/share/mole_excavation_mapping/geometries/beam_segment_2m/beam_segment_2m_40cmDepth.stl"

ros2 run mole_excavation_mapping mesh_to_excavation_grid_map.py preview \
  "$GEOM" \
  --output /tmp/target_preview.svg \
  --authoring-frame CABIN_CONTROL \
  --mesh-anchor-x max \
  --mesh-x 7.0 \
  --mesh-y 0.0 \
  --mesh-anchor-y origin \
  --align-major-axis x
```

Then apply the same placement to the live map:

```bash
ros2 run mole_excavation_mapping mesh_to_excavation_grid_map.py apply \
  "$GEOM" \
  --authoring-frame CABIN_CONTROL \
  --mesh-anchor-x max \
  --mesh-x 7.0 \
  --mesh-y 0.0 \
  --mesh-anchor-y origin \
  --align-major-axis x \
  --reference-mode local_min \
  --mesh-reference-z max \
  --map-topic /excavation_mapping/grid_map \
  --load-service /excavation_mapping/load_excavation_map \
  --dry-run-output /tmp/target_applied \
  --timeout-sec 60 \
  --tf-timeout-sec 30 \
  --force
```

Rules for this mesh workflow:

- Use `--align-major-axis x` for beam/trench STLs whose long axis should match the arm pull direction.
- Use `--authoring-frame CABIN_CONTROL --mesh-anchor-x max --mesh-x 7.0` when the farthest `+X` point
  of the imported target should land 7 m from the live cabin-control origin.
- Use `--mesh-y 0.0` to keep the target centerline on the cabin-control strip centerline.
- Use `--reference-mode local_min --mesh-reference-z max` for local trench-like cut volumes so the target
  is anchored to one stable footprint height instead of following local soil noise.
- The importer writes a `runtime_target` marker layer; local excavation mapping should log
  `load_excavation_map applied (... runtime_target=true)`.
- If Foxglove still shows the old target after a successful apply, restart the `perception` pane and reapply
  the STL. Do not restart only OCS2 or Dig3D to fix a stale target map.

For a quick ROS readback, inspect the loaded `dig_zone` bounds in `/excavation_mapping/grid_map`. With the
7 m farthest-point example, the continuous target max-X is 7.0 m; the last occupied cell center will be
slightly inside that value because of grid resolution.

### planner

In the split Terra dev loop, start the workspace planner server in its own window:

```bash
ros2 launch workspace_planner single_workspace.launch.py \
  use_sim_time:=true \
  robot_namespace:=mole \
  frame_id:=map
```

Apply the workspace geometry once before starting the executor, and re-run it only when the workspace config changes:

```bash
ros2 run mole_bringup apply_workspace.py --ros-args \
  -p use_sim_time:=true \
  -p timeout_sec:=60.0 \
  -p apply_only:=true \
  -p tf_prefix:= \
  -p runtime_apply_spec_path:=$MW_LOCAL_WORKSPACE_CONFIG \
  -p apply_service_name:=/excavation_mapping/apply_runtime_profile \
  -p apply_zones_service_name:=/excavation_mapping/apply_runtime_zones \
  -p grid_map_topic:=/excavation_mapping/grid_map
```

### ocs2

In the split Terra dev loop, mirror the integrated `single_workspace` OCS2 wiring:

```bash
ros2 launch mole_ocs2_arm_controller ocs2_arm.launch.py \
  use_sim_time:=true \
  robot_namespace:=mole \
  auto_handover:=false \
  taskProfile:=real_collisions \
  elevation_map_topic:=/excavation_mapping/grid_map \
  elevation_map_layer:=elevation \
  command_lag_comp_sec:=0.0 \
  delay_enable:=true \
  delay_command_prefilter_enable:=true \
  launch_move_leg:=true \
  launch_dump_leg:=true \
  launch_dump_scheduler:=true \
  launch_policy_visualizer:=true \
  bootstrap_auto_hold_on_configure:=false \
  turn_servo_enable:=true \
  boom_servo_enable:=true \
  stick_servo_enable:=true \
  tele_servo_enable:=true \
  pitch_servo_enable:=true
```

Starship-specific runtime rule for the split OCS2 loop:
- On `starship` / the local PC, pin both the MPC node and the arm controller to P-cores `8-11`, not `22-23`. The `22-23` convention is for the robot machine and maps to the wrong cores on Starship.
- Pass `mpc_cpu_affinity:=8-11 arm_cpu_affinity:=8-11` in the launch command when you are on Starship.
- After launch, verify the effective affinity with `taskset -pc <pid>`. If the launch helper still applied `22-23`, repin live to `8-11`.
- Reapply `SCHED_FIFO/99` to the non-DDS threads after launch. Do not assume the internal realtime request succeeded; check with `ps -L -p <pid> -o pid,tid,cls,rtprio,pri,psr,comm`.

Example live correction on Starship:

```bash
MPC_PID=$(pgrep -f '(^|/)mobile_manipulator_mpc_node($| )' | head -n1)
ARM_PID=$(pgrep -f '(^|/)mole_arm_mpc_controller($| )' | head -n1)
taskset -pc 8-11 "$MPC_PID"
taskset -pc 8-11 "$ARM_PID"
for pid in "$MPC_PID" "$ARM_PID"; do
  ps -L -p "$pid" -o tid=,comm= | awk '$2 !~ /^dds/ {print $1}' | while read -r tid; do
    sudo -n chrt -f -p 99 "$tid"
  done
done
```

### single_ws

For the full local Terra single-workspace workflow, prefer one integrated launch over the split
`robot` + `state_pub` + `perception` + `dig` bringup. This launch manages OCS2 arm MPC, Dig3D,
workspace planning, and the Terra executor in one place. There should be no hold-controller nodes
in this workflow.

Use the canonical bundled local workspace preset unless the user explicitly asks for a different
testing geometry:
- `workspace_planner/config/canonical_dig_dump_workspace.yaml`
- integrated bringup arg: `workspace_config_name:=canonical_dig_dump_workspace.yaml`

Use the Hong no-holes local seed on both sides:
- Newton: `--elevation-map "$MW_LOCAL_SURFACE_MCAP"`
- ROS: `design_bag_path:=$MW_LOCAL_SURFACE_MCAP`

For the default Hong no-holes single-workspace start pose, reset the base to `yaw=180 deg` after any Newton-side
runtime-profile apply and before starting the integrated ROS stack:

This is not just a convention: on the Hong no-holes surface, `yaw=0 deg` puts the arm on the obstacle side.
Use `yaw=180 deg` for single-workspace fan/dig/dump tests unless the test specifically needs the obstacle-side
orientation.

```bash
ros2 service call /mole/reset_robot_pose mole_msgs/srv/ResetRobotPose "{
  x_m: $MW_LOCAL_SURFACE_DEFAULT_X_M,
  y_m: $MW_LOCAL_SURFACE_DEFAULT_Y_M,
  yaw_deg: $MW_LOCAL_SURFACE_DEFAULT_YAW_DEG
}"
```

Do not do this reset before `/mole/newton/apply_runtime_profile` when the Hong no-holes workspace target is also
being authored, because the runtime-profile reset can put the bucket back on top of the obstacle.

Do not start `single_workspace.launch.py` against a Newton viewer that was not launched with the same
surface artifact. That creates exactly the perception mismatch where lidar sees one terrain and ROS
excavation mapping is seeded from another.

If you are not intentionally testing a separate planner overlay, do not source an extra temporary workspace
before this launch. The default `moleworks_ros` install is the expected overlay. Only source an additional
workspace when the user explicitly wants that planner build.

#### Local Workspace Planner Bootstrap

When the user explicitly wants the local workspace-planner flow on the Hong no-holes terrain, use the
surface-only seed bag and the canonical local fan workspace. This is the current integrated bootstrap:

```bash
bash -lc 'source /workspace/moleworks/ros2_ws/install/setup.bash && \
  ros2 launch mole_bringup single_workspace.launch.py \
  use_sim_time:=true \
  on_machine:=false \
  launch_rviz:=false \
  robot_namespace:=mole \
  workspace_config_name:=canonical_dig_dump_workspace.yaml \
  design_map_name:=none \
  design_bag_path:=$MW_LOCAL_SURFACE_MCAP \
  workspace_planner_timeout_sec:=60.0 \
  ocs2_task_profile:=real_collisions \
  enable_robot_self_filter:=true'
```

Why this exact contract:

- `canonical_dig_dump_workspace.yaml` is the current local dig/dump fan preset used by the workspace planner.
- `design_bag_path:=$MW_LOCAL_SURFACE_MCAP` seeds excavation mapping with the Hong surface-only bag so
  `/excavation_mapping/apply_runtime_profile` can succeed before Terra starts.
- `design_map_name:=none` avoids looking for a nonexistent `hong0326_no_holes_design` artifact; the explicit
  bag path is the intended local bootstrap.
- `enable_robot_self_filter:=true` keeps elevation mapping on the filtered-cloud local contract and keeps
  excavation mapping on the final processed `inpaint` layer.

Recommended run order for single-workspace:

1. Start Newton headless with the requested `--elevation-map`.
2. Wait for `/mole/reset_robot_pose`.
3. Reset the Hong no-holes floating base to `yaw=180 deg`.
4. Launch `single_workspace.launch.py` with the same `design_bag_path`.
5. Verify there are no hold-controller nodes:

```bash
ros2 node list | rg 'hold|wheel|turn_hold|arm_hold'
```

6. Watch for this expected early sequence in the stack log:
   - `LoadWorkspaceActionNode: workspace loaded successfully`
   - `ComputeWorkspaceNextActionActionNode: DIG_PASS`
   - `ResetArmMpcActionNode: MPC reset succeeded`
   - `EnableArmMpcController -> SUCCESS`
   - `move_to_dig -> SUCCESS`
   - `dig_action -> RUNNING`

Current known failure signature on `real_collisions`:
- startup/bootstrap is no longer the blocker if the flags above are off
- the first cycle can still fail later at `move_to_dump` with
  `ArmMpcMoveActionNode: target not reached within 30.00 s`
- treat that as a dump-leg motion problem, not a map-load or arm-controller activation problem

### dig

Bring up Dig3D only after Newton, robot, state publisher, and perception are healthy:

```bash
ros2 launch mole_highlevel_controller_cpp dig_3d_controller_cpp.launch.py \
  use_sim_time:=true \
  config:=no_aoa \
  mode:=weightedobs_rate0050_s203_1750 \
  activate_controller:=true \
  run_action:=false
```

### foxglove

Optional:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:=8765 \
  use_sim_time:=true
```

For isolated side stacks, prefer a non-default port and state it explicitly, for example:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:=8766 \
  address:=0.0.0.0 \
  use_sim_time:=true
```

If Foxglove connects but the excavator is not visible, check for these topics first:

```bash
ros2 topic list | rg '^/mole/(joint_states|robot_description)$'
ros2 topic info /mole/robot_description -v
```

That failure mode is usually missing model publishers, not a bridge failure.

### ackermann

Use the Ackermann drive controller as the low-level base executor for Nav2:

```bash
ros2 launch mole_highlevel_controller_cpp joy_drive_cpp.launch.py \
  use_sim_time:=true \
  on_machine:=false \
  activate_controller:=true \
  robot_namespace:=mole \
  cmd_vel_remap:=cmd_vel_smoothed
```

### nav2

Bring up Nav2 without RViz in its own window:

```bash
ros2 launch mole_nav2_bringup bringup.launch.py \
  use_sim_time:=true \
  on_machine:=false \
  launch_rviz:=false \
  robot_namespace:=mole \
  endeffector_type:=shovel_400mm_without_teeth \
  publish_self_footprint:=true \
  publish_static_map_odom_tf:=true
```

For Terra/workspace-commit runs, verify the self footprint before executing a plan. `mole_nav2_bringup` now publishes it
directly; if a split stack was started from an older command, start `mole_nav2_utils dynamic_footprint_publisher` in its
own tmux pane before resuming.

```bash
ros2 topic info /mole/global_costmap/footprint --verbose
timeout 12 ros2 topic echo /mole/global_costmap/footprint geometry_msgs/msg/Polygon --once
```

### benchmark

For the standard side-target regression, run the benchmark directly in tmux and log it:

```bash
python3 /workspace/moleworks/ros2_ws/src/moleworks_ros/mole_bringup/scripts/nav2_side_target_benchmark.py \
  --robot-ns mole \
  --lateral-m 1.0 \
  --random-targets 0 \
  --goal-timeout-sec 180.0 2>&1 | tee /tmp/nav2_side_target_benchmark.log
```

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

For the fast Nav2 layout, also verify these before sending a goal:

```bash
timeout 8 ros2 topic hz /clock
timeout 15 ros2 topic echo /mole/state --once
timeout 8 ros2 run tf2_ros tf2_echo map BASE_GRAV --ros-args -r __ns:=/mole
ros2 action list | sort
ros2 topic info /mole/actuator_commands -v
ros2 topic info /mole/cmd_vel_smoothed -v
```

Expected result:

- `/clock` is live at a stable sim rate
- `map -> BASE_GRAV` resolves
- `/mole/navigate_to_pose` and `/mole/follow_path` exist
- `/mole/actuator_commands` has exactly one `ackermann_drive_controller` publisher in the Nav2 layout; no hold-controller publishers
- `/mole/cmd_vel_smoothed` has exactly one `velocity_smoother` publisher

Do not use `ros2 topic echo --once /mole/cmd_vel_smoothed` as the primary health check before a goal is active. That topic can be idle until Nav2 is actually driving.

For sim-nav bringup, confirm the drive chain is healthy:

```bash
ros2 lifecycle get /mole/ackermann_drive_controller
ros2 topic info /mole/actuator_commands -v
ros2 topic info /mole/cmd_vel_smoothed -v
ros2 node list | sort | uniq -d
```

Expected state after startup:
- `/mole/ackermann_drive_controller`: `active [3]` for the fast Nav2 layout
- `/mole/actuator_commands`: exactly one `ackermann_drive_controller` publisher in the Nav2 layout, or one dig/executor publisher in the Terra layout; no stale extras and no hold-controller publishers in the Nav2 layout
- `/mole/cmd_vel_smoothed`: exactly one `velocity_smoother` publisher when the Nav2 layout is up
- `ros2 node list | sort | uniq -d`: no duplicate node names

For integrated `single_workspace`, also check:

```bash
ros2 lifecycle get /mole/mole_arm_mpc_controller
ros2 node list | rg 'hold|wheel|turn_hold|arm_hold' || true
ros2 topic info /mole/actuator_commands -v
```

Expected state:
- no hold-controller nodes
- `mole_arm_mpc_controller` may still show as `inactive` before the first `EnableArmMpcController`
- `/mole/actuator_commands` may list both `mole_arm_mpc_controller` and `dig_3d_controller`; that is expected for this integrated workflow

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

If the arm is visibly sinking or shaking, do not look for deleted hold-controller nodes. Reset the robot and check for competing actuator publishers instead:

```bash
ros2 service call /mole/reset_robot std_srvs/srv/Trigger '{}'
ros2 topic info /mole/actuator_commands -v
```

Use `/reset` only when you want the full Newton environment reset. Use `/mole/reset_robot` when you want to respawn the robot without changing terrain.

## 5) Dig Deployment Order

For Dig3D parity runs, keep the rollout order strict:

1. Start Newton with the requested surface via `--elevation-map`.
2. Start `robot.launch.py`.
3. Start `mole_state_publisher` if TF is split.
4. If the Newton terrain seed is wrong or stale, stop here and restart Newton with the correct `--elevation-map` before continuing.
5. Start local perception and excavation mapping with the same surface via `design_bag_path`.
6. Apply the requested target:
   - for analytic runtime profiles, mirror the profile to both `/mole/excavation_mapping/apply_runtime_profile`
     and `/mole/newton/apply_runtime_profile`
   - for premade STL geometry, apply the STL to `/excavation_mapping/load_excavation_map` with
     `mesh_to_excavation_grid_map.py apply`. Use
     `--authoring-frame CABIN_CONTROL --mesh-anchor-x max --mesh-x <distance>` when the target far
     edge is specified from the cabin-control origin.
7. On Hong no-holes, if the user did not request a different spawn pose, call `/mole/reset_robot_pose` with
   `x=-0.001212`, `y=0.299973`, `yaw=180 deg` after the runtime-profile apply.
8. Only then launch Dig3D and send `/run_dig_3d`.

Do not send the goal before the runtime profile or premade target geometry is applied.
Do not treat “map loaded” as complete unless both the Newton soil and ROS excavation mapping were seeded from the same artifact.
Do not mirror a runtime profile into Newton on top of a stale split stack. Fix the Newton terrain seed first, then relaunch the downstream ROS stack.

For `single_workspace.launch.py`, the executor owns that sequencing internally after Newton is up and
the runtime workspace apply succeeds, so do not also launch the split `dig` window on top of it.

### executor

In the split Terra dev loop, start the BT executor in its own window instead of the integrated `single_ws` launch:

```bash
ros2 launch terra_planner terra_executor.launch.py \
  use_sim_time:=true \
  robot_namespace:=mole \
  frame_id:=map \
  single_workspace_mode:=true \
  workspace_config_path:=$MW_LOCAL_WORKSPACE_CONFIG \
  dig_action_name:=run_dig_3d \
  workspace_planner_timeout_sec:=60.0
```

If the BT fails but Newton, perception, planner, OCS2, and Dig3D are still healthy, restart only the executor first:

```bash
ros2 service call /mole/terra_executor/restart std_srvs/srv/Trigger "{}"
```

If that does not recover the loop, escalate in this order:

1. restart `executor`
2. restart `dig`
3. restart `ocs2`
4. reset robot pose / workspace only if the failure is geometry-state related

#### Packaged Foundation Execution

For packaged flat-foundation plans, load `terra-foundation-execution`. This startup skill intentionally does not carry
the full foundation launch/checklist so it can stay focused on generic Newton runtime bringup and failure recovery.

## 6) Failure Retry Checkpoint And Snapshot

Before teardown or restart after a failed Newton/Terra run, save the retry checkpoint first. A Newton restart empties
the bucket, so a mid-dig current map is not an exact replay of a full-bucket controller state. For controller retries,
use the latest Terra checkpoint saved before the failed action. Also save a current map/pose snapshot for diagnosis, but
label it as diagnostic unless the failure happened at a clean BT boundary with an empty bucket.

Prefer the run directory when one exists so the bundle sits next to `logs/`, `runtime/`, and `checkpoints/`:

```bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-24}
source /workspace/moleworks/ros2_ws/install/setup.bash
RUN_DIR=${RUN_DIR:-/tmp/newton_failure_$(date -u +%Y%m%d_%H%M%S)}
STATE_DIR="$RUN_DIR/failure_state/$(date -u +%Y%m%d_%H%M%S)"
CURRENT_SNAPSHOT_DIR="$STATE_DIR/current_snapshot"
mkdir -p "$STATE_DIR"

{
  echo "date_utc=$(date -u --iso-8601=seconds)"
  echo "host=$(hostname)"
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  echo "RUN_DIR=$RUN_DIR"
  echo "PWD=$PWD"
} > "$STATE_DIR/context.txt"

LATEST_CHECKPOINT=$(
  find "$RUN_DIR/checkpoints" -path '*checkpoint.yaml' -type f -printf '%T@ %p\n' 2>/dev/null | \
    sort -nr | head -1 | sed 's/^[^ ]* //'
)
PAIR_INDEX=${PAIR_INDEX:-1}
COMPLETED_DIGGING_LOOP_INDEX=${COMPLETED_DIGGING_LOOP_INDEX:-0}
if [ -n "$LATEST_CHECKPOINT" ]; then
  echo "$LATEST_CHECKPOINT" > "$STATE_DIR/retry_checkpoint_source.txt"
  rm -rf "$STATE_DIR/retry_checkpoint"
  cp -a "$(dirname "$LATEST_CHECKPOINT")" "$STATE_DIR/retry_checkpoint"
  {
    echo "python3 src/moleworks_ros/scripts/resume_from_checkpoint.py \\"
    echo "  $STATE_DIR/retry_checkpoint/checkpoint.yaml \\"
    echo "  --on_machine false"
  } > "$STATE_DIR/retry_resume_command.txt"
  read -r PAIR_INDEX COMPLETED_DIGGING_LOOP_INDEX < <(
    python3 - "$LATEST_CHECKPOINT" <<'PY'
import sys, yaml
data = yaml.safe_load(open(sys.argv[1])) or {}
print(int(data.get("pair_index", data.get("workspace_pair_index", 1))),
      int(data.get("completed_digging_loop_index", data.get("digging_loop_index", 0))))
PY
  )
fi

python3 - "$CURRENT_SNAPSHOT_DIR" "$PAIR_INDEX" "$COMPLETED_DIGGING_LOOP_INDEX" "${ROBOT_NAMESPACE:-mole}" \
  "${MAP_FRAME:-map}" "${BASE_FRAME:-base_link}" <<'PY'
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from mole_excavation_mapping.srv import SaveGridMap
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

snapshot_dir = Path(sys.argv[1])
pair_index = int(sys.argv[2])
completed_index = int(sys.argv[3])
robot_namespace = sys.argv[4].strip("/") or "mole"
map_frame = sys.argv[5]
base_frame = sys.argv[6]
snapshot_dir.mkdir(parents=True, exist_ok=True)

rclpy.init()
node = rclpy.create_node("newton_failure_current_snapshot_capture")
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)
del tf_listener

tf = None
deadline = time.monotonic() + 15.0
last_error = None
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        tf = tf_buffer.lookup_transform(map_frame, base_frame, Time())
        break
    except Exception as exc:
        last_error = exc
if tf is None:
    raise RuntimeError(f"TF {map_frame} -> {base_frame} unavailable: {last_error}")

client = node.create_client(SaveGridMap, "/excavation_mapping/save_map")
if not client.wait_for_service(timeout_sec=10.0):
    raise RuntimeError("/excavation_mapping/save_map service not available")
req = SaveGridMap.Request()
req.uri = str(snapshot_dir / "excavation_map")
req.topic = "grid_map"
req.storage_id = "mcap"
req.overwrite = True
req.include_layers = []
future = client.call_async(req)
deadline = time.monotonic() + 20.0
while rclpy.ok() and not future.done() and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
if not future.done():
    raise RuntimeError("/excavation_mapping/save_map timed out")
result = future.result()
if result is None or not result.success:
    message = "no response" if result is None else result.message
    raise RuntimeError(f"/excavation_mapping/save_map failed: {message}")

q = tf.transform.rotation
yaw_deg = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
metadata = {
    "version": 1,
    "created_at": time.strftime("%Y%m%d_%H%M%S"),
    "map_name": "manual_failure_current_snapshot",
    "checkpoint_dir": str(snapshot_dir),
    "checkpoint_root": str(snapshot_dir.parent),
    "plan_path": "",
    "robot_namespace": robot_namespace,
    "tf_prefix": "",
    "workspace_pair_index": pair_index,
    "pair_index": pair_index,
    "digging_loop_index": completed_index,
    "completed_digging_loop_index": completed_index,
    "waypoint_index_after_load": pair_index * 2,
    "excavation_map_uri": "excavation_map",
    "excavation_map_topic": "grid_map",
    "excavation_map_storage_id": "mcap",
    "notes": "Diagnostic current snapshot. Newton restart empties bucket contents; use retry_checkpoint for controller retries after mid-dig failures.",
    "robot_pose": {
        "parent_frame": tf.header.frame_id,
        "child_frame": tf.child_frame_id,
        "x": float(tf.transform.translation.x),
        "y": float(tf.transform.translation.y),
        "z": float(tf.transform.translation.z),
        "qx": float(q.x),
        "qy": float(q.y),
        "qz": float(q.z),
        "qw": float(q.w),
        "yaw_deg": float(yaw_deg),
    },
}
(snapshot_dir / "checkpoint.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False))
(snapshot_dir / "resume_command.txt").write_text(
    "python3 src/moleworks_ros/scripts/resume_from_checkpoint.py "
    f"{snapshot_dir / 'checkpoint.yaml'} --on_machine false\n"
)
print(snapshot_dir / "checkpoint.yaml")
node.destroy_node()
rclpy.shutdown()
PY

tmux list-sessions > "$STATE_DIR/tmux_sessions.txt" 2>&1 || true
tmux list-panes -a -F '#{session_name}:#{window_index}.#{pane_index} #{window_name} pid=#{pane_pid} cmd=#{pane_current_command}' \
  > "$STATE_DIR/tmux_panes.txt" 2>&1 || true
while read -r pane _; do
  [ -z "$pane" ] && continue
  safe_pane=$(echo "$pane" | tr ':.' '__')
  tmux capture-pane -p -S -500 -t "$pane" > "$STATE_DIR/tmux_${safe_pane}.log" 2>&1 || true
done < "$STATE_DIR/tmux_panes.txt"

ps -eo pid,ppid,stat,etime,%cpu,%mem,cmd > "$STATE_DIR/processes.txt" 2>&1 || true
free -h > "$STATE_DIR/memory.txt" 2>&1 || true
nvidia-smi --query-gpu=index,name,memory.used,memory.total,utilization.gpu --format=csv,noheader,nounits \
  > "$STATE_DIR/gpu.txt" 2>&1 || true

ros2 node list > "$STATE_DIR/ros_nodes.txt" 2>&1 || true
ros2 topic list -t > "$STATE_DIR/ros_topics.txt" 2>&1 || true
ros2 service list -t > "$STATE_DIR/ros_services.txt" 2>&1 || true
for topic in /clock /mole/state /mole/joint_states /mole/actuator_commands /excavation_mapping/grid_map \
  /controller_status /dig_3d/scooped_soil_volume; do
  topic_file=$(echo "$topic" | sed 's#^/##; s#/#_#g')
  timeout 10 ros2 topic echo "$topic" --once --full-length > "$STATE_DIR/topic_${topic_file}.txt" 2>&1 || true
  ros2 topic info "$topic" -v > "$STATE_DIR/topic_${topic_file}_info.txt" 2>&1 || true
done
for node in /mole/terra_executor /mole/dig_3d_controller /mole/workspace_planner_server /mole/mole_arm_mpc_controller; do
  ros2 param dump "$node" > "$STATE_DIR/params_$(basename "$node").yaml" 2>&1 || true
done

find "${RUN_DIR:-/tmp}" -path '*checkpoint.yaml' -type f -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -20 \
  > "$STATE_DIR/recent_checkpoints.txt" || true
cp "$RUN_DIR"/logs/*.log "$STATE_DIR"/ 2>/dev/null || true

echo "$STATE_DIR"
```

### Resume From A Checkpoint

After the failure bundle is captured and the full Newton/Terra stack has been restarted, replay from the copied retry
checkpoint. Keep the same `ROS_DOMAIN_ID`, run from the ROS workspace root, and source the same overlay first:

```bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-24}
cd /workspace/moleworks/ros2_ws
source install/setup.bash
python3 src/moleworks_ros/scripts/resume_from_checkpoint.py \
  "$CHECKPOINT_YAML_OR_DIR" \
  --on_machine false \
  --robot-namespace "${ROBOT_NAMESPACE:-mole}"
```

For the bundle created above, prefer the generated command:

```bash
cd /workspace/moleworks/ros2_ws
bash "$STATE_DIR/retry_resume_command.txt"
```

`resume_from_checkpoint.py` accepts either the checkpoint directory or `checkpoint.yaml`. It waits for
`/<robot_namespace>/terra_executor/resume` before mutating state, then in simulation mode calls
`/<robot_namespace>/reset_robot_pose`, `/<robot_namespace>/load_soil_map`,
`/<robot_namespace>/elevation_mapping_cupy/clear_map`, `/excavation_mapping/load_excavation_map`, and finally
`/<robot_namespace>/terra_executor/resume` with `skip_navigation=true` for the resumed workspace.

For multi-waypoint Terra plans in Newton with `skip_navigation:=true`, do not acknowledge the manual-navigation gate
while the status log still reports a nonzero distance to the target. The plan waypoint `agent_state.pos_base` is the
base pose expected for the next workspace; if Newton is not physically driving there, call `/mole/reset_robot_pose` to
the displayed target pose first, then call `/mole/manual_navigation_done`. Acknowledging without moving leaves the live
BASE pose at the old station, and the workspace planner will correctly reject dump targets against the wrong base
keepaway.

Checkpoint leaf directories use `<pair_index>_<completed_digging_loop_index>`. For example, `3_2` means resume the
third Terra workspace pair after two completed dig/dump loops in that workspace. In `single_workspace_mode`, only
`1_<loop>` is valid; pair indices above 1 require normal Terra plan mode with the matching `design_map_name` installed.

Use `failure_state/.../retry_checkpoint/checkpoint.yaml` for controller retries. Only use
`failure_state/.../current_snapshot/checkpoint.yaml` when the failure was at a clean BT boundary with an empty bucket, or
when you explicitly want a diagnostic replay rather than an exact controller retry. The resume path restores the saved
excavation map, Newton soil map, and base pose; it does not restore bucket contents or full joint/controller internal
state.

When adding the result to a validation note or replay manifest, record:

- run directory
- exact launch command or tmux pane log
- retry `checkpoint.yaml` from `failure_state/.../retry_checkpoint/`
- failure signal from `stack.log`
- diagnostic `failure_state/.../current_snapshot/excavation_map`
- `topic_mole_state.txt` and `topic_mole_joint_states.txt` if the failure depends on arm/base state

The retry checkpoint restores the map and simulator base pose through the current resume services, and the bucket is
expected to be empty after restart. The current snapshot records `/mole/state` and `/mole/joint_states` for diagnosis,
but it is not an exact full-bucket replay unless the simulator gains explicit bucket-content and full-joint restore
services.

Do not restart Newton or kill the stack before this capture unless the process is actively harming the machine. If
Newton itself is unstable but ROS callbacks are still alive, call `/mole/newton_pause` first and then capture:

```bash
ros2 service call /mole/newton_pause std_srvs/srv/SetBool "{data: true}" || true
```

## 7) Teardown

Inside the container:

```bash
tmux kill-session -t newton_sim 2>/dev/null || true
self=$$
mw_stack_patterns=(
  'standalone_fee_terra_newton_env.py'
  'standalone_dig_newton_env.py'
  'native_fee_terra_default_viewer.py'
  'newton_bridge.launch.py'
  'robot.launch.py'
  'mole_state_publisher.launch.py'
  'mole_perception_bringup'
  'dig_3d_controller_cpp.launch.py'
  'compare_dig3d_live_obs.py'
  'foxglove_bridge'
  'ackermann_drive_controller_node'
  'controller_server'
  'planner_server'
  'behavior_server'
  'bt_navigator'
  'velocity_smoother'
  'collision_monitor'
  'waypoint_follower'
  'opennav_docking'
  'lifecycle_manager_navigation'
  'odom_nav2_adapter'
  'dynamic_footprint_publisher'
  'robot_state_publisher/robot_state_publisher'
  'mole_joint_state_publisher_node'
  'elevation_mapping_node.py'
)
for pat in "${mw_stack_patterns[@]}"; do
  for pid in $(pgrep -f "$pat" || true); do
    [ "$pid" = "$self" ] && continue
    kill "$pid" 2>/dev/null || true
  done
done
sleep 2
for pat in "${mw_stack_patterns[@]}"; do
  for pid in $(pgrep -f "$pat" || true); do
    [ "$pid" = "$self" ] && continue
    kill -9 "$pid" 2>/dev/null || true
  done
done
```

Re-check resources before the next launch:

```bash
free -h
nvidia-smi --query-gpu=index,name,memory.used,memory.total,utilization.gpu --format=csv,noheader,nounits
ps -eo pid,ppid,%mem,%cpu,etime,cmd | rg 'standalone_dig_newton_env|newton_bridge|robot.launch.py|mole_state_publisher|mole_perception_bringup|excavation_mapping|dig_3d_controller|rviz|ros2 launch'
```
