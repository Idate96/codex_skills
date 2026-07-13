---
name: terra-foundation-execution
description: "Execute packaged Terra foundation plans in Newton with Nav2, checkpoints, stall measurement, recovery, and failure capture."
---

# Terra Foundation Execution

## Scope

Use this skill for the actual packaged foundation run: selecting the `flat_foundation` or
`flat_foundation_depth_0p5` Terra plan, launching Terra in multi-waypoint mode, verifying Nav2 can navigate between
stations, measuring whether execution stalls after dumping, and preserving enough state to debug or resume failures.

This is not the generic startup runbook. First satisfy the relevant `newton-sim-ros-startup` preflight: correct container,
clean tmux session, isolated `ROS_DOMAIN_ID`, correct `/workspace/moleworks/ros2_ws/install/setup.bash` overlay, Newton
process discipline, and basic `/clock`, `/mole/state`, and TF health.

## Non-Negotiables

- Work from `/workspace/moleworks/ros2_ws` inside the Moleworks ROS runtime container.
- Keep the same `ROS_DOMAIN_ID` in Newton, Terra, monitors, and any helper panes.
- Use the main workspace install, not a nested `src/moleworks_ros/install`.
- Packaged foundation plans are schema-v2 multi-waypoint plans: use `single_workspace_mode:=false`.
- In Newton simulation, let Nav2 drive between foundation workspaces: use `skip_navigation:=false`.
- Do not acknowledge manual navigation for these runs unless intentionally resuming a checkpoint path that already reset
  the simulated base pose.
- Use `mapping_profile:=site` for packaged foundation plans. The local dig profile publishes the OccupancyGrid on a
  relative `map` topic under `/mole`, which can leave Nav2 waiting forever on `/map`.
- Before a teardown after failure, preserve the latest checkpoint, current excavation-map snapshot, tmux panes, ROS graph,
  and relevant topic/param samples. Use the failure bundle procedure from `newton-sim-ros-startup`.
- If comparing against older behavior, create or use a separate reference checkout/worktree at a pre-May-23 revision.
  Do not revert the active main worktree mid-run.

## Fixtures

Set the canonical packaged foundation fixtures after sourcing the ROS overlay:

```bash
export MW_TERRA_FLAT_FOUNDATION_DEPTH1_ROOT=/workspace/moleworks/ros2_ws/src/moleworks_maps/maps/flat_foundation
export MW_TERRA_FLAT_FOUNDATION_DEPTH1_PLAN=$MW_TERRA_FLAT_FOUNDATION_DEPTH1_ROOT/map/terra_plan.json
export MW_TERRA_FLAT_FOUNDATION_DEPTH1_DESIGN_MCAP=$MW_TERRA_FLAT_FOUNDATION_DEPTH1_ROOT/flat_foundation_design/flat_foundation_design.mcap

export MW_TERRA_FLAT_FOUNDATION_DEPTH05_ROOT=/workspace/moleworks/ros2_ws/src/moleworks_maps/maps/flat_foundation_depth_0p5
export MW_TERRA_FLAT_FOUNDATION_DEPTH05_PLAN=$MW_TERRA_FLAT_FOUNDATION_DEPTH05_ROOT/map/terra_plan.json
export MW_TERRA_FLAT_FOUNDATION_DEPTH05_DESIGN_MCAP=$MW_TERRA_FLAT_FOUNDATION_DEPTH05_ROOT/flat_foundation_depth_0p5_design/flat_foundation_depth_0p5_design.mcap
```

Pick exactly one target:

```bash
# 0.5 m foundation cut, default for current end-to-end validation:
export MW_TERRA_TEST_PLAN=$MW_TERRA_FLAT_FOUNDATION_DEPTH05_PLAN
export MW_TERRA_TEST_DESIGN_MCAP=$MW_TERRA_FLAT_FOUNDATION_DEPTH05_DESIGN_MCAP
export MW_TERRA_TEST_MAP_NAME=flat_foundation_depth_0p5

# 1.0 m foundation cut:
# export MW_TERRA_TEST_PLAN=$MW_TERRA_FLAT_FOUNDATION_DEPTH1_PLAN
# export MW_TERRA_TEST_DESIGN_MCAP=$MW_TERRA_FLAT_FOUNDATION_DEPTH1_DESIGN_MCAP
# export MW_TERRA_TEST_MAP_NAME=flat_foundation
```

Use the same design MCAP on both sides:

- Newton soil seed: `--elevation-map "$MW_TERRA_TEST_DESIGN_MCAP" --elevation-layer elevation --max-depth-layer desired_elevation`
- ROS excavation mapping: `design_map_name:=$MW_TERRA_TEST_MAP_NAME` with `mapping_profile:=site`
- Terra executor: `plan_path:=$MW_TERRA_TEST_PLAN`, or integrated `terra.launch.py` with the same `design_map_name`

## Launch Pattern

Use one tmux session per domain. Suggested names:

```bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-31}
export RUN_DIR=/tmp/terra_foundation_${MW_TERRA_TEST_MAP_NAME}_${ROS_DOMAIN_ID}_$(date -u +%Y%m%d_%H%M%S)
mkdir -p "$RUN_DIR/logs"
tmux new-session -d -s terra_foundation_${ROS_DOMAIN_ID}
```

Newton pane:

```bash
cd /home/lorenzo/moleworks/moleworks_newton
source /workspace/moleworks/ros2_ws/install/setup.bash
source .venv/bin/activate
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-31}
python scripts/ros/standalone_fee_terra_newton_env.py \
  --elevation-map "$MW_TERRA_TEST_DESIGN_MCAP" \
  --elevation-layer elevation \
  --max-depth-layer desired_elevation
```

Before starting Terra in Newton simulation, fast-start the robot at the first packaged waypoint. This keeps Nav2 enabled
for the workflow while avoiding a long initial drive across the seeded map. Do this only after Newton is publishing
`/clock` and `/mole/state`, and only before Terra has started:

```bash
source /workspace/moleworks/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-31}

timeout 60 ros2 topic echo /clock --once >/dev/null
timeout 60 ros2 topic echo /mole/state --once --qos-reliability best_effort >/dev/null

eval "$(
python3 - <<'PY'
import json
import math
import os
from pathlib import Path

plan = json.loads(Path(os.environ["MW_TERRA_TEST_PLAN"]).read_text())
alignment = plan["alignment"]
waypoint = plan["waypoints"][0]
pos_base = waypoint["agent_state"]["pos_base"]
meters_per_tile = float(alignment["meters_per_tile"])
origin_x, origin_y = [float(v) for v in alignment["origin_map_xy_m"]]
yaw_map_from_plan = float(alignment["yaw_map_from_plan_rad"])
plan_x = float(pos_base[0]) * meters_per_tile
plan_y = float(pos_base[1]) * meters_per_tile
c = math.cos(yaw_map_from_plan)
s = math.sin(yaw_map_from_plan)
map_x = origin_x + c * plan_x - s * plan_y
map_y = origin_y + s * plan_x + c * plan_y
map_yaw = math.atan2(
    math.sin(float(waypoint["agent_state"]["angle_base_rad"]) + yaw_map_from_plan),
    math.cos(float(waypoint["agent_state"]["angle_base_rad"]) + yaw_map_from_plan),
)
print(f"export MW_TERRA_FIRST_BASE_X_M={map_x:.9f}")
print(f"export MW_TERRA_FIRST_BASE_Y_M={map_y:.9f}")
print(f"export MW_TERRA_FIRST_BASE_YAW_DEG={math.degrees(map_yaw):.9f}")
PY
)"

ros2 service call /mole/reset_robot_pose mole_msgs/srv/ResetRobotPose \
  "{x_m: ${MW_TERRA_FIRST_BASE_X_M}, y_m: ${MW_TERRA_FIRST_BASE_Y_M}, yaw_deg: ${MW_TERRA_FIRST_BASE_YAW_DEG}}"
```

Integrated Terra pane:

```bash
cd /workspace/moleworks/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-31}
export MW_EXPECTED_ROS_PREFIX=/workspace/moleworks/ros2_ws/install
ros2 launch mole_bringup terra.launch.py \
  use_sim_time:=true \
  on_machine:=false \
  launch_rviz:=false \
  launch_foxglove:=false \
  launch_nav_rviz:=false \
  robot_namespace:=mole \
  design_map_name:=$MW_TERRA_TEST_MAP_NAME \
  single_workspace_mode:=false \
  skip_navigation:=false \
  nav2_ready_gate_enabled:=true \
  nav2_ready_timeout_sec:=300.0 \
  ocs2_task_profile:=sim_no_collision \
  recompute_terrain_sdf_on_target:=false \
  keep_arm_mpc_active_between_cycles:=true \
  checkpoint_async_timeout_sec:=5.0 \
  workspace_planner_timeout_sec:=60.0 \
  workspace_completion_profile:=completion_foundation_strict.yaml \
  terra_start_delay:=5.0 \
  enable_camera:=false \
  enable_sam3:=false \
  mapping_profile:=site \
  excavation_mapping_publish_grid_map:=true \
  excavation_mapping_publish_map:=true
```

Use a split launch only when debugging one owner. For the end-to-end acceptance run, prefer the integrated launch above.

## Pre-Execution Gates

Do these before treating Terra logs as meaningful:

```bash
source /workspace/moleworks/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-31}

timeout 30 ros2 topic echo /clock --once
timeout 30 ros2 topic echo /mole/state --once --qos-reliability best_effort
timeout 15 ros2 run tf2_ros tf2_echo map BASE

ros2 topic info /map -v
ros2 topic info /excavation_mapping/grid_map -v
ros2 param get /mole/excavation_mapping map_topic
ros2 lifecycle get /mole/controller_server
ros2 lifecycle get /mole/planner_server
ros2 lifecycle get /mole/bt_navigator
```

Expected:

- `/map` has one transient-local publisher from excavation mapping and Nav2 global costmap subscribes to it.
- `/excavation_mapping/grid_map` has a publisher.
- `map_topic` is `/map`, not `map`.
- Nav2 lifecycle nodes are active before the executor starts or before the Nav2-ready gate expires.

If `/mole/map` has a publisher but `/map` has none, the run is not valid for Nav2. Restart with `mapping_profile:=site`
or fix the launch path before continuing.

## Milestones To Watch

The first complete cycle should show:

1. Terra loads the packaged plan and sends a Nav2 goal for the first foundation station.
2. Nav2 reports progress and reaches the station without a manual navigation ack.
3. Workspace planner computes a `DIG_PASS`.
4. MPC move-leg receives the new target and reaches the dig pose.
5. Dig controller executes the scoop.
6. Soil carving finishes against the post-dig map.
7. Dump action executes and clears bucket state.
8. Checkpoint save is best-effort async and does not block the next target longer than the configured timeout.
9. The next workspace-target computation starts from the carved map, not from the pre-carve map.
10. The loop either continues to the next station or exits with `COVERAGE_DONE`.

Treat these as acceptance checks, not just logs. If the order is wrong, preserve the run artifacts before restarting.

## Post-Dump Stall Measurement

Define post-dump stall as: after a dump action reports success, no checkpoint completion, workspace-target request,
navigation goal, move-leg target, or terminal status appears for more than 60 seconds while Terra remains running.

Start lightweight monitors before the first dig:

```bash
mkdir -p "$RUN_DIR/monitors"
ros2 topic echo /controller_status --full-length > "$RUN_DIR/monitors/controller_status.log" 2>&1
ros2 topic echo /mole/cmd_vel_smoothed > "$RUN_DIR/monitors/cmd_vel_smoothed.log" 2>&1
ros2 topic echo /dig_3d/scooped_soil_volume > "$RUN_DIR/monitors/scooped_soil_volume.log" 2>&1
```

Also keep the Terra pane log. On failure or completion, record:

- time of last dump-start and dump-success log
- time of next checkpoint, planner request, Nav2 goal, move-leg target, or terminal status
- whether `/mole/cmd_vel_smoothed` or `/mole/actuator_commands` still had active publishers
- latest checkpoint path before the stalled action

## Recovery Discipline

If Nav2 stalls before the first workspace, first check `/map`; do not debug MPC or workspace planning until Nav2 has a
valid global map.

If workspace planning fails after a dump, verify the map was carved before target recomputation. Use the latest Terra
checkpoint and the current excavation-map snapshot as the replay inputs.

If Newton is restarted for any reason, restart the full ROS-side stack too. Do not keep the old Terra, OCS2, perception,
or Nav2 panes alive across a sim-time jump.

If a controller or BT owner fails while Newton and perception are still healthy, preserve a failure bundle first, then
restart the smallest owner that can recover the loop. For exact controller retries, resume from the latest pre-action
checkpoint, not from a current snapshot captured mid-action.
