# Beam6 Two-Stage STL Workflow Reference

Use this reference for exact command shapes. The live run should still use the `apply_target` and `run_stage` commands printed by `generate_trench_sequence_plans.py`, because those commands include the current map-frame trench-axis values.

## Contents

- Runtime setup
- Live anchored plan generation and registration
- Flange stage
- Flange-to-bottom map handoff
- Bottom stage
- Failure signs

## Runtime Setup

```bash
cd /workspace/moleworks/ros2_ws
source install/setup.bash
: "${ROS_DOMAIN_ID:?Set ROS_DOMAIN_ID to the verified Newton simulation domain}"
export MW_EXPECTED_ROS_PREFIX=/workspace/moleworks/ros2_ws/install
```

Start the simulation through the active Newton or Isaac/Terra runbook. Start the ROS base stack with generic Terra disabled:

```bash
launch_terra:=false
```

Beam6 still launches Terra for the generated workspaces. The base stack keeps generic Terra disabled so that `beam6_sequence_stage.launch.py` can own the stage-specific `workspace_planner_server` and `terra_executor` for the generated `<stage>_terra_plan.json`.

For a stock-shovel Newton flange run, match analytical bucket geometry:

```bash
--bucket-width 1.3 --bucket-y-sample-half-width 0.65
```

## Generate Live Anchored Beam6 Plans

Run this only after TF is live and the robot is at the first base station:

```bash
python3 src/moleworks_ros/high_level_planning/terra_planner/scripts/generate_trench_sequence_plans.py \
  --config src/moleworks_ros/high_level_planning/terra_planner/config/beam6_flange_bottom_sequence.yaml \
  --output-root /tmp/beam6_flange_bottom_sequence \
  --live-first-base-frame BASE
```

The generator uses `BASE` for first-station translation and fixed `BASE_CONTROL` for trench-axis yaw. It writes:

- `/tmp/beam6_flange_bottom_sequence/flange6_terra_plan.json`
- `/tmp/beam6_flange_bottom_sequence/bottom6_terra_plan.json`
- `/tmp/beam6_flange_bottom_sequence/station_anchor.json`
- `/tmp/beam6_flange_bottom_sequence/flange6_target_width_1.300m.stl`

For the current recipe, live first-station anchoring shifts the target farthest X from `7.0` to `6.375` in the station frame. Do not move the base between generation and target application.

## Shared Registration Contract

Both targets are authored in `BASE_CONTROL` and applied into `map` through the live `map -> BASE_CONTROL` TF lookup:

```bash
--authoring-frame BASE_CONTROL \
--mesh-anchor-x max \
--mesh-x 6.375 \
--mesh-y 0.0 \
--mesh-anchor-y origin \
--align-major-axis x \
--reference-mode local_min \
--mesh-reference-z max
```

Do not use `--authoring-frame BASE`, live `CABIN_CONTROL`, or legacy `--cabin-control-farthest-x-m` for Beam6. The generator rejects those old conventions.

Before bottom target application, compare the live transforms against `station_anchor.json`:

```bash
cat /tmp/beam6_flange_bottom_sequence/station_anchor.json
timeout 15 bash -lc 'ros2 run tf2_ros tf2_echo map BASE 2>&1' | head -40
timeout 15 bash -lc 'ros2 run tf2_ros tf2_echo map BASE_CONTROL 2>&1' | head -40
```

## Flange Stage

Prevent completed `dug_zone` bookkeeping from becoming Nav2 obstacles:

```bash
EM_NODE=$(ros2 node list | grep -E '(^|/)excavation_mapping$' | head -1)
test -n "$EM_NODE"
ros2 param set "$EM_NODE" occupancy_mark_dug_zone_occupied false
```

Apply target:

```bash
ros2 run mole_excavation_mapping mesh_to_excavation_grid_map.py apply \
  /tmp/beam6_flange_bottom_sequence/flange6_target_width_1.300m.stl \
  --authoring-frame BASE_CONTROL \
  --mesh-anchor-x max \
  --mesh-x 6.375 \
  --mesh-y 0.0 \
  --mesh-anchor-y origin \
  --align-major-axis x \
  --reference-mode local_min \
  --mesh-reference-z max \
  --map-topic /excavation_mapping/grid_map \
  --load-service /excavation_mapping/load_excavation_map \
  --timeout-sec 60.0 \
  --tf-timeout-sec 5.0 \
  --force
```

Zero-yaw smoke-test stage command shape:

```bash
ros2 launch terra_planner beam6_sequence_stage.launch.py \
  stage:=flange6 \
  sequence_output_root:=/tmp/beam6_flange_bottom_sequence \
  endeffector_type:=shovel \
  workspace_planner_blade_width_m:=1.3 \
  workspace_plan_trench_axis_start_x:=-0.825 \
  workspace_plan_trench_axis_start_y:=-0.274 \
  workspace_plan_trench_axis_end_x:=6.375 \
  workspace_plan_trench_axis_end_y:=-0.274 \
  workspace_plan_trench_target_half_width_m:=0.65 \
  workspace_plan_trench_legal_half_width_m:=0.75 \
  workspace_plan_trench_entry_margin_m:=0.25 \
  workspace_plan_trench_exit_margin_m:=0.25 \
  workspace_dig_boundary_margin_m:=0.8 \
  grading_only:=true \
  dig_start_soil_carving_service:=excavation_mapping/start_target_clamped_soil_carving \
  trench_axis_finish_mask_mode:=target_depth \
  remaining_height_done_threshold_m_excavate:=0.05 \
  dig_3d_policy_id:=r14c_tbar200_net1024_s213_3750
```

## Flange To Bottom Handoff

Save the live flange map before stopping the flange stack:

```bash
BEAM6_CHECKPOINT_ROOT=/tmp/beam6_flange_bottom_sequence/checkpoints
BEAM6_FLANGE_BAG="$BEAM6_CHECKPOINT_ROOT/after_flange_$(date -u +%Y%m%d_%H%M%S)"

timeout 20 ros2 service call /excavation_mapping/save_map mole_excavation_mapping/srv/SaveGridMap \
  "{uri: '$BEAM6_FLANGE_BAG', topic: 'grid_map', storage_id: 'mcap', overwrite: false}"
```

Require a successful service response, then resolve and validate the latest saved map while the
flange stack is still available:

```bash
eval "$(
  python3 src/moleworks_ros/high_level_planning/terra_planner/applications/beam6_flange_bottom_sequence/beam6_latest_map.py \
    --root /tmp/beam6_flange_bottom_sequence/checkpoints \
    --format shell
)"

test -n "$BEAM6_LATEST_MAP_BAG"
test -n "$BEAM6_LATEST_MAP_MCAP"
```

Only then stop every process that embeds stock-shovel geometry: robot state publisher, self-filter,
OCS2, dig controller, workspace planner, and Terra executor. Use the scoped teardown from
`newton-sim-ros-startup`; do not use broad process-name kills.

If Newton uses a desired-elevation clamp, do not pass the flange-only map directly to the real bottom dig. First start a temporary 400 mm alignment stack, take the generator-printed bottom `apply_target` command, and add a dry-run output to materialize the bottom target into the saved flange terrain:

```bash
BOTTOM_SEED_BAG=/tmp/beam6_flange_bottom_sequence/checkpoints/bottom_target_seed_$(date -u +%Y%m%d_%H%M%S)
# Run the generator-printed bottom apply command with:
#   --dry-run-output "$BOTTOM_SEED_BAG" --dry-run-storage-id mcap
```

Then start Newton with the 400 mm robot and the MCAP inside `$BOTTOM_SEED_BAG`:

```bash
--robot-variant m445_terra_400mm_without_teeth --bucket-width 0.4 --bucket-y-sample-half-width 0.2
```

For ROS excavation mapping, load the same bottom-target seed bag. Do not load the flange-only bag on one side and the bottom seed on the other.

```bash
python3 src/moleworks_ros/high_level_planning/terra_planner/applications/beam6_flange_bottom_sequence/beam6_latest_map.py \
  --root "$BOTTOM_SEED_BAG" \
  --load \
  --load-service /excavation_mapping/load_excavation_map
```

## Bottom Stage

After the 400 mm stack is healthy, repeat the `dug_zone` occupancy parameter:

```bash
EM_NODE=$(ros2 node list | grep -E '(^|/)excavation_mapping$' | head -1)
test -n "$EM_NODE"
ros2 param set "$EM_NODE" occupancy_mark_dug_zone_occupied false
```

Apply the bottom target from the station saved in `station_anchor.json` by running the exact generator-printed bottom `apply_target` command. Do not reuse the old hardcoded Downloads path or zero-yaw placement for a live run.

Clear costmaps:

```bash
ros2 service call /mole/global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
ros2 service call /mole/local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
```

Zero-yaw smoke-test stage command shape:

```bash
ros2 launch terra_planner beam6_sequence_stage.launch.py \
  stage:=bottom6 \
  sequence_output_root:=/tmp/beam6_flange_bottom_sequence \
  terra_start_delay:=15.0 \
  skip_navigation:=false \
  endeffector_type:=shovel_400mm_without_teeth \
  workspace_planner_blade_width_m:=0.4 \
  workspace_planner_mode:=1 \
  workspace_planner_profile:=stripwise_max_volume \
  workspace_plan_trench_axis_start_x:=-0.825 \
  workspace_plan_trench_axis_start_y:=-0.274 \
  workspace_plan_trench_axis_end_x:=6.375 \
  workspace_plan_trench_axis_end_y:=-0.274 \
  workspace_plan_trench_target_half_width_m:=0.2 \
  workspace_plan_trench_legal_half_width_m:=0.3 \
  workspace_plan_trench_entry_margin_m:=0.25 \
  workspace_plan_trench_exit_margin_m:=0.25 \
  workspace_dig_boundary_margin_m:=0.3 \
  grading_only:=false \
  enable_grading_pass:=true \
  use_dig_zone_if_nonempty:=false \
  dig_3d_dig_zone_layer_name:=current_dig_workspace \
  dig_3d_pullup_boundary_min_distance_m:=0.0 \
  grading_workspace_extension_m:=0.500 \
  workspace_completion_profile:=completion_foundation_strict.yaml \
  dig_start_soil_carving_service:=excavation_mapping/start_target_clamped_soil_carving \
  trench_axis_finish_mask_mode:=target_depth \
  grading_completion_mode:=local_open \
  remaining_height_done_threshold_m_excavate:=0.05 \
  dig_3d_policy_id:=shovel400_r47_b6p60_toprbf_net256_s214_6000
```

## Failure Signs

- `station_anchor.json` is missing after live plan generation.
- Target apply uses `BASE` or live `CABIN_CONTROL` instead of `BASE_CONTROL`.
- `map -> BASE` translation or `map -> BASE_CONTROL` yaw differs before bottom apply.
- Bottom starts with stock-shovel robot model, self-filter, OCS2, dig controller, or policy.
- The live map no longer contains completed flange terrain.
- `desired_elevation` or `dig_zone` does not show the current target footprint.
- Nav2 costmaps retain completed flange occupancy.
- The workspace planner reports immediate completion while target cells still have remaining height.

Fix the failed condition and relaunch the stage. For a tool mismatch, stop the full stack and restart the 400 mm stack; do not patch one node in isolation.
