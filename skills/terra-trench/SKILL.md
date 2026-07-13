---
name: terra-trench
description: "Run or debug autonomous Beam6 Terra trenching: flange/bottom stages, target registration, planning, simulation or robot bringup, and tool handoff."
---

# Terra Trench

## Use This With

Use this skill for Beam6-style autonomous trench runs where target geometry, live map state, workspace masks, controllers, and Terra execution must stay aligned across a staged dig.

Pair it with:

- `newton-sim-ros-startup` for the single-container Newton workflow.
- `ros2-debugging` when checking TF, topics, services, or action servers.
- `dig-bag-recording` if the run needs a rosbag artifact.

For the split Isaac/Terra stack, use the active repository runbook. The former
`moleworks-terra-stack` skill is not present in this workspace, so do not route
through it.

The current application is:

- Config: `high_level_planning/terra_planner/config/beam6_flange_bottom_sequence.yaml`
- Runbook: `high_level_planning/terra_planner/applications/beam6_flange_bottom_sequence/README.md`
- Generator: `high_level_planning/terra_planner/scripts/generate_trench_sequence_plans.py`
- Stage launcher: `high_level_planning/terra_planner/launch/beam6_sequence_stage.launch.py`
- Executor: `high_level_planning/terra_planner/launch/terra_executor.launch.py`

## Core Contract

Beam6 is a two-stage application-specific flow, not a generic Terra retarget:

1. Start simulation and the ROS base stack with the stock 1300 mm shovel.
2. Keep generic Terra disabled in the base bringup: `launch_terra:=false`.
3. Generate stage plans after TF is live and the machine is at the first station.
4. Apply and run the `flange6` target with the stock shovel.
5. Save the resulting flange terrain.
6. Stop every ROS-side process that embeds stock-shovel geometry.
7. Restart or reseed the simulation and ROS stack with the 400 mm shovel.
8. Return to the saved first station pose before applying bottom target geometry.
9. Apply and run the `bottom6` target with `shovel_400mm_without_teeth`.

Do not use `resume_from_checkpoint.py` to switch from flange to bottom. Checkpoint resume continues the same target and planner state; it is not a target-change mechanism.

Terminology: Beam6 "generated/manual workspaces" means the generator-created workspaces in `<stage>_terra_plan.json`. We still launch Terra, but through `beam6_sequence_stage.launch.py`, which wraps `terra_executor.launch.py` and runs `terra_execution.xml`. The generated plan provides the workspaces, and `LoadWorkspaceFromPlan` loads them at each waypoint. This is different from the ad hoc single-workspace YAML path (`single_workspace.launch.py`, `trench_workspace.launch.py`, or `single_workspace_executor.launch.py`).

## Robot Test Quick Path

On the robot, keep the same Beam6 ownership model as simulation: the base stack runs without generic Terra, and `beam6_sequence_stage.launch.py` owns the stage-specific workspace planner plus Terra executor.

1. Pull/build the current `moleworks_ros` `main` on the robot.
2. Start the normal robot/base stack with `launch_terra:=false` and `dig_3d_dig_zone_layer_name:=current_dig_workspace`.
3. Park at the first trench station, verify `map -> BASE`, `map -> BASE_CONTROL`, `/mole/state`, and `/excavation_mapping/grid_map`, then generate live-anchored plans with `generate_trench_sequence_plans.py --live-first-base-frame BASE`.
4. Use the generator-printed `apply_target` and `run_stage` commands. Do not hand-edit trench-axis coordinates unless this is an intentional zero-yaw smoke run.
5. For `flange6`, run the stock shovel stack: `endeffector_type:=shovel`, `workspace_planner_blade_width_m:=1.3`, `dig_3d_policy_id:=r14c_tbar200_net1024_s213_3750`, `dig_start_soil_carving_service:=excavation_mapping/start_target_clamped_soil_carving`.
6. After flange completes, save the live map with `/excavation_mapping/save_map` before stopping the flange stack.
7. Stop every process that embeds stock-shovel geometry: robot state publisher, self-filter, OCS2, Dig3D, workspace planner, and Terra executor.
8. Restart the robot/base stack with `shovel_400mm_without_teeth` everywhere: `workspace_planner_blade_width_m:=0.4`, `dig_3d_policy_id:=shovel400_r47_b6p60_toprbf_net256_s214_6000`, `dig_3d_pullup_boundary_min_distance_m:=0.0`, `dig_3d_dig_zone_layer_name:=current_dig_workspace`, and `move_leg_explicit_target_z_trench_mode:=true`.
9. Before bottom target application, compare live `map -> BASE` and `map -> BASE_CONTROL` against `/tmp/beam6_flange_bottom_sequence/station_anchor.json`; do not continue if the station or yaw moved.
10. Load the saved flange map into excavation mapping if needed, apply the generated bottom target, clear Nav2 costmaps, then run `bottom6`.

Bottom stage must use `terra_start_delay:=15.0`, `skip_navigation:=false`, `grading_only:=false`, `enable_grading_pass:=true`, `grading_completion_mode:=local_open`, `use_dig_zone_if_nonempty:=false`, `dig_zone_boundary_margin_m:=0.000`, `workspace_dig_boundary_margin_m:=0.3`, `grading_workspace_extension_m:=0.500`, and `workspace_completion_profile:=completion_foundation_strict.yaml`.

For Beam6 bottom Dig3D calibration, use the application config field
`dig_3d_desired_elevation_offset_m`; the generator copies it into
`bottom6_terra_plan.json` and Terra applies it to Dig3D before excavation
passes. This is an observation bias only. It must not change the
`desired_elevation` map used for completion or precision validation.

Stop instead of patching live nodes if Foxglove shows the wrong bucket, the bottom `desired_elevation` or `dig_zone` is wrong, Nav2 costmaps still contain stale flange occupancy, or the planner reports completion while visible bottom material remains.

## Frame And Geometry Registration

The target STLs are authored in `BASE_CONTROL`, a fixed child of `BASE` that coincides with `CABIN_CONTROL` when `J_TURN=0`. Do not apply Beam6 trench targets with live `CABIN_CONTROL` or raw `BASE`.

The live anchored generator uses two frames:

- `BASE` provides the first station translation in `map`.
- `BASE_CONTROL` provides the trench-axis yaw in `map`.

The generator writes `/tmp/beam6_flange_bottom_sequence/station_anchor.json`. Treat that file as the bottom-stage registration contract. The bottom target must be applied only after the robot is back at that saved station pose, within the tolerances in the file.

Current Beam6 placement rules:

- `map_apply.authoring_frame: BASE_CONTROL`
- `--mesh-anchor-x max`
- `--mesh-anchor-y origin`
- `--align-major-axis x`
- `--reference-mode local_min`
- `--mesh-reference-z max`
- The nominal farthest X is `7.0 m`, but live first-station anchoring shifts the current recipe to `6.375 m`.
- `plan.center_y_m` and `map_apply.mesh_y_m` must match.
- The generated Terra stages use `workspace_center_y_m=-0.274`, matching the fixed zero-turn control offset after live application.

`mesh_to_excavation_grid_map.py apply` freezes the target into `map` at apply time by looking up `map -> BASE_CONTROL`. If the base pose changes between generation and apply, the desired geometry and the precomputed Terra plan no longer match.

To apply Beam6 to a different live or saved map, first load that map into `/excavation_mapping/grid_map`, park at the first trench station for that map, verify `map -> BASE` and `map -> BASE_CONTROL`, then rerun `generate_trench_sequence_plans.py --live-first-base-frame BASE`. Use the newly printed `apply_target` and `run_stage` lines; do not reuse commands from another map or station.

## Simulation Run Order

Inside the ROS runtime environment:

```bash
cd /workspace/moleworks/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=24
export MW_EXPECTED_ROS_PREFIX=/workspace/moleworks/ros2_ws/install
```

Start the simulation with the relevant runtime skill. For split Isaac/Terra, bring up the Isaac side first, then start the ROS base stack with generic Terra disabled. The exact base launch can vary by current runbook, but the Beam6 invariant is:

```bash
launch_terra:=false
dig_3d_dig_zone_layer_name:=current_dig_workspace
```

Beam6 stage execution applies `current_dig_workspace` before each dig action.
Use that layer for Dig3D dynamic pull-up locking so the controller locks against
the active workspace boundary, not the full trench target.

For a direct Newton stock-shovel start, the analytical bucket geometry must match the flange stage:

```bash
--bucket-width 1.3 --bucket-y-sample-half-width 0.65
```

After `/clock`, TF, `/mole/state`, and `/excavation_mapping/grid_map` are live and the robot is at the first station, generate the live anchored plans:

```bash
python3 src/moleworks_ros/high_level_planning/terra_planner/scripts/generate_trench_sequence_plans.py \
  --config src/moleworks_ros/high_level_planning/terra_planner/config/beam6_flange_bottom_sequence.yaml \
  --output-root /tmp/beam6_flange_bottom_sequence \
  --live-first-base-frame BASE
```

Use the exact `apply_target` and `run_stage` commands printed by the generator. They contain map-frame trench-axis values after live anchoring. Do not substitute the zero-yaw examples unless the run is intentionally a smoke test at zero yaw.

Before applying each target, keep completed `dug_zone` bookkeeping cells out of Nav2 occupancy:

```bash
EM_NODE=$(ros2 node list | grep -E '(^|/)excavation_mapping$' | head -1)
test -n "$EM_NODE"
ros2 param set "$EM_NODE" occupancy_mark_dug_zone_occupied false
```

Flange stage:

- Apply the generated widened target, normally `/tmp/beam6_flange_bottom_sequence/flange6_target_width_1.300m.stl`.
- Use `endeffector_type:=shovel`.
- Use `workspace_planner_blade_width_m:=1.3`.
- Use policy `r14c_tbar200_net1024_s213_3750`.
- Use target-clamped soil carving service `excavation_mapping/start_target_clamped_soil_carving`.
- Use `trench_axis_finish_mask_mode:=target_depth`.
- Use `remaining_height_done_threshold_m_excavate:=0.05`.

After flange completes, save the live map while the flange stack is still alive:

```bash
BEAM6_CHECKPOINT_ROOT=/tmp/beam6_flange_bottom_sequence/checkpoints
BEAM6_FLANGE_BAG="$BEAM6_CHECKPOINT_ROOT/after_flange_$(date -u +%Y%m%d_%H%M%S)"

ros2 service call /excavation_mapping/save_map mole_excavation_mapping/srv/SaveGridMap \
  "{uri: '$BEAM6_FLANGE_BAG', topic: 'grid_map', storage_id: 'mcap', overwrite: false}"
```

Then stop stock-shovel robot state publisher, self-filter, OCS2, dig controller, workspace planner, and Terra executor panes.

If Newton is restarted for bottom with `--max-depth-layer desired_elevation`, do not dig from the flange-only saved map. First materialize the bottom target into the saved flange terrain from a temporary 400 mm alignment stack by running the generated bottom `apply_target` command with `--dry-run-output <bottom-target-seed-bag> --dry-run-storage-id mcap`. Then restart Newton with the bottom-target seed MCAP and restart ROS excavation mapping with the bottom-target seed bag. The flange map is the source terrain; the bottom-target seed is the map that must be loaded by both Newton and ROS for bottom digging.

Find the latest saved flange map:

```bash
eval "$(
  python3 src/moleworks_ros/high_level_planning/terra_planner/applications/beam6_flange_bottom_sequence/beam6_latest_map.py \
    --root /tmp/beam6_flange_bottom_sequence/checkpoints \
    --format shell
)"

test -n "$BEAM6_LATEST_MAP_BAG"
test -n "$BEAM6_LATEST_MAP_MCAP"
```

Use the saved flange map only to create a bottom-target seed before the real bottom dig when Newton uses a desired-elevation clamp. Do not point Newton at `$BEAM6_LATEST_MAP_MCAP` and then apply the bottom target only on the ROS side; that leaves Newton clamped to the flange target and can make the workspace planner stall with soil still remaining.

Restart the bottom stack with the 400 mm tool propagated everywhere:

```bash
endeffector_type:=shovel_400mm_without_teeth
workspace_planner_blade_width_m:=0.4
dig_3d_policy_id:=shovel400_r47_b6p60_toprbf_net256_s214_6000
grading_only:=false
enable_grading_pass:=true
dig_3d_pullup_boundary_min_distance_m:=0.0
dig_3d_dig_zone_layer_name:=current_dig_workspace
move_leg_explicit_target_z_trench_mode:=true
```

For Beam6 bottom, do not keep the generic 3.1 m Dig3D pull-up floor. Use
`dig_3d_pullup_boundary_min_distance_m:=0.0` so the dynamic boundary lock is the
max of the active workspace boundary candidate and the machine-collision
candidate. Use `dig_3d_dig_zone_layer_name:=current_dig_workspace` so Dig3D
does not lock against stale or future cells from the full trench target.

For direct Newton restart, match the robot variant and analytical bucket:

```bash
--robot-variant m445_terra_400mm_without_teeth --bucket-width 0.4 --bucket-y-sample-half-width 0.2
```

Before bottom apply, verify the current station:

```bash
cat /tmp/beam6_flange_bottom_sequence/station_anchor.json
timeout 15 bash -lc 'ros2 run tf2_ros tf2_echo map BASE 2>&1' | head -40
timeout 15 bash -lc 'ros2 run tf2_ros tf2_echo map BASE_CONTROL 2>&1' | head -40
```

Reload the saved flange map into ROS excavation mapping if needed:

```bash
python3 src/moleworks_ros/high_level_planning/terra_planner/applications/beam6_flange_bottom_sequence/beam6_latest_map.py \
  --root /tmp/beam6_flange_bottom_sequence/checkpoints \
  --load \
  --load-service /excavation_mapping/load_excavation_map
```

Bottom stage:

- Apply the generated bottom target from the original Beam6 STL path printed by the generator.
- Use `endeffector_type:=shovel_400mm_without_teeth`.
- Use `workspace_planner_blade_width_m:=0.4`.
- Use `workspace_planner_mode:=1`.
- Use `workspace_planner_profile:=stripwise_max_volume`.
- Use target/legal half widths `0.2` and `0.3`.
- Use `workspace_dig_boundary_margin_m:=0.3`.
- Use `grading_only:=false` and leave `enable_grading_pass:=true` so the 400 mm stage performs normal Dig3D excavation first, then a grading cleanup pass.
- Use target-clamped soil carving service `excavation_mapping/start_target_clamped_soil_carving`.
- Use policy `shovel400_r47_b6p60_toprbf_net256_s214_6000`.
- In simulation, do not skip navigation: use `skip_navigation:=false`.
- For the open trench bottom pass, use `grading_completion_mode:=local_open`, not `dump`.
- Until `beam6_sequence_stage.launch.py` has an explicit workspace-service readiness gate, use `terra_start_delay:=15.0` for full-navigation runs. Earlier skip-navigation runs implicitly delayed long enough for `LoadWorkspaceFromPlan`; full-navigation runs can reach `/mole/load_workspace` much sooner.

Clear Nav2 costmaps after bottom target apply:

```bash
ros2 service call /mole/global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
ros2 service call /mole/local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
```

## Activation Chain

`beam6_sequence_stage.launch.py` starts the stage-specific runtime after the base stack is already up:

- It validates the active ROS overlay through `MW_EXPECTED_ROS_PREFIX` or `expected_ros_prefix`.
- It starts `workspace_planner_server` under the robot namespace.
- It passes the generated stage plan to `terra_executor.launch.py`.
- It forwards trench-axis metadata, blade width, finish-mask mode, remaining-height threshold, carving service, checkpoint root, tool type, bucket frame, and dig policy.
- It should run after the base stack, Nav2, excavation mapping, and the workspace planner's map subscriptions have had enough time to settle. For now, pass `terra_start_delay:=15.0` on full-navigation Beam6 runs.

The normal non-single Terra behavior tree is `terra_execution.xml`:

1. Disable Ackermann and dig controller, configure dig backend, reset arm MPC.
2. For each waypoint, navigate unless `skip_navigation` is active.
3. Load the next workspace from the generated plan.
4. Apply map-fixed dig and dump masks through excavation mapping.
5. Load trench-axis metadata into the workspace planner.
6. Ask the workspace planner for the next action.
7. For `DIG_PASS`, enable arm MPC, move the arm, configure dig controller target layer and policy, lifecycle-activate the selected dig controller, start soil carving, disable upstream map updates, run `/mole/run_dig_3d`, finish carving, re-enable updates, deactivate the controller, recompute terrain SDF, dump/spawn soil, mark dump area, checkpoint, and loop.
8. For `GRADE_PASS`, run the grading action, checkpoint, and loop.
9. On workspace completion, commit `dug_zone`, checkpoint, and advance to the next waypoint.

The dig action itself sends an empty `mole_highlevel_msgs/action/RunAction` goal. Pull length, target layer, and policy selection are owned by the controller and the planner state, not by the action goal.

## Verification Checklist

Before plan generation:

- `/clock` advances in sim time.
- `ros2 pkg prefix terra_planner` resolves to the expected overlay.
- `map -> BASE` and `map -> BASE_CONTROL` are available.
- The robot is at the intended first station.
- `/excavation_mapping/grid_map` publishes finite `elevation`.

Before each stage:

- The active robot model, self-filter, OCS2, and dig controller match the target shovel.
- `desired_elevation` exists and is lower than `elevation` inside the target.
- `dig_zone` matches the current target footprint.
- Workspace masks overlap the current `dig_zone`.
- Nav2 costmaps have no stale completed-flange occupancy.
- The first planner action is not immediate `DONE` unless the run intentionally starts from completed terrain.

Before bottom:

- A flange map was saved successfully.
- Newton, if restarted with a desired-elevation clamp, uses the materialized bottom-target seed map, and ROS excavation mapping loads the same seed bag.
- `station_anchor.json` exists.
- `map -> BASE` matches the saved station within tolerance.
- `map -> BASE_CONTROL` yaw matches the generated axis yaw.
- No stock-shovel process remains alive.

Stop instead of continuing if any of these fail:

- Target application fails.
- The current station does not match `station_anchor.json`.
- The live map lost the flange terrain before bottom.
- The bottom stack still uses stock-shovel geometry or policy.
- The workspace planner immediately reports bottom complete with obvious remaining height.
- Nav2 clear services fail or `/map` still contains stale flange occupancy.

## References

Use `references/two-stage-stl-workflow.md` for exact Beam6 command shapes and zero-yaw examples. Always prefer the commands printed by `generate_trench_sequence_plans.py` for live anchored runs.
