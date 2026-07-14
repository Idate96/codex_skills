---
name: terra-trench
description: "Run or debug autonomous Beam6 Terra trenching. Use for flange/bottom stages, live target registration, generated workspaces, simulation or robot bringup, map handoff, and shovel changes."
---

# Terra Trench

Beam6 is a two-stage, application-specific workflow. Pair it with `newton-sim-ros-startup` for Newton, `ros2-debugging` for graph/TF diagnosis, and `dig-bag-recording` when a run artifact is required.

Work from the selected `moleworks_ros` repository root. Verify these current entrypoints before use:

- `high_level_planning/terra_planner/config/beam6_flange_bottom_sequence.yaml`
- `high_level_planning/terra_planner/applications/beam6_flange_bottom_sequence/README.md`
- `high_level_planning/terra_planner/scripts/generate_trench_sequence_plans.py`
- `high_level_planning/terra_planner/launch/beam6_sequence_stage.launch.py`

Read [references/two-stage-stl-workflow.md](references/two-stage-stl-workflow.md) for exact command shapes after selecting the active checkout. For live anchored runs, always prefer the generator-printed `apply_target` and `run_stage` commands over static examples.

## Core Contract

1. Start the base stack with `launch_terra:=false` and the stock 1300 mm shovel.
2. Park at the first station; verify `map -> BASE`, `map -> BASE_CONTROL`, `/mole/state`, and `/excavation_mapping/grid_map`.
3. Generate plans with `--live-first-base-frame BASE`. The generator uses `BASE` translation and `BASE_CONTROL` yaw and writes `station_anchor.json`.
4. Apply and run `flange6` with the stock shovel.
5. Save the live flange map, require a successful response, and resolve/validate the saved bag before stopping the flange stack.
6. Stop every process that embeds stock-shovel geometry: robot state publisher, self-filter, OCS2, DIG controller, workspace planner, and Terra executor.
7. Start/reseed Newton and ROS with `shovel_400mm_without_teeth` everywhere.
8. If Newton uses `--max-depth-layer desired_elevation`, materialize the bottom target into the saved flange terrain first and load the same bottom-target seed in Newton and ROS.
9. Return to the saved station pose and yaw before applying the bottom target.
10. Run `bottom6` with normal 400 mm excavation followed by grading cleanup.

Do not use `resume_from_checkpoint.py` to switch targets. Resume continues the same target and planner state; it is not a flange-to-bottom retarget mechanism.

## Registration And Tool Gates

- Target authoring frame: `BASE_CONTROL`, never live `CABIN_CONTROL` or raw `BASE`.
- Treat `/tmp/beam6_flange_bottom_sequence/station_anchor.json` as the bottom-stage registration contract.
- Do not move the base between plan generation and target application.
- Stock stage: `endeffector_type:=shovel`, blade width `1.3`, policy `r14c_tbar200_net1024_s213_3750`.
- Bottom stage: `endeffector_type:=shovel_400mm_without_teeth`, blade width `0.4`, policy `shovel400_r47_b6p60_toprbf_net256_s214_6000`.
- Bottom Dig3D boundary: `dig_3d_pullup_boundary_min_distance_m:=0.0` and `dig_3d_dig_zone_layer_name:=current_dig_workspace`.
- `dig_3d_desired_elevation_offset_m` is an observation bias only; it must not alter the target map used for completion or precision.

## Bottom-Stage Invariants

Use:

- `terra_start_delay:=15.0`
- `skip_navigation:=false`
- `grading_only:=false`
- `enable_grading_pass:=true`
- `grading_completion_mode:=local_open`
- `use_dig_zone_if_nonempty:=false`
- `workspace_dig_boundary_margin_m:=0.3`
- `grading_workspace_extension_m:=0.500`
- `workspace_completion_profile:=completion_foundation_strict.yaml`

Clear Nav2 costmaps after bottom target application.

## Verification And Stop Conditions

Before each stage, require the expected overlay, advancing `/clock` in simulation, finite map elevation, correct tool model/controller/policy, target/workspace overlap, and a planner action that is not spuriously `DONE`.

Before bottom, require:

- a successfully saved flange map;
- the same bottom-target seed loaded in Newton and ROS when desired-elevation clamping is active;
- a matching station translation and `BASE_CONTROL` yaw;
- no surviving stock-shovel process;
- no stale completed-flange occupancy in Nav2 costmaps.

Stop instead of patching live nodes when target application fails, registration moved, the flange terrain was lost, any process still uses the wrong tool, the planner reports completion with visible remaining material, or costmap clearing fails.
