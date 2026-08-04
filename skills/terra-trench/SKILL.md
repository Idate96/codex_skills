---
name: terra-trench
description: "Prepare or troubleshoot the Beam6 two-stage Terra trench application. Use for live stage-manifest generation, flange-to-bottom handoff, station registration, or tool changes; route stack execution through terra-pipeline."
---

# Terra Trench

Treat Beam6 as application data for the generic Terra trench owner. Use `$terra-pipeline` to start,
monitor, or stop a generated stage. Use `$ros2-debugging` for read-only graph/TF diagnosis and
`$dig-bag-recording` for run evidence.

## Source Of Truth

Work from the selected `moleworks_ros` repository root and read the current application runbook:

- `high_level_planning/terra_planner/config/beam6_flange_bottom_sequence.yaml`
- `high_level_planning/terra_planner/applications/beam6_flange_bottom_sequence/README.md`
- `high_level_planning/terra_planner/scripts/generate_trench_sequence_plans.py`
- `mole_bringup/config/terra/trench.yaml`
- `description/mole_description/config/bucket_specs.yaml`

The application README owns the exact generation, receipt binding, snapshot, and stage-transition
commands. Do not copy its launch arguments into this skill.

## Current Contract

1. Start only low-level control and `mole_estimator` as machine prerequisites with the effective
   stage tool. Do not start separate perception, mapping, controller, planner, or executor owners.
2. Park at the first station and verify `map -> BASE` and `map -> BASE_CONTROL`.
3. Generate one immutable schema-v4 bundle with explicit `--base-template`, `--bucket-specs`, and
   `--live-first-base-frame BASE`.
4. Resolve `GENERATION_ROOT` from `current` once and retain that immutable path for both stages.
   Machine launch rejects the mutable `current` symlink and offline manifests.
5. Run `flange6_stage.json` through `$terra-pipeline` with `autostart=false`.
6. Save the flange map and bind it with `bind_trench_handoff.py`; require the receipt before stop.
7. Stop the complete flange owner and both tool-dependent prerequisites, change the physical tool,
   then restart prerequisites with `shovel_400mm_without_teeth`.
8. Run `bottom6_stage.json` with the bound flange bag, or first create the documented target
   snapshot when a simulator must boot from bottom-target terrain.

The recipe owns tool geometry, policy IDs, navigation, workspace, soil carving, grading, and
completion behavior. Change the owning YAML and regenerate; do not recreate removed per-component
launch arguments. `beam6_sequence_stage.launch.py` and the old `/tmp` loose-plan workflow are not
current entrypoints.

## Stop Conditions

Stop at the public owner boundary when manifest/receipt validation fails, station pose changes,
tool/TF/policy differs from the manifest, flange terrain is missing, overlays are mixed, or duplicate
high-level owners exist. Do not patch individual live nodes around worker validation.
