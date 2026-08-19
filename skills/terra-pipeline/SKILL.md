---
name: terra-pipeline
description: Start, resume, monitor, or stop the current Moleworks Terra application on robot or simulation. Use for profile-driven normal Terra, manifest-driven generated trenches, low-level and estimator prerequisites, one-shot terrain-SDF refresh, Foxglove, and single-owner recovery.
---

# Terra Pipeline

Use one current public owner:

- normal Terra: `mole_bringup terra.launch.py` with one reviewed schema-v3 profile
- generated trench: `mole_bringup trench.launch.py` with one immutable stage manifest

Do not assemble perception, navigation, Dig3D, OCS2, workspace planning, or the
executor beside that owner. The refactored public launch owns the complete
high-level graph. On the machine, start only low-level control and
`mole_estimator` as prerequisites.

## Runtime Image and DDS

- Use `rslheap/moleworks_ros:latest` by default. Record `rslheap/moleworks_ros:sha-<merge-sha>` when exact image provenance matters.
- A fresh remote shell gives normal nodes the runtime DDS CLIENT profile and the ROS 2 CLI daemon the observer SUPER_CLIENT profile. Run plain `ros2 ...` commands.
- The canonical owner scopes its Foxglove child to observer DDS. Record with `dig-bag-recording` or `mole_bag_tools rosbag_record.launch.py`; the canonical recorder scopes only its bag children to observer DDS.
- Do not paste Fast DDS exports around commands. If the runtime/observer variables are missing or legacy discovery variables remain, pull the published image, recreate the container, and open fresh tmux panes.

## Machine Contract

- Treat stack startup as non-motion authorization. Keep `autostart:=false` until the operator approves execution and all interlocks, TF, tool, target, controller ownership, and map checks pass.
- Use the same reviewed effective tool for low-level, estimator, and the Terra profile or trench stage.
- Keep exactly one public Terra/trench owner. Stop it before changing profile, stage, or owner type.
- `/mole/terra_executor/restart` starts or restarts execution; it is not an abort service.
- While Terra is waiting at the non-motion manual-navigation gate, do not fuzzy-correct an ambiguous operator phrase such as `we top now` into a stop request. Treat it as a possible position update, keep the gate closed, inspect the reported distance, and ask one short clarification only if the intent remains unclear. Stop only for an explicit `stop`, `halt`, `shutdown`, emergency-stop request, or an unsafe condition.
- For an unsafe motion, use the physical emergency stop and machine procedure. For a controlled stop, make the machine safe and press `Ctrl-C` once in the owner pane.

## Tmux Start

Normal Terra on the machine:

```bash
TERRA_PROFILE="$(ros2 pkg prefix --share mole_bringup)/config/terra/default.yaml"
rg -n 'profile_id:|tool:|policy_id:|recompute_terrain_sdf_on_target:' "${TERRA_PROFILE}"

~/.codex/skills/terra-pipeline/scripts/terra_pipeline_tmux.sh \
  --application normal \
  --profile "${TERRA_PROFILE}" \
  --runtime-mode machine \
  --effective-tool shovel \
  --autostart false \
  --visualization foxglove \
  --attach
```

The packaged profile is only a resolvable example. Copy and review it for a
materially different run. Profiles do not inherit and the public launch does
not accept removed component-level arguments.

Generated trench on the machine:

```bash
: "${STAGE_MANIFEST:?Set STAGE_MANIFEST to a reviewed immutable stage manifest}"

~/.codex/skills/terra-pipeline/scripts/terra_pipeline_tmux.sh \
  --application trench \
  --stage-manifest "${STAGE_MANIFEST}" \
  --runtime-mode machine \
  --effective-tool shovel \
  --autostart false \
  --visualization foxglove \
  --attach
```

Add `--handoff-bag /path/to/bag` only for a reviewed trench handoff. If
low-level and the estimator already run under another maintained startup
workflow, add `--owner-only` to avoid duplicate prerequisites.

The script uses three core windows in tmux session `ros`:

- `low_level`: machine low-level control, unless simulation or `--owner-only`
- `estimator`: machine estimator, unless simulation or `--owner-only`
- `terra_owner`: the sole `terra.launch.py` or `trench.launch.py` owner

When `--visualization foxglove` or `rviz_foxglove` is selected, the script adds
a dedicated `foxglove` window. The Terra owner receives `none` or `rviz`,
respectively, and the separate bridge uses the observer DDS profile and the
profile's configured port. The wrapper starts it last, after a short grace for
the application graph. Restarting that window does not restart perception,
mapping, planning, or control. The window remains part of the one canonical
Terra workflow; do not start another bridge beside it.

It starts only idle windows and never kills an existing process. Make the owner
safe and stop it manually before relaunching a changed configuration.

## Single Local Workspace

Use the packaged `local_workspace.yaml` path for workspace-planner-only DIG
experiments. Terra owns perception and, after the first finite map arrives,
authors the configured workspace from the current `BASE` pose and freezes both
runtime geometry and the generated waypoint pair in `map`. Keep machine
`autostart=false`. Do not add a preparation service or defer target application:
the visible target is what lets the operator move the cabin/bucket for complete
coverage without driving the base.

After the coverage sweep, require a stable `map <- BASE` transform. Then start
recording and call `/mole/terra_executor/restart`. With
`plan.navigation.kind: manual`, Terra starts no Nav2 process and sends no
chassis command; `/mole/manual_navigation_done` is the explicit operator gate
and checks that the base remains near the map pose captured at registration. If
estimator drift makes the check fail, do not navigate or widen the tolerance.
Stop the run, recover the estimator, and restart the owner so the one-shot
runtime profile and plan are registered together again.

## Terrain SDF Refresh

Keep `recompute_terrain_sdf_on_target: false` in the reviewed profile or stage.
That disables automatic rebuilds on target updates and policy execution. Terra's
separate `recompute_terrain_sdf_before_arm_motion` behavior-tree gate remains
enabled.

For normal Terra cycles, let the behavior tree own the explicit service calls:

- DIG pass before move: refresh the current mapped terrain, then enable arm MPC and run the dig move leg.
- DIG pass before dump: refresh again after soil carving finishes and mapping updates resume, then enable arm MPC and run the dump leg.
- GRADE pass before move: refresh before the grade-start move leg.
- GRADE pass ending in DUMP: refresh after the grade pass and immediately before the dump leg.

These are the terrain-change/arm-motion boundaries; they are not policy-loop
work. Do not add another service call for every policy step or target. Confirm
the executor reports a successful `RecomputeTerrainSdf` action at each
applicable gate; a skipped or failed action is a blocker before that arm motion.

For standalone OCS2 or recovery outside the Terra owner, wait for the intended
map snapshot and call the service once before the next terrain-constrained arm
motion:

```bash
timeout 10 ros2 topic echo /excavation_mapping/grid_map --once >/dev/null
ros2 service call \
  /mole/mobile_manipulator_mpc_node/terrain_collision/recompute_sdf \
  std_srvs/srv/Trigger \
  "{}"
```

## Pre-Release Checks

Use plain commands from a fresh shell:

```bash
show_dds_mode
ros2 topic info /mole/actuator_commands -v
timeout 20 ros2 run tf2_ros tf2_echo map BASE
ros2 lifecycle get /mole/dig_3d_controller
ros2 lifecycle get /mole/mole_arm_mpc_controller
ros2 node list | rg '/mole/terra_executor|mole_arm_mpc_controller|dig_3d_controller'
```

Record the run before releasing execution. Then, only with operator approval:

```bash
ros2 service call /mole/terra_executor/restart std_srvs/srv/Trigger "{}"
# Wait until Terra reports an active manual gate and an in-tolerance pose.
ros2 service call /mole/manual_navigation_done std_srvs/srv/Trigger "{}"
```

Do not derive the gate service from the restart service hierarchy: the endpoint
is `/<robot_namespace>/manual_navigation_done`, not
`/<robot_namespace>/terra_executor/manual_navigation_done`. After calling it,
require the owner log to report `Manual navigation completion accepted` before
assuming execution advanced.

## Failure Triage

When Terra stops after a scoop starts, read the first executor failure and the
latest Dig3D termination blocker before changing geometry or restarting. A
`Dig action timed out after <N>s` failure means the executor deadline expired;
it is not evidence that a safety margin fired. Check whether Dig3D was still
publishing commands and which completion gate remained false (for example
`curl`, `back_clearance`, or `finish_pose`). Preserve the recording and report
that blocker before deciding whether to change the action timeout or controller
termination logic.

## Recording

Use `$dig-bag-recording` for the normal full split. For a narrow OCS2 evidence
run, use the canonical `record_ocs2:=true` split described by
`$ocs2-arm-experiments`. Do not use raw remote `ros2 bag record` with a manual
DDS environment.

## Resource

- Script: `scripts/terra_pipeline_tmux.sh`
