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

## Fast Machine Iteration

Machine time is scarce. Finish edits, focused tests and installation before the
next operator test; reuse verified session facts instead of repeating discovery.
Existing operator authorization covers the requested cycle and its ordinary
recovery. Do not ask again unless the intended motion or conditions change.

- Start by inspecting the current owner, latest failed BT leaf and selected
  profile. Reuse running low-level control, estimator and Foxglove. Restart only
  the sole application owner when the changed installed code requires it.
- **Register once, then keep the registration for controller retries.** A request
  to restart the cycle does not mean move the trench to a fresh BASE estimate.
  Preserve the validated placed plan, merged design/terrain bag and registration
  receipt outside `/tmp`. Run that frozen package with `plan.placement: fixed`;
  remove `plan.terrain_snapshot` because the frozen design already contains the
  terrain captured at placement. Keep the same site datum. A new location or
  operator-requested re-registration is a separate operation.
- On rslpc keep `auto_set_arm_controller_rt_prio=false` and retain the reviewed
  CPU affinity (MPC 1,2; arm 3). Verify installed launch defaults once after a
  relevant build. Blanket FIFO90 on all arm threads caused a ROS executor
  readiness loop to starve its peers on the shared core. Normal time-sharing
  scheduling is the supported arm default; do not restore the old boost while
  tuning MPC speed. See the OCS2 arm validation runbook's CPU placement section.
- Before releasing execution, check the actual machine interlocks/RPM, fresh
  state, `map` to the selected tool TF, current pose tolerance and recorder
  readiness. A stale-source SAFE_STOP needs normal lifecycle/owner recovery;
  newly fresh data alone does not prove the latch was cleared. Keep the guard.
- Start the canonical recorder for the test, not for a long build/debug wait.
  Check free space and observed growth. Full elevation-map capture consumed
  about 53 GiB during a no-motion debugging interval; a configured split is not
  a storage budget. Finalize while blocked, retain controller failure evidence,
  then start a new run before motion. Use `$dig-bag-recording` to verify splits.
- Follow actual BT transitions through move, DIG, pullup, dump and next DIG.
  A successful service response only accepts a request. Report the last
  completed stage and first failed leaf; never describe the full cycle as
  successful until those transitions occurred.

For an existing registered rslpc session, inspect the saved profile and receipt
under `~/Downloads/terra_machine_plans_20260925/site_registration_v2/current_base`
before preparing another placement. The frozen session profile is
`straight_frozen_local.yaml`, used by `start_terra.sh`; the older
`straight_current_base.yaml` would re-register on every start. Preserve newer
post-dig terrain when restarting the frozen package. These are session artifacts,
not a default site for future experiments.

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
For a normal site-backed profile, the wrapper passes `plan.design_map` to the
estimator, which loads the fixed reference named by the map metadata. Restart
the estimator when changing sites.

## Incremental Builds

After updating source, rebuild only the affected boundary before starting the
robot graph:

- Use the verified main robot workspace; do not create a second workspace or
  overlay only for an ordinary rebuild.
- For an isolated change, use
  `colcon build --packages-select <changed-package>`.
- When several Terra application packages changed, or the exact boundary is
  unclear within the Mole bringup dependency closure, use
  `colcon build --packages-up-to mole_bringup`.
- If a ROS message/service/action package or a C++ library ABI changed, use
  `colcon build --packages-above <changed-package>` so the package and its
  reverse dependents are rebuilt together.
- Add `--cmake-clean-cache` only to a package whose cache reports a different
  source directory or install mode. Do not use it for every build.
- Upstream Nav2 comes from `/opt/nav2_underlay`. Do not rebuild workspace Nav2
  sources unless `MOLE_ALLOW_NAV2_WORKSPACE_OVERLAY=true` is an intentional
  Nav2 development experiment.

Machine-workspace builds use copied installs; omit `--symlink-install`.
`mole_highlevel_controller_cpp` must install its trusted policy inventory and
models as regular files:

```bash
colcon build --packages-select mole_highlevel_controller_cpp \
  --cmake-clean-cache \
  --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
```

When converting that package from an older symlink install, CMake can leave
the old generated links as "up to date." Remove only its generated
`build/mole_highlevel_controller_cpp` and
`install/mole_highlevel_controller_cpp` directories, rebuild, and require
`test ! -L` for the installed policy inventory and selected model. Do not
clear the workspace or source tree.

Keep compile-time and runtime dependency prefixes consistent; in particular,
Terra's Nav2 dependencies must resolve from `/opt/nav2_underlay`. Let package
guards fail loudly, then clean only the affected package. Run one focused test
at the changed controller or ABI boundary and verify the installed resource,
not a broad workspace preflight.

For example, after changing `workspace_planner_msgs`, rebuild its actual ABI
consumers with:

```bash
colcon build --packages-above workspace_planner_msgs
```

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

Decision (2026-09-27): **one SDF refresh per arm motion**, then reuse that
snapshot throughout the motion. Terra must explicitly pass `sdf_auto_update=false`
and `sdf_max_age_sec=0`; the arm launch defaults alone are not the Terra contract.
Verify the live `tuning.terrainCollision.sdfAutoUpdate` parameter is false.
`auto_update_new_grid_map` rebuilds during motion are a configuration regression.
Keep mapping live and retain the successful post-request-map admission gate;
zero age expiration does not permit skipping that gate.

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

Capture the first failure and fix that boundary before retrying. Do not loop
through whole-stack restarts or relax an unrelated safety condition to get past
it. For a known intermittent failure, one clean retry is useful; recurrence
calls for evidence, not another identical restart.

**A hung arm:** if `ros:arm_stall_watch` is capturing, do not restart until it
prints `capture complete`. Inspect the capture, not just the last ROS log line.
On rslpc the helper is `current_base/arm_stall_watchdog.py`; its manual mode is
`python3 arm_stall_watchdog.py --capture-now PID`. It records threads, kernel
stacks, shared-memory mappings, log tail and GDB without commanding the robot.
The GDB step has a 180 s timeout. Check `kernel.yama.ptrace_scope` when attach is
rejected; it was set to 0 for this session and resets at reboot. Do not assume
an old permission failure still applies.

If GDB attachment to the main thread times out while one FIFO thread consumes a
core, wait for the original debugger to exit, identify the busy TID from
`/proc/PID/task/*/schedstat`, then capture that TID directly with bounded GDB.
This succeeded where main-thread attach could not. Do not run two debuggers or
change live priorities to unblock an armed controller: save the evidence, stop
the owner normally, and apply scheduling changes on its next start.

**Registration errors:** compare the loader's BASE sample and the planner's
later sample. Small yaw variation previously exceeded a 0.001 rad heading-search
margin; the current tested margin is 0.01 rad. Physical lane permission,
full-blade checks, station tolerance and the 0.15 rad maximum window still apply.
Do not re-register the trench to work around every controller restart. A raster
rotation failure during redundant re-registration is a reason to reuse the
already validated frozen placement, not to drop conflicting cells silently.


**Pullup stops near completion:** compare the last requested joint velocities
with measured motion and every actual completion gate before changing a timeout.
A taper can command velocities below the hydraulic deadband while positive
clearance error remains. The current shared pullup ramps to its configured lift
speed and retains it until measured completion or the finite height backstop.
Do not treat absence of measured motion as proof of collision or an empty bucket.
For `loaded_carry`, the loaded extraction curl window is distinct from the
ordinary `term_curled_enough` flag; false on that ordinary flag need not block
loaded completion. Whole-bucket clearance also differs from rear/tip clearance.
Inspect `term_bucket_clearance`, `term_close_threshold`, loaded cut/fill/curl
qualification and the command/measurement trace together. An armed dump action
waiting on a moving handoff is not evidence that dumping has started.

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
