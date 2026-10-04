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

Machine time is scarce. When the operator requests immediate hardware testing,
finish the affected build and installation, then perform essential live checks
and proceed under existing motion authorization. Offline test suites must not
block the requested robot slot: run them later or in parallel when they do not
compete with control, and report pending validation honestly. Preserve actual
interlocks, fresh state, command ownership, terrain admission and physical
limits. Reuse verified session facts instead of repeating discovery.
Existing operator authorization covers the requested cycle and its ordinary
recovery. Do not ask again unless the intended motion or conditions change.

- After a symlink-install of the controller package, check the installed Dig3D
  policy inventory: Terra requires a regular file. Materialize its exact reviewed
  bytes at the installed path if the build regenerated a symlink; do not weaken
  policy admission or change the selected model to get past this error.
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
- During idle debugging pauses after execution has stopped, set and verify 900 RPM
  when the operator requests reduced idle speed. Restore and verify 1600 RPM
  before releasing motion. Do not throttle down during an active move, DIG,
  pullup, dump, or momentary controller handoff.
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

## Direct Pullup-to-Dump Handoff

Current decision (2026-09-27): UGEP owns pullup to measured completion. Terra
then acknowledges STOP_DIG, disables DIG, finalizes carving, resumes map updates,
refreshes the SDF once, selects destination endpoints and resumes MPC through
its stopped-token handshake. DumpLeg gives MPC a fixed transport pose target;
MPC generates the connecting motion. Do not restore early concurrent DumpLeg,
PREPARE_EXTRACTION, moving-prefix trajectories or carry-splice deadlines.
MPC terrain/joint/source guards and measured opening gates remain in force.
Preopening starts within the 0.65 m measured dump zone, even with carry-angle
tracking error; do not require attaining the closed-angle preference before
commanding opening. This restores approach-overlapped opening. Destination
dumps admit only preopen and release endpoints (moleworks_ros `53d3c908a`):
there is no closed-entry waypoint. DumpLeg carries at the closed angle toward
the admitted preopen point, with the orientation target at that point's cabin
heading (`482a116fb`), and the perceptive MPC owns the route. A closed-entry
witness had rejected all 164 ranked sites by 2 mm once the spoil pile grew
near the reach limit. Do not restore it or loosen physical reserves instead.
Full release starts on fresh measured arrival within 0.10 m of the preopen
target, without an intermediate joint-rest or angle gate. Do not restore the
September 23 five-second no-progress watchdog: a valid .045 m/.17-degree
preopen arrival was aborted while waiting for rest. Keep the overall action
timeout, fresh state/contact ownership, physical limits and final release rest.
The 0.65 m approach-opening lead is separate from Terra's permitted deposition
mask; it never enlarges where soil may be deposited.

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

**Dump fails immediately after successful pullup:** distinguish reachable destination
admission from solver convergence. The September 23 loaded-carry relaxed-log
penalty caused every policy to be rejected when the measured bucket was about
22 degrees below the carry preference. The September 27 main-workspace fix uses
a moderate squared-hinge preference, retaining the requested closed angle and
all physical bounds. Inspect the first IPM failure's group/knot/row/margin;
policy staleness and the later no-progress timeout can be consequences, not the
cause. Preserve the recorded observation/target and compare the component
benchmark before moving the dump point or weakening limits.

**Registration errors:** compare the loader's BASE sample and the planner's
later sample. Small yaw variation previously exceeded a 0.001 rad heading-search
margin; the current tested margin is 0.01 rad. Physical lane permission,
full-blade checks, station tolerance and the 0.15 rad maximum window still apply.
Do not re-register the trench to work around every controller restart. A raster
rotation failure during redundant re-registration is a reason to reuse the
already validated frozen placement, not to drop conflicting cells silently.


**One pullup completion path (2026-09-27):** with shared vertical extraction
enabled, every scoop finishes through active pullup at measured whole-bucket,
rear/tip clearance, carry curl, lift and pullup reach. Fill does not select or
block a completion route. Do not add timeout-to-success, empty/full alternatives
or restore strict policy curl as an alternative pullup finish. The small-scoop
regression had 10.5% fill against a 20% loaded threshold while scripted curl
settled inside the carry window but outside ordinary strict curl. The correct
handoff is immediate when carry geometry qualifies, before the timeout; timeout
still fails when the physical pose does not qualify. Recording stays off when
the operator has asked to stop it, including subsequent retries.

**Pullup stops near completion:** compare the last requested joint velocities
with measured motion and every actual completion gate before changing a timeout.
A taper can command velocities below the hydraulic deadband while positive
clearance error remains. The current shared pullup ramps to its configured lift
speed and retains it until measured completion or the finite height backstop.
Do not treat absence of measured motion as proof of collision or an empty bucket.
For `loaded_carry`, the loaded extraction curl window is distinct from the
ordinary `term_curled_enough` flag; false on that ordinary flag need not block
loaded completion. Whole-bucket clearance also differs from rear/tip clearance.
Inspect `term_bucket_clearance`, `term_close_threshold`, carry curl, rear/tip
clearance, lift and the command/measurement trace together. An armed dump action
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

## September 27 completion and TF notes

Trench completion excludes the authored free-end 0.5 m and sidewall 0.1 m
bands, measured from the global trench geometry rather than workspace seams.
Use elevation sample centres to rasterize the required core; footprint overlap
otherwise puts samples from the aliasing allowance back into completion. After
rigid local registration, rebuild this completion partition from the transformed
authored geometry rather than forward-splatting required samples into the bands.
When changing the saved global core, also update every frozen waypoint
`dig_completion_mask` and `metadata.required_cells`; explicit workspace masks
are validated against the global core, not automatically clipped. Assert that
the union of station assignments still covers the entire new core. Plan arrays
are `[x, y]`; the GridMap codec arrays are `[y, x]`.
Keep the full digging permission and target elevations unchanged. A flat observed
bottom alone does not prove agreement with the authored depth.

A 0.375 s TF delivery gap aborted a nearly finished dump. MoveLeg/DumpLeg now
suspend phase decisions for TF older than the 0.25 s settle limit but no older
than 0.5 s, retaining the admitted target and resetting rest dwell. They never
use that stale pose to publish a new target, open, or finish. Longer gaps still
fail; source-state SAFE_STOP is unchanged.

### DIG moves slowly or leaves a high, flat trench bottom

Compare planner and DIG `desired_elevation` topic/layer and DIG offset first.
Check the selected policy's terrain sampling contract before tuning depth or
completion tolerances. V42's packaged training config has
`ros_controller_soil_sampling: false`: its 142D actor expects tip-anchored rows
inward along cabin X, not the older boom-offset ROS grid in the opposite row
order. The controller README owns the exact coordinates and normalization.
Verify the loaded library was restarted after an observation-contract fix.
Then inspect the depth shield's actual `projection_active`, `correction_norm`,
and `qp_fallback_count`; `boom_scale` reports the boom-only fallback even when
the QP owns the command, so it is not evidence of the commanded boom velocity.
The QP must use the local target-profile height at each cutting-edge point,
not the highest sample elsewhere in the profile. A higher bank or tilted
observation frame otherwise raises its effective floor above the local design.
V42's recorded training configuration has depth projection disabled, whereas
the robot may apply the ROS shield after the policy; do not assume runtime parity.
Keep recording off when requested; a bounded live diagnostic subscription is
sufficient to inspect intervention without starting a bag recorder.
Capture observations, raw policy actions, post-shield commands, measured joint
motion and target/tool TF together before attributing the residual to hydraulic
delay. Policy-publication latency alone does not measure hydraulic response.

## September 27 evening notes

- **Owner shell.** Start the owner from a fresh shell (`exec bash -l` in the
  pane, or a new window). A pane that once sourced an overlay keeps it in
  `AMENT_PREFIX_PATH`, and `start_terra.sh` inherits it; check the preflight
  `workspace_planner=` path after every start.
- **Frozen design bag topic.** The loader reads GridMap only on `grid_map` or
  `/grid_map`. A snapshot recorded from `/excavation_mapping/grid_map` aborts the
  mapper ("Design bag missing/unreadable"); rewrite the same bytes under
  `grid_map`. Terra checkpoints already use `grid_map`.
- **Resume.** Terra writes a checkpoint before every scoop under
  `~/.local/state/moleworks/terra/checkpoints/<map>/<session>/<pair>_<loop>/`
  (`checkpoint.yaml` + `excavation_map/`). Nothing reads it back
  (`resume_from_checkpoint.py` was removed on 2026-07-14). To resume, install the
  latest checkpoint's `excavation_map` as the frozen design; the executor restarts
  at waypoint 0 and planner progress memory resets.
- **Stale-policy SAFE_STOP.** It latches until the arm controller's lifecycle is
  reset; `/mole/terra_executor/restart` does that. Find the rejecting check
  first. On-time solves were being discarded for millimetre violations: fixed
  by the verifier's 2 cm terrain allowance and 5e-3 published-bound tolerance
  (moleworks_ros `0f70a6eeb`) plus IPM restoration within 5e-3 (ocs2 `ros2`
  `516c59c41`). Restoration failures on `group=state row=19` are the J_EE_PITCH
  upper stop during loaded carry.
- **Fake terrain in the loaded bucket.** Soil heaped above the bucket opening
  passed the 0.25 m self-filter pad and entered the height-only map, so the MPC
  saw the parked bucket 0.1-0.35 m inside terrain. SHOVEL_1300 z padding is now
  0.45 m (`9e73c1ef9`).
- **Dig reach.** V42 with the 1300 mm shovel tracks design depth only at
  r = 4.1-5.5 m from BASE in hard soil: pullup starts at 3.60-3.80 m, and entry
  at 6.25-6.56 m needs 0.75-1.0 m of travel. A station whose required core lies
  outside that band cannot reach COVERAGE_DONE; see
  `high_level_planning/terra_planner/docs/dig_reach_requirements.md`.
- **Tests and the live graph.** Never run ROS tests on domain 0 on the robot PC:
  a planner test published fake GridMaps on the live `/excavation_mapping/grid_map`.
- **CPU placement.** The MPC is pinned to CPUs 1-2 but not isolated; their SMT
  siblings 13-14 carry DDS, LiDAR backprojection and Foxglove. Suspect this for
  a silent >0.5 s solve gap.
- **Tracing.** `current_base/trace_dig_height.py` now records J_TURN. Use the
  encoder, not cabin TF, for turn smoothness; `map->CABIN` includes estimator yaw.

## September 29 notes

- **Navigation between stations: use the proper Nav2 stack.** Set
  `plan.navigation: {kind: nav2, ready_timeout_sec: 300.0}` in the machine profile;
  `terra.launch.py` then starts Nav2 itself with the hardware parameters
  (`mole_nav2_bringup/config/nav2_params_hw.yaml`) and converted plans already carry
  Nav2's goal tolerance as their station tolerance. Do not use `kind: dummy` on a
  straight trench: it only reverses in a straight line, never corrects forward or
  lateral error, and on 2026-09-29 each executor restart reversed further until the
  base sat 0.49 m behind its station. `manual` remains the operator-gate option.
- **Foxglove must never interfere with the machine.** The wrapper now starts the
  bridge with `scripts/foxglove_bridge_low_impact.yaml` (best-effort subscriptions on
  every topic, queue depth 1) at `nice 19`, pinned to `FOXGLOVE_CPUS` (default 20-23)
  on the machine. A default-QoS bridge viewing the 20-40 MB planner/excavation grid
  maps ran at 90% CPU, starved excavation mapping (80%), and caused a 9 s carve
  commit and a stale-terrain Dig3D abort. Restarting only the `foxglove` window is safe.
- **`is_using_gravis_commands` is unreliable** (Gravis confirmed); do not act on it.
  The Dig3D machine-command guard is now off by default
  (`machine_status_gate_abort_timeout_s: 0.0`): its hold-on-false turned a 1 Hz flag
  flicker into a 54 s frozen scoop. The reliable lock-out signal is
  `machine_specific_fields` `radio_control/upper_wagon_movement_release` (false while
  the machine withholds arm motion; returns ~2 s after commands go to zero). Keep the
  small `record_state`/`record_commands` bag running to give Gravis exact timestamps.
- **Resuming mid-trench.** The executor always starts at waypoint 0 and has no start
  index. To resume at station N without driving back, copy the frozen
  `plans/default.json` and drop the first 2*(N-1) waypoints (renumber `step`); before
  every owner restart install the newest checkpoint `excavation_map` as the frozen
  `<map>_design` (move the old one aside, do not delete). Paired dig/dump waypoints must
  carry identical `dig_permission_wkt`.
- **Other robustness changes (uncommitted as of 12:30 UTC):** a station completes when
  every required cell is observed, >=90% are within 5 cm and none is >0.15 m high;
  extraction finishes at its lift target instead of aborting and lifts to 0.5 m
  whole-bucket clearance for the MPC; MoveLeg/DumpLeg fail after 6 s without tool
  motion and the BT retries each once; the machine carve acknowledgement is 15 s; Dig3D
  keeps digging on map-registration TF up to 1 s old.

## October 4 notes (what worked on the machine)

- **Read the field contracts first:** `docs/src/subsystems/information/terra_field_contracts.md` in moleworks_ros.
  It pins the operator-approved planner, termination, pull-up, workspace-shape and metrics rules.

**Running the machine while agents edit or build the checkout**
- Hold the colcon lock for the whole run:
  `tmux new-window -d -t ros -n build_lock "flock /tmp/colcon_ros2_ws.lock sleep 14400; exec bash"`.
- Agents build with `flock /tmp/colcon_ros2_ws.lock colcon build ...`.
- `workspace_planner` is a symlink install, so the owner imports its Python at start. Agents must keep files
  importable at every save.
- Release the lock (Ctrl-C in `build_lock`) after the owner exits. Restart the whole owner to pick up builds.

**Release sequence** (worked at 13:35 and 15:29 UTC):
1. Poll `/machine_status`. `is_autonomy_switch_on` can read false for up to about 30 s after the operator flips it.
2. Run `set-engine-rpm 1600` once autonomy is on.
3. Check that `/mole/state` status is 1.
4. Compare `tf2_echo map BASE` with the plan station. Inside 0.35 m / 0.10 rad Terra logs `BASE already
   satisfies the station … skipping Nav2`.
5. Start the recorder and confirm it grows.
6. Start the `planner_watch` and `progress` windows on `<bag>/derived/planner_watch.log`.
7. Call `/mole/terra_executor/restart`.

**Run-event log**
- After the owner starts, `~/.local/state/moleworks/terra_runs/<time>_<profile>.jsonl` must exist.
- After the run, copy it into the bag's `derived/` and run
  `ros2 run workspace_planner workspace-planner-run-summary <jsonl> --csv <dir> --png <dir>`. It reports `seq`
  gaps per source. The collect run produced 54 events with no gaps.

**Ending a workspace on purpose:** `ros2 service call /mole/force_workspace_done std_srvs/srv/Trigger "{}"`.
- Planner services are `/<ns>/<name>`, not `/<ns>/workspace_planner/<name>`.
- Arm it only after `Loaded workspace`; before that it is refused.
- It acts on the next compute, so the current scoop finishes. The next workspace load clears it.
- Nav2 then drives to the next station: 2.1 m backwards took 13 s and arrived 0.345 m / +3.1° off.

**Stopping after N scoops:** wait for `scoop_progress: attempt=N ` in the planner_watch log, then call
`/mole/terra_executor/stop`. The owner exits; stop the recorder afterwards.

**Starting mid-plan:** `resume_from` refuses multi-station plans. Use a subset plan that starts at the next
workspace (see the September 29 notes); `radial_rect3_r1_ws23_site_20261004` is an example.

**Collect workspaces (`collect_dumped_soil`)**
- Use `free_lane_max_volume` only.
- Keep grading enabled; with it off, free-lane waits for a stationary-noise report.
- Check that the executor logs `Configured dig controller target_elevation_layer='original_elevation'` and that
  Dig3D's lock line shows `zone_layer='current_dig_workspace'`.

**V42 pull-up:** check `Locked pull-up distance X m (dig_zone=…, collision=…)` for every scoop. Terra profiles carry
`collision_guard_distance_margin_m: -0.2`, which keeps the guard at machine + 0.3 m.

**Foxglove**
- Excavation map: `/excavation_mapping/grid_map_rviz`.
- Nav2 obstacle maps: `/mole/global_costmap/costmap` and `/mole/local_costmap/costmap`, with Color mode set to
  Costmap and display frame `map`.
