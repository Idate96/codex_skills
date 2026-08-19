---
name: robot-startup
description: "Start or restart real-robot Moleworks ROS tmux prerequisites, add Nav2 on top of verified running prerequisites, or launch an optional standalone perception/DIG stack. Use for bounded machine bringup, Nav2-only robot startup, tool/namespace/TF selection, hydraulic unlock, or authorized engine-RPM setup."
---

# Robot Startup

Use the bundled tmux wrapper for direct robot bringup. Use `terra-pipeline` for
a Terra application, `dig-controllers` when the prerequisites already run, and
`set-engine-rpm` for RPM-only work.

For repeated manual-navigation Terra debugging where perception must survive
controller or planner rebuilds, use the enumerated `terra_research.launch.py`
components described below. This is the only supported split Terra graph.

## Authority And Sources

- A startup request authorizes software bringup only.
- Unlock hydraulics or change RPM only when the user explicitly asks to make
  the machine ready, unlock it, or set an RPM. Keep an operator present.
- Inspect the existing `ros` tmux session and command owners before changing
  anything. Use `--restart` only when a clean restart is explicitly in scope.
- Read `<repo>/docs/robot_agent/ROBOT_OPERATIONS_GUIDE.md` for current machine
  prerequisites and shutdown, then confirm launch arguments in the owning
  package README/launch file. Do not copy an old quick-reference command.
- Use the sourced environment that owns the live graph. The current real-robot
  image is ROS 2 Jazzy; source the selected workspace's `install/setup.bash`
  and verify `ROS_DISTRO=jazzy` rather than hard-coding a Humble underlay.
  On a robot installation this may be the host workspace; a container is not a
  universal requirement.

## Start The Stack

Resolve the installed skill path and run the wrapper:

```bash
STARTUP="${CODEX_HOME:-$HOME/.codex}/skills/robot-startup/scripts/robot_startup_tmux.sh"
: "${ROBOT_WS:?Set ROBOT_WS to the workspace verified from the owning tmux/process environment}"
"$STARTUP" --ws "$ROBOT_WS"
```

The default managed layout is `low_level`, `perception`, `estimator`, then
`foxglove`. It uses `mole`, an empty TF prefix, the geometry-only `shovel`
tool, and `mapping_profile:=local`. Useful explicit options are:

- `--attach`, `--restart`, `--session <name>`, `--ws <path>`
- `--robot-namespace mole --tf-prefix <prefix>`
- `--endeffector-type <geometry-only-tool>`; never pass a retired
  `*_calibrated` value
- `--mapping-profile <analytical|local|site>` and
  `--design-map-name <name>` for a design-backed profile
- `--no-perception` when the later application owner supplies perception
- `--no-estimator` only for an intentional inspection/low-level diagnostic;
  machine applications require estimator readiness
- `--no-foxglove` when the application workflow supplies visualization
- `--dig-controller <dig3d|newton|dig|dig-ee>` to launch a standalone DIG
  controller without lifecycle activation; after state, TF, map, and publisher checks, use
  `dig-controllers` for the separately authorized activation/action boundary
- `--nav2-only` to add a `nav2` pane while preserving and reusing matching
  `low_level`, `estimator`, and `perception` panes in the existing session
- `--nav2-overlay-ws <path>` with `--nav2-only` to source one intentionally
  built development overlay after the image-pinned `/opt/nav2_underlay`

Keep the same effective tool, namespace, and TF prefix across low level,
estimator, perception, and the application. For Terra, follow its owning skill
and operator guide: Terra owns enabled perception and its tmux workflow manages
one dedicated `foxglove` window, so do not prestart either component here. The
bridge uses the observer `SUPER_CLIENT` DDS profile; normal robot and Terra
nodes remain runtime `CLIENT` participants. Restarting only the Foxglove pane
must not restart perception or clear the accumulated map.

Without `--restart`, the wrapper preflights every managed pane, starts only
idle panes, and refuses a busy pane whose owner is wrong or unknown. It never
sends Ctrl+C or respawns a busy process implicitly. For a standalone stack it
starts Foxglove last, in its own pane, after earlier requested launches are
present, and scopes only that bridge to the observer `SUPER_CLIENT` profile.
`--restart` kills and recreates the named tmux session. It does not change
global tmux continuum settings.

A matching pane role is not proof of matching configuration. Before relying on
an accepted busy pane, compare its working directory, process environment, and
launch arguments with the selected workspace, ROS domain, tool, namespace, TF
prefix, and profile. Stop on ambiguity; do not start the missing panes beside a
mixed stack.

For `--nav2-only`, first verify `MoleState.STATUS_OK`, `map` to the effective
`BASE` transform, the configured filtered front-cloud topic, and absence of an
existing Nav2 action server. The wrapper checks tmux ownership, preserves every
busy prerequisite pane, and launches `mole_bringup nav2_on_robot.launch.py`,
which owns one unconfigured Ackermann controller wired to the collision
monitor's guarded output plus the Nav2 stack. Stop any existing application
owner such as Terra before using this entry. The wrapper does not use ROS graph
discovery as a launch gate and does not configure or activate the drive
controller. Use the optional overlay only for a reviewed focused build;
`nav2_core` must still resolve from `/opt/nav2_underlay`.

## Mapping Contract

Use `local` for a standalone live local map. `site` and `analytical` are
design-backed and require an explicit saved design map. Do not load a site map
silently.

For Terra's single local-workspace experiment, let the Terra owner start
perception and apply the configured target as soon as the first finite map is
available. The target is authored from the current `BASE` pose and frozen in
`map`; do not delay target application until after the coverage sweep. Once the
target is visible, the operator may move the cabin/bucket to expose occluded
terrain without driving the base. Require a stable `map <- BASE` transform
before execution. Route the executor release and manual no-Nav2 acknowledgement
through `terra-pipeline`.

## Split Terra Research Loop

Use this only for a machine profile with `plan.navigation.kind: manual`. Never
run these components beside `terra.launch.py` or `trench.launch.py`. Keep the
verified `low_level` and `estimator` prerequisites, then give every research
owner its own window in the existing `ros` session:

- `perception`: LiDAR, filtering, elevation mapping, and excavation mapping
- `dig3d`: Dig3D controller only
- `ocs2`: arm MPC, move-leg, dump-leg, and OCS2 diagnostics
- `workspace`: workspace planner; for `local_workspace` it also applies the
  configured geometry and materializes the plan
- `terra`: inactive drive controller and Terra executor only
- `foxglove`: observer bridge, started last after all requested owners are up

Resolve one reviewed profile and plan path, source the main robot workspace,
and launch the components in the order above. For a packaged plan, `PLAN` is
the reviewed packaged plan JSON. For `local_workspace`, `PLAN` is the output
path that the `workspace` component materializes; do not start `terra` until it
exists.

```bash
: "${ROBOT_WS:?Set ROBOT_WS to the verified main workspace}"
: "${PROFILE:?Set PROFILE to the reviewed manual Terra profile}"
: "${PLAN:?Set PLAN to the reviewed or materialized plan JSON}"

for component in perception dig3d ocs2 workspace terra; do
  tmux new-window -d -t ros -n "$component" \
    "cd '$ROBOT_WS' && source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
     ros2 launch mole_bringup terra_research.launch.py profile:='$PROFILE' \
       component:='$component' plan_path:='$PLAN'; exec bash"
done
```

Create and verify one component at a time in that order; the compact loop is a
command template, not permission to ignore a busy or mismatched window. A
packaged-plan `workspace` owner does not reapply geometry. Restart only the
window whose installed code changed. Restarting `dig3d`, `ocs2`, `workspace`,
or `terra` must leave `perception` running so the accumulated elevation map is
preserved. Restart the dedicated Foxglove bridge after the owners are stable;
scope only that bridge to the observer `SUPER_CLIENT` profile. Release the
manual gate only through `/mole/manual_navigation_done`, after the executor has
reported the expected target pose and all machine preflight checks pass.

Apply runtime desired geometry through
`/excavation_mapping/apply_runtime_profile` only when the user requested a
target. Supported profile types are `constant`, `slanted`, `trench`, and
`polar_sector`. The service accepts one profile per excavation-mapping
lifetime; restart that owner before applying a different target. Use the
owning perception README/service definition for the request fields.

## Optional Machine-Ready Actions

Continue only after explicit user intent. Use 1600 RPM as the standard
autonomous target when the operator asks to raise RPM or make the machine ready
without naming a value. An explicit numeric target overrides it. Never apply
an RPM change for generic software startup. Keep automatic startup targets in
the maintained 900-1600 RPM envelope. The 1600 target requires the direct
`SetRPM` service because the stepwise `/engine_speed` fallback stops at 1500;
the owning wrapper enforces that distinction. A target above 1600 requires a
separately reviewed interface-specific workflow.

1. Wait at most 60 seconds for `/machine_status` and `/hydraulic_lock`.
2. Read one `machine_msgs/msg/MachineStatus` sample and require all of:
   `is_armrest_unlocked`, `is_radio_estop_unlocked`,
   `is_manual_operation_unlocked`, and `is_autonomy_switch_on`.
3. Call `/hydraulic_lock` with `std_srvs/srv/SetBool {data: true}` only after
   those gates pass.
4. Re-read status and require both `is_hydraulilock_unlocked` and
   `is_autonomous_operation_unlocked`.
5. If RPM was requested, invoke `set-engine-rpm` with the explicit or standard
   target and report measured readback.

Use bounded read-only preflight commands before the authorized service call:

```bash
timeout 60 bash -lc 'until ros2 topic list | grep -Fxq /machine_status; do sleep 1; done'
timeout 60 bash -lc 'until ros2 service list | grep -Fxq /hydraulic_lock; do sleep 1; done'
status="$(timeout 10 ros2 topic echo /machine_status machine_msgs/msg/MachineStatus --once)"
grep -F 'is_armrest_unlocked: true' <<<"$status"
grep -F 'is_radio_estop_unlocked: true' <<<"$status"
grep -F 'is_manual_operation_unlocked: true' <<<"$status"
grep -F 'is_autonomy_switch_on: true' <<<"$status"
```

After explicit authorization and successful gates:

```bash
timeout 20 ros2 service call /hydraulic_lock std_srvs/srv/SetBool '{data: true}'
status="$(timeout 10 ros2 topic echo /machine_status machine_msgs/msg/MachineStatus --once)"
grep -F 'is_hydraulilock_unlocked: true' <<<"$status"
grep -F 'is_autonomous_operation_unlocked: true' <<<"$status"
```

Stop at the first failed prerequisite, missing endpoint, or failed readback. A
zero ROS command is not an emergency stop and is not part of startup recovery.

## Verify

Verify requested tmux panes, publisher ownership, estimator `STATUS_OK`, and
the `map` to effective `BASE`/tool TFs using the operator guide. For a
machine-ready request, report hydraulic state and measured RPM, not only
successful service calls. Use `ros2-debugging` for read-only diagnosis.

## Focused Rebuild Handoff

Use the verified main `ROBOT_WS`; do not create another workspace or overlay
only to rebuild robot code. Keep hydraulics locked and stop the application
owner before replacing installed artifacts.

- When the changed owning package is known, build only it:
  `colcon build --packages-select <package>`.
- When several application packages changed, or the exact boundary is unclear
  within the Mole bringup dependency closure, use
  `colcon build --packages-up-to mole_bringup`.
- After a message, service, action, or C++ ABI change, build the reverse
  dependents with `colcon build --packages-above <changed-package>`.
- Do not rebuild the whole workspace unless one of those narrower boundaries
  is proven insufficient.

Machine-workspace builds use copied installs. Omit `--symlink-install` so the
runtime does not depend on mutable source paths. This is mandatory for
`mole_highlevel_controller_cpp`: its trusted policy inventory and model files
must be regular installed files. Build it with:

```bash
colcon build --packages-select mole_highlevel_controller_cpp \
  --cmake-clean-cache \
  --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
```

If that package was previously built with symlink install, CMake may leave the
old generated links as "up to date." Remove only its generated
`build/mole_highlevel_controller_cpp` and
`install/mole_highlevel_controller_cpp` directories, rerun the command, then
verify the installed policy inventory and selected model with `test ! -L`.
Never clear the workspace or source tree for this repair.

Run one focused test at the changed boundary and verify the installed resource,
not only the source file, before relaunching the single owner. Route
Terra/Nav2-specific dependency details to `terra-pipeline`.
