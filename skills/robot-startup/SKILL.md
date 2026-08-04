---
name: robot-startup
description: "Start or restart the real-robot Moleworks ROS tmux prerequisites and optional standalone perception/DIG stack. Use for bounded machine bringup, tool/namespace/TF selection, hydraulic unlock, or authorized engine-RPM setup."
---

# Robot Startup

Use the bundled tmux wrapper for direct robot bringup. Use `terra-pipeline` for
a Terra application, `dig-controllers` when the prerequisites already run, and
`set-engine-rpm` for RPM-only work.

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
- `--no-foxglove` when the application owns visualization
- `--dig-controller <dig3d|newton|dig|dig-ee>` to launch a standalone DIG
  controller without lifecycle activation; after state, TF, map, and publisher checks, use
  `dig-controllers` for the separately authorized activation/action boundary

Keep the same effective tool, namespace, and TF prefix across low level,
estimator, perception, and the application. For Terra, follow its owning skill
and operator guide: Terra owns enabled perception and visualization, so do not
prestart duplicates.

Without `--restart`, the wrapper preflights every managed pane, starts only
idle panes, and refuses a busy pane whose owner is wrong or unknown. It never
sends Ctrl+C or respawns a busy process implicitly. It delays Foxglove until
earlier requested launches are present. `--restart` kills and recreates the
named tmux session. It does not change global tmux continuum settings.

A matching pane role is not proof of matching configuration. Before relying on
an accepted busy pane, compare its working directory, process environment, and
launch arguments with the selected workspace, ROS domain, tool, namespace, TF
prefix, and profile. Stop on ambiguity; do not start the missing panes beside a
mixed stack.

## Mapping Contract

Use `local` for a standalone live local map. `site` and `analytical` are
design-backed and require an explicit saved design map. Do not load a site map
silently.

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
