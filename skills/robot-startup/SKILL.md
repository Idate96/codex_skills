---
name: robot-startup
description: "Start or restart the real-robot Moleworks ROS tmux stack. Use for base bringup, optional DIG-controller launch, hydraulic unlock, estimator selection, and engine-RPM setup."
---

# Robot Startup

Start the real-robot stack through the bundled tmux wrapper. Use `newton-sim-ros-startup` for Newton and `sim-startup` for split IsaacLab/Terra simulation.

## Authority Boundary

- A request to start the ROS stack authorizes software bringup only.
- Unlock hydraulics or change engine RPM only when the user explicitly asks to make the machine ready, unlock it, or set an RPM.
- Keep an operator present for machine-side actions and require the Menzi interlocks below.
- Use `--restart` only when the user requested a clean restart or killing the existing managed session is otherwise clearly in scope.

## Start The Stack

Run the wrapper immediately for a normal base-stack request:

```bash
/home/lorenzo/codex_skills/skills/robot-startup/scripts/robot_startup_tmux.sh
```

Useful explicit options:

- attach: `--attach`
- workspace override: `--ws <path>`; otherwise the script checks `~/ros2_ws`, then `~/moleworks/ros2_ws`
- clean restart: `--restart`
- omit estimator: `--no-estimator`
- add a controller: `--dig-controller <dig3d|newton|dig|dig-ee>`
- select tool geometry: `--endeffector-type <type>`
- load a saved site design: `--mapping-profile site --design-map-name <name>`

Base stack is the default. If the base is already running and only a controller is needed, use `dig-controllers` instead.

The wrapper manages named tmux windows (`low_level`, `perception`, optional `estimator`, optional `dig`, then `foxglove`). It leaves unrelated busy windows alone, starts Foxglove only after earlier managed launches pass readiness, and disables tmux continuum restore unless `--keep-continuum-restore` is explicit.

## DDS And Container Default

Use a freshly pulled or rebuilt `rslheap/moleworks_ros:latest` container. Its shell configures normal ROS processes as Fast DDS CLIENT and the ROS 2 daemon as an observer SUPER_CLIENT. Source the workspace, then run the wrapper and ordinary `ros2 ...` commands directly.

Do not paste DDS profile exports/unsets in front of each command. If that plain-command contract is missing, the container is stale: pull or rebuild the image and recreate the container instead of patching its environment command by command. The startup wrapper intentionally does not duplicate image-level DDS setup.

## Mapping Contract

Default to `mapping_profile:=local`; do not silently load a site-scale design bag. Dig controllers consume both current and target terrain from `/excavation_mapping/grid_map`.

After local bringup, apply runtime desired geometry through `/excavation_mapping/apply_runtime_profile` only when the user asked for a target. Supported families are `constant`, `slanted`, `trench`, and `polar_sector`. The service is one-shot per excavation-mapping lifetime; restart excavation mapping before changing the target.

## Optional Machine-Ready Actions

Only continue here after explicit user intent.

1. Resolve and source the same workspace used by startup.
2. Wait at most 60 seconds for `/machine_status` and `/hydraulic_lock`; do not wait indefinitely.
3. Read `/machine_status` with a bounded timeout and require:
   - `is_armrest_unlocked: true`
   - `is_radio_estop_unlocked: true`
   - `is_manual_operation_unlocked: true`
   - `is_autonomy_switch_on: true`
4. Call `/hydraulic_lock` only after those gates pass.
5. Re-read status and require `is_hydraulilock_unlocked: true`; when available, also require `is_autonomous_operation_unlocked: true`.
6. If the user requested an RPM, use `set-engine-rpm` and verify readback. Do not assume `1600` from a generic startup request.

Bounded readiness pattern:

```bash
WS="$HOME/ros2_ws"
[[ -f "$WS/install/setup.bash" ]] || WS="$HOME/moleworks/ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

timeout 60 bash -lc 'until ros2 topic list | grep -Eq "^/machine_status$"; do sleep 1; done'
timeout 60 bash -lc 'until ros2 service list | grep -Eq "^/hydraulic_lock$"; do sleep 1; done'
status="$(timeout 10 ros2 topic echo --once /machine_status)"
grep -Fq 'is_armrest_unlocked: true' <<<"$status"
grep -Fq 'is_radio_estop_unlocked: true' <<<"$status"
grep -Fq 'is_manual_operation_unlocked: true' <<<"$status"
grep -Fq 'is_autonomy_switch_on: true' <<<"$status"
ros2 service call /hydraulic_lock std_srvs/srv/SetBool '{data: true}'
status="$(timeout 10 ros2 topic echo --once /machine_status)"
grep -Fq 'is_hydraulilock_unlocked: true' <<<"$status"
```

Stop and report the failed gate if a prerequisite, service, or readback does not pass. Do not invent alternate unlock commands.

## Verification

After startup, verify the requested managed windows are running their intended launches. For machine-ready requests, report both hydraulic status and measured RPM rather than only successful service calls.

For general interlock or stack diagnosis, use `robot-ros`. For RPM changes, use `set-engine-rpm`.
