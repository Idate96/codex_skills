---
name: robot-startup
description: Start the real-robot Moleworks ROS tmux stack, optionally add a DIG controller, unlock hydraulics, and set engine RPM.
---

# Robot Startup

Create a tmux session that matches the standard on-machine window layout and starts the usual launch commands (no hardcoded window indexes).

This skill is for the real robot stack only.

Do not stretch it to cover simulation startup:
- for Newton sim in the ROS container, use `newton-sim-ros-startup`
- for IsaacLab/Terra sim, use `sim-startup`

Reason:
- `robot-startup` includes machine-only actions like hydraulic unlock and engine RPM handling
- sim startup uses different containers, `use_sim_time:=true`, different window layouts, and different health checks
- merging them would make the hardware workflow less predictable

Base stack only remains the default. If the user explicitly asks for base stack plus a dig controller, pass `--dig-controller <dig3d|newton|dig|dig-ee>` to the startup script so the `dig` window is created too.

Unless the user explicitly opts out or requests different post-start settings, this skill also performs the standard machine-ready steps after startup:
- unlock hydraulics
- set engine RPM to `1600`

The startup script launches `foxglove` last, after the other managed windows are confirmed to be running their intended managed launches and a short settle delay has passed. If that readiness gate times out, the script leaves `foxglove` stopped instead of starting it too early. On reruns, if one of the earlier managed windows is restarted, the script re-sequences `foxglove` behind that restart.

Perception now defaults to the native `local` mapping contract again: no saved design artifact is loaded unless you opt in explicitly. This keeps excavation mapping base-local/empty-map friendly instead of silently loading a site-scale design bag into the local stack. To start a saved design artifact on purpose, use `--mapping-profile site --design-map-name <name>`.

Runtime desired geometry is a post-start step in local mode. `robot-startup` should bring up the stack first, then apply the target surface once through `/excavation_mapping/apply_runtime_profile`. Current supported geometry families are `constant`, `slanted`, `trench`, and `polar_sector`.

Dig controllers now consume both current terrain and target terrain from `/excavation_mapping/grid_map`. Do not route `desired_elevation` through elevation mapping, and do not treat elevation mapping as the source of the digging target.

This assumes DDS/discovery is already configured in the container shell environment and that the robot bringup uses `~/ros2_ws`.

## Execute (Fast)

If the user asks to "execute" (or uses phrasing like "execute fast"), run the startup script immediately, then do the default post-start actions (hydraulic unlock + RPM `1600`) unless the user said not to. Do not do extra debugging like `tmux ls` or pane log capture unless asked for it, but do wait for the required ROS services and `/machine_status` before issuing the post-start commands.

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh
```

Add flags only if the user asked for them:

- attach: `--attach`
- clean slate: `--restart`
- legacy 3-window layout (skip estimator): `--no-estimator`
- base stack plus dig controller: `--dig-controller dig3d` (or `newton`, `dig`, `dig-ee`)

After the tmux startup script returns, remember that the bringup is still starting inside tmux. Source `~/ros2_ws`, wait until `/machine_status` and `/hydraulic_lock` are available, confirm the operator-side autonomy prerequisites via `/machine_status`, and only then issue the post-start commands:

```bash
set -euo pipefail
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
until ros2 topic list | grep -Eq '^/machine_status$'; do sleep 1; done
until ros2 service list | grep -Eq '^/hydraulic_lock$'; do sleep 1; done
status="$(ros2 topic echo --once /machine_status)"
echo "$status" | grep -Fq 'is_armrest_unlocked: true'
echo "$status" | grep -Fq 'is_radio_estop_unlocked: true'
echo "$status" | grep -Fq 'is_manual_operation_unlocked: true'
echo "$status" | grep -Fq 'is_autonomy_switch_on: true'
ros2 service call /hydraulic_lock std_srvs/srv/SetBool "data: true"
until ros2 service list | grep -Eq '^/(set_diesel_speed|set_rpm|engine_speed)$'; do sleep 1; done
if ros2 service list | grep -Eq '^/set_diesel_speed$'; then
  ros2 service call /set_diesel_speed mole_msgs/srv/SetRPM "{target_rpm: 1600}"
elif ros2 service list | grep -Eq '^/set_rpm$'; then
  ros2 service call /set_rpm mole_highlevel_msgs/srv/SetRPM "{target_rpm: 1600}"
else
  python3 ~/ros2_ws/src/moleworks_ros/high_level_controllers/mole_highlevel_controller_cpp/scripts/set_engine_rpm.py 1600
fi
status="$(ros2 topic echo --once /machine_status)"
echo "$status" | grep -Fq 'is_hydraulilock_unlocked: true'
echo "$status" | grep -Fq 'is_autonomous_operation_unlocked: true'
rpm="$(echo "$status" | awk '/measured_engine_rpm:/ {print $2; exit}')"
if ! awk -v rpm="$rpm" 'BEGIN { exit !(rpm >= 1550 && rpm <= 1650) }'; then
  python3 ~/ros2_ws/src/moleworks_ros/high_level_controllers/mole_highlevel_controller_cpp/scripts/set_engine_rpm.py 1600
  status="$(ros2 topic echo --once /machine_status)"
  rpm="$(echo "$status" | awk '/measured_engine_rpm:/ {print $2; exit}')"
  awk -v rpm="$rpm" 'BEGIN { exit !(rpm >= 1550 && rpm <= 1650) }'
fi
```

Keep the RPM step pinned to `~/ros2_ws`: prefer the direct services when available, and only fall back to the Python helper from `~/ros2_ws`.

## Quick Start

Run the setup script:

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh --attach
```

Run the base stack and also launch `dig3d`:

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh \
  --dig-controller dig3d \
  --attach
```

Disable the estimator window (legacy 3-window layout):

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh --no-estimator --attach
```

Optionally override the estimator config:

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh \
  --estimator-config ~/ros2_ws/src/moleworks_ros/mole_estimator/config/mole_estimator_robot_heading_corrected.yaml \
  --attach
```

Start with the default local empty-map perception contract:

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh --attach
```

After local bringup, apply a supported runtime profile once through excavation mapping:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 service call /excavation_mapping/apply_runtime_profile mole_excavation_mapping/srv/ApplyRuntimeProfile \
  "{authoring_frame_id: BASE, profile_type: trench, start_x_m: 8.0, end_x_m: 4.0, center_y_m: 0.0, half_width_m: 1.0, depth_m: 0.0, angle_deg: 20.0, entry_angle_deg: 20.0, exit_angle_deg: 20.0, trench_length_m: 2.0, trench_depth_m: -1.5}"
```

This service is one-shot per excavation-mapping lifetime. If the profile footprint is not fully covered in `elevation` yet, wait and retry. If you need a different runtime target later, restart excavation mapping or restart the perception stack and apply again.

Example for a 60 degree annular sector in front of the robot, from radius 4 m to 8 m:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 service call /excavation_mapping/apply_runtime_profile mole_excavation_mapping/srv/ApplyRuntimeProfile \
  "{authoring_frame_id: BASE, profile_type: polar_sector, depth_m: -1.5, radius_min_m: 4.0, radius_max_m: 8.0, theta_min_deg: -30.0, theta_max_deg: 30.0}"
```

Opt into a saved design-backed site runtime explicitly:

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh \
  --mapping-profile site \
  --design-map-name hong0304 \
  --attach
```

Use a non-default session:

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh \
  --session ros_test \
  --endeffector-type shovel_w_teeth \
  --attach
```

If your tmux uses `tmux-continuum` auto-restore and you explicitly want to keep it enabled during startup:

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh --keep-continuum-restore --attach
```

If you want a clean slate (kills the existing session):

```bash
~/.codex/skills/robot-startup/scripts/robot_startup_tmux.sh --restart --attach
```

## What It Creates

tmux session (default: `ros`) with base windows in order by name:

- `low_level`: `ros2 launch mole_low_level_bringup bringup.launch.py use_sim_time:=false on_machine:=true activate_trajectory_controller:=false`
- `perception`: `ros2 launch mole_perception_bringup bringup.launch.py use_sim_time:=false enable_lidar:=true enable_robot_self_filter:=true enable_elevation_mapping:=true mapping_profile:=local`
- `estimator`: `ros2 launch mole_estimator mole_estimator.launch.py ...`
- `foxglove`: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml` (default port `8765`)

Saved design artifacts are opt-in. When needed, use `--mapping-profile site --design-map-name <name>` so the perception stack follows the site/design-backed contract instead of mixing a site artifact into the local runtime.

Important distinction:

- if the user only wants a different dig area or dump area, update the workspace-planner masks
- if the user wants a different target surface in `desired_elevation`, use `/excavation_mapping/apply_runtime_profile`
- if the requested target geometry is not `constant|slanted|trench|polar_sector`, the current runtime service does not support it yet and the skill should say so plainly
- `trench_workspace.launch.py` is still trench-task-specific; for non-trench targets such as `polar_sector`, use `/excavation_mapping/apply_runtime_profile` directly after local bringup

`polar_sector` is the current way to express a base-centered wedge/circular section target surface in front of the robot. If the requested target is not centered at `BASE`, or it needs a curved depth law instead of constant depth over the sector, that still needs a new geometry family such as `arc_section`.

If `--dig-controller` is requested, it also adds:

- `dig`: split tmux window started via the `dig-controllers` script for the selected controller, with launch-driven auto-activation enabled. For `dig3d`, the default policy is now the non-AoA `fkfix_s203_3750` preset.

The script prompts for `endeffector_type` when run interactively (defaults to `shovel_calibrated` if omitted) and passes it into low_level, perception, and estimator launches.

`foxglove` is started after the other managed windows are already running their intended launches, with a short delay to let the ROS graph settle first. On reruns, if one of the earlier managed windows is restarted, the script stops `foxglove` first and only brings it back after that gate passes. If the readiness gate times out, `foxglove` is left stopped so the bridge is not started in a bad state.

Window names are locked (automatic rename disabled), and the order is enforced each run.

By default, the script does not touch windows that already have a non-shell process running, except when it detects a managed launch in the wrong managed window (for example, `foxglove` running in `estimator`) or when it needs to re-sequence `foxglove` behind restarted earlier windows. In those foxglove cases, the script stops `foxglove` as needed and only starts it again after the readiness gate passes; if that gate times out, the `foxglove` window is left stopped instead. Use `--restart` to recreate everything.

To keep startup deterministic, the script force-disables tmux `@continuum-restore` by default for the current tmux server (to avoid old windows/history being auto-restored into `ros`). Use `--keep-continuum-restore` to opt out.

Use `robot-startup` when the user asks for base stack only or base stack plus dig. Use the separate `dig-controllers` skill when the base stack is already running and they only want to add or restart a dig controller.

## Default Post-Start Actions

When this skill is used for robot bringup, assume the operator wants the machine ready for testing unless they say otherwise:

- wait for bringup readiness before any machine-side command
- unlock hydraulics after the low-level stack is available
- set engine RPM to `1600`

Use the authoritative Menzi/robot workflow for the machine-side part:
- operator prerequisites still apply for autonomy/hydraulic unlock
- require `/machine_status` before unlock and confirm the machine is in the correct pre-unlock state
- wait for `/hydraulic_lock` before unlock and wait for one of `/set_diesel_speed`, `/set_rpm`, or `/engine_speed` before RPM changes
- if service discovery is required, prefer `ros2 service list | grep -Ei "hydraul|lock|unlock|engine|set_diesel_speed|set_rpm"`

Minimum readiness checks before calling `/hydraulic_lock`:

- `is_armrest_unlocked: true`
- `is_radio_estop_unlocked: true`
- `is_manual_operation_unlocked: true`
- `is_autonomy_switch_on: true`

After calling `/hydraulic_lock`, re-check `/machine_status` and expect `is_hydraulilock_unlocked: true`. When available, also confirm `is_autonomous_operation_unlocked: true` before treating the machine as ready.

Prefer direct services in this order:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 service call /hydraulic_lock std_srvs/srv/SetBool "data: true"
ros2 service call /set_diesel_speed mole_msgs/srv/SetRPM "{target_rpm: 1600}"
ros2 service call /set_rpm mole_highlevel_msgs/srv/SetRPM "{target_rpm: 1600}"
```

If the RPM service type is not available in `~/ros2_ws`, fall back to `~/ros2_ws/src/moleworks_ros/high_level_controllers/mole_highlevel_controller_cpp/scripts/set_engine_rpm.py 1600` instead of guessing.

## Resources

- Script: `scripts/robot_startup_tmux.sh`
