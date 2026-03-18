---
name: robot-startup
description: Set up the standard Moleworks ROS 2 tmux session for on-machine work with 4 windows in order (low_level, perception, estimator, foxglove), then by default unlock hydraulics and set engine RPM to 1600 unless the user asks otherwise. Use when asked to create/recreate the tmux window layout and start the stack quickly (dig controllers are started separately).
---

# Robot Startup

Create a tmux session that matches the standard on-machine window layout (4 windows in order, pinned names) and starts the usual launch commands (no hardcoded window indexes).

Unless the user explicitly opts out or requests different post-start settings, this skill also performs the standard machine-ready steps after startup:
- unlock hydraulics
- set engine RPM to `1600`

The startup script launches `foxglove` last, after the other managed windows are confirmed to be running their intended managed launches and a short settle delay has passed. If that readiness gate times out, the script leaves `foxglove` stopped instead of starting it too early. On reruns, if one of the earlier managed windows is restarted, the script re-sequences `foxglove` behind that restart.

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
  python3 ~/ros2_ws/src/moleworks_ros/high_level_controllers/mole_highlevel_controller/mole_highlevel_controller/utils/set_engine_rpm.py 1600
fi
status="$(ros2 topic echo --once /machine_status)"
echo "$status" | grep -Fq 'is_hydraulilock_unlocked: true'
echo "$status" | grep -Fq 'is_autonomous_operation_unlocked: true'
rpm="$(echo "$status" | awk '/measured_engine_rpm:/ {print $2; exit}')"
if ! awk -v rpm="$rpm" 'BEGIN { exit !(rpm >= 1550 && rpm <= 1650) }'; then
  python3 ~/ros2_ws/src/moleworks_ros/high_level_controllers/mole_highlevel_controller/mole_highlevel_controller/utils/set_engine_rpm.py 1600
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

tmux session (default: `ros`) with 4 windows in order by name:

- `low_level`: `ros2 launch mole_low_level_bringup bringup.launch.py use_sim_time:=false on_machine:=true activate_trajectory_controller:=false`
- `perception`: `ros2 launch mole_perception_bringup bringup.launch.py use_sim_time:=false enable_lidar:=true enable_robot_self_filter:=true enable_elevation_mapping:=true map_name:=none`
- `estimator`: `ros2 launch mole_estimator mole_estimator.launch.py ...`
- `foxglove`: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml` (default port `8765`)

The script prompts for `endeffector_type` when run interactively (defaults to `shovel` if omitted) and passes it into low_level, perception, and estimator launches.

`foxglove` is started after the other managed windows are already running their intended launches, with a short delay to let the ROS graph settle first. On reruns, if one of the earlier managed windows is restarted, the script stops `foxglove` first and only brings it back after that gate passes. If the readiness gate times out, `foxglove` is left stopped so the bridge is not started in a bad state.

Window names are locked (automatic rename disabled), and the order is enforced each run.

By default, the script does not touch windows that already have a non-shell process running, except when it detects a managed launch in the wrong managed window (for example, `foxglove` running in `estimator`) or when it needs to re-sequence `foxglove` behind restarted earlier windows. In those foxglove cases, the script stops `foxglove` as needed and only starts it again after the readiness gate passes; if that gate times out, the `foxglove` window is left stopped instead. Use `--restart` to recreate everything.

To keep startup deterministic, the script force-disables tmux `@continuum-restore` by default for the current tmux server (to avoid old windows/history being auto-restored into `ros`). Use `--keep-continuum-restore` to opt out.

To start dig controllers, use the separate `dig-controllers` skill.

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

If the RPM service type is not available in `~/ros2_ws`, fall back to `~/ros2_ws/src/moleworks_ros/high_level_controllers/mole_highlevel_controller/mole_highlevel_controller/utils/set_engine_rpm.py 1600` instead of guessing.

## Resources

- Script: `scripts/robot_startup_tmux.sh`
