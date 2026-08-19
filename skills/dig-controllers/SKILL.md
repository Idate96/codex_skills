---
name: dig-controllers
description: "Start or restart one Mole DIG controller in tmux when the base robot or Newton stack is already running. Covers Dig3D, Newton, legacy DIG/DIG-EE, and UGEP dry-run bringup."
---

# DIG Controllers

This skill owns controller-only launch and lifecycle/action handling. Route elsewhere when the request includes more:

- base robot stack startup or recovery: `robot-startup`
- ROS graph, TF, topic, or lifecycle diagnosis: `ros2-debugging`
- split-bag capture: `dig-bag-recording`
- offline bag visualization: `dig-bag-replay`
- OCS2 arm operation or tuning: the applicable OCS2 skill

## Safety and preflight

1. Confirm hardware (`use_sim_time:=false`) versus Newton (`use_sim_time:=true`) with the operator; do not infer it only from the graph.
2. Confirm the built workspace and effective ROS/DDS environment. The current
   real-robot image is ROS 2 Jazzy: source the selected workspace's
   `install/setup.bash` and verify `ROS_DISTRO=jazzy`. Do not hard-code a
   Humble underlay path.
3. Check required state, TF, and map inputs. Current DIG policies consume `/excavation_mapping/grid_map`; that map must contain live `elevation` and, when the selected policy requires it, `desired_elevation`.
4. Inspect publishers on `/<robot_namespace>/actuator_commands`. Do not activate into an unexpected competing command owner. On hardware, require the robot interlocks and an operator.
5. `--restart-window` kills only the managed tmux window. Use it only for an authorized restart.

The helper launches with the controller launch file's own auto-activation disabled, then configures and activates through lifecycle services. Use `--no-activate` for inspection. Motion remains separate: `--run-action` is explicit opt-in.

The current Jazzy robot image uses an ordinary Fast DDS Discovery Server client
profile. Short-lived graph-oriented commands such as `ros2 lifecycle get` can
intermittently report `Node not found` even while known services and actions
communicate normally. The helper therefore waits for and calls the controller's
known lifecycle services directly. Do not switch DDS profiles during a live run
because of one failed global graph lookup.

## Controller keys

| Key | Launch file | Default node/action under namespace `mole` |
|---|---|---|
| `dig3d` | `dig_3d_controller_cpp.launch.py` | `/mole/dig_3d_controller`, `/mole/run_dig_3d` |
| `newton` | `dig_newton_controller.launch.py` | `/mole/dig_newton_controller`, `/mole/run_dig_newton` |
| `dig` | `dig_controller_cpp.launch.py` | `/mole/dig_controller`, `/mole/run_dig` |
| `dig-ee` | `dig_ee_controller_cpp.launch.py` | `/mole/dig_ee_controller`, `/mole/run_dig_ee` |
| `ugep` | `dig_ugep_controller_cpp.launch.py` | `/mole/dig_ugep_controller`, `/mole/run_dig_ugep` |

UGEP requires a policy manifest supplied after `--`, for example `policy_manifest_path:=...`. Its launch defaults to the isolated `actuator_commands_ugep_dryrun` topic and `allow_live_actuation:=false`; do not override those for hardware without separate explicit authorization.

## Commands

Real-robot Dig3D:

```bash
SKILL_DIR="${CODEX_HOME:-$HOME/.codex}/skills/dig-controllers"
"$SKILL_DIR/scripts/dig_controllers_tmux.sh" \
  --controller dig3d --use-sim-time false --attach
```

Newton simulation:

```bash
SKILL_DIR="${CODEX_HOME:-$HOME/.codex}/skills/dig-controllers"
"$SKILL_DIR/scripts/dig_controllers_tmux.sh" \
  --controller newton --use-sim-time true --attach
```

Inspection-only launch:

```bash
SKILL_DIR="${CODEX_HOME:-$HOME/.codex}/skills/dig-controllers"
"$SKILL_DIR/scripts/dig_controllers_tmux.sh" \
  --controller dig3d --use-sim-time false --no-activate --attach
```

Pass supported launch overrides after `--`; the helper already owns `use_sim_time`, `activate_controller`, `run_action`, `robot_namespace`, and `tf_prefix`.

The managed window defaults to `ros:dig`. Its left pane owns the launch; its right pane waits for the namespaced node and handles lifecycle/action commands.

Manual lifecycle recovery uses the known services directly (`1=configure`,
`3=activate`, `4=deactivate`):

```bash
ros2 service call /mole/dig_3d_controller/change_state \
  lifecycle_msgs/srv/ChangeState '{transition: {id: 3}}'
ros2 service call /mole/dig_3d_controller/get_state \
  lifecycle_msgs/srv/GetState '{}'
```

Runtime target profiles are one-shot per excavation-mapping process. To replace an applied profile, restart the owning mapping stack and call `/excavation_mapping/apply_runtime_profile` again. `desired_elevation` belongs to excavation mapping, not elevation mapping.
