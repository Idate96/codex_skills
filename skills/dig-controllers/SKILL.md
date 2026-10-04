---
name: dig-controllers
description: "Start or restart one Mole DIG controller, handle repeat scoops, or change an existing DIG/UGEP target depth with the base stack running. Covers Dig3D, Newton, legacy DIG/DIG-EE, and UGEP bringup."
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
known lifecycle services directly with bounded timeouts. Never use `ros2 service
list` as the readiness gate for a known controller lifecycle endpoint on this
robot; probe `/<namespace>/<controller>/get_state` directly. Do not switch DDS
profiles during a live run because of one failed global graph lookup.

### Same-session repeat action fast path

Do not repeat the full bringup preflight before every scoop. When the same
controller process, policy, target, workspace, recorder, and command publisher
ownership were already verified in the current session, the preceding action
ended normally, and the operator says `go`, `again`, or equivalent:

1. Read the controller's known lifecycle state and one `/machine_status`
   sample. If recording is in scope, also confirm the existing recorder pane is
   still busy; do not inspect every split while it is running.
2. Require the controller to be inactive or active-waiting and the usual
   hydraulic/autonomy interlocks to remain enabled. Activate only if inactive,
   then send the action immediately.

Reuse the earlier estimator, TF, map, policy, target, topic, and publisher
checks. Run the full preflight again only after a controller restart, policy or
target change, graph-owner change, failed/abnormal action, stale/missing input,
or another observed change that invalidates the earlier evidence. A manual
dump or arm reposition alone does not invalidate software ownership checks.

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

## Existing target depth changes

Change an existing `desired_elevation` through
`/excavation_mapping/apply_shared_excavation_patch`; this does not require a
perception restart or controller reconfiguration. `apply_runtime_profile` is
one-shot initial authoring, not the interface for these edits. The owning
controller package README's **Runtime target edits** section documents the
contract. For the flat-target hardware fast path, use
[the target helper](scripts/update_flat_target.py).

- Reuse the session's established reference and footprint. With old depth
  `d_old` and requested depth `d_new`, use
  `z_new = z_old + d_old - d_new`. State the chosen reference briefly. If the
  user says either reference is acceptable, choose the existing reference and
  proceed; do not keep a clarification pending. Ask only when the choice is
  material and cannot be inferred. Do not derive a new datum from freshly dug
  soil unless a new soil-relative target is intended.
- Finish the current action, make the sole controller inactive, and keep
  hardware hydraulic acceptance locked while editing. This is preparation;
  the helper never activates, unlocks, or sends a goal. Retain existing session
  authorization for subsequent testing and honor operator readiness.
- Source the live workspace and use its DDS environment. The helper takes an
  explicit new map Z, expected old flat map Z, expected footprint cell count,
  and a new output directory. Run `--help` for arguments; `--apply` writes once,
  while omission saves a read-only preview. Do not blindly retry after a
  service timeout: inspect the published map and saved receipt first.
- It saves the original serialized map and patch, verifies inactive/locked
  state and the controller's elevation validity, preserves geometry and
  circular indices, writes only `desired_elevation` plus the configured mask,
  and verifies the published target and unchanged footprint. Outside values
  must remain unchanged or follow live soil in both snapshots, as the mapping
  owner normally maintains them; do not require the entire live map to freeze.
  Live `elevation` is never sent in the patch. After an edit, refresh the
  controller input checks before the first scoop; later scoops use the repeat
  fast path above.
- The helper supports an existing flat footprint on hardware. For other
  geometry or simulation, follow the same package contract with the
  appropriate lifecycle/interlocks and explicit patch geometry. A target below
  the configured validity minimum requires reviewing that parameter and
  reconfiguring; do not weaken depth-shield checks to make it pass.

`desired_elevation` belongs to excavation mapping, not elevation mapping.

On `/excavation_mapping/grid_map`, `desired_elevation` is the authored target
height surface. `runtime_target` is only a binary footprint mask (0 outside,
1 inside); it is never a valid target-elevation layer. Keep DIG/UGEP pointed at
`desired_elevation`, including after applying a runtime profile. The
`set_target_elevation_layer` service checks that a layer exists but cannot
distinguish a mask from a height surface, so never retarget it to
`runtime_target`. Selecting that mask can make the policy decode a target many
metres above the soil and command a strong upward lift.
