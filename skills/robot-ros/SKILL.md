---
name: robot-ros
description: "Operate and debug real-robot Moleworks ROS 2 stacks, including Menzi GPS gateway and RTK correction recovery. Route Gravis CAT323 to its integration workflow; use the M4 runbook for Menzi bringup, interlocks, controllers, and health checks."
---

# Robot ROS

## Overview

Use the repo runbooks and scripts as the single source of truth for operating and debugging the robot stack, and keep operator-facing docs consistent with the current code/topic names.

## Workflow (Always Do This First)

For Gravis **CAT323** or the `integration@10.27.0.13` x86, use [gravis-cat323](../gravis-cat323/SKILL.md) before the M4 steps below. Its native Gravis estimation/mapping, numeric status fields, hydraulic-service contract, and command gateway differ. The M4 startup, interlock, and latch-recovery examples below do not apply to CAT323. A Gravis interface on Menzi still uses the M4 route; select by machine, not the word “Gravis” alone.

1. Resolve the active robot workspace and repo root; prefer `$HOME/ros2_ws` on-machine and fall back to `$HOME/moleworks/ros2_ws` when present.
2. Treat `<workspace>/src/moleworks_ros/docs/robot_agent/ROBOT_OPERATIONS_GUIDE.md` as the single source of truth for operator commands.
3. For Menzi M4 machine-specific interlocks (autonomy/hydraulics/ignition), cross-check `~/git/menzi_docs/M4/`.
4. Prefer the health/monitor scripts under `<workspace>/src/moleworks_ros/mole_utils/scripts/` over ad-hoc commands.
5. If giving ROS CLI commands, verify they exist for Jazzy (`ros2 <verb> -h`) and match the installed message definitions.

## Common Tasks

### GPS Gateway / RTK Correction Recovery (Menzi M4)

For GPS Internet sharing, receiver gateway changes, or missing RTK corrections,
read [GPS gateway and RTK recovery](references/gps-gateway.md). The Septentrio
receiver at `192.168.19.4` has its own gateway; changing rslpc's default route
does not change it. The bundled helper reads status and changes only the
receiver's current gateway, with backup and readback. A normal startup request
does not imply changing the receiver's network configuration.

### Bringup / Startup

- Open `src/moleworks_ros/docs/robot_agent/ROBOT_OPERATIONS_GUIDE.md` and use the launch commands from the “Quick Reference”.
- If the user’s question implies different stacks, pick the closest launch entry point:
  - `mole_bringup nav2_and_moveit.launch.py` for full stack
  - `mole_bringup nav2.launch.py` for Nav2-only
  - `mole_bringup moveit.launch.py` for MoveIt-only
  - `mole_bringup dig.launch.py` for dig stack

### Enable Autonomous Mode / Hydraulic Unlock (Menzi M4)

- Do not invent an “unlock” command. Interlock services are machine/LLC specific.
- Confirm operator-side prerequisites (armrest up; press the radio/antenna button in the Menzi display and confirm with the push button).
- Verify readiness via `/machine_status` (fields like `is_autonomous_operation_unlocked`, `is_hydraulilock_unlocked`, etc.).
- Discover services with `ros2 service list | grep -Ei "hydraul|lock|unlock|autonom|ignition|engine"`.
- For the canonical Menzi docs and exact service names, consult:
  - `~/git/menzi_docs/M4/M4_operation_workflow.md`
  - `~/git/menzi_docs/M4/M4_Checklist.md`

### Health Checks / Debugging

- Prefer:
  - `src/moleworks_ros/mole_utils/scripts/monitor_pipeline_flow.sh`
  - `src/moleworks_ros/mole_utils/scripts/monitor_perception_stack.sh`
  - `src/moleworks_ros/mole_utils/scripts/monitor_highlevel_controllers.sh`
- If a command/topic name seems stale, confirm the authoritative names by searching:
  - `rg -n "<topic_or_service_or_action>" src/moleworks_ros`
  - Message types in `src/mole_msgs/msg/` and `src/machine_msgs/msg/`
  - Launch files under `src/moleworks_ros/*/launch/`

### Menzi M4 Gravis Command Latch Recovery

Use this when `/machine_status` shows `is_using_gravis_commands: false` during testing.

1. Stop/interrupt the active command publisher (for example a matrix runner).
2. Publish zero velocity commands on `/mole/actuator_commands` at 20 Hz for 2 seconds.
3. Re-check `/machine_status` and only continue when `is_using_gravis_commands: true`.
4. Restart the interrupted test and skip already-completed cases when possible.

Reference command:

```bash
timeout --signal=INT 2s ros2 topic pub -r 20 /mole/actuator_commands \
  mole_msgs/msg/MoleActuatorCommands \
  "{actuators: [{joint_name: 'J_TURN', mode: 2, velocity: 0.0}, {joint_name: 'J_BOOM', mode: 2, velocity: 0.0}, {joint_name: 'J_STICK', mode: 2, velocity: 0.0}, {joint_name: 'J_TELE', mode: 2, velocity: 0.0}, {joint_name: 'J_EE_PITCH', mode: 2, velocity: 0.0}]}" || [[ $? -eq 124 || $? -eq 130 ]]
```

## Maintaining Robot-Agent Docs (Avoid Duplication)

When updating operator docs:
- Update `src/moleworks_ros/docs/robot_agent/ROBOT_OPERATIONS_GUIDE.md` (add/modify content there).
- Keep `src/moleworks_ros/docs/robot_agent/ROBOT_AGENT_QUICK_REFERENCE.md` as a pointer/alias (do not duplicate content).
- Validate commands against the current repo (launch args, topic names, message fields, ROS 2 Jazzy CLI).
