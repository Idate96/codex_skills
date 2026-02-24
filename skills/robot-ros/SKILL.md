---
name: robot-ros
description: Moleworks ROS 2 (Jazzy) robot operations + runbook assistant for the Mole/Menzi M4 stack. Use for startup/shutdown, enabling autonomous mode, hydraulic unlock, running Nav2/MoveIt/dig controllers, checking topic health/rates, debugging lifecycle/action servers, and maintaining the robot-agent docs under src/moleworks_ros/docs/robot_agent/.
---

# Robot ROS

## Overview

Use the repo runbooks and scripts as the single source of truth for operating and debugging the robot stack, and keep operator-facing docs consistent with the current code/topic names.

## Workflow (Always Do This First)

1. Treat `src/moleworks_ros/docs/robot_agent/ROBOT_OPERATIONS_GUIDE.md` as the single source of truth for operator commands.
2. For Menzi M4 machine-specific interlocks (autonomy/hydraulics/ignition), cross-check `~/git/menzi_docs/M4/`.
3. Prefer running the health/monitor scripts in `src/moleworks_ros/mole_utils/scripts/` over ad-hoc commands.
4. If giving ROS CLI commands, verify they exist for Jazzy (`ros2 <verb> -h`) and match message definitions in `src/*_msgs/msg/`.

## Common Tasks

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
  - `~/git/menzi_docs/M4/Running code on rslpc.md`
  - `~/git/menzi_docs/M4/M4 Startup and Shutdown.md`

### Health Checks / Debugging

- Prefer:
  - `src/moleworks_ros/mole_utils/scripts/monitor_pipeline_flow.sh`
  - `src/moleworks_ros/mole_utils/scripts/monitor_perception_stack.sh`
  - `src/moleworks_ros/mole_utils/scripts/monitor_highlevel_controllers.sh`
- If a command/topic name seems stale, confirm the authoritative names by searching:
  - `rg -n "<topic_or_service_or_action>" src/moleworks_ros`
  - Message types in `src/mole_msgs/msg/` and `src/machine_msgs/msg/`
  - Launch files under `src/moleworks_ros/*/launch/`

### Gravis Command Latch Recovery

Use this when `/machine_status` shows `is_using_gravis_commands: false` during testing.

1. Stop/interrupt the active command publisher (for example a matrix runner).
2. Publish zero velocity commands on `/mole/actuator_commands` for 2 seconds.
3. Re-check `/machine_status` and only continue when `is_using_gravis_commands: true`.
4. Restart the interrupted test and skip already-completed cases when possible.

Reference command:

```bash
ros2 topic pub -r 20 -t 2 /mole/actuator_commands mole_msgs/msg/MoleActuatorCommands \
  "{actuators: [{joint_name: 'J_TURN', mode: 2, velocity: 0.0}, {joint_name: 'J_BOOM', mode: 2, velocity: 0.0}, {joint_name: 'J_STICK', mode: 2, velocity: 0.0}, {joint_name: 'J_TELE', mode: 2, velocity: 0.0}, {joint_name: 'J_EE_PITCH', mode: 2, velocity: 0.0}]}"
```

## Maintaining Robot-Agent Docs (Avoid Duplication)

When updating operator docs:
- Update `src/moleworks_ros/docs/robot_agent/ROBOT_OPERATIONS_GUIDE.md` (add/modify content there).
- Keep `src/moleworks_ros/docs/robot_agent/ROBOT_AGENT_QUICK_REFERENCE.md` as a pointer/alias (do not duplicate content).
- Validate commands against the current repo (launch args, topic names, message fields, ROS 2 Jazzy CLI).
