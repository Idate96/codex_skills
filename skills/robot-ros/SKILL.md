---
name: robot-ros
description: "Route Mole/Menzi M4 ROS operations to the current owning runbook or narrow skill. Use to orient robot bringup, controllers, interlocks, motion, monitoring, recovery, recording, or code/runbook maintenance without duplicating subsystem procedures."
---

# Robot ROS Router

Use this skill to identify the owner, establish the safety boundary, and hand
off. Do not grow it into a second robot runbook.

## Establish Context Read-Only

1. Resolve the workspace that owns the live graph and its
   `src/moleworks_ros` checkout. Prefer explicit process/tmux evidence over a
   guessed path.
2. Read `docs/robot_agent/robot_agent.md`, then the linked operations or
   troubleshooting guide relevant to the request.
3. Inspect the existing tmux/process owners, `ROS_DOMAIN_ID`, requested
   endpoints, and publisher ownership with bounded read-only probes before any
   restart or control action.
4. Use the sourced environment that owns the graph. The contributor container
   is a supported default, but the current robot installation may run directly
   from its host workspace.
5. Treat physical motion, hydraulics, ignition, RPM, controller activation,
   service calls that advance execution, and process replacement as separate
   authorization boundaries.

## Route To The Narrow Owner

| Request | Owner |
|---|---|
| Start/restart base robot panes, tool geometry, hydraulics | `robot-startup` |
| Read or set engine RPM | `set-engine-rpm` |
| Start/restart only a DIG controller | `dig-controllers` |
| Direct short joint direction/motion check | `robot-move-check` |
| Guarded closed-loop arm positioning | `robot-move-to-position` |
| PID step tuning or LUT recollection | `mole-pid-tuning` / `open-loop-lut-recollection` |
| OCS2 arm run or action experiment | `ocs2-arm-experiments` |
| OCS2 tuning iteration | `ocs2-tuning-fastloop` |
| Terra application start/resume/monitor/stop | `terra-pipeline` |
| Beam6 manifest generation or handoff preparation | `terra-trench` |
| Generic topic, node, service, TF, DDS, or tmux diagnosis | `ros2-debugging` |
| DIG recording or replay | `dig-bag-recording` / `dig-bag-replay` |
| Estimator bag reprocessing/evaluation | `state-estimator-evaluate-bags` |
| Estimator-container bringup | `mole-graph-msf-container` |
| Sparse or camera-colored LiDAR export | `mole-lidar-accumulator` / `open3d-mapping` |

If no narrow skill owns the operation, use the current package README and
launch/interface source instead of adding procedure here.

## Current Documentation Owners

- Operator entrypoint: `docs/robot_agent/robot_agent.md`
- Bringup/readiness/shutdown: `docs/robot_agent/ROBOT_OPERATIONS_GUIDE.md`
- First-response diagnosis: `docs/robot_agent/TROUBLESHOOTING_GUIDE.md`
- Top-level applications: `mole_bringup/README.md` and `mole_bringup/launch/`
- Low-level ownership/interlocks: `low_level/README.md` plus imported
  `machine_msgs`/`gravis_bridge` interfaces
- Perception surface: `perception/mole_perception_bringup/README.md` and its
  launch file
- Estimator readiness: `mole_estimator/README.md`
- Menzi/Gravis raw interface:
  `docs/src/subsystems/information/menzi_m445x_gravis_rack.md`

For network gateway selection on `rslpc`, use the Network section of the
operations guide and verify `ip route show default`; do not copy gateway or
netplan edits into this router.

## Command-Acceptance Recovery

When `/machine_status.is_using_gravis_commands` is false, stop the active
high-level command owner, make the machine physically safe, and follow the
troubleshooting guide. Inspect publishers on
`/${ROBOT_NAMESPACE:-mole}/actuator_commands` and
`/${ROBOT_NAMESPACE:-mole}/current_commands` before acting.

Do not use a hand-written partial zero `MoleActuatorCommands` message as a
generic recovery step. The current operator guide explicitly keeps raw command
recovery outside this router; a zero ROS message is not an emergency stop.

## Maintaining Robot Documentation

Verify every changed launch argument, topic, service, action, message field,
TF frame, and profile against current code and the installed Jazzy CLI. Update
the owning package README or robot-agent guide and link to it. Do not recreate
retired quick-reference aliases or duplicate complete workflows in skills,
`AGENTS.md`, and operator docs.
