---
name: robot-move-check
description: "Quickly command Mole/Menzi M4 joint velocities on `/mole/actuator_commands`. Use for short, direct real-robot motion and direction checks with an operator present."
---

# Robot Move Check

Use direct JOINTVELOCITY commands only for an explicitly requested, short real-hardware motion check. Keep an operator at the controls and preserve a clear stop path.

## Required Preflight

Before publishing a nonzero command:

1. Resolve and source the active robot workspace (`~/ros2_ws` on-machine, otherwise `~/moleworks/ros2_ws` when present).
2. Confirm `/machine_status` shows the required autonomy and hydraulic interlocks unlocked.
3. Confirm `/mole/joint_states` is fresh, inspect the requested joint position, and verify the commanded direction moves away from its limit.
4. Inspect `/mole/actuator_commands` publishers. Stop if another controller owns the topic unless the user explicitly requested that handoff and the controller is safely deactivated.
5. Require an explicit joint, signed velocity, and duration. Do not infer them from an earlier run.

Use bounded readbacks during the preflight:

```bash
timeout 10 ros2 topic echo /machine_status --once
timeout 10 ros2 topic echo /mole/joint_states --once
timeout 10 ros2 topic info /mole/actuator_commands --verbose
```

Interpret the exact interlock fields using the active robot runbook; do not guess field names from an
older machine-status schema.

Do not use a one-shot nonzero command: it may remain latched. Do not leave an unbounded publisher running.

## Bounded Command

Adapt `JOINT`, `VELOCITY`, and `DURATION_SEC` to the user's explicit request. Keep the zero burst in the `EXIT` trap:

```bash
set -euo pipefail
WS="$HOME/ros2_ws"
[[ -f "$WS/install/setup.bash" ]] || WS="$HOME/moleworks/ros2_ws"
[[ -f "$WS/install/setup.bash" ]] || { echo "No built robot workspace found" >&2; exit 2; }
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

: "${JOINT:?Set the explicitly requested joint, for example J_BOOM}"
: "${VELOCITY:?Set the explicitly requested signed velocity}"
: "${DURATION_SEC:?Set the explicitly requested duration in seconds}"

zero_cmd() {
  ros2 topic pub -r 20 -t 10 /mole/actuator_commands \
    mole_msgs/msg/MoleActuatorCommands \
    "{actuators: [{joint_name: '$JOINT', mode: 2, velocity: 0.0}]}" >/dev/null
}
trap zero_cmd EXIT INT TERM

timeout --signal=INT "${DURATION_SEC}s" ros2 topic pub -r 10 /mole/actuator_commands \
  mole_msgs/msg/MoleActuatorCommands \
  "{actuators: [{joint_name: '$JOINT', mode: 2, velocity: $VELOCITY}]}" || [[ $? -eq 124 || $? -eq 130 ]]
```

Afterward, repeat the bounded joint-state, publisher, and `/machine_status` readbacks and verify the
joint velocity returned to zero. If the command publisher, state feedback, or zero burst is uncertain,
stop the test and use the operator E-stop path.
