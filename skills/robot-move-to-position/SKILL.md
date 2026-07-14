---
name: robot-move-to-position
description: "Move Mole/Menzi M4 arm joints to target positions with closed-loop velocity control and joint-state feedback. Use for bounded real-robot positioning with an operator present."
---

# Robot Move To Position

Use the bundled `scripts/move_joints_to_targets.py` to move one or more joints to explicit targets with feedback control.

Before motion, keep an operator present, confirm the machine interlocks are unlocked, verify fresh joint states and target limits, and confirm no other publisher owns `/mole/actuator_commands`.

## Quick Commands

Single joint:

```bash
WS="$HOME/ros2_ws"; [[ -f "$WS/install/setup.bash" ]] || WS="$HOME/moleworks/ros2_ws"
source "$WS/install/setup.bash"
python3 /home/lorenzo/codex_skills/skills/robot-move-to-position/scripts/move_joints_to_targets.py \
  --target J_BOOM=-0.72 \
  --max-vel 0.35 --tol 0.02 --timeout-sec 90
```

Multiple joints:

```bash
WS="$HOME/ros2_ws"; [[ -f "$WS/install/setup.bash" ]] || WS="$HOME/moleworks/ros2_ws"
source "$WS/install/setup.bash"
python3 /home/lorenzo/codex_skills/skills/robot-move-to-position/scripts/move_joints_to_targets.py \
  --target J_BOOM=-0.72 \
  --target J_STICK=1.65 \
  --target J_TELE=0.10 \
  --max-vel 0.30 --tol 0.02 --timeout-sec 120
```

Continuous-joint note (`J_TURN`):

- The script now wraps position error to the nearest `2*pi` equivalent for joints in `--wrap-joints` (default includes `J_TURN`).
- This prevents long unnecessary spins when commanding `J_TURN=0.0` from large wrapped angles.
- Override with `--wrap-joints ""` to disable.

## Operating Rules

- Confirm machine is unlocked before motion (`is_hydraulilock_unlocked`, `is_autonomous_operation_unlocked`, `is_using_gravis_commands`).
- Require fresh `/mole/joint_states` and no conflicting `/mole/actuator_commands` publisher.
- Keep targets inside known joint limits.
- Use conservative `--max-vel` near boundaries.
- Let the script finish; it automatically publishes zero commands at exit.
- Abort anytime with `Ctrl+C`; script still sends zero commands.
