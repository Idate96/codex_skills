---
name: robot-move-to-position
description: "Move Mole/Menzi M445 arm joints to bounded target positions with closed-loop velocity control, live processed and raw sensor-validity gates, machine interlocks, and exclusive command ownership. Use for guarded real-robot pre-positioning with an operator present."
---

# Robot Move To Position

Use `scripts/move_joints_to_targets.py` for one direct operation: move one or more M445 arm joints to explicit targets.

Keep an operator at the machine with a working emergency stop. The script refuses motion unless:

- every target is finite, names an enumerated M445 arm joint, and is inside its built-in soft limits;
- `/mole/measurements`, `/machine_measurements`, and `/machine_status` have been received within `--max-feedback-age-sec`;
- raw `/machine_measurements.status` is `OPERATIONAL`, and position plus velocity sensor status is `CURRENT_VALID` for every target joint (`J_DIPPER` maps to `J_STICK`);
- `is_hydraulilock_unlocked`, `is_autonomous_operation_unlocked`, and `is_using_gravis_commands` are all true; and
- the ROS graph reports a stable command subscriber and no other discovered publisher on `/mole/actuator_commands`.

It rechecks processed/raw feedback validity, interlocks, the matched subscriber, and publisher exclusion every control cycle. It hard-caps velocity per joint and refuses any bounded-joint command projected to cross a soft limit. Any failure, timeout, `Ctrl+C`, or `SIGTERM` takes the same exit path and repeatedly publishes zero arm velocities after command ownership has been claimed. A failed final zero burst makes the command fail.

## Run

Single joint:

```bash
WS="$HOME/ros2_ws"; [[ -f "$WS/install/setup.bash" ]] || WS="$HOME/moleworks/ros2_ws"
source "$WS/install/setup.bash"
python3 /home/lorenzo/codex_skills/skills/robot-move-to-position/scripts/move_joints_to_targets.py \
  --confirm-hardware --confirm-safe-start \
  --target J_BOOM=-0.72 \
  --max-vel 0.35 --tol 0.02 --timeout-sec 90
```

Multiple joints use repeated `--target` arguments:

```bash
WS="$HOME/ros2_ws"; [[ -f "$WS/install/setup.bash" ]] || WS="$HOME/moleworks/ros2_ws"
source "$WS/install/setup.bash"
python3 /home/lorenzo/codex_skills/skills/robot-move-to-position/scripts/move_joints_to_targets.py \
  --confirm-hardware --confirm-safe-start \
  --target J_BOOM=-0.72 \
  --target J_STICK=1.65 \
  --target J_TELE=0.10 \
  --max-vel 0.30 --tol 0.02 --timeout-sec 120
```

`J_TURN` always uses the nearest `2*pi`-equivalent error, avoiding a long spin from an unwrapped measured angle.

## Built-in M445 Limits

| Joint | Soft minimum | Soft maximum | Max velocity |
|---|---:|---:|---:|
| `J_TURN` | -3.12 | 3.12 | 0.86 |
| `J_BOOM` | -1.39 | 0.34 | 0.45 |
| `J_STICK` | 0.61 | 2.78 | 0.62 |
| `J_TELE` | 0.02 | 1.584 | 0.64 |
| `J_EE_PITCH` | -0.64 | 2.278 | 1.30 |

Do not bypass a refusal. Stop the conflicting controller or correct the stale/locked input, then rerun the command. Use conservative `--max-vel` values and abort with `Ctrl+C` if the observed motion is wrong.
