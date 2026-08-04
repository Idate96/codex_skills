---
name: robot-move-to-position
description: "Move Mole/Menzi M445 arm joints, including tool roll, to bounded target positions with closed-loop velocity control, processed/raw sensor-validity gates, machine interlocks, and exclusive command ownership. Use for guarded real-robot pre-positioning with an operator present."
---

# Robot Move To Position

Use `scripts/move_joints_to_targets.py` for one direct operation: move one or more M445 arm joints to explicit targets.

Keep an operator at the machine with a working emergency stop. The script refuses motion unless:

- every target is finite, names an enumerated M445 arm joint, and is inside its built-in soft limits;
- `/mole/measurements`, `/machine_measurements`, and `/machine_status` have been received within `--max-feedback-age-sec`;
- raw `/machine_measurements.status` is `OPERATIONAL`, and position plus velocity sensor status is `CURRENT_VALID` for every target joint (`J_DIPPER` maps to `J_STICK`);
- `is_hydraulilock_unlocked`, `is_autonomous_operation_unlocked`, and `is_using_gravis_commands` are all true; and
- the ROS graph reports exactly one `mole_pid_joint_controller` command subscriber and no other discovered publisher on `/mole/actuator_commands`.

It creates a command-topic probe plus a disarmed writer before claiming ownership so a Discovery
Server CLIENT can discover competing writers and the expected PID reader. It publishes nothing until
all preflight gates pass. It rechecks processed/raw feedback validity, interlocks, the matched PID
subscriber, and publisher exclusion every control cycle. It hard-caps velocity per joint and refuses
any bounded-joint command projected to cross a soft limit. Any failure, timeout, `Ctrl+C`, or
`SIGTERM` takes the same exit path and repeatedly publishes zero arm velocities after command
ownership has been claimed. A failed final zero burst makes the command fail.

## Run

Single joint:

```bash
WS="${MOLE_ROS_WS:-$HOME/ros2_ws}"; [[ -f "$WS/install/setup.bash" ]] || WS="$HOME/moleworks/ros2_ws"
source "$WS/install/setup.bash"
SKILL_DIR="${CODEX_HOME:-$HOME/.codex}/skills/robot-move-to-position"
python3 "$SKILL_DIR/scripts/move_joints_to_targets.py" \
  --confirm-hardware --confirm-safe-start \
  --target J_BOOM=-0.72 \
  --max-vel 0.35 --tol 0.02 --timeout-sec 90
```

Multiple joints use repeated `--target` arguments:

```bash
WS="${MOLE_ROS_WS:-$HOME/ros2_ws}"; [[ -f "$WS/install/setup.bash" ]] || WS="$HOME/moleworks/ros2_ws"
source "$WS/install/setup.bash"
SKILL_DIR="${CODEX_HOME:-$HOME/.codex}/skills/robot-move-to-position"
python3 "$SKILL_DIR/scripts/move_joints_to_targets.py" \
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
| `J_TURN` | -3.14 | 3.14 | 0.86 |
| `J_BOOM` | -1.33 | -0.38 | 0.45 |
| `J_STICK` | 0.67 | 2.72 | 0.62 |
| `J_TELE` | 0.08 | 1.524 | 0.64 |
| `J_EE_PITCH` | -0.58 | 2.218 | 1.30 |
| `J_EE_ROLL` | -0.596 | 0.505 | 0.25 |

These duplicate the maintained soft bounds in `mole_sysid.robot_tuning.M445_ARM_SAFETY`; compare
them during future audits. Do not target `J_EE_YAW` while its raw velocity feedback is unsupported.
Do not bypass a refusal. Stop the conflicting controller or correct the stale/locked input, then rerun
the command. Use conservative `--max-vel` values and abort with `Ctrl+C` if the observed motion is wrong.
