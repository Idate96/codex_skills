---
name: set-engine-rpm
description: Set or read the Mole/Menzi M4 engine RPM on the robot machine. Use when a user asks to increase/decrease/check engine RPM (e.g., "set engine rpm", "increase rpm to 1500", "check current rpm"), or when the `set_engine_rpm` console script fails and you need a direct on-machine workflow.
---

# Set Engine RPM

## Overview

Set the engine RPM on-machine. Prefer the direct SetRPM services if available (from `gravis_bridge` or the high-level controller) and fall back to the legacy `/engine_speed` stepper via `set_engine_rpm.py` when needed.

## Quick Start

- If `gravis_bridge` exposes `/set_diesel_speed`, call it directly:
  - `ros2 service call /set_diesel_speed mole_msgs/srv/SetRPM "{target_rpm: 1500}"`
- If the high-level controller exposes `/set_rpm`, call it directly:
  - `ros2 service call /set_rpm mole_highlevel_msgs/srv/SetRPM "{target_rpm: 1500}"`
- Wrapper fallback (works with `/engine_speed`): `scripts/run_set_engine_rpm.sh 1500` or `scripts/run_set_engine_rpm.sh --current`.

## Workflow

1. Ensure the robot stack is running and `/machine_status` is available.
2. Check which RPM service exists:
   - `ros2 service list | grep -Ei "set_diesel_speed|set_rpm|engine_speed"`
3. Preferred: call `/set_diesel_speed` (mole_msgs/SetRPM) if present.
4. Otherwise call `/set_rpm` (mole_highlevel_msgs/SetRPM) if present.
5. Fallback: use the wrapper script which steps `/engine_speed`.

## Commands

```bash
# Preferred (direct SetRPM services)
ros2 service call /set_diesel_speed mole_msgs/srv/SetRPM "{target_rpm: 1500}"
ros2 service call /set_rpm mole_highlevel_msgs/srv/SetRPM "{target_rpm: 1500}"

# Wrapper fallback (steps /engine_speed)
/home/lorenzo/.codex/skills/set-engine-rpm/scripts/run_set_engine_rpm.sh 1500
/home/lorenzo/.codex/skills/set-engine-rpm/scripts/run_set_engine_rpm.sh --current

# Direct fallback (no wrapper)
source /opt/ros/jazzy/setup.bash
source ~/newton_ros2_ws/install/setup.bash
python3 ~/newton_ros2_ws/src/moleworks_ros/high_level_controllers/mole_highlevel_controller/mole_highlevel_controller/utils/set_engine_rpm.py 1500
```

## Troubleshooting

- If `/set_diesel_speed` is missing, verify `gravis_bridge` is running and check `ros2 node list | grep -i gravis`.
- If `/set_rpm` is missing, verify the high-level controller node is running.
- If `set_engine_rpm` (console script) fails with pinocchio import errors, use the wrapper or direct Python path above.
- If `/engine_speed` is missing, start the low-level controller/bringup so the service exists before retrying.

## Resources

### scripts/
- `run_set_engine_rpm.sh`: Sources ROS + workspace and runs the Python RPM setter reliably.
