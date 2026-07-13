---
name: set-engine-rpm
description: Read or set Mole/Menzi M4 engine RPM using the available ROS service or legacy fallback.
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

# Explicit workspace override when auto-discovery is ambiguous
MOLEWORKS_ROS_WS=~/moleworks/ros2_ws \
  /home/lorenzo/.codex/skills/set-engine-rpm/scripts/run_set_engine_rpm.sh 1500
```

## Troubleshooting

- If `/set_diesel_speed` is missing, verify `gravis_bridge` is running and check `ros2 node list | grep -i gravis`.
- If `/set_rpm` is missing, verify the high-level controller node is running.
- If `set_engine_rpm` (console script) fails with pinocchio import errors, use the wrapper. It checks `~/ros2_ws`, `~/moleworks/ros2_ws`, and legacy `~/newton_ros2_ws`; set `MOLEWORKS_ROS_WS` to override discovery.
- If `/engine_speed` is missing, start the low-level controller/bringup so the service exists before retrying.

## Resources

### scripts/
- `run_set_engine_rpm.sh`: Sources ROS plus the first available Moleworks workspace, prefers live RPM services, and falls back to the Python setter.
