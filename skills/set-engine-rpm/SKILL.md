---
name: set-engine-rpm
description: "Read or set Mole/Menzi M4 engine RPM using the available ROS service or legacy fallback. Use for real-robot RPM checks, startup setup, and RPM-service troubleshooting."
---

# Set Engine RPM

Use the bundled wrapper so workspace discovery, service selection, valid-range checks, and measured-RPM verification stay consistent.

## Safety Contract

- Reading current RPM is non-mutating.
- Change RPM only when the user explicitly requested a target value or an already-authorized workflow specifies one.
- Require the real robot stack and `/machine_status`; do not use this skill in simulation.
- Accept only the service-defined `900-2000 RPM` range. Do not clamp or invent a target.
- Report measured readback. A successful service call without readback is not completion.

## Commands

```bash
SKILL=/home/lorenzo/codex_skills/skills/set-engine-rpm

# Read only
"$SKILL/scripts/run_set_engine_rpm.sh" --current

# Set the explicit requested target and verify within 50 RPM
"$SKILL/scripts/run_set_engine_rpm.sh" 1500

# Override workspace discovery when needed
MOLEWORKS_ROS_WS="$HOME/moleworks/ros2_ws" \
  "$SKILL/scripts/run_set_engine_rpm.sh" 1500
```

The wrapper prefers `/set_diesel_speed`, then `/set_rpm`, and falls back to the installed `set_engine_rpm.py` stepper. It checks `~/ros2_ws`, `~/moleworks/ros2_ws`, and legacy `~/newton_ros2_ws`; use `MOLEWORKS_ROS_WS` when that is ambiguous.

If no RPM service is present, verify the low-level/Gravis stack instead of guessing a service name. If readback fails or is outside ±50 RPM, report the failure and leave further machine action to the operator.
