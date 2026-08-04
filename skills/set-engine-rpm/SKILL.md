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
- Use `1600 RPM` as the maintained autonomous operating target.
- Accept `900-1600 RPM` through this wrapper. The direct `SetRPM` interface is
  documented for 900-2000 RPM, while the legacy stepwise `/engine_speed`
  fallback is limited to 900-1500 RPM; therefore 1600 requires the direct
  service. Targets above 1600 require a separately reviewed machine-specific
  workflow.
- Report measured readback. A successful service call without readback is not completion.

## Commands

```bash
SKILL_DIR="${CODEX_HOME:-$HOME/.codex}/skills/set-engine-rpm"
SET_RPM="$SKILL_DIR/scripts/run_set_engine_rpm.sh"

# Read only
"$SET_RPM" --current

# Set the explicit requested target and verify within 50 RPM
"$SET_RPM" 1600

# Select a custom install overlay when needed
MOLEWORKS_ROS_SETUP=/path/to/install/setup.bash "$SET_RPM" 1600
```

The wrapper reads `/machine_status` directly for `--current`. For a change it prefers
`/set_diesel_speed`, then `/set_rpm`, after verifying that the discovered service request contains
`target_rpm`; it falls back to the installed `mole_highlevel_controller_cpp set_engine_rpm` stepper,
which uses `/engine_speed`. It checks `~/ros2_ws`, `~/moleworks/ros2_ws`, and legacy
`~/newton_ros2_ws`; use `MOLEWORKS_ROS_WS` when that is ambiguous.
Use `MOLEWORKS_ROS_SETUP` when the desired install is not `<workspace>/install`, such as a dated clean-build overlay.

If no RPM service is present, verify the low-level/Gravis stack instead of guessing a service name. If readback fails or is outside ±50 RPM, report the failure and leave further machine action to the operator.
