#!/usr/bin/env bash
set -euo pipefail

if [[ $# -eq 0 ]]; then
  echo "Usage: $(basename "$0") <rpm|--current>" >&2
  echo "  Examples: $(basename "$0") 1500 | $(basename "$0") --current" >&2
  exit 1
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi

set +u
# shellcheck disable=SC1091
source "$HOME/newton_ros2_ws/install/setup.bash"
set -u

SCRIPT_PATH="$HOME/newton_ros2_ws/src/moleworks_ros/high_level_controllers/mole_highlevel_controller/mole_highlevel_controller/utils/set_engine_rpm.py"
if [[ ! -f "$SCRIPT_PATH" ]]; then
  echo "Error: set_engine_rpm.py not found at $SCRIPT_PATH" >&2
  exit 1
fi

if [[ "${1:-}" == "--current" ]]; then
  python3 "$SCRIPT_PATH" "$@"
  exit $?
fi

if command -v ros2 >/dev/null 2>&1; then
  diesel_srv="$(ros2 service list 2>/dev/null | grep -E "/set_diesel_speed$" | head -n1 || true)"
  if [[ -n "$diesel_srv" ]]; then
    diesel_type="$(ros2 service type "$diesel_srv" 2>/dev/null || true)"
    if [[ -n "$diesel_type" ]]; then
      ros2 service call "$diesel_srv" "$diesel_type" "{target_rpm: $1}"
      exit $?
    fi
  fi

  rpm_srv="$(ros2 service list 2>/dev/null | grep -E "/set_rpm$" | head -n1 || true)"
  if [[ -n "$rpm_srv" ]]; then
    rpm_type="$(ros2 service type "$rpm_srv" 2>/dev/null || true)"
    if [[ -n "$rpm_type" ]]; then
      ros2 service call "$rpm_srv" "$rpm_type" "{target_rpm: $1}"
      exit $?
    fi
  fi
fi

python3 "$SCRIPT_PATH" "$@"
