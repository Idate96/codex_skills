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

workspace_candidates=()
if [[ -n "${MOLEWORKS_ROS_WS:-}" ]]; then
  workspace_candidates+=("$MOLEWORKS_ROS_WS")
fi
workspace_candidates+=(
  "$HOME/ros2_ws"
  "$HOME/moleworks/ros2_ws"
  "$HOME/newton_ros2_ws"
)

ACTIVE_WS=""
SCRIPT_PATH=""
FALLBACK_WS=""
for ws in "${workspace_candidates[@]}"; do
  if [[ -z "$FALLBACK_WS" && -f "$ws/install/setup.bash" ]]; then
    FALLBACK_WS="$ws"
  fi
  for relative_script in \
    "high_level_controllers/mole_highlevel_controller_cpp/scripts/set_engine_rpm.py" \
    "high_level_controllers/mole_highlevel_controller/mole_highlevel_controller/utils/set_engine_rpm.py"; do
    candidate="$ws/src/moleworks_ros/$relative_script"
    if [[ -f "$ws/install/setup.bash" && -f "$candidate" ]]; then
      ACTIVE_WS="$ws"
      SCRIPT_PATH="$candidate"
      break 2
    fi
  done
done

if [[ -z "$ACTIVE_WS" ]]; then
  ACTIVE_WS="$FALLBACK_WS"
fi
if [[ -n "$ACTIVE_WS" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$ACTIVE_WS/install/setup.bash"
  set -u
fi

if [[ "${1:-}" == "--current" ]]; then
  if [[ -z "$SCRIPT_PATH" ]]; then
    echo "Error: set_engine_rpm.py not found in any known Moleworks ROS workspace." >&2
    echo "Set MOLEWORKS_ROS_WS to the workspace root and retry." >&2
    exit 1
  fi
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

if [[ -z "$SCRIPT_PATH" ]]; then
  echo "Error: set_engine_rpm.py not found in any known Moleworks ROS workspace." >&2
  echo "Set MOLEWORKS_ROS_WS to the workspace root and retry." >&2
  exit 1
fi

python3 "$SCRIPT_PATH" "$@"
