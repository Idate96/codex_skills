#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $(basename "$0") <rpm|--current>" >&2
  echo "  Examples: $(basename "$0") 1600 | $(basename "$0") --current" >&2
  exit 1
fi

if [[ "${1:-}" != "--current" ]]; then
  if ! awk -v rpm="${1:-}" 'BEGIN { exit !(rpm ~ /^[0-9]+([.][0-9]+)?$/ && rpm >= 900 && rpm <= 1600) }'; then
    echo "Error: target RPM must be numeric and within the maintained 900-1600 machine range." >&2
    exit 2
  fi
fi

source_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source_setup /opt/ros/jazzy/setup.bash
fi

if [[ -n "${MOLEWORKS_ROS_SETUP:-}" ]]; then
  if [[ ! -f "$MOLEWORKS_ROS_SETUP" ]]; then
    echo "Error: MOLEWORKS_ROS_SETUP is not a file: $MOLEWORKS_ROS_SETUP" >&2
    exit 1
  fi
  source_setup "$MOLEWORKS_ROS_SETUP"
else
  workspace_candidates=(
    "${MOLEWORKS_ROS_WS:-}"
    "${MOLE_ROS_WS:-}"
    "$HOME/ros2_ws"
    "$HOME/moleworks/ros2_ws"
    "$HOME/newton_ros2_ws"
  )
  for workspace in "${workspace_candidates[@]}"; do
    if [[ -n "$workspace" && -f "$workspace/install/setup.bash" ]]; then
      source_setup "$workspace/install/setup.bash"
      break
    fi
  done
fi

machine_status_once() {
  timeout 5 ros2 topic echo --once /machine_status
}

status="$(machine_status_once)" || {
  echo "Error: no /machine_status sample received within 5 seconds." >&2
  exit 1
}
current_rpm="$(awk '/measured_engine_rpm:/ {print $2; exit}' <<<"$status")"
if [[ -z "$current_rpm" ]]; then
  echo "Error: /machine_status did not contain measured_engine_rpm." >&2
  exit 1
fi

if [[ "${1:-}" == "--current" ]]; then
  echo "Current engine RPM: $current_rpm"
  exit 0
fi

verify_machine_status_rpm() {
  local target="$1"
  local status measured deadline=$((SECONDS + 30))
  while (( SECONDS < deadline )); do
    status="$(timeout 3 ros2 topic echo --once /machine_status 2>/dev/null || true)"
    measured="$(awk '/measured_engine_rpm:/ {print $2; exit}' <<<"$status")"
    if [[ -n "$measured" ]] && awk -v rpm="$measured" -v target="$target" 'BEGIN { exit !(rpm >= target - 50 && rpm <= target + 50) }'; then
      echo "Verified measured engine RPM: $measured (target: $target)"
      return 0
    fi
    sleep 1
  done
  echo "Error: measured RPM did not reach within 50 RPM of target $target in 30 seconds (last: ${measured:-unavailable})." >&2
  return 1
}

service_list="$(timeout 10 ros2 service list 2>/dev/null || true)"
for rpm_service in /set_diesel_speed /set_rpm; do
  if ! awk -v service="$rpm_service" '$0 == service { found=1 } END { exit !found }' <<<"$service_list"; then
    continue
  fi
  rpm_type="$(timeout 10 ros2 service type "$rpm_service" 2>/dev/null || true)"
  if [[ -z "$rpm_type" ]] || ! timeout 10 ros2 interface show "$rpm_type" 2>/dev/null | awk '
      /^---$/ { exit }
      $2 == "target_rpm" { found=1 }
      END { exit !found }
    '; then
    echo "Error: $rpm_service has no verified target_rpm request field (type: ${rpm_type:-unknown})." >&2
    exit 1
  fi
  timeout 20 ros2 service call "$rpm_service" "$rpm_type" "{target_rpm: $1}"
  verify_machine_status_rpm "$1"
  exit $?
done

if ros2 pkg executables mole_highlevel_controller_cpp 2>/dev/null | \
    awk '$2 == "set_engine_rpm" { found=1 } END { exit !found }'; then
  if awk -v rpm="$1" 'BEGIN { exit !(rpm > 1500) }'; then
    echo "Error: target $1 RPM requires the direct SetRPM service; the legacy /engine_speed fallback is limited to 1500 RPM." >&2
    exit 1
  fi
  timeout 45 ros2 run mole_highlevel_controller_cpp set_engine_rpm "$1"
  verify_machine_status_rpm "$1"
  exit $?
fi

echo "Error: no target-RPM service and no installed mole_highlevel_controller_cpp/set_engine_rpm fallback." >&2
echo "Verify the low-level/Gravis stack and selected ROS overlay; do not guess a service name." >&2
exit 1
