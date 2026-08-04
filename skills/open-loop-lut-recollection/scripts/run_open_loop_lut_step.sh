#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 --confirm-hardware --confirm-safe-start <joint> <pos|neg> <abs_current> <step_s> [settle_s] [steady_window_s]" >&2
}
if [[ "${1:-}" != "--confirm-hardware" || "${2:-}" != "--confirm-safe-start" ]]; then
  usage
  exit 2
fi
shift 2
if (( $# < 4 || $# > 6 )); then
  usage
  exit 2
fi

source_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

source_setup /opt/ros/jazzy/setup.bash

if [[ -n "${MOLEWORKS_ROS_SETUP:-}" ]]; then
  setup_file="${MOLEWORKS_ROS_SETUP}"
else
  tuning_ws="${ROS_WS:-${MOLE_ROS_WS:-}}"
  if [[ -z "${tuning_ws}" || ! -f "${tuning_ws}/install/setup.bash" ]]; then
    tuning_ws="${HOME}/ros2_ws"
  fi
  if [[ ! -f "${tuning_ws}/install/setup.bash" ]]; then
    tuning_ws="${HOME}/moleworks/ros2_ws"
  fi
  setup_file="${tuning_ws}/install/setup.bash"
fi
if [[ ! -f "${setup_file}" ]]; then
  echo "No built ROS setup found; set MOLEWORKS_ROS_SETUP or ROS_WS explicitly." >&2
  exit 2
fi
source_setup "${setup_file}"

if ! ros2 pkg executables mole_sysid 2>/dev/null | awk '$2 == "mole_sysid_lut_collect" { found=1 } END { exit !found }'; then
  echo "The selected ROS overlay does not provide mole_sysid_lut_collect." >&2
  exit 2
fi

bag_root="${LUT_BAG_ROOT:-${HOME}/mcap/open_loop_lut}"
mkdir -p "${bag_root}"
exec ros2 run mole_sysid mole_sysid_lut_collect \
  --confirm-hardware --confirm-safe-start \
  --joint "$1" --direction "$2" --abs-current "$3" --step-s "$4" \
  --output-root "${bag_root}" \
  --settle-s "${5:-1.0}" --steady-window-s "${6:-1.0}"
