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

source /opt/ros/jazzy/setup.bash
tuning_ws="${ROS_WS:-${HOME}/moleworks/ros2_ws}"
if [[ ! -f "${tuning_ws}/install/setup.bash" ]]; then
  tuning_ws="${HOME}/ros2_ws"
fi
if [[ ! -f "${tuning_ws}/install/setup.bash" ]]; then
  echo "No built ROS workspace found; set ROS_WS explicitly." >&2
  exit 2
fi
source "${tuning_ws}/install/setup.bash"

bag_root="${LUT_BAG_ROOT:-${HOME}/mcap/open_loop_lut}"
mkdir -p "${bag_root}"
exec ros2 run mole_sysid mole_sysid_lut_collect \
  --confirm-hardware --confirm-safe-start \
  --joint "$1" --direction "$2" --abs-current "$3" --step-s "$4" \
  --output-root "${bag_root}" \
  --settle-s "${5:-1.0}" --steady-window-s "${6:-1.0}"
