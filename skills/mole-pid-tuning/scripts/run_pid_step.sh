#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 --confirm-hardware --confirm-safe-start <joint> <pos|neg> <abs_velocity> <step_s>" >&2
}
if [[ "${1:-}" != "--confirm-hardware" || "${2:-}" != "--confirm-safe-start" ]]; then
  usage
  exit 2
fi
shift 2
if (( $# != 4 )); then
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

bag_root="${PID_BAG_ROOT:-${HOME}/mcap/pid_tuning}"
mkdir -p "${bag_root}"
exec ros2 run mole_sysid mole_sysid_pid_step \
  --confirm-hardware --confirm-safe-start \
  --joint "$1" --direction "$2" --abs-velocity "$3" --step-s "$4" \
  --output-root "${bag_root}"
