#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" != "--confirm-hardware" || "${2:-}" != "--confirm-safe-start" ]]; then
  echo "usage: $0 --confirm-hardware --confirm-safe-start <joint> <pos|neg> <abs_current> <step_phase_s> [settle_s] [steady_window_s]" >&2
  exit 2
fi
shift 2
if [ "$#" -lt 4 ] || [ "$#" -gt 6 ]; then
  echo "usage: $0 --confirm-hardware --confirm-safe-start <joint> <pos|neg> <abs_current> <step_phase_s> [settle_s] [steady_window_s]" >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
if [[ -f "${HOME}/ros2_ws/install/setup.bash" ]]; then
  ROS_WS="${HOME}/ros2_ws"
elif [[ -f "${HOME}/moleworks/ros2_ws/install/setup.bash" ]]; then
  ROS_WS="${HOME}/moleworks/ros2_ws"
else
  echo "No built ROS workspace found under ~/ros2_ws or ~/moleworks/ros2_ws" >&2
  exit 2
fi
source "$ROS_WS/install/setup.bash"

joint="$1"
direction="$2"
abs_current="$3"
step_phase_s="$4"
settle_s="${5:-1.0}"
steady_window_s="${6:-1.0}"
bag_root="${LUT_BAG_ROOT:-${HOME}/mcap/open_loop_lut}"

python3 - "$abs_current" "$step_phase_s" "$settle_s" "$steady_window_s" <<'PY'
import math
import sys

current, phase, settle, window = map(float, sys.argv[1:])
if not all(map(math.isfinite, (current, phase, settle, window))):
    raise SystemExit("all numeric arguments must be finite")
if not 0.0 < current <= 1.0:
    raise SystemExit("abs_current must be in (0, 1.0] A")
if phase <= 0.0 or settle < 0.0 or window <= 0.0:
    raise SystemExit("step phase/window must be positive and settle must be non-negative")
PY

if timeout 10 ros2 node list | rg -q 'mole_pid_joint_controller'; then
  echo "Refusing: mole_pid_joint_controller is still running" >&2
  exit 2
fi
current_topic_info="$(timeout 10 ros2 topic info /mole/current_commands 2>/dev/null || true)"
if printf '%s\n' "$current_topic_info" | rg -q 'Publisher count: [1-9]'; then
  echo "Refusing: /mole/current_commands already has a publisher" >&2
  printf '%s\n' "$current_topic_info" >&2
  exit 2
fi
timeout 10 ros2 topic echo /mole/state --once >/dev/null
timeout 10 ros2 topic echo /machine_measurements --once >/dev/null

case "$direction" in
  pos) step_current="$abs_current" ;;
  neg) step_current="-$abs_current" ;;
  *)
    echo "direction must be pos or neg" >&2
    exit 2
    ;;
esac

imil="$(python3 -c 'import sys; print(int(round(float(sys.argv[1]) * 1000)))' "$abs_current")"
joint_tag="$(printf '%s' "$joint" | tr '[:upper:]' '[:lower:]')"
tag="${joint_tag}_lut_${direction}_i$(printf '%03d' "$imil")_$(date +%Y%m%d_%H%M%S)"
bag="${bag_root}/${tag}"
analysis="${bag_root}/analysis_${tag}_lut"

echo "RUN ${joint} ${direction} ${step_current}A"
echo "BAG ${bag}"
echo "ANALYSIS ${analysis}"

ros2 bag record -s mcap --storage-preset-profile fastwrite \
  -o "$bag" \
  --topics /mole/current_commands /mole/actuator_commands /mole/state /mole/joint_states /mole/measurements /machine_measurements /tf \
  </dev/null >|"${bag}.record.log" 2>&1 &
rec_pid=$!

cleanup_recorder() {
  if [[ -n "${rec_pid:-}" ]] && kill -0 "$rec_pid" 2>/dev/null; then
    kill -INT "$rec_pid" 2>/dev/null || true
    sleep 1
    kill -TERM "$rec_pid" 2>/dev/null || true
  fi
  if [[ -n "${rec_pid:-}" ]]; then
    wait "$rec_pid" 2>/dev/null || true
  fi
}
trap cleanup_recorder EXIT

sleep 2

ros2 run mole_sysid mole_sysid_joint_current_sequence \
  --joint-name "$joint" \
  --step-current "$step_current" \
  --max-abs-current 1.0 \
  --skip-return \
  --baseline-s 1.0 \
  --step-phase-s "$step_phase_s" \
  --pre-reset-stop-s 0.0 \
  --final-hold-s 1.0 \
  --rate 50

cleanup_recorder
trap - EXIT

test -f "$bag/metadata.yaml"
find "$bag" -maxdepth 1 -type f -name '*.mcap' -print -quit | rg -q .
ros2 bag info "$bag" >/dev/null

ros2 run mole_sysid mole_sysid_tune_lut \
  "$bag" \
  "$joint" \
  "$analysis" \
  --command-topic /mole/current_commands \
  --state-topic /machine_measurements \
  --direction "$direction" \
  --settle-s "$settle_s" \
  --steady-window-s "$steady_window_s" \
  --low-speed-settle-s "$settle_s" \
  --low-speed-steady-window-s "$steady_window_s" \
  --max-command-std 0.03 \
  --low-speed-max-command-std 0.03 \
  --low-speed-focus-abs-command 0.12 \
  --plot
