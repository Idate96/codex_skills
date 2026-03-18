#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 4 ] || [ "$#" -gt 6 ]; then
  echo "usage: $0 <joint> <pos|neg> <abs_current> <step_phase_s> [settle_s] [steady_window_s]" >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

joint="$1"
direction="$2"
abs_current="$3"
step_phase_s="$4"
settle_s="${5:-1.0}"
steady_window_s="${6:-1.0}"
bag_root="${LUT_BAG_ROOT:-/home/lorenzo/rosbags_from_diego_20260304}"

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

kill -INT "$rec_pid" 2>/dev/null || true
sleep 1
if kill -0 "$rec_pid" 2>/dev/null; then
  kill -TERM "$rec_pid" 2>/dev/null || true
fi
wait "$rec_pid" || true

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
