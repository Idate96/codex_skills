#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
dig_split_recording_tmux.sh

Start split recording for dig/newton runs in tmux using estimator-style bags plus a
separate elevation_map bag.

Creates one run directory with:
  sensors/ state/ commands/ lidar/ camera/ elevation_map/

Usage:
  dig_split_recording_tmux.sh --scenario NAME [options]

Options:
  --scenario NAME           Required scenario/tag (e.g. dig_newton)
  --session NAME            tmux session (default: ros)
  --window NAME             tmux window (default: record)
  --ws PATH                 ROS2 workspace (default: ~/ros2_ws)
  --output-root PATH        Output root (default: ~/rosbags/dig)
  --timestamp TS            Fixed timestamp (default: now YYYYMMDD_HHMMSS)
  --elevation-topic TOPIC   Elevation map topic (default: /mole/elevation_map_filter)
  --use-sim-time BOOL       true|false (default: false)
  --restart-window          Kill and recreate tmux window before launching
  --attach                  Attach to tmux session after setup
  -h, --help                Show this help
USAGE
}

is_bool() {
  [[ "$1" == "true" || "$1" == "false" ]]
}

SESSION="ros"
WINDOW="record"
WS="${HOME}/ros2_ws"
OUTPUT_ROOT="${HOME}/rosbags/dig"
ELEVATION_TOPIC="/mole/elevation_map_filter"
USE_SIM_TIME="false"
RESTART_WINDOW="false"
ATTACH="false"
SCENARIO=""
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario)
      SCENARIO="${2:-}"; shift 2 ;;
    --session)
      SESSION="${2:-}"; shift 2 ;;
    --window)
      WINDOW="${2:-}"; shift 2 ;;
    --ws)
      WS="${2:-}"; shift 2 ;;
    --output-root)
      OUTPUT_ROOT="${2:-}"; shift 2 ;;
    --timestamp)
      TIMESTAMP="${2:-}"; shift 2 ;;
    --elevation-topic)
      ELEVATION_TOPIC="${2:-}"; shift 2 ;;
    --use-sim-time)
      USE_SIM_TIME="${2:-}"; shift 2 ;;
    --restart-window)
      RESTART_WINDOW="true"; shift ;;
    --attach)
      ATTACH="true"; shift ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "Unknown arg: $1" >&2
      usage
      exit 2 ;;
  esac
done

if [[ -z "$SCENARIO" ]]; then
  echo "--scenario is required" >&2
  usage
  exit 2
fi
if ! is_bool "$USE_SIM_TIME"; then
  echo "--use-sim-time must be true|false (got: $USE_SIM_TIME)" >&2
  exit 2
fi

WS="$(realpath -m "$WS")"
OUTPUT_ROOT="$(realpath -m "$OUTPUT_ROOT")"
RUN_DIR="${OUTPUT_ROOT}/${SCENARIO}_${TIMESTAMP}"

if [[ ! -d "$WS" ]]; then
  echo "Workspace not found: $WS" >&2
  exit 2
fi
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "Missing $WS/install/setup.bash (build workspace first)" >&2
  exit 2
fi

ESTIMATOR_SCRIPT="$WS/src/moleworks_ros/mole_utils/mole_utils/rosbag_recording/estimator_eval_session.py"
if [[ -f "$ESTIMATOR_SCRIPT" ]]; then
  ESTIMATOR_RECORDER="python3 \"$ESTIMATOR_SCRIPT\""
elif command -v record_estimator_eval_session >/dev/null 2>&1; then
  ESTIMATOR_RECORDER="record_estimator_eval_session"
else
  ESTIMATOR_RECORDER="ros2 run mole_utils record_estimator_eval_session"
fi

mkdir -p "$OUTPUT_ROOT"

TMUX_HAS_SESSION="false"
if tmux has-session -t "$SESSION" 2>/dev/null; then
  TMUX_HAS_SESSION="true"
fi

if [[ "$TMUX_HAS_SESSION" == "false" ]]; then
  tmux new-session -d -s "$SESSION" -n "$WINDOW" -c "$WS"
fi

if tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -Fxq "$WINDOW"; then
  if [[ "$RESTART_WINDOW" == "true" ]]; then
    tmux kill-window -t "$SESSION:$WINDOW"
    tmux new-window -t "$SESSION" -n "$WINDOW" -c "$WS"
  fi
else
  tmux new-window -t "$SESSION" -n "$WINDOW" -c "$WS"
fi

tmux set-option -w -t "$SESSION:$WINDOW" allow-rename off
tmux set-option -w -t "$SESSION:$WINDOW" automatic-rename off

pane_count="$(tmux list-panes -t "$SESSION:$WINDOW" | wc -l | tr -d ' ')"
if [[ "$pane_count" -lt 2 ]]; then
  tmux split-window -h -t "$SESSION:$WINDOW" -c "$WS"
fi

left_pane="$(tmux list-panes -t "$SESSION:$WINDOW" -F '#{pane_index} #{pane_id}' | sort -n | awk 'NR==1{print $2}')"
right_pane="$(tmux list-panes -t "$SESSION:$WINDOW" -F '#{pane_index} #{pane_id}' | sort -n | awk 'NR==2{print $2}')"

tmux send-keys -t "$left_pane" C-c
sleep 0.2
tmux send-keys -t "$right_pane" C-c
sleep 0.2

EST_CMD="cd \"$WS\" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
$ESTIMATOR_RECORDER \"$SCENARIO\" \
  --output-root \"$OUTPUT_ROOT\" \
  --timestamp \"$TIMESTAMP\" \
  --record-sensors \
  --record-state \
  --record-commands \
  --record-lidar \
  --record-camera \
  --record-tf-in-sensors"

if [[ "$USE_SIM_TIME" == "true" ]]; then
  EST_CMD+=" --use-sim-time"
fi

ELEV_CMD="cd \"$WS\" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
ros2 bag record -s mcap \
  --max-cache-size 104857600 \
  --output \"$RUN_DIR/elevation_map\" \
  --topics \"$ELEVATION_TOPIC\""

tmux send-keys -t "$left_pane" "echo 'Starting estimator split recorder in: $RUN_DIR'" C-m
tmux send-keys -t "$left_pane" "$EST_CMD" C-m

tmux send-keys -t "$right_pane" "echo 'Starting elevation_map recorder in: $RUN_DIR/elevation_map'" C-m
tmux send-keys -t "$right_pane" "$ELEV_CMD" C-m

echo "Recording started in tmux: $SESSION:$WINDOW"
echo "Run directory: $RUN_DIR"
echo "Stop both panes with Ctrl-C when finished."

if [[ "$ATTACH" == "true" ]]; then
  exec tmux attach -t "$SESSION"
fi
