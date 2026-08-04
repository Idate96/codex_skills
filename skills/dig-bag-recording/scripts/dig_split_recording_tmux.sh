#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
dig_split_recording_tmux.sh

Start split recording for dig/newton runs in tmux using the canonical rosbag_record launch.

Creates one run directory with:
  raw/sensors/ raw/state/ raw/commands/ raw/lidar/ raw/camera/ raw/elevation_map/
For the Dig3D controller it also records:
  raw/dig3d_special_obs/

Usage:
  dig_split_recording_tmux.sh --scenario NAME [options]

Options:
  --scenario NAME           Required scenario/tag (e.g. dig_newton)
  --controller NAME         dig3d|newton|dig|dig-ee|ugep (default: infer from scenario)
  --session NAME            tmux session (default: ros)
  --window NAME             tmux window (default: record)
  --ws PATH                 ROS2 workspace (default: auto-detect a built local workspace)
  --output-root PATH        Output root (default: ~/mcap/dig)
  --timestamp TS            Fixed timestamp (default: now YYYYMMDD_HHMMSS)
  --robot-namespace NS      Robot namespace (default: mole)
  --actuator-commands-topic TOPIC
                            Command topic override (default: namespace-derived)
  --elevation-topic TOPIC   Primary map topic (default: /excavation_mapping/grid_map)
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
if [[ -f "${HOME}/ros2_ws/install/setup.bash" ]]; then
  WS="${HOME}/ros2_ws"
elif [[ -f "${HOME}/moleworks/ros2_ws/install/setup.bash" ]]; then
  WS="${HOME}/moleworks/ros2_ws"
else
  WS="${HOME}/ros2_ws"
fi
OUTPUT_ROOT="${HOME}/mcap/dig"
ELEVATION_TOPIC="/excavation_mapping/grid_map"
ROBOT_NAMESPACE="mole"
ACTUATOR_COMMANDS_TOPIC=""
USE_SIM_TIME="false"
RESTART_WINDOW="false"
ATTACH="false"
SCENARIO=""
CONTROLLER=""
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario)
      SCENARIO="${2:-}"; shift 2 ;;
    --controller)
      CONTROLLER="${2:-}"; shift 2 ;;
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
    --robot-namespace)
      ROBOT_NAMESPACE="${2:-}"; shift 2 ;;
    --actuator-commands-topic)
      ACTUATOR_COMMANDS_TOPIC="${2:-}"; shift 2 ;;
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
if [[ ! "$SCENARIO" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "--scenario must be a path-safe tag using letters, digits, dot, underscore, or dash" >&2
  exit 2
fi
if [[ ! "$TIMESTAMP" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "--timestamp must be a path-safe value using letters, digits, dot, underscore, or dash" >&2
  exit 2
fi
if [[ -z "$ROBOT_NAMESPACE" ]]; then
  echo "--robot-namespace cannot be empty" >&2
  exit 2
fi
if ! is_bool "$USE_SIM_TIME"; then
  echo "--use-sim-time must be true|false (got: $USE_SIM_TIME)" >&2
  exit 2
fi

if [[ -z "$CONTROLLER" ]]; then
  case "$SCENARIO" in
    dig3d*|dig_3d*|dig-3d*) CONTROLLER="dig3d" ;;
    *newton*) CONTROLLER="newton" ;;
    dig-ee*|dig_ee*) CONTROLLER="dig-ee" ;;
    *ugep*) CONTROLLER="ugep" ;;
    dig|dig-*) CONTROLLER="dig" ;;
    *) CONTROLLER="none" ;;
  esac
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

mkdir -p "$OUTPUT_ROOT"
echo "Output filesystem free space:"
df -h "$OUTPUT_ROOT"

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
  else
    while IFS='|' read -r pane_id pane_command; do
      case "$pane_command" in
        bash|zsh|fish|sh) ;;
        *)
          echo "Refusing to replace busy pane $pane_id in $SESSION:$WINDOW (command: ${pane_command:-unknown}). Use --restart-window only when replacement is explicitly authorized." >&2
          exit 3
          ;;
      esac
    done < <(tmux list-panes -t "$SESSION:$WINDOW" -F '#{pane_id}|#{pane_current_command}')
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

RECORD_DIG3D_SPECIAL_OBS="false"
RECORD_ACTIONS="true"
ACTION_NAME=""
EXTRA_STATE_TOPICS=""
case "$CONTROLLER" in
  dig3d)
    RECORD_DIG3D_SPECIAL_OBS="true"
    ACTION_NAME="/$ROBOT_NAMESPACE/run_dig_3d"
    EXTRA_STATE_TOPICS="/dig_3d/actual_joint_velocity,/dig_3d/torque_lower_limit,/dig_3d/torque_upper_limit,/dig_3d/torque_saturation"
    ;;
  newton) ACTION_NAME="/$ROBOT_NAMESPACE/run_dig_newton" ;;
  dig) ACTION_NAME="/$ROBOT_NAMESPACE/run_dig" ;;
  dig-ee|dig_ee) ACTION_NAME="/$ROBOT_NAMESPACE/run_dig_ee" ;;
  ugep)
    ACTION_NAME="/$ROBOT_NAMESPACE/run_dig_ugep"
    if [[ -z "$ACTUATOR_COMMANDS_TOPIC" ]]; then
      ACTUATOR_COMMANDS_TOPIC="/$ROBOT_NAMESPACE/actuator_commands_ugep_dryrun"
    fi
    EXTRA_STATE_TOPICS="/$ROBOT_NAMESPACE/dig_ugep/observations,/$ROBOT_NAMESPACE/dig_ugep/observations_raw,/$ROBOT_NAMESPACE/dig_ugep/policy_action_stamped,/$ROBOT_NAMESPACE/dig_ugep/commanded_joint_velocity,/$ROBOT_NAMESPACE/dig_ugep/pullup_distance_locked,/$ROBOT_NAMESPACE/dig_ugep/scooped_soil_volume,/$ROBOT_NAMESPACE/dig_ugep/filled_soil_volume"
    ;;
  none) RECORD_ACTIONS="false" ;;
  *)
    echo "Unknown --controller: $CONTROLLER" >&2
    exit 2
    ;;
esac

# Keep the launch argument explicit. Passing an empty `:=` value through the
# tmux command string is malformed, and the documented default is the
# namespace-derived live command topic for every non-UGEP controller.
if [[ -z "$ACTUATOR_COMMANDS_TOPIC" ]]; then
  ACTUATOR_COMMANDS_TOPIC="/$ROBOT_NAMESPACE/actuator_commands"
fi

RECORD_CMD="cd \"$WS\" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
ros2 launch mole_bag_tools rosbag_record.launch.py \
  bag_path:=\"$RUN_DIR\" \
  append_timestamp:=false \
  record_sensors:=true \
  record_state:=true \
  record_commands:=true \
  record_lidar:=true \
  record_camera:=true \
  record_elevation_map:=true \
  record_dig3d_special_obs:=\"$RECORD_DIG3D_SPECIAL_OBS\" \
  record_controller_observations:=true \
  record_actions:=\"$RECORD_ACTIONS\" \
  action_name:=\"$ACTION_NAME\" \
  robot_namespace:=\"$ROBOT_NAMESPACE\" \
  actuator_commands_topic:=\"$ACTUATOR_COMMANDS_TOPIC\" \
  extra_state_topics:=\"$EXTRA_STATE_TOPICS\" \
  use_sim_time:=\"$USE_SIM_TIME\" \
  elevation_map_topic:=\"$ELEVATION_TOPIC\""

WATCH_CMD="printf 'Watching bags under: %s\n\n' \"$RUN_DIR\"; while true; do \
  date; \
  find \"$RUN_DIR/raw\" -maxdepth 2 -type f -name metadata.yaml | sort; \
  sleep 2; \
  printf '\033[2J\033[H'; \
done"

tmux send-keys -t "$left_pane" "echo 'Starting canonical split recorder in: $RUN_DIR'" C-m
tmux send-keys -t "$left_pane" "$RECORD_CMD" C-m

tmux send-keys -t "$right_pane" "$WATCH_CMD" C-m

echo "Recording started in tmux: $SESSION:$WINDOW"
echo "Run directory: $RUN_DIR"
echo "Recording profile: controller=$CONTROLLER robot_namespace=$ROBOT_NAMESPACE"
echo "Stop the recorder with Ctrl-C in the left pane when finished."

if [[ "$ATTACH" == "true" ]]; then
  exec tmux attach -t "$SESSION"
fi
