#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
terra_pipeline_tmux.sh

Start the current canonical Terra owner and, on machine, its two prerequisites.

Usage:
  terra_pipeline_tmux.sh --application normal|trench [options]

Options:
  --application TYPE       normal|trench (default: normal)
  --profile PATH           reviewed normal Terra schema-v3 profile
  --stage-manifest PATH    reviewed generated-trench stage manifest
  --handoff-bag PATH       optional reviewed trench handoff bag
  --runtime-mode MODE      machine|simulation (default: machine)
  --effective-tool TYPE    required on machine (for example shovel)
  --robot-namespace NS     robot namespace (default: mole)
  --tf-prefix PREFIX       optional TF prefix (default: empty)
  --autostart BOOL         true|false (default: false)
  --visualization MODE     none|rviz|foxglove|rviz_foxglove (default: foxglove)
  --session NAME           tmux session (default: ros)
  --ws PATH                ROS workspace (default: ~/ros2_ws)
  --owner-only             do not start low-level or estimator prerequisites
  --attach                 attach after setup
  -h, --help               show this help
EOF
}

is_bool() {
  [[ "$1" == "true" || "$1" == "false" ]]
}

shell_quote() {
  printf '%q' "$1"
}

APPLICATION="normal"
PROFILE=""
STAGE_MANIFEST=""
HANDOFF_BAG=""
RUNTIME_MODE="machine"
EFFECTIVE_TOOL=""
ROBOT_NAMESPACE="mole"
TF_PREFIX=""
AUTOSTART="false"
VISUALIZATION="foxglove"
SESSION="ros"
WS="${HOME}/ros2_ws"
OWNER_ONLY="false"
ATTACH="false"
FOXGLOVE_START_DELAY_SEC=5

while [[ $# -gt 0 ]]; do
  case "$1" in
    --application) APPLICATION="${2:-}"; shift 2 ;;
    --profile) PROFILE="${2:-}"; shift 2 ;;
    --stage-manifest) STAGE_MANIFEST="${2:-}"; shift 2 ;;
    --handoff-bag) HANDOFF_BAG="${2:-}"; shift 2 ;;
    --runtime-mode) RUNTIME_MODE="${2:-}"; shift 2 ;;
    --effective-tool) EFFECTIVE_TOOL="${2:-}"; shift 2 ;;
    --robot-namespace) ROBOT_NAMESPACE="${2:-}"; shift 2 ;;
    --tf-prefix) TF_PREFIX="${2:-}"; shift 2 ;;
    --autostart) AUTOSTART="${2:-}"; shift 2 ;;
    --visualization) VISUALIZATION="${2:-}"; shift 2 ;;
    --session) SESSION="${2:-}"; shift 2 ;;
    --ws) WS="${2:-}"; shift 2 ;;
    --owner-only) OWNER_ONLY="true"; shift ;;
    --attach) ATTACH="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

case "$APPLICATION" in
  normal|trench) ;;
  *) echo "--application must be normal|trench" >&2; exit 2 ;;
esac
case "$RUNTIME_MODE" in
  machine|simulation) ;;
  *) echo "--runtime-mode must be machine|simulation" >&2; exit 2 ;;
esac
case "$VISUALIZATION" in
  none|rviz|foxglove|rviz_foxglove) ;;
  *) echo "Unsupported --visualization: $VISUALIZATION" >&2; exit 2 ;;
esac
OWNER_VISUALIZATION="$VISUALIZATION"
START_FOXGLOVE="false"
case "$VISUALIZATION" in
  foxglove)
    OWNER_VISUALIZATION="none"
    START_FOXGLOVE="true"
    ;;
  rviz_foxglove)
    OWNER_VISUALIZATION="rviz"
    START_FOXGLOVE="true"
    ;;
esac
if ! is_bool "$AUTOSTART"; then
  echo "--autostart must be true|false" >&2
  exit 2
fi
if [[ -z "$SESSION" || -z "$WS" || -z "$ROBOT_NAMESPACE" ]]; then
  echo "session, workspace, and robot namespace must be non-empty" >&2
  exit 2
fi

WS="$(realpath -m "$WS")"
if [[ ! -f "$WS/install/local_setup.bash" ]]; then
  echo "Missing $WS/install/local_setup.bash" >&2
  exit 2
fi

if [[ "$APPLICATION" == "normal" ]]; then
  if [[ -z "$PROFILE" ]]; then
    PROFILE="$WS/src/moleworks_ros/mole_bringup/config/terra/default.yaml"
  fi
  PROFILE="$(realpath -m "$PROFILE")"
  if [[ ! -f "$PROFILE" ]]; then
    echo "Normal Terra profile not found: $PROFILE" >&2
    exit 2
  fi
  if grep -Eq '^[[:space:]]*recompute_terrain_sdf_on_target:[[:space:]]*true([[:space:]]*(#.*)?)?$' "$PROFILE"; then
    echo "Profile enables target-triggered SDF recompute: $PROFILE" >&2
    echo "Use a reviewed schema-v3 profile with recompute_terrain_sdf_on_target: false." >&2
    exit 2
  fi
elif [[ -z "$STAGE_MANIFEST" ]]; then
  echo "--stage-manifest is required for --application trench" >&2
  exit 2
else
  STAGE_MANIFEST="$(realpath -m "$STAGE_MANIFEST")"
  if [[ ! -f "$STAGE_MANIFEST" ]]; then
    echo "Stage manifest not found: $STAGE_MANIFEST" >&2
    exit 2
  fi
  stage_auto_recompute="$(python3 -c '
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    document = json.load(stream)
print(document["effective_config"]["base_graph"]["recompute_terrain_sdf_on_target"])
' "$STAGE_MANIFEST")" || {
    echo "Could not inspect the stage SDF recompute contract: $STAGE_MANIFEST" >&2
    exit 2
  }
  if [[ "$stage_auto_recompute" != "False" ]]; then
    echo "Stage must set effective_config.base_graph.recompute_terrain_sdf_on_target=false." >&2
    exit 2
  fi
fi

FOXGLOVE_PORT="8765"
if [[ "$START_FOXGLOVE" == "true" && "$APPLICATION" == "normal" ]]; then
  FOXGLOVE_PORT="$(python3 -c '
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    print(yaml.safe_load(stream)["visualization"]["foxglove_port"])
' "$PROFILE")" || {
    echo "Could not read visualization.foxglove_port from $PROFILE" >&2
    exit 2
  }
fi
if [[ "$START_FOXGLOVE" == "true" ]]; then
  if [[ ! "$FOXGLOVE_PORT" =~ ^[0-9]+$ ]] \
      || (( 10#$FOXGLOVE_PORT < 1 || 10#$FOXGLOVE_PORT > 65535 )); then
    echo "Invalid Foxglove port: $FOXGLOVE_PORT" >&2
    exit 2
  fi
fi

if [[ -n "$HANDOFF_BAG" ]]; then
  if [[ "$APPLICATION" != "trench" ]]; then
    echo "--handoff-bag is valid only for --application trench" >&2
    exit 2
  fi
  HANDOFF_BAG="$(realpath -m "$HANDOFF_BAG")"
  if [[ ! -d "$HANDOFF_BAG" ]]; then
    echo "Handoff bag not found: $HANDOFF_BAG" >&2
    exit 2
  fi
fi

if [[ "$RUNTIME_MODE" == "machine" && "$OWNER_ONLY" == "false" ]]; then
  if [[ -z "$EFFECTIVE_TOOL" ]]; then
    echo "--effective-tool is required when starting machine prerequisites" >&2
    exit 2
  fi
fi

if [[ -n "${ROS_DISCOVERY_SERVER:-}" || -n "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]]; then
  echo "Legacy DDS variables are set; recreate a fresh container shell." >&2
  exit 2
fi
if [[ -n "${MOLE_DDS_RUNTIME_PROFILE:-}" || -n "${MOLE_DDS_OBSERVER_PROFILE:-}" ]]; then
  if [[ -z "${MOLE_DDS_RUNTIME_PROFILE:-}" || -z "${MOLE_DDS_OBSERVER_PROFILE:-}" ]]; then
    echo "DDS runtime/observer split is incomplete; recreate a fresh container shell." >&2
    exit 2
  fi
fi
if [[ -n "${DDS_DISCOVERY_SERVER_IP:-}" && "${DDS_DISCOVERY_SERVER_IP}" != "127.0.0.1" ]]; then
  if [[ -z "${MOLE_DDS_RUNTIME_PROFILE:-}" || -z "${MOLE_DDS_OBSERVER_PROFILE:-}" ]]; then
    echo "Remote DDS split is unavailable; pull the current image and recreate the container." >&2
    exit 2
  fi
fi

tmux_has_session() {
  tmux has-session -t "$SESSION" 2>/dev/null
}

tmux_has_window() {
  local window="$1"
  tmux list-windows -t "$SESSION" -F '#{window_name}' 2>/dev/null | grep -Fxq "$window"
}

ensure_window() {
  local window="$1"
  if ! tmux_has_window "$window"; then
    tmux new-window -t "$SESSION" -n "$window" -c "$WS"
  fi
  tmux set-option -w -t "$SESSION:$window" allow-rename off
  tmux set-option -w -t "$SESSION:$window" automatic-rename off
}

window_command() {
  local window="$1"
  tmux list-panes -t "$SESSION:$window" -F '#{pane_current_command}' | head -n 1
}

window_is_idle() {
  case "$(window_command "$1")" in
    bash|zsh|fish|sh) return 0 ;;
    *) return 1 ;;
  esac
}

base_prefix() {
  printf 'cd %q && source /opt/ros/jazzy/setup.bash && source %q' \
    "$WS" "$WS/install/local_setup.bash"
}

start_if_idle() {
  local window="$1" command="$2"
  if window_is_idle "$window"; then
    tmux send-keys -t "$SESSION:$window" "$command" C-m
  else
    echo "$SESSION:$window already running ($(window_command "$window")); leaving it untouched." >&2
  fi
}

start_owner() {
  local command="$1"
  if ! window_is_idle terra_owner; then
    echo "$SESSION:terra_owner is already running ($(window_command terra_owner)); refusing a second owner." >&2
    return 3
  fi
  tmux send-keys -t "$SESSION:terra_owner" "$command" C-m
}

if ! tmux_has_session; then
  tmux new-session -d -s "$SESSION" -n terra_owner -c "$WS"
fi

ensure_window terra_owner

if [[ "$RUNTIME_MODE" == "simulation" ]]; then
  OWNER_ONLY="true"
fi

TF_ARG=""
if [[ -n "$TF_PREFIX" ]]; then
  TF_ARG=" tf_prefix:=$(shell_quote "$TF_PREFIX")"
fi

if [[ "$OWNER_ONLY" == "false" ]]; then
  ensure_window low_level
  ensure_window estimator

  LOW_LEVEL_CMD="$(base_prefix) && ros2 launch mole_low_level_bringup bringup.launch.py \
use_sim_time:=false on_machine:=true \
endeffector_type:=$(shell_quote "$EFFECTIVE_TOOL") \
robot_namespace:=$(shell_quote "$ROBOT_NAMESPACE")${TF_ARG}"

  ESTIMATOR_CMD="$(base_prefix) && ros2 launch mole_estimator mole_estimator.launch.py \
use_sim_time:=false urdf_xacro_endeffector_type:=$(shell_quote "$EFFECTIVE_TOOL") \
robot_namespace:=$(shell_quote "$ROBOT_NAMESPACE")${TF_ARG}"

  start_if_idle low_level "$LOW_LEVEL_CMD"
  start_if_idle estimator "$ESTIMATOR_CMD"
fi

if [[ "$APPLICATION" == "normal" ]]; then
  OWNER_CMD="$(base_prefix) && ros2 launch mole_bringup terra.launch.py \
profile:=$(shell_quote "$PROFILE") runtime_mode:=$RUNTIME_MODE \
robot_namespace:=$(shell_quote "$ROBOT_NAMESPACE")${TF_ARG} \
autostart:=$AUTOSTART visualization:=$OWNER_VISUALIZATION"
else
  OWNER_CMD="$(base_prefix) && ros2 launch mole_bringup trench.launch.py \
stage_manifest:=$(shell_quote "$STAGE_MANIFEST") runtime_mode:=$RUNTIME_MODE \
robot_namespace:=$(shell_quote "$ROBOT_NAMESPACE")${TF_ARG} \
autostart:=$AUTOSTART visualization:=$OWNER_VISUALIZATION"
  if [[ -n "$HANDOFF_BAG" ]]; then
    OWNER_CMD+=" handoff_bag:=$(shell_quote "$HANDOFF_BAG")"
  fi
fi

start_owner "$OWNER_CMD"

if [[ "$START_FOXGLOVE" == "true" ]]; then
  sleep "$FOXGLOVE_START_DELAY_SEC"
  ensure_window foxglove
  FOXGLOVE_ENV=""
  if [[ -n "${MOLE_DDS_OBSERVER_PROFILE:-}" ]]; then
    FOXGLOVE_ENV="export FASTRTPS_DEFAULT_PROFILES_FILE=$(shell_quote "$MOLE_DDS_OBSERVER_PROFILE") ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT && unset FASTDDS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER RMW_IMPLEMENTATION CYCLONEDDS_URI && "
  fi
  USE_SIM_TIME="false"
  if [[ "$RUNTIME_MODE" == "simulation" ]]; then
    USE_SIM_TIME="true"
  fi
  FOXGLOVE_CMD="$(base_prefix) && ${FOXGLOVE_ENV}ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=$FOXGLOVE_PORT use_sim_time:=$USE_SIM_TIME"
  start_if_idle foxglove "$FOXGLOVE_CMD"
fi

echo "Terra tmux session ready: $SESSION"
echo "Owner: $APPLICATION ($RUNTIME_MODE), autostart=$AUTOSTART"
if [[ "$START_FOXGLOVE" == "true" ]]; then
  echo "Foxglove: separate $SESSION:foxglove window on port $FOXGLOVE_PORT"
fi
echo "Attach with: tmux attach -t $SESSION"

if [[ "$ATTACH" == "true" ]]; then
  if [[ -n "${TMUX:-}" ]]; then
    exec tmux switch-client -t "$SESSION"
  fi
  exec tmux attach -t "$SESSION"
fi
