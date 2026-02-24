#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
robot_startup_tmux.sh

Create (or update) a tmux session with standard Moleworks robot windows (in order):
  - low_level
  - perception
  - estimator
  - foxglove

Defaults assume you are already inside the Moleworks container with DDS configured.

Usage:
  robot_startup_tmux.sh [--session NAME] [--ws PATH] [--endeffector-type TYPE] [--no-estimator] [--estimator-config PATH] [--restart] [--attach]

Options:
  --session NAME   tmux session name (default: ros)
  --ws PATH        workspace path (default: ~/ros2_ws)
  --endeffector-type TYPE  end-effector type for URDF (default: prompt or 'shovel')
  --no-estimator   do not start mole_estimator (restores the legacy 3-window layout)
  --estimator-config PATH  optional config YAML to pass to mole_estimator (default: package default)
  --restart        kill existing session and recreate
  --attach         attach to the session after setup
EOF
}

SESSION="ros"
WS="${HOME}/ros2_ws"
RESTART="false"
ATTACH="false"
ENDEFFECTOR_TYPE=""
LAUNCH_ESTIMATOR="true"
ESTIMATOR_CONFIG=""
WINDOW_ORDER=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session)
      SESSION="${2:-}"; shift 2 ;;
    --ws)
      WS="${2:-}"; shift 2 ;;
    --endeffector-type)
      ENDEFFECTOR_TYPE="${2:-}"; shift 2 ;;
    --no-estimator)
      LAUNCH_ESTIMATOR="false"; shift ;;
    --estimator-config)
      ESTIMATOR_CONFIG="${2:-}"; shift 2 ;;
    --restart)
      RESTART="true"; shift ;;
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

if [[ -z "$SESSION" ]]; then
  echo "--session cannot be empty" >&2
  exit 2
fi
if [[ -z "$ENDEFFECTOR_TYPE" ]]; then
  if [[ -t 0 ]]; then
    read -r -p "End-effector type (shovel or shovel_w_teeth) [shovel]: " ENDEFFECTOR_TYPE
    ENDEFFECTOR_TYPE="${ENDEFFECTOR_TYPE:-shovel}"
  else
    ENDEFFECTOR_TYPE="shovel"
  fi
fi
if [[ -z "$ENDEFFECTOR_TYPE" ]]; then
  echo "--endeffector-type cannot be empty" >&2
  exit 2
fi

WINDOW_ORDER=(low_level perception foxglove)
if [[ "$LAUNCH_ESTIMATOR" == "true" ]]; then
  # Keep foxglove last.
  WINDOW_ORDER=(low_level perception estimator foxglove)
fi

WS="$(realpath -m "$WS")"
if [[ ! -d "$WS" ]]; then
  echo "Workspace not found: $WS" >&2
  exit 2
fi
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "Missing $WS/install/setup.bash (did you build + source the workspace?)" >&2
  exit 2
fi

tmux_has_session() {
  tmux has-session -t "$SESSION" 2>/dev/null
}

tmux_has_window() {
  local win_name="$1"
  tmux list-windows -t "$SESSION" -F '#{window_name}' 2>/dev/null | grep -Fxq "$win_name"
}

tmux_kill_session_if_exists() {
  if tmux_has_session; then
    tmux kill-session -t "$SESSION"
  fi
}

tmux_new_session_if_missing() {
  if ! tmux_has_session; then
    tmux new-session -d -s "$SESSION" -n "low_level" -c "$WS"
  fi
}

tmux_disable_auto_rename() {
  local target="$1"
  tmux set-option -w -t "$target" allow-rename off
  tmux set-option -w -t "$target" automatic-rename off
}

tmux_ensure_window() {
  local win_name="$1"
  if ! tmux_has_window "$win_name"; then
    tmux new-window -t "$SESSION" -n "$win_name" -c "$WS"
  fi
  tmux_disable_auto_rename "$SESSION:$win_name"
}

tmux_send_to_active_pane() {
  local target_window="$1"
  local cmd="$2"
  tmux select-window -t "$SESSION:$target_window"
  tmux send-keys -t "$SESSION:$target_window" "$cmd" C-m
}

tmux_base_index() {
  local base
  base="$(tmux show-options -gqv base-index 2>/dev/null || true)"
  if [[ -z "$base" ]]; then
    base=0
  fi
  echo "$base"
}

tmux_reorder_windows() {
  local base idx win current_idx
  base="$(tmux_base_index)"
  idx="$base"
  for win in "${WINDOW_ORDER[@]}"; do
    if ! tmux_has_window "$win"; then
      idx=$((idx + 1))
      continue
    fi
    current_idx="$(tmux list-windows -t "$SESSION" -F '#{window_index} #{window_name}' | awk -v w="$win" '$2==w {print $1; exit}')"
    if [[ -z "$current_idx" || "$current_idx" == "$idx" ]]; then
      idx=$((idx + 1))
      continue
    fi
    if tmux list-windows -t "$SESSION" -F '#{window_index}' | grep -Fxq "$idx"; then
      tmux swap-window -d -s "$SESSION:$win" -t "$SESSION:$idx"
    else
      tmux move-window -d -s "$SESSION:$win" -t "$SESSION:$idx"
    fi
    idx=$((idx + 1))
  done
}

start_low_level() {
  tmux_send_to_active_pane "low_level" "cd \"$WS\" && source install/setup.bash && ros2 launch mole_low_level_bringup bringup.launch.py use_sim_time:=false on_machine:=true activate_trajectory_controller:=false endeffector_type:=$ENDEFFECTOR_TYPE"
}

start_perception() {
  tmux_send_to_active_pane "perception" "cd \"$WS\" && source install/setup.bash && ros2 launch mole_perception_bringup bringup.launch.py use_sim_time:=false enable_lidar:=true enable_robot_self_filter:=true enable_elevation_mapping:=true map_name:=none endeffector_type:=$ENDEFFECTOR_TYPE"
}

start_foxglove() {
  tmux_send_to_active_pane "foxglove" "cd \"$WS\" && source install/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
}

start_estimator() {
  local cfg_arg=""
  if [[ -n "$ESTIMATOR_CONFIG" ]]; then
    # quote for the shell; ros2 launch expects config:=<path>
    cfg_arg=" config:=$(printf '%q' "$ESTIMATOR_CONFIG")"
  fi

  tmux_send_to_active_pane "estimator" "cd \"$WS\" && source install/setup.bash && ros2 launch mole_estimator mole_estimator.launch.py use_sim_time:=false urdf_xacro_endeffector_type:=$ENDEFFECTOR_TYPE${cfg_arg}"
}

start_window() {
  local win="$1"
  case "$win" in
    low_level) start_low_level ;;
    perception) start_perception ;;
    foxglove) start_foxglove ;;
    estimator) start_estimator ;;
  esac
}

if [[ "$RESTART" == "true" ]]; then
  tmux_kill_session_if_exists
fi

tmux_new_session_if_missing
tmux_disable_auto_rename "$SESSION"

# Create windows (by name; no fixed indexes).
for win in "${WINDOW_ORDER[@]}"; do
  tmux_ensure_window "$win"
done
tmux_reorder_windows

if [[ "$RESTART" == "true" ]]; then
  for win in "${WINDOW_ORDER[@]}"; do
    start_window "$win"
  done
else
  # Start commands only in windows that look idle (single pane running a shell).
  for win in "${WINDOW_ORDER[@]}"; do
    cmd="$(tmux list-panes -t "$SESSION:$win" -F '#{pane_current_command}' 2>/dev/null | head -n 1 || true)"
    if [[ "$cmd" == "bash" || "$cmd" == "zsh" || "$cmd" == "fish" || "$cmd" == "sh" ]]; then
      start_window "$win"
    fi
  done
fi

if [[ "$ATTACH" == "true" ]]; then
  exec tmux attach -t "$SESSION"
fi

echo "tmux session ready: tmux attach -t $SESSION"
