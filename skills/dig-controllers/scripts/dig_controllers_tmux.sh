#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
dig_controllers_tmux.sh

Create (or update) a tmux window split into 2 panes:
  - left: launch selected dig controller
  - right: lifecycle helpers + action send_goal helper

Defaults assume you are already inside the Moleworks container with DDS configured.

Usage:
  dig_controllers_tmux.sh --controller NAME [options] [-- <extra ros2 launch args...>]

Controllers:
  dig3d    -> dig_3d_controller_cpp.launch.py
  newton   -> dig_newton_controller.launch.py
  dig      -> dig_controller_cpp.launch.py
  dig-ee   -> dig_ee_controller_cpp.launch.py
  ugep     -> dig_ugep_controller_cpp.launch.py (manifest required via extra args)

Options:
  --controller NAME     controller key (required)
  --session NAME        tmux session name (default: ros)
  --window NAME         tmux window name (default: dig)
  --ws PATH             workspace path (default: auto-detect ~/ros2_ws, then ~/moleworks/ros2_ws)
  --use-sim-time BOOL   true|false (default: false)
  --robot-namespace NS  controller namespace (default: mole)
  --tf-prefix PREFIX    robot-local TF prefix (default: empty)
  --no-activate         don't auto configure/activate (only print commands)
  --run-action          send the action goal after activation
  --restart-window      kill and recreate the tmux window
  --attach              attach to the session after setup
EOF
}

is_bool() {
  [[ "$1" == "true" || "$1" == "false" ]]
}

SESSION="ros"
WINDOW="dig"
WS=""
USE_SIM_TIME="false"
AUTO_ACTIVATE="true"
RUN_ACTION="false"
RESTART_WINDOW="false"
ATTACH="false"
ROBOT_NAMESPACE="mole"
TF_PREFIX=""
CONTROLLER=""
EXTRA_LAUNCH_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --controller)
      CONTROLLER="${2:-}"; shift 2 ;;
    --session)
      SESSION="${2:-}"; shift 2 ;;
    --window)
      WINDOW="${2:-}"; shift 2 ;;
    --ws)
      WS="${2:-}"; shift 2 ;;
    --use-sim-time)
      USE_SIM_TIME="${2:-}"; shift 2 ;;
    --robot-namespace)
      ROBOT_NAMESPACE="${2:-}"; shift 2 ;;
    --tf-prefix)
      TF_PREFIX="${2:-}"; shift 2 ;;
    --no-activate)
      AUTO_ACTIVATE="false"; shift ;;
    --run-action)
      RUN_ACTION="true"; shift ;;
    --restart-window)
      RESTART_WINDOW="true"; shift ;;
    --attach)
      ATTACH="true"; shift ;;
    --)
      shift
      EXTRA_LAUNCH_ARGS=("$@")
      break ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "Unknown arg: $1" >&2
      usage
      exit 2 ;;
  esac
done

if [[ -z "$CONTROLLER" ]]; then
  echo "--controller is required" >&2
  usage
  exit 2
fi
if [[ -z "$SESSION" ]]; then
  echo "--session cannot be empty" >&2
  exit 2
fi
if [[ -z "$WINDOW" ]]; then
  echo "--window cannot be empty" >&2
  exit 2
fi
if ! is_bool "$USE_SIM_TIME"; then
  echo "--use-sim-time must be true|false (got: $USE_SIM_TIME)" >&2
  exit 2
fi
if [[ -z "$ROBOT_NAMESPACE" ]]; then
  echo "--robot-namespace cannot be empty" >&2
  exit 2
fi
if [[ ! "$ROBOT_NAMESPACE" =~ ^[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$ ]]; then
  echo "--robot-namespace must be a valid relative ROS namespace" >&2
  exit 2
fi
if [[ -n "$TF_PREFIX" && ! "$TF_PREFIX" =~ ^[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$ ]]; then
  echo "--tf-prefix must contain only ROS frame-prefix characters" >&2
  exit 2
fi
if [[ "$CONTROLLER" == "ugep" ]]; then
  have_manifest="false"
  for arg in "${EXTRA_LAUNCH_ARGS[@]}"; do
    if [[ "$arg" == policy_manifest_path:=* && "$arg" != "policy_manifest_path:=" ]]; then
      have_manifest="true"
      break
    fi
  done
  if [[ "$have_manifest" != "true" ]]; then
    echo "UGEP requires policy_manifest_path:=... after --" >&2
    exit 2
  fi
fi

if [[ -z "$WS" ]]; then
  for candidate in "${HOME}/ros2_ws" "${HOME}/moleworks/ros2_ws"; do
    if [[ -f "$candidate/install/setup.bash" ]]; then
      WS="$candidate"
      break
    fi
  done
fi
if [[ -z "$WS" ]]; then
  echo "Could not auto-detect a built workspace; pass --ws PATH" >&2
  exit 2
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

PKG=""
LAUNCH_FILE=""
NODE_NAME=""
ACTION_NAME=""
BASE_LAUNCH_ARGS=()

qualify_node_name() {
  local namespace="$1"
  local node="$2"
  if [[ -z "$namespace" ]]; then
    printf '/%s\n' "$node"
  else
    printf '/%s/%s\n' "$namespace" "$node"
  fi
}

case "$CONTROLLER" in
  dig3d)
    PKG="mole_highlevel_controller_cpp"
    LAUNCH_FILE="dig_3d_controller_cpp.launch.py"
    NODE_NAME="$(qualify_node_name "$ROBOT_NAMESPACE" "dig_3d_controller")"
    ACTION_NAME="$(qualify_node_name "$ROBOT_NAMESPACE" "run_dig_3d")"
    BASE_LAUNCH_ARGS=(
      "use_sim_time:=$USE_SIM_TIME"
      "activate_controller:=false"
      "run_action:=false"
      "robot_namespace:=$ROBOT_NAMESPACE"
    )
    ;;
  newton)
    PKG="mole_highlevel_controller_cpp"
    LAUNCH_FILE="dig_newton_controller.launch.py"
    NODE_NAME="$(qualify_node_name "$ROBOT_NAMESPACE" "dig_newton_controller")"
    ACTION_NAME="$(qualify_node_name "$ROBOT_NAMESPACE" "run_dig_newton")"
    BASE_LAUNCH_ARGS=(
      "use_sim_time:=$USE_SIM_TIME"
      "activate_controller:=false"
      "run_action:=false"
      "robot_namespace:=$ROBOT_NAMESPACE"
    )
    ;;
  dig)
    PKG="mole_highlevel_controller_cpp"
    LAUNCH_FILE="dig_controller_cpp.launch.py"
    NODE_NAME="$(qualify_node_name "$ROBOT_NAMESPACE" "dig_controller")"
    ACTION_NAME="$(qualify_node_name "$ROBOT_NAMESPACE" "run_dig")"
    BASE_LAUNCH_ARGS=(
      "use_sim_time:=$USE_SIM_TIME"
      "activate_controller:=false"
      "run_action:=false"
      "robot_namespace:=$ROBOT_NAMESPACE"
    )
    ;;
  dig-ee|dig_ee)
    PKG="mole_highlevel_controller_cpp"
    LAUNCH_FILE="dig_ee_controller_cpp.launch.py"
    NODE_NAME="$(qualify_node_name "$ROBOT_NAMESPACE" "dig_ee_controller")"
    ACTION_NAME="$(qualify_node_name "$ROBOT_NAMESPACE" "run_dig_ee")"
    BASE_LAUNCH_ARGS=(
      "use_sim_time:=$USE_SIM_TIME"
      "activate_controller:=false"
      "run_action:=false"
      "robot_namespace:=$ROBOT_NAMESPACE"
    )
    ;;
  ugep)
    PKG="mole_highlevel_controller_cpp"
    LAUNCH_FILE="dig_ugep_controller_cpp.launch.py"
    NODE_NAME="$(qualify_node_name "$ROBOT_NAMESPACE" "dig_ugep_controller")"
    ACTION_NAME="$(qualify_node_name "$ROBOT_NAMESPACE" "run_dig_ugep")"
    BASE_LAUNCH_ARGS=(
      "use_sim_time:=$USE_SIM_TIME"
      "activate_controller:=false"
      "run_action:=false"
      "robot_namespace:=$ROBOT_NAMESPACE"
    )
    ;;
  *)
    echo "Unknown controller: $CONTROLLER" >&2
    usage
    exit 2
    ;;
esac

# An empty value is the launch-file default. Emitting a bare `tf_prefix:=`
# through tmux is parsed by the shell as a malformed launch argument.
if [[ -n "$TF_PREFIX" ]]; then
  BASE_LAUNCH_ARGS+=("tf_prefix:=$TF_PREFIX")
fi

tmux_has_session() {
  tmux has-session -t "$SESSION" 2>/dev/null
}

tmux_has_window() {
  tmux list-windows -t "$SESSION" -F '#{window_name}' 2>/dev/null | grep -Fxq "$WINDOW"
}

tmux_disable_auto_rename() {
  local target="$1"
  tmux set-option -w -t "$target" allow-rename off
  tmux set-option -w -t "$target" automatic-rename off
}

tmux_get_window_panes_sorted() {
  # Return pane_id values sorted by pane_index, one per line.
  tmux list-panes -t "$SESSION:$WINDOW" -F '#{pane_index} #{pane_id}' 2>/dev/null | sort -n | awk '{print $2}'
}

tmux_pane_cmd() {
  local target="$1"
  tmux display-message -p -t "$target" '#{pane_current_command}' 2>/dev/null || true
}

is_shell_cmd() {
  case "$1" in
    bash|zsh|fish|sh) return 0 ;;
    *) return 1 ;;
  esac
}

ensure_session() {
  if ! tmux_has_session; then
    tmux new-session -d -s "$SESSION" -n "$WINDOW" -c "$WS"
  fi
}

ensure_window() {
  if [[ "$RESTART_WINDOW" == "true" ]] && tmux_has_session && tmux_has_window; then
    tmux kill-window -t "$SESSION:$WINDOW"
  fi
  if ! tmux_has_window; then
    tmux new-window -t "$SESSION" -n "$WINDOW" -c "$WS"
  fi
  tmux_disable_auto_rename "$SESSION:$WINDOW"
}

ensure_two_panes() {
  tmux select-window -t "$SESSION:$WINDOW"
  local pane_count
  pane_count="$(tmux list-panes -t "$SESSION:$WINDOW" 2>/dev/null | wc -l | tr -d ' ')"
  if [[ "$pane_count" -lt 2 ]]; then
    tmux split-window -h -t "$SESSION:$WINDOW" -c "$WS"
  fi
  tmux select-layout -t "$SESSION:$WINDOW" even-horizontal >/dev/null 2>&1 || true
}

start_controller_if_idle() {
  local left_pane
  left_pane="$(tmux_get_window_panes_sorted | sed -n '1p')"
  if [[ -z "$left_pane" ]]; then
    echo "Failed to resolve left pane id for $SESSION:$WINDOW" >&2
    return 1
  fi
  local cmd
  cmd="$(tmux_pane_cmd "$left_pane")"
  if is_shell_cmd "$cmd"; then
    tmux send-keys -t "$left_pane" "cd \"$WS\" && source install/setup.bash && ros2 launch $PKG $LAUNCH_FILE ${BASE_LAUNCH_ARGS[*]} ${EXTRA_LAUNCH_ARGS[*]}" C-m
  fi
}

start_helpers_if_idle() {
  local right_pane
  right_pane="$(tmux_get_window_panes_sorted | sed -n '2p')"
  if [[ -z "$right_pane" ]]; then
    echo "Failed to resolve right pane id for $SESSION:$WINDOW" >&2
    return 1
  fi
  local cmd
  cmd="$(tmux_pane_cmd "$right_pane")"
  if ! is_shell_cmd "$cmd"; then
    return 0
  fi

  tmux send-keys -t "$right_pane" "cd \"$WS\" && source install/setup.bash" C-m
  tmux send-keys -t "$right_pane" "echo \"Controller: $CONTROLLER ($NODE_NAME)\"" C-m
  tmux send-keys -t "$right_pane" "echo \"Waiting for $NODE_NAME...\"" C-m
  tmux send-keys -t "$right_pane" "deadline=\$((SECONDS + 60)); until ros2 node list --no-daemon --spin-time 2 2>/dev/null | grep -Fxq $NODE_NAME; do if (( SECONDS >= deadline )); then echo 'Timed out waiting for $NODE_NAME' >&2; exit 1; fi; sleep 1; done" C-m

  if [[ "$AUTO_ACTIVATE" == "true" ]]; then
    tmux send-keys -t "$right_pane" "ros2 lifecycle set --no-daemon --spin-time 2 $NODE_NAME configure" C-m
    tmux send-keys -t "$right_pane" "ros2 lifecycle set --no-daemon --spin-time 2 $NODE_NAME activate" C-m
    tmux send-keys -t "$right_pane" "ros2 lifecycle get --no-daemon --spin-time 2 $NODE_NAME" C-m
  else
    tmux send-keys -t "$right_pane" "echo \"Manual lifecycle (optional):\"" C-m
    tmux send-keys -t "$right_pane" "echo \"  ros2 lifecycle set $NODE_NAME configure\"" C-m
    tmux send-keys -t "$right_pane" "echo \"  ros2 lifecycle set $NODE_NAME activate\"" C-m
  fi

  tmux send-keys -t "$right_pane" "echo \"Send goal (when ready): ros2 action send_goal --feedback $ACTION_NAME mole_highlevel_msgs/action/RunAction \\\"{}\\\"\"" C-m

  if [[ "$RUN_ACTION" == "true" ]]; then
    tmux send-keys -t "$right_pane" "ros2 action send_goal --feedback $ACTION_NAME mole_highlevel_msgs/action/RunAction \"{}\"" C-m
  fi
}

ensure_session
tmux_disable_auto_rename "$SESSION"
ensure_window
ensure_two_panes

if [[ "$RESTART_WINDOW" == "true" ]]; then
  start_controller_if_idle
  start_helpers_if_idle
else
  start_controller_if_idle
  start_helpers_if_idle
fi

if [[ "$ATTACH" == "true" ]]; then
  exec tmux attach -t "$SESSION"
fi

echo "tmux window ready: tmux attach -t $SESSION  (window: $WINDOW)"
