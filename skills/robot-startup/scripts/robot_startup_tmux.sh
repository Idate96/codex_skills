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

Optionally add:
  - dig

Defaults assume you are already inside the Moleworks container with DDS configured.

Usage:
  robot_startup_tmux.sh [--session NAME] [--ws PATH] [--endeffector-type TYPE] [--no-estimator] [--estimator-config PATH] [--no-elevation-mapping] [--dig-controller NAME] [--restart] [--attach] [--keep-continuum-restore]

Options:
  --session NAME   tmux session name (default: ros)
  --ws PATH        workspace path (default: ~/ros2_ws)
  --endeffector-type TYPE  end-effector type for URDF (default: prompt or 'shovel')
  --no-estimator   do not start mole_estimator (restores the legacy 3-window layout)
  --estimator-config PATH  optional config YAML to pass to mole_estimator (default: package default)
  --no-elevation-mapping  disable elevation mapping in the perception launch
  --dig-controller NAME  optional dig controller to launch after base stack is up (dig3d|newton|dig|dig-ee)
  --restart        kill existing session and recreate
  --attach         attach to the session after setup
  --keep-continuum-restore  do not force-disable tmux @continuum-restore
EOF
}

SESSION="ros"
WS="${HOME}/ros2_ws"
RESTART="false"
ATTACH="false"
ENDEFFECTOR_TYPE=""
LAUNCH_ESTIMATOR="true"
ESTIMATOR_CONFIG=""
KEEP_CONTINUUM_RESTORE="false"
ENABLE_ELEVATION_MAPPING="true"
DIG_CONTROLLER=""
DIG_WINDOW="dig"
WINDOW_ORDER=()
PRE_FOXGLOVE_WINDOWS=()
FOXGLOVE_START_DELAY_SEC=5

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
    --no-elevation-mapping)
      ENABLE_ELEVATION_MAPPING="false"; shift ;;
    --dig-controller)
      DIG_CONTROLLER="${2:-}"; shift 2 ;;
    --restart)
      RESTART="true"; shift ;;
    --attach)
      ATTACH="true"; shift ;;
    --keep-continuum-restore)
      KEEP_CONTINUUM_RESTORE="true"; shift ;;
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
if [[ -n "$DIG_CONTROLLER" ]]; then
  case "$DIG_CONTROLLER" in
    dig3d|newton|dig|dig-ee|dig_ee) ;;
    *)
      echo "--dig-controller must be one of: dig3d, newton, dig, dig-ee" >&2
      exit 2
      ;;
  esac
fi

WINDOW_ORDER=(low_level perception foxglove)
if [[ "$LAUNCH_ESTIMATOR" == "true" ]]; then
  # Keep foxglove last.
  WINDOW_ORDER=(low_level perception estimator foxglove)
fi
PRE_FOXGLOVE_WINDOWS=(low_level perception)
if [[ "$LAUNCH_ESTIMATOR" == "true" ]]; then
  PRE_FOXGLOVE_WINDOWS+=(estimator)
fi
if [[ -n "$DIG_CONTROLLER" ]]; then
  WINDOW_ORDER=("${PRE_FOXGLOVE_WINDOWS[@]}" "$DIG_WINDOW" foxglove)
  PRE_FOXGLOVE_WINDOWS+=("$DIG_WINDOW")
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

tmux_disable_continuum_restore() {
  if [[ "$KEEP_CONTINUUM_RESTORE" == "true" ]]; then
    return 0
  fi

  local current
  current="$(tmux show-options -gqv @continuum-restore || true)"
  if [[ "$current" == "on" ]]; then
    tmux set-option -g @continuum-restore off
    echo "Disabled tmux @continuum-restore to prevent stale session auto-restore into '$SESSION'." >&2
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
  local cmd
  cmd="cd \"$WS\" && source install/setup.bash && ros2 launch mole_low_level_bringup bringup.launch.py use_sim_time:=false on_machine:=true activate_trajectory_controller:=false endeffector_type:=$ENDEFFECTOR_TYPE"
  tmux_send_to_active_pane "low_level" "$cmd"
}

start_perception() {
  local cmd
  cmd="cd \"$WS\" && source install/setup.bash && ros2 launch mole_perception_bringup bringup.launch.py use_sim_time:=false enable_lidar:=true enable_robot_self_filter:=true enable_elevation_mapping:=$ENABLE_ELEVATION_MAPPING mapping_profile:=local endeffector_type:=$ENDEFFECTOR_TYPE"
  tmux_send_to_active_pane "perception" "$cmd"
}

start_dig() {
  "$HOME/.codex/skills/dig-controllers/scripts/dig_controllers_tmux.sh" \
    --controller "$DIG_CONTROLLER" \
    --session "$SESSION" \
    --window "$DIG_WINDOW" \
    --ws "$WS" \
    --restart-window \
    -- activate_controller:=true
}

start_foxglove() {
  local cmd
  cmd="cd \"$WS\" && source install/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
  tmux_send_to_active_pane "foxglove" "$cmd"
}

start_estimator() {
  local cfg_arg=""
  local cmd
  if [[ -n "$ESTIMATOR_CONFIG" ]]; then
    # quote for the shell; ros2 launch expects config:=<path>
    cfg_arg=" config:=$(printf '%q' "$ESTIMATOR_CONFIG")"
  fi

  # Prefer /usr/local/lib first so estimator picks the locally installed GTSAM runtime when present.
  cmd="cd \"$WS\" && source install/setup.bash && export LD_LIBRARY_PATH=\"/usr/local/lib:${LD_LIBRARY_PATH:-}\" && ros2 launch mole_estimator mole_estimator.launch.py use_sim_time:=false urdf_xacro_endeffector_type:=$ENDEFFECTOR_TYPE${cfg_arg}"
  tmux_send_to_active_pane "estimator" "$cmd"
}

start_window() {
  local win="$1"
  case "$win" in
    low_level) start_low_level ;;
    perception) start_perception ;;
    foxglove) start_foxglove ;;
    estimator) start_estimator ;;
    dig) start_dig ;;
  esac
}

restart_window_in_place() {
  local win="$1"
  tmux send-keys -t "$SESSION:$win" C-c
  sleep 0.3
  start_window "$win"
}

stop_window_to_shell() {
  local win="$1"
  local timeout_sec="${2:-10}"
  local deadline cmd

  tmux send-keys -t "$SESSION:$win" C-c
  deadline=$((SECONDS + timeout_sec))

  while (( SECONDS < deadline )); do
    cmd="$(window_primary_command "$win")"
    if [[ -z "$cmd" ]] || is_shell_command "$cmd"; then
      return 0
    fi
    sleep 1
  done

  tmux respawn-window -k -t "$SESSION:$win" -c "$WS"
  tmux_disable_auto_rename "$SESSION:$win"
  return 0
}

window_primary_command() {
  local win="$1"
  tmux list-panes -t "$SESSION:$win" -F '#{pane_current_command}' 2>/dev/null | head -n 1 || true
}

window_primary_tty() {
  local win="$1"
  tmux list-panes -t "$SESSION:$win" -F '#{pane_tty}' 2>/dev/null | head -n 1 || true
}

is_shell_command() {
  local cmd="$1"
  [[ "$cmd" == "bash" || "$cmd" == "zsh" || "$cmd" == "fish" || "$cmd" == "sh" ]]
}

wait_for_pre_foxglove_windows() {
  local timeout_sec="${1:-30}"
  local deadline detected_role win
  deadline=$((SECONDS + timeout_sec))

  while (( SECONDS < deadline )); do
    local all_ready="true"

    for win in "${PRE_FOXGLOVE_WINDOWS[@]}"; do
      detected_role="$(detect_managed_role_in_window "$win")"
      if [[ "$detected_role" != "$win" ]]; then
        all_ready="false"
        break
      fi
    done

    if [[ "$all_ready" == "true" ]]; then
      sleep "$FOXGLOVE_START_DELAY_SEC"
      return 0
    fi

    sleep 1
  done

  echo "Warning: timed out waiting for managed launches in ${PRE_FOXGLOVE_WINDOWS[*]}; leaving foxglove stopped." >&2
  return 1
}

detect_managed_role_in_window() {
  local win="$1"
  local tty tty_short ps_out

  tty="$(window_primary_tty "$win")"
  if [[ -z "$tty" ]]; then
    return 0
  fi
  tty_short="${tty#/dev/}"
  ps_out="$(ps -t "$tty_short" -o args= 2>/dev/null || true)"

  if echo "$ps_out" | grep -Fq "mole_low_level_bringup bringup.launch.py"; then
    echo "low_level"
    return 0
  fi
  if echo "$ps_out" | grep -Fq "mole_perception_bringup bringup.launch.py"; then
    echo "perception"
    return 0
  fi
  if echo "$ps_out" | grep -Fq "mole_estimator mole_estimator.launch.py"; then
    echo "estimator"
    return 0
  fi
  if echo "$ps_out" | grep -Eq "mole_highlevel_controller_cpp (dig_3d_controller_cpp|dig_newton_controller|dig_controller_cpp|dig_ee_controller_cpp)\.launch\.py"; then
    echo "dig"
    return 0
  fi
  if echo "$ps_out" | grep -Fq "foxglove_bridge foxglove_bridge_launch.xml"; then
    echo "foxglove"
    return 0
  fi

  return 0
}

if [[ "$RESTART" == "true" ]]; then
  tmux_kill_session_if_exists
fi

tmux_new_session_if_missing
tmux_disable_continuum_restore
tmux_disable_auto_rename "$SESSION"

# Create windows (by name; no fixed indexes).
for win in "${WINDOW_ORDER[@]}"; do
  tmux_ensure_window "$win"
done
tmux_reorder_windows

if [[ "$RESTART" == "true" ]]; then
  for win in "${PRE_FOXGLOVE_WINDOWS[@]}"; do
    start_window "$win"
  done
  if wait_for_pre_foxglove_windows; then
    start_window "foxglove"
  fi
else
  pre_foxglove_changed="false"

  # Start commands only in windows that look idle (single pane running a shell).
  for win in "${PRE_FOXGLOVE_WINDOWS[@]}"; do
    cmd="$(window_primary_command "$win")"
    if is_shell_command "$cmd"; then
      start_window "$win"
      pre_foxglove_changed="true"
      continue
    fi

    # If a managed launch is running in the wrong managed window name
    # (e.g. foxglove under "estimator"), force-correct only that window.
    detected_role="$(detect_managed_role_in_window "$win")"
    if [[ -n "$detected_role" && "$detected_role" != "$win" ]]; then
      echo "Correcting stale managed launch in $SESSION:$win (detected $detected_role)." >&2
      restart_window_in_place "$win"
      pre_foxglove_changed="true"
    fi
  done

  cmd="$(window_primary_command "foxglove")"
  detected_role="$(detect_managed_role_in_window "foxglove")"
  foxglove_running_correctly="false"
  if [[ "$detected_role" == "foxglove" ]] && ! is_shell_command "$cmd"; then
    foxglove_running_correctly="true"
  fi

  if [[ "$pre_foxglove_changed" == "true" && "$foxglove_running_correctly" == "true" ]]; then
    stop_window_to_shell "foxglove"
    cmd="$(window_primary_command "foxglove")"
    detected_role="$(detect_managed_role_in_window "foxglove")"
    foxglove_running_correctly="false"
  fi

  if [[ "$foxglove_running_correctly" == "true" ]]; then
    if ! wait_for_pre_foxglove_windows; then
      stop_window_to_shell "foxglove"
      cmd="$(window_primary_command "foxglove")"
      detected_role="$(detect_managed_role_in_window "foxglove")"
      foxglove_running_correctly="false"
    fi
  fi

  if is_shell_command "$cmd"; then
    if wait_for_pre_foxglove_windows; then
      start_window "foxglove"
    fi
  else
    if [[ -n "$detected_role" && "$detected_role" != "foxglove" ]]; then
      echo "Correcting stale managed launch in $SESSION:foxglove (detected $detected_role)." >&2
      if wait_for_pre_foxglove_windows; then
        restart_window_in_place "foxglove"
      else
        stop_window_to_shell "foxglove"
      fi
    fi
  fi
fi

tmux_reorder_windows

if [[ "$ATTACH" == "true" ]]; then
  exec tmux attach -t "$SESSION"
fi

echo "tmux session ready: tmux attach -t $SESSION"
