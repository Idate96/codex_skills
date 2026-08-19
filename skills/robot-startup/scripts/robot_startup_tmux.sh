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
  - nav2 (reuse already-running low-level, estimator, and perception panes)

Run this from the sourced environment that owns the robot ROS graph.

Usage:
  robot_startup_tmux.sh [--session NAME] [--ws PATH] [--robot-namespace NS] [--tf-prefix PREFIX] [--endeffector-type TYPE] [--mapping-profile PROFILE] [--design-map-name NAME] [--excavation-mapping-upstream-layer LAYER] [--no-perception] [--no-estimator] [--estimator-config PATH] [--no-elevation-mapping] [--dig-controller NAME] [--nav2-only] [--nav2-overlay-ws PATH] [--no-foxglove] [--restart] [--attach] [--keep-continuum-restore]

Options:
  --session NAME   tmux session name (default: ros)
  --ws PATH        workspace path (default: auto-detect ~/ros2_ws, then ~/moleworks/ros2_ws)
  --robot-namespace NS  robot-local ROS namespace (default: mole)
  --tf-prefix PREFIX  optional robot-local TF prefix (default: empty)
  --endeffector-type TYPE  geometry-only end-effector type for URDF (default: prompt or 'shovel')
  --mapping-profile PROFILE  perception mapping contract to use (default: local; choices: analytical, local, site)
  --design-map-name NAME  optional excavation design map artifact for perception (default: empty)
  --excavation-mapping-upstream-layer LAYER  optional upstream elevation_map_filter layer override (default: profile default)
  --no-perception  do not start the standalone perception stack (use when an application owner starts perception)
  --no-estimator   do not start mole_estimator (intentional inspection/low-level diagnostics only)
  --estimator-config PATH  optional config YAML to pass to mole_estimator (default: package default)
  --no-elevation-mapping  disable elevation mapping in the perception launch
  --dig-controller NAME  optional DIG controller launch, left inactive (dig3d|newton|dig|dig-ee)
  --nav2-only     add/start only the nav2 pane, reusing verified low_level, estimator, and perception panes
  --nav2-overlay-ws PATH  optional built development overlay sourced after the image Nav2 underlay (requires --nav2-only)
  --no-foxglove   do not start the standalone Foxglove bridge
  --restart        kill existing session and recreate
  --attach         attach to the session after setup
  --keep-continuum-restore  deprecated no-op; this helper never changes global tmux settings
EOF
}

SESSION="ros"
WS=""
ROBOT_NAMESPACE="mole"
TF_PREFIX=""
RESTART="false"
ATTACH="false"
ENDEFFECTOR_TYPE=""
MAPPING_PROFILE="local"
DESIGN_MAP_NAME=""
EXCAVATION_MAPPING_UPSTREAM_LAYER=""
LAUNCH_ESTIMATOR="true"
LAUNCH_PERCEPTION="true"
LAUNCH_FOXGLOVE="true"
ESTIMATOR_CONFIG=""
ENABLE_ELEVATION_MAPPING="true"
DIG_CONTROLLER=""
DIG_WINDOW="dig"
NAV2_ONLY="false"
NAV2_OVERLAY_WS=""
NAV2_WINDOW="nav2"
WINDOW_ORDER=()
PRE_FOXGLOVE_WINDOWS=()
FOXGLOVE_START_DELAY_SEC=5

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session)
      SESSION="${2:-}"; shift 2 ;;
    --ws)
      WS="${2:-}"; shift 2 ;;
    --robot-namespace)
      ROBOT_NAMESPACE="${2:-}"; shift 2 ;;
    --tf-prefix)
      TF_PREFIX="${2:-}"; shift 2 ;;
    --endeffector-type)
      ENDEFFECTOR_TYPE="${2:-}"; shift 2 ;;
    --mapping-profile)
      MAPPING_PROFILE="${2:-}"; shift 2 ;;
    --design-map-name)
      DESIGN_MAP_NAME="${2:-}"; shift 2 ;;
    --excavation-mapping-upstream-layer)
      EXCAVATION_MAPPING_UPSTREAM_LAYER="${2:-}"; shift 2 ;;
    --no-estimator)
      LAUNCH_ESTIMATOR="false"; shift ;;
    --no-perception)
      LAUNCH_PERCEPTION="false"; shift ;;
    --estimator-config)
      ESTIMATOR_CONFIG="${2:-}"; shift 2 ;;
    --no-elevation-mapping)
      ENABLE_ELEVATION_MAPPING="false"; shift ;;
    --dig-controller)
      DIG_CONTROLLER="${2:-}"; shift 2 ;;
    --nav2-only)
      NAV2_ONLY="true"; shift ;;
    --nav2-overlay-ws)
      NAV2_OVERLAY_WS="${2:-}"; shift 2 ;;
    --no-foxglove)
      LAUNCH_FOXGLOVE="false"; shift ;;
    --restart)
      RESTART="true"; shift ;;
    --attach)
      ATTACH="true"; shift ;;
    --keep-continuum-restore)
      shift ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "Unknown arg: $1" >&2
      usage
      exit 2 ;;
  esac
done

trim_whitespace() {
  local value="$1"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "$value"
}

normalize_optional_map_name() {
  local value
  value="$(trim_whitespace "$1")"
  if [[ -z "$value" || "${value,,}" == "none" ]]; then
    printf '%s' ""
    return 0
  fi
  printf '%s' "$value"
}

if [[ -z "$SESSION" ]]; then
  echo "--session cannot be empty" >&2
  exit 2
fi
ROBOT_NAMESPACE="$(trim_whitespace "$ROBOT_NAMESPACE")"
TF_PREFIX="$(trim_whitespace "$TF_PREFIX")"
if [[ -z "$ROBOT_NAMESPACE" ]]; then
  echo "--robot-namespace cannot be empty for the current machine stack" >&2
  exit 2
fi
if [[ -z "$ENDEFFECTOR_TYPE" ]]; then
  if [[ -t 0 ]]; then
    read -r -p "Geometry-only end-effector type (for example shovel, shovel_400mm_without_teeth, or shovel_w_teeth) [shovel]: " ENDEFFECTOR_TYPE
    ENDEFFECTOR_TYPE="${ENDEFFECTOR_TYPE:-shovel}"
  else
    ENDEFFECTOR_TYPE="shovel"
  fi
fi
ENDEFFECTOR_TYPE="$(trim_whitespace "$ENDEFFECTOR_TYPE")"
if [[ -z "$ENDEFFECTOR_TYPE" ]]; then
  echo "--endeffector-type cannot be empty" >&2
  exit 2
fi
if [[ "${ENDEFFECTOR_TYPE,,}" == *_calibrated ]]; then
  echo "--endeffector-type must be a geometry-only tool name; *_calibrated values are no longer supported" >&2
  exit 2
fi
DESIGN_MAP_NAME="$(normalize_optional_map_name "$DESIGN_MAP_NAME")"
MAPPING_PROFILE="$(trim_whitespace "$MAPPING_PROFILE")"
EXCAVATION_MAPPING_UPSTREAM_LAYER="$(trim_whitespace "$EXCAVATION_MAPPING_UPSTREAM_LAYER")"
case "$MAPPING_PROFILE" in
  analytical|local|site) ;;
  *)
    echo "--mapping-profile must be one of: analytical, local, site" >&2
    exit 2
    ;;
esac
if [[ "$MAPPING_PROFILE" != "local" && -z "$DESIGN_MAP_NAME" ]]; then
  echo "--mapping-profile $MAPPING_PROFILE requires --design-map-name <name>" >&2
  exit 2
fi
if [[ "$MAPPING_PROFILE" == "local" && -n "$DESIGN_MAP_NAME" ]]; then
  echo "Warning: --design-map-name with --mapping-profile local loads saved design geometry into the local stack." >&2
  echo "Prefer --mapping-profile site for saved design artifacts, or omit --design-map-name for true local/base-centered bringup." >&2
fi
if [[ -n "$DIG_CONTROLLER" ]]; then
  case "$DIG_CONTROLLER" in
    dig3d|newton|dig|dig-ee|dig_ee) ;;
    *)
      echo "--dig-controller must be one of: dig3d, newton, dig, dig-ee" >&2
      exit 2
      ;;
  esac
  if [[ "$LAUNCH_PERCEPTION" != "true" ]]; then
    echo "--dig-controller cannot be combined with --no-perception; the managed DIG stack requires its map owner" >&2
    exit 2
  fi
  if [[ "$LAUNCH_ESTIMATOR" != "true" ]]; then
    echo "--dig-controller cannot be combined with --no-estimator; the managed DIG stack requires robot state and TF" >&2
    exit 2
  fi
  if [[ "$ENABLE_ELEVATION_MAPPING" != "true" ]]; then
    echo "--dig-controller cannot be combined with --no-elevation-mapping; the managed DIG stack requires current terrain" >&2
    exit 2
  fi
fi

if [[ "$NAV2_ONLY" == "true" ]]; then
  if [[ "$RESTART" == "true" ]]; then
    echo "--nav2-only cannot be combined with --restart because it reuses the existing robot session" >&2
    exit 2
  fi
  if [[ -n "$DIG_CONTROLLER" ]]; then
    echo "--nav2-only cannot be combined with --dig-controller" >&2
    exit 2
  fi
  LAUNCH_ESTIMATOR="false"
  LAUNCH_PERCEPTION="false"
  LAUNCH_FOXGLOVE="false"
  WINDOW_ORDER=("$NAV2_WINDOW")
  PRE_FOXGLOVE_WINDOWS=("$NAV2_WINDOW")
else
  if [[ -n "$NAV2_OVERLAY_WS" ]]; then
    echo "--nav2-overlay-ws requires --nav2-only" >&2
    exit 2
  fi
  WINDOW_ORDER=(low_level)
  PRE_FOXGLOVE_WINDOWS=(low_level)
  if [[ "$LAUNCH_PERCEPTION" == "true" ]]; then
    WINDOW_ORDER+=(perception)
    PRE_FOXGLOVE_WINDOWS+=(perception)
  fi
  if [[ "$LAUNCH_ESTIMATOR" == "true" ]]; then
    WINDOW_ORDER+=(estimator)
    PRE_FOXGLOVE_WINDOWS+=(estimator)
  fi
  if [[ -n "$DIG_CONTROLLER" ]]; then
    WINDOW_ORDER+=("$DIG_WINDOW")
    PRE_FOXGLOVE_WINDOWS+=("$DIG_WINDOW")
  fi
  if [[ "$LAUNCH_FOXGLOVE" == "true" ]]; then
    WINDOW_ORDER+=(foxglove)
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
  echo "Could not auto-detect a built robot workspace; pass --ws PATH" >&2
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
if [[ "$NAV2_ONLY" == "true" ]]; then
  if [[ ! -f /opt/nav2_underlay/setup.bash ]]; then
    echo "--nav2-only requires the image-pinned /opt/nav2_underlay" >&2
    exit 2
  fi
  if [[ -n "$NAV2_OVERLAY_WS" ]]; then
    NAV2_OVERLAY_WS="$(realpath -m "$NAV2_OVERLAY_WS")"
    if [[ ! -f "$NAV2_OVERLAY_WS/install/local_setup.bash" ]]; then
      echo "Missing $NAV2_OVERLAY_WS/install/local_setup.bash (build the focused Nav2 overlay first)" >&2
      exit 2
    fi
  fi
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
  local cmd
  cmd="cd \"$WS\" && source install/setup.bash && ros2 launch mole_low_level_bringup bringup.launch.py use_sim_time:=false on_machine:=true endeffector_type:=$(printf '%q' "$ENDEFFECTOR_TYPE") robot_namespace:=$(printf '%q' "$ROBOT_NAMESPACE")"
  if [[ -n "$TF_PREFIX" ]]; then
    cmd+=" tf_prefix:=$(printf '%q' "$TF_PREFIX")"
  fi
  tmux_send_to_active_pane "low_level" "$cmd"
}

start_perception() {
  local cmd
  cmd="cd \"$WS\" && source install/setup.bash && ros2 launch mole_perception_bringup bringup.launch.py use_sim_time:=false on_machine:=true enable_lidar:=true enable_robot_self_filter:=true enable_elevation_mapping:=$ENABLE_ELEVATION_MAPPING mapping_profile:=$(printf '%q' "$MAPPING_PROFILE") endeffector_type:=$(printf '%q' "$ENDEFFECTOR_TYPE") robot_namespace:=$(printf '%q' "$ROBOT_NAMESPACE")"
  if [[ -n "$TF_PREFIX" ]]; then
    cmd+=" tf_prefix:=$(printf '%q' "$TF_PREFIX")"
  fi
  if [[ -n "$DESIGN_MAP_NAME" ]]; then
    cmd+=" design_map_name:=$(printf '%q' "$DESIGN_MAP_NAME")"
  fi
  if [[ -n "$EXCAVATION_MAPPING_UPSTREAM_LAYER" ]]; then
    cmd+=" excavation_mapping_upstream_layer:=$(printf '%q' "$EXCAVATION_MAPPING_UPSTREAM_LAYER")"
  fi
  tmux_send_to_active_pane "perception" "$cmd"
}

start_dig() {
  local skills_root
  local helper_args=(
    --controller "$DIG_CONTROLLER"
    --session "$SESSION"
    --window "$DIG_WINDOW"
    --ws "$WS"
    --robot-namespace "$ROBOT_NAMESPACE"
    --no-activate
  )
  local extra_launch_args=()
  if [[ -n "$TF_PREFIX" ]]; then
    helper_args+=(--tf-prefix "$TF_PREFIX")
  fi
  case "$DIG_CONTROLLER" in
    dig3d|newton)
      extra_launch_args+=("urdf_endeffector_type:=$ENDEFFECTOR_TYPE")
      ;;
  esac
  skills_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
  "$skills_root/dig-controllers/scripts/dig_controllers_tmux.sh" \
    "${helper_args[@]}" \
    -- "${extra_launch_args[@]}"
}

start_nav2() {
  local cmd
  cmd="cd \"$WS\" && source /opt/ros/jazzy/setup.bash && source /opt/nav2_underlay/setup.bash && source install/local_setup.bash"
  if [[ -n "$NAV2_OVERLAY_WS" ]]; then
    cmd+=" && source $(printf '%q' "$NAV2_OVERLAY_WS/install/local_setup.bash")"
  fi
  cmd+=" && ros2 launch mole_bringup nav2_on_robot.launch.py use_sim_time:=false on_machine:=true launch_rviz:=false activate_controller:=false endeffector_type:=$(printf '%q' "$ENDEFFECTOR_TYPE") robot_namespace:=$(printf '%q' "$ROBOT_NAMESPACE")"
  if [[ -n "$TF_PREFIX" ]]; then
    cmd+=" tf_prefix:=$(printf '%q' "$TF_PREFIX")"
  fi
  tmux_send_to_active_pane "$NAV2_WINDOW" "$cmd"
}

start_foxglove() {
  local cmd
  cmd="cd \"$WS\" && source install/setup.bash"
  if [[ -n "${MOLE_DDS_OBSERVER_PROFILE:-}" ]]; then
    cmd+=" && export FASTRTPS_DEFAULT_PROFILES_FILE=$(printf '%q' "$MOLE_DDS_OBSERVER_PROFILE") ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT"
    cmd+=" && unset FASTDDS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER RMW_IMPLEMENTATION CYCLONEDDS_URI"
  fi
  cmd+=" && ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
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
  cmd="cd \"$WS\" && source install/setup.bash && export LD_LIBRARY_PATH=\"/usr/local/lib:${LD_LIBRARY_PATH:-}\" && ros2 launch mole_estimator mole_estimator.launch.py use_sim_time:=false urdf_xacro_endeffector_type:=$(printf '%q' "$ENDEFFECTOR_TYPE") robot_namespace:=$(printf '%q' "$ROBOT_NAMESPACE")${cfg_arg}"
  if [[ -n "$TF_PREFIX" ]]; then
    cmd+=" tf_prefix:=$(printf '%q' "$TF_PREFIX")"
  fi
  if [[ -n "$DESIGN_MAP_NAME" ]]; then
    cmd+=" design_map:=$(printf '%q' "$DESIGN_MAP_NAME")"
  fi
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
    nav2) start_nav2 ;;
  esac
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
  local timeout_sec=30
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
  if echo "$ps_out" | grep -Fq "mole_bringup robot.launch.py" &&
    echo "$ps_out" | grep -Fq "launch_low_level:=false" &&
    echo "$ps_out" | grep -Fq "launch_perception:=true"; then
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
  if echo "$ps_out" | grep -Fq "mole_bringup nav2_on_robot.launch.py" ||
    echo "$ps_out" | grep -Fq "mole_nav2_bringup bringup.launch.py"; then
    echo "nav2"
    return 0
  fi

  return 0
}

preflight_existing_managed_windows() {
  local cmd detected_role win
  if ! tmux_has_session; then
    return 0
  fi

  for win in "${WINDOW_ORDER[@]}"; do
    if ! tmux_has_window "$win"; then
      continue
    fi
    cmd="$(window_primary_command "$win")"
    if is_shell_command "$cmd"; then
      continue
    fi
    detected_role="$(detect_managed_role_in_window "$win")"
    if [[ "$detected_role" != "$win" ]]; then
      echo "Refusing to modify session '$SESSION': busy pane $SESSION:$win has role ${detected_role:-unknown} and command ${cmd:-unknown}. Use --restart only when replacing the session is explicitly authorized." >&2
      return 1
    fi
  done
}

preflight_nav2_prerequisite_windows() {
  local detected_role prerequisite
  if [[ "$NAV2_ONLY" != "true" ]]; then
    return 0
  fi
  if ! tmux_has_session; then
    echo "--nav2-only requires an existing '$SESSION' session with robot prerequisites" >&2
    return 1
  fi
  for prerequisite in low_level estimator perception; do
    if ! tmux_has_window "$prerequisite"; then
      echo "--nav2-only requires the existing $SESSION:$prerequisite pane" >&2
      return 1
    fi
    detected_role="$(detect_managed_role_in_window "$prerequisite")"
    if [[ "$detected_role" != "$prerequisite" ]]; then
      echo "--nav2-only prerequisite $SESSION:$prerequisite has role ${detected_role:-unknown}" >&2
      return 1
    fi
  done
}

preflight_nav2_prerequisite_windows
if [[ "$RESTART" == "true" ]]; then
  tmux_kill_session_if_exists
else
  preflight_existing_managed_windows
fi

tmux_new_session_if_missing
tmux_disable_auto_rename "$SESSION"

# Create windows (by name; no fixed indexes).
for win in "${WINDOW_ORDER[@]}"; do
  tmux_ensure_window "$win"
done
if [[ "$NAV2_ONLY" != "true" ]]; then
  tmux_reorder_windows
fi

if [[ "$RESTART" == "true" ]]; then
  for win in "${PRE_FOXGLOVE_WINDOWS[@]}"; do
    start_window "$win"
  done
  if [[ "$LAUNCH_FOXGLOVE" == "true" ]] && wait_for_pre_foxglove_windows; then
    start_window "foxglove"
  fi
else
  # Preflight every managed window before starting anything. Without --restart,
  # never interrupt or replace a busy pane.
  for win in "${PRE_FOXGLOVE_WINDOWS[@]}"; do
    cmd="$(window_primary_command "$win")"
    if is_shell_command "$cmd"; then
      continue
    fi

    detected_role="$(detect_managed_role_in_window "$win")"
    if [[ "$detected_role" != "$win" ]]; then
      echo "Refusing to replace busy pane $SESSION:$win without --restart (detected role: ${detected_role:-unknown}, command: ${cmd:-unknown})." >&2
      exit 1
    fi
  done

  if [[ "$LAUNCH_FOXGLOVE" == "true" ]]; then
    cmd="$(window_primary_command "foxglove")"
    detected_role="$(detect_managed_role_in_window "foxglove")"
    if ! is_shell_command "$cmd" && [[ "$detected_role" != "foxglove" ]]; then
      echo "Refusing to replace busy pane $SESSION:foxglove without --restart (detected role: ${detected_role:-unknown}, command: ${cmd:-unknown})." >&2
      exit 1
    fi
  fi

  for win in "${PRE_FOXGLOVE_WINDOWS[@]}"; do
    cmd="$(window_primary_command "$win")"
    if is_shell_command "$cmd"; then
      start_window "$win"
    fi
  done

  if [[ "$LAUNCH_FOXGLOVE" == "true" ]]; then
    cmd="$(window_primary_command "foxglove")"
    if is_shell_command "$cmd" && wait_for_pre_foxglove_windows; then
      start_window "foxglove"
    fi
  fi
fi

if [[ "$NAV2_ONLY" != "true" ]]; then
  tmux_reorder_windows
fi

if [[ "$ATTACH" == "true" ]]; then
  exec tmux attach -t "$SESSION"
fi

echo "tmux session ready: tmux attach -t $SESSION"
