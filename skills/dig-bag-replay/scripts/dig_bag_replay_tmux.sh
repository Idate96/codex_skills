#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
dig_bag_replay_tmux.sh

Replay selected splits from one canonical DIG run in a dedicated
moleworks_ros container, with a robot-description publisher and Foxglove.
This helper visualizes recorded topics; it does not regenerate mapping.

Usage:
  dig_bag_replay_tmux.sh --bag-root PATH [options]

Options:
  --bag-root PATH           Run root (with raw/) or raw split root
  --container-bag-root P   Corresponding path inside container (default: host path)
  --splits CSV             Splits to replay (default: sensors,state,lidar,elevation_map,dig3d_special_obs,camera)
  --include-commands       Also replay raw/commands (offline container only)
  --session NAME           tmux session name (default: bag_replay)
  --container NAME         Docker container name (default: moleworks_ros)
  --container-ws PATH      Workspace inside container (default: auto-detect)
  --playback-rate RATE     ros2 bag replay rate (default: 0.2)
  --foxglove-port PORT     Foxglove bridge port (default: 8766)
  --endeffector-type TYPE  URDF end-effector type (default: shovel)
  --confirm-offline-container
                            Confirm the container is dedicated to replay
  --attach                  Attach to tmux after startup
  -h, --help                Show this help
USAGE
}

SESSION="bag_replay"
CONTAINER="moleworks_ros"
CONTAINER_WS=""
PLAYBACK_RATE="0.2"
FOXGLOVE_PORT="8766"
ENDEFFECTOR_TYPE="shovel"
SPLITS_CSV="sensors,state,lidar,elevation_map,dig3d_special_obs,camera"
INCLUDE_COMMANDS="false"
ATTACH="false"
CONFIRM_OFFLINE_CONTAINER="false"
BAG_ROOT=""
CONTAINER_BAG_ROOT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag-root) BAG_ROOT="${2:-}"; shift 2 ;;
    --container-bag-root) CONTAINER_BAG_ROOT="${2:-}"; shift 2 ;;
    --splits) SPLITS_CSV="${2:-}"; shift 2 ;;
    --include-commands) INCLUDE_COMMANDS="true"; shift ;;
    --session) SESSION="${2:-}"; shift 2 ;;
    --container) CONTAINER="${2:-}"; shift 2 ;;
    --container-ws) CONTAINER_WS="${2:-}"; shift 2 ;;
    --playback-rate) PLAYBACK_RATE="${2:-}"; shift 2 ;;
    --foxglove-port) FOXGLOVE_PORT="${2:-}"; shift 2 ;;
    --endeffector-type) ENDEFFECTOR_TYPE="${2:-}"; shift 2 ;;
    --confirm-offline-container) CONFIRM_OFFLINE_CONTAINER="true"; shift ;;
    --attach) ATTACH="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "$BAG_ROOT" ]]; then
  echo "--bag-root is required" >&2
  usage
  exit 2
fi
if [[ "$CONFIRM_OFFLINE_CONTAINER" != "true" ]]; then
  echo "Refusing replay without --confirm-offline-container." >&2
  echo "Use a container dedicated to offline replay; never confirm a live robot container." >&2
  exit 2
fi
if [[ -z "$SESSION" || -z "$CONTAINER" ]]; then
  echo "Session and container must be non-empty" >&2
  exit 2
fi
if [[ ! "$PLAYBACK_RATE" =~ ^[0-9]+([.][0-9]+)?$ ]] || [[ "$PLAYBACK_RATE" == "0" || "$PLAYBACK_RATE" == "0.0" ]]; then
  echo "--playback-rate must be a positive number" >&2
  exit 2
fi
if [[ ! "$FOXGLOVE_PORT" =~ ^[0-9]+$ ]]; then
  echo "--foxglove-port must be numeric" >&2
  exit 2
fi

BAG_ROOT="$(realpath -m "$BAG_ROOT")"
if [[ ! -d "$BAG_ROOT" ]]; then
  echo "Bag root not found: $BAG_ROOT" >&2
  exit 2
fi
if [[ -f "$BAG_ROOT/upload_name_map.yaml" ]]; then
  echo "Bag root looks like a flattened Kleinkram mission payload: $BAG_ROOT" >&2
  echo "Reconstruct split runs with the kleinkram-upload skill first." >&2
  exit 2
fi

if [[ -d "$BAG_ROOT/raw" ]]; then
  SPLIT_ROOT="$BAG_ROOT/raw"
else
  SPLIT_ROOT="$BAG_ROOT"
fi
if [[ -z "$CONTAINER_BAG_ROOT" ]]; then
  CONTAINER_SPLIT_ROOT="$SPLIT_ROOT"
else
  CONTAINER_BAG_ROOT="${CONTAINER_BAG_ROOT%/}"
  if [[ -d "$BAG_ROOT/raw" ]]; then
    CONTAINER_SPLIT_ROOT="$CONTAINER_BAG_ROOT/raw"
  else
    CONTAINER_SPLIT_ROOT="$CONTAINER_BAG_ROOT"
  fi
fi

IFS=',' read -r -a REQUESTED_SPLITS <<< "$SPLITS_CSV"
if [[ "$INCLUDE_COMMANDS" == "true" ]]; then
  REQUESTED_SPLITS+=("commands")
fi

SELECTED_SPLITS=()
for raw_split in "${REQUESTED_SPLITS[@]}"; do
  split="${raw_split//[[:space:]]/}"
  [[ -n "$split" ]] || continue
  case "$split" in
    sensors|state|commands|lidar|camera|elevation_map|dig3d_special_obs|ocs2) ;;
    *) echo "Unsupported split: $split" >&2; exit 2 ;;
  esac
  if [[ ! -d "$SPLIT_ROOT/$split" ]]; then
    echo "Skipping absent optional split: $split"
    continue
  fi
  if [[ ! -f "$SPLIT_ROOT/$split/metadata.yaml" ]]; then
    echo "Missing metadata.yaml in split: $SPLIT_ROOT/$split" >&2
    exit 2
  fi
  if ! find "$SPLIT_ROOT/$split" -maxdepth 1 -type f -name '*.mcap' -print -quit | grep -q .; then
    echo "No .mcap file in split: $SPLIT_ROOT/$split" >&2
    exit 2
  fi
  SELECTED_SPLITS+=("$split")
done
if [[ ${#SELECTED_SPLITS[@]} -eq 0 ]]; then
  echo "None of the requested splits exist under $SPLIT_ROOT" >&2
  exit 2
fi

if [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null || true)" != "true" ]]; then
  echo "Container is not running: $CONTAINER" >&2
  exit 2
fi
if [[ -z "$CONTAINER_WS" ]]; then
  CONTAINER_WS="$(docker exec "$CONTAINER" bash -lc '
    for candidate in "${MOLE_ROS_WS:-}" /workspaces/moleworks_ws /workspace/moleworks/ros2_ws "$HOME/ros2_ws" "$HOME/moleworks/ros2_ws"; do
      if [[ -n "$candidate" && -f "$candidate/install/setup.bash" ]]; then
        printf "%s\n" "$candidate"
        exit 0
      fi
    done
    find /home -maxdepth 4 -type f -path "*/ros2_ws/install/setup.bash" -printf "%h\n" 2>/dev/null | sed "s#/install$##" | head -n 1
    exit 1
  ' 2>/dev/null || true)"
fi
if [[ -z "$CONTAINER_WS" ]]; then
  echo "Could not auto-detect a built workspace in container; pass --container-ws" >&2
  exit 2
fi
if ! docker exec "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash && source '$CONTAINER_WS/install/setup.bash' && command -v xacro >/dev/null && ros2 pkg prefix foxglove_bridge >/dev/null && ros2 pkg prefix mole_ocs2_arm_controller >/dev/null && ros2 pkg prefix mole_description >/dev/null"; then
  echo "Replay dependencies are unavailable in container workspace: $CONTAINER_WS" >&2
  exit 2
fi
for split in "${SELECTED_SPLITS[@]}"; do
  printf -v container_split_q '%q' "$CONTAINER_SPLIT_ROOT/$split"
  if ! docker exec "$CONTAINER" bash -lc "test -f $container_split_q/metadata.yaml"; then
    echo "Split is not visible inside container: $CONTAINER_SPLIT_ROOT/$split" >&2
    echo "Use --container-bag-root when the host and container paths differ." >&2
    exit 2
  fi
done

# The tmux sessions own replay processes. Avoid broad pkill cleanup inside the
# container: a similarly named process may belong to another offline workflow.
tmux kill-session -t "$SESSION" 2>/dev/null || true
if [[ "$SESSION" != "segment_replay" ]]; then
  tmux kill-session -t segment_replay 2>/dev/null || true
fi

tmux new-session -d -s "$SESSION" -n replay
tmux new-window -t "$SESSION" -n monitor
for win in replay monitor; do
  tmux set-option -w -t "$SESSION:$win" allow-rename off
  tmux set-option -w -t "$SESSION:$win" automatic-rename off
done

PLAY_INPUTS=""
for split in "${SELECTED_SPLITS[@]}"; do
  printf -v container_split_q '%q' "$CONTAINER_SPLIT_ROOT/$split"
  PLAY_INPUTS+=" --input $container_split_q"
done

printf -v container_q '%q' "$CONTAINER"
printf -v ws_q '%q' "$CONTAINER_WS"
printf -v port_q '%q' "$FOXGLOVE_PORT"
printf -v rate_q '%q' "$PLAYBACK_RATE"
printf -v ee_q '%q' "$ENDEFFECTOR_TYPE"
REPLAY_CMD="docker exec -it $container_q bash -lc 'cd $ws_q && source /opt/ros/jazzy/setup.bash && source install/setup.bash && { urdf_file=/tmp/dig_bag_replay_mole.urdf; xacro install/mole_description/share/mole_description/xacro/mole.urdf.xacro enable_continuous_joints:=true limit_j_ee:=false endeffector_type:=$ee_q > \$urdf_file && ros2 run mole_ocs2_arm_controller mole_robot_description_publisher.py --ros-args -r __node:=mole_robot_description_publisher -r __ns:=/mole -p urdf_file:=\$urdf_file -p topic:=robot_description >/tmp/dig_bag_robot_description.log 2>&1 & desc_pid=\$!; ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=$port_q use_sim_time:=true >/tmp/dig_bag_foxglove_$port_q.log 2>&1 & bridge_pid=\$!; trap \"kill \$desc_pid \$bridge_pid 2>/dev/null || true\" EXIT; sleep 2; ros2 bag play$PLAY_INPUTS --clock 100 --rate $rate_q --disable-keyboard-controls; }'"
MONITOR_CMD="docker exec -it $container_q bash -lc 'cd $ws_q && source /opt/ros/jazzy/setup.bash && source install/setup.bash && echo Ready in container $CONTAINER. Foxglove: ws://localhost:$FOXGLOVE_PORT && exec bash --noprofile --norc'"

tmux send-keys -t "$SESSION:replay" "$REPLAY_CMD" C-m
tmux send-keys -t "$SESSION:monitor" "$MONITOR_CMD" C-m

echo "Replay session ready: tmux attach -t $SESSION"
echo "Foxglove: ws://localhost:$FOXGLOVE_PORT"
echo "Split root: $SPLIT_ROOT"
echo "Selected splits: ${SELECTED_SPLITS[*]}"
echo "Recorded topics are replayed as-is; no self-filter or mapping node is regenerated."

if [[ "$ATTACH" == "true" ]]; then
  exec tmux attach -t "$SESSION"
fi
