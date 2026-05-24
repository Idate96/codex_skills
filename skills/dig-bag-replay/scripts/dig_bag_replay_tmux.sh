#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
dig_bag_replay_tmux.sh

Replay a split DIG bag in the moleworks_ros container with live self-filter,
live elevation mapping, live excavation mapping, and Foxglove.

Usage:
  dig_bag_replay_tmux.sh --bag-root PATH [options]

Options:
  --bag-root PATH         Required split bag root with sensors/state/commands/lidar
  --session NAME         tmux session name (default: bag_replay)
  --container NAME       Docker container name (default: moleworks_ros)
  --container-ws PATH    Workspace path inside container (default: /workspace/moleworks/ros2_ws)
  --playback-rate RATE   ros2 bag replay rate (default: 0.2)
  --foxglove-port PORT   Foxglove bridge port (default: 8766)
  --endeffector-type T   shovel or shovel_w_teeth (default: shovel)
  --design-bag-path P    Explicit excavation preload bag path
  --attach               Attach to tmux after startup
  -h, --help             Show this help
USAGE
}

SESSION="bag_replay"
CONTAINER="moleworks_ros"
CONTAINER_WS="/workspace/moleworks/ros2_ws"
PLAYBACK_RATE="0.2"
FOXGLOVE_PORT="8766"
ENDEFFECTOR_TYPE="shovel"
DESIGN_BAG_PATH="package://mole_maps/maps/hong0326_no_holes/hong0326_no_holes_surface"
ATTACH="false"
BAG_ROOT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag-root)
      BAG_ROOT="${2:-}"; shift 2 ;;
    --session)
      SESSION="${2:-}"; shift 2 ;;
    --container)
      CONTAINER="${2:-}"; shift 2 ;;
    --container-ws)
      CONTAINER_WS="${2:-}"; shift 2 ;;
    --playback-rate)
      PLAYBACK_RATE="${2:-}"; shift 2 ;;
    --foxglove-port)
      FOXGLOVE_PORT="${2:-}"; shift 2 ;;
    --endeffector-type)
      ENDEFFECTOR_TYPE="${2:-}"; shift 2 ;;
    --design-bag-path)
      DESIGN_BAG_PATH="${2:-}"; shift 2 ;;
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

if [[ -z "$BAG_ROOT" ]]; then
  echo "--bag-root is required" >&2
  usage
  exit 2
fi

BAG_ROOT="$(realpath -m "$BAG_ROOT")"
if [[ ! -d "$BAG_ROOT" ]]; then
  echo "Bag root not found: $BAG_ROOT" >&2
  exit 2
fi
if [[ -f "$BAG_ROOT/upload_name_map.yaml" ]]; then
  echo "Bag root looks like a flattened Kleinkram mission payload: $BAG_ROOT" >&2
  echo "Reconstruct split runs first, then pass one reconstructed run root to --bag-root." >&2
  echo "Helper: /home/lorenzo/codex_skills/skills/kleinkram-upload/scripts/reconstruct_split_bags.py" >&2
  exit 2
fi
if [[ ! -f "$BAG_ROOT/sensors/metadata.yaml" ]]; then
  echo "Missing $BAG_ROOT/sensors/metadata.yaml" >&2
  exit 2
fi
if [[ ! -f "$BAG_ROOT/state/metadata.yaml" ]]; then
  echo "Missing $BAG_ROOT/state/metadata.yaml" >&2
  exit 2
fi
if [[ ! -f "$BAG_ROOT/commands/metadata.yaml" ]]; then
  echo "Missing $BAG_ROOT/commands/metadata.yaml" >&2
  exit 2
fi
if [[ ! -f "$BAG_ROOT/lidar/metadata.yaml" ]]; then
  echo "Missing $BAG_ROOT/lidar/metadata.yaml" >&2
  exit 2
fi

if [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null || true)" != "true" ]]; then
  echo "Container is not running: $CONTAINER" >&2
  exit 2
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux kill-session -t segment_replay 2>/dev/null || true

docker exec -i "$CONTAINER" bash -s <<'EOF'
set +e
pkill -f "ros2 launch mole_bringup dig_bag_replay.launch.py" || true
pkill -f "/opt/ros/jazzy/lib/foxglove_bridge/foxglove_bridge" || true
pkill -f "mole_robot_description_publisher.py" || true
pkill -f "mole_excavation_mapping_node" || true
pkill -f "elevation_mapping_node.py" || true
pkill -f "publish_grid_map_layers.py" || true
pkill -f "robot_self_filter/self_filter" || true
pkill -f "standalone_dig_newton_env.py" || true
pkill -f "mole_tf_publisher_node" || true
pkill -f "mole_joint_state_publisher.launch.py" || true
pkill -f "mole_joint_state_publisher_node" || true
pkill -f "mole_pid_joint_controller.launch.py" || true
pkill -f "mole_pid_joint_controller_node.py" || true
pkill -f "/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher" || true
pkill -f "/opt/ros/jazzy/lib/tf2_ros/static_transform_publisher" || true
pkill -f "ros2 bag play" || true
sleep 1
EOF

tmux new-session -d -s "$SESSION" -n replay
tmux new-window -t "$SESSION" -n monitor

for win in replay monitor; do
  tmux set-option -w -t "$SESSION:$win" allow-rename off
  tmux set-option -w -t "$SESSION:$win" automatic-rename off
done

REPLAY_CMD="docker exec -it $CONTAINER bash -lc 'cd $CONTAINER_WS && source /opt/ros/jazzy/setup.bash && source install/setup.bash && { urdf_file=/tmp/dig_bag_replay_mole.urdf; xacro $CONTAINER_WS/install/mole_description/share/mole_description/xacro/mole.urdf.xacro enable_continuous_joints:=true limit_j_ee:=false endeffector_type:=$ENDEFFECTOR_TYPE > \$urdf_file && ros2 run mole_ocs2_arm_controller mole_robot_description_publisher.py --ros-args -r __node:=mole_robot_description_publisher -r __ns:=/mole -p urdf_file:=\$urdf_file -p topic:=robot_description >/tmp/dig_bag_robot_description.log 2>&1 & desc_pid=\$!; trap \"kill \$desc_pid 2>/dev/null || true\" EXIT; ros2 launch mole_bringup dig_bag_replay.launch.py bag_root:=$BAG_ROOT replay_commands_bag:=true loop:=false playback_rate:=$PLAYBACK_RATE foxglove_port:=$FOXGLOVE_PORT endeffector_type:=$ENDEFFECTOR_TYPE design_bag_path:=$DESIGN_BAG_PATH; }'"
MONITOR_CMD="docker exec -it $CONTAINER bash -lc 'cd $CONTAINER_WS && source /opt/ros/jazzy/setup.bash && source install/setup.bash && echo Ready in container $CONTAINER. Foxglove: ws://localhost:$FOXGLOVE_PORT && exec bash --noprofile --norc'"

tmux send-keys -t "$SESSION:replay" "$REPLAY_CMD" C-m
tmux send-keys -t "$SESSION:monitor" "$MONITOR_CMD" C-m

echo "Replay session ready: tmux attach -t $SESSION"
echo "Foxglove: ws://localhost:$FOXGLOVE_PORT"
echo "Bag root: $BAG_ROOT"
echo "Design bag path: $DESIGN_BAG_PATH"

if [[ "$ATTACH" == "true" ]]; then
  exec tmux attach -t "$SESSION"
fi
