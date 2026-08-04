#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
dig_segment_replay_tmux.sh

Replay one monolithic single-scoop segment bag in the moleworks_ros container
for fast Foxglove review.

Usage:
  dig_segment_replay_tmux.sh --segment-dir PATH [options]
  dig_segment_replay_tmux.sh --manifest PATH --segment-name NAME [options]

Options:
  --segment-dir PATH      Segment bag directory with metadata.yaml
  --container-segment-dir PATH
                          Corresponding segment path inside container
  --manifest PATH         CSV manifest with output_dir and segment_name columns
  --segment-name NAME     Segment name to resolve from the manifest
  --session NAME          tmux session name (default: segment_replay)
  --container NAME        Docker container name (default: moleworks_ros)
  --container-ws PATH     Workspace path inside container (default: auto-detect)
  --playback-rate RATE    ros2 bag replay rate (default: 0.2)
  --foxglove-port PORT    Foxglove bridge port (default: 8766)
  --endeffector-type TYPE URDF end-effector type (default: shovel)
  --confirm-offline-container
                          Confirm this container is dedicated to offline replay
  --attach                Attach to tmux after startup
  -h, --help              Show this help
USAGE
}

SESSION="segment_replay"
CONTAINER="moleworks_ros"
CONTAINER_WS=""
PLAYBACK_RATE="0.2"
FOXGLOVE_PORT="8766"
ENDEFFECTOR_TYPE="shovel"
ATTACH="false"
CONFIRM_OFFLINE_CONTAINER="false"
SEGMENT_DIR=""
CONTAINER_SEGMENT_DIR=""
MANIFEST=""
SEGMENT_NAME=""
SEGMENT_RECOMMENDATION=""
SEGMENT_LABEL=""
SEGMENT_NOTES=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --segment-dir)
      SEGMENT_DIR="${2:-}"; shift 2 ;;
    --container-segment-dir)
      CONTAINER_SEGMENT_DIR="${2:-}"; shift 2 ;;
    --manifest)
      MANIFEST="${2:-}"; shift 2 ;;
    --segment-name)
      SEGMENT_NAME="${2:-}"; shift 2 ;;
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
    --confirm-offline-container)
      CONFIRM_OFFLINE_CONTAINER="true"; shift ;;
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

if [[ "$CONFIRM_OFFLINE_CONTAINER" != "true" ]]; then
  echo "Refusing replay without --confirm-offline-container." >&2
  echo "Use a container dedicated to offline replay; never confirm a live robot container." >&2
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

if [[ -n "$SEGMENT_DIR" && ( -n "$MANIFEST" || -n "$SEGMENT_NAME" ) ]]; then
  echo "Use either --segment-dir or --manifest + --segment-name" >&2
  exit 2
fi

if [[ -z "$SEGMENT_DIR" ]]; then
  if [[ -z "$MANIFEST" || -z "$SEGMENT_NAME" ]]; then
    echo "Need --segment-dir or --manifest + --segment-name" >&2
    usage
    exit 2
  fi
  MANIFEST="$(realpath -m "$MANIFEST")"
  if [[ ! -f "$MANIFEST" ]]; then
    echo "Manifest not found: $MANIFEST" >&2
    exit 2
  fi
  mapfile -t RESOLVED < <(
    python3 - "$MANIFEST" "$SEGMENT_NAME" <<'PY'
import csv
import sys

manifest_path, segment_name = sys.argv[1], sys.argv[2]
matches = []
with open(manifest_path, newline="") as f:
    for row in csv.DictReader(f):
        if row.get("segment_name") == segment_name:
            matches.append(row)

if not matches:
    raise SystemExit(f"segment_name not found in manifest: {segment_name}")
if len(matches) != 1:
    raise SystemExit(f"segment_name is not unique in manifest: {segment_name}")

row = matches[0]
print(row["output_dir"])
print(row.get("recommendation", ""))
print(row.get("label", ""))
print(row.get("notes", ""))
PY
  )
  SEGMENT_DIR="${RESOLVED[0]}"
  SEGMENT_RECOMMENDATION="${RESOLVED[1]}"
  SEGMENT_LABEL="${RESOLVED[2]}"
  SEGMENT_NOTES="${RESOLVED[3]}"
  if [[ "$SEGMENT_DIR" != /* ]]; then
    SEGMENT_DIR="$(dirname "$MANIFEST")/$SEGMENT_DIR"
  fi
fi

SEGMENT_DIR="$(realpath -m "$SEGMENT_DIR")"
if [[ -f "$SEGMENT_DIR" && "$SEGMENT_DIR" == *.mcap ]]; then
  SEGMENT_DIR="$(dirname "$SEGMENT_DIR")"
fi
if [[ ! -d "$SEGMENT_DIR" ]]; then
  echo "Segment dir not found: $SEGMENT_DIR" >&2
  exit 2
fi
if [[ ! -f "$SEGMENT_DIR/metadata.yaml" ]]; then
  echo "Missing $SEGMENT_DIR/metadata.yaml" >&2
  exit 2
fi
if ! find "$SEGMENT_DIR" -maxdepth 1 -type f -name '*.mcap' | grep -q .; then
  echo "No .mcap file found under $SEGMENT_DIR" >&2
  exit 2
fi
if [[ -z "$CONTAINER_SEGMENT_DIR" ]]; then
  CONTAINER_SEGMENT_DIR="$SEGMENT_DIR"
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
printf -v container_segment_q '%q' "$CONTAINER_SEGMENT_DIR"
if ! docker exec "$CONTAINER" bash -lc "test -f $container_segment_q/metadata.yaml"; then
  echo "Segment is not visible inside container: $CONTAINER_SEGMENT_DIR" >&2
  echo "Use --container-segment-dir when the host and container paths differ." >&2
  exit 2
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true
if [[ "$SESSION" != "bag_replay" ]]; then
  tmux kill-session -t bag_replay 2>/dev/null || true
fi

tmux new-session -d -s "$SESSION" -n replay
tmux new-window -t "$SESSION" -n monitor

for win in replay monitor; do
  tmux set-option -w -t "$SESSION:$win" allow-rename off
  tmux set-option -w -t "$SESSION:$win" automatic-rename off
done

printf -v container_q '%q' "$CONTAINER"
printf -v ws_q '%q' "$CONTAINER_WS"
printf -v segment_q '%q' "$CONTAINER_SEGMENT_DIR"
printf -v rate_q '%q' "$PLAYBACK_RATE"
printf -v port_q '%q' "$FOXGLOVE_PORT"
printf -v ee_q '%q' "$ENDEFFECTOR_TYPE"
REPLAY_CMD="docker exec -it $container_q bash -lc 'cd $ws_q && source /opt/ros/jazzy/setup.bash && source install/setup.bash && { urdf_file=/tmp/dig_segment_replay_mole.urdf; xacro install/mole_description/share/mole_description/xacro/mole.urdf.xacro enable_continuous_joints:=true limit_j_ee:=false endeffector_type:=$ee_q > \$urdf_file && ros2 run mole_ocs2_arm_controller mole_robot_description_publisher.py --ros-args -r __node:=mole_robot_description_publisher -r __ns:=/mole -p urdf_file:=\$urdf_file -p topic:=robot_description >/tmp/dig_segment_robot_description.log 2>&1 & desc_pid=\$!; ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=$port_q use_sim_time:=true >/tmp/dig_segment_foxglove_$port_q.log 2>&1 & bridge_pid=\$!; trap \"kill \$desc_pid \$bridge_pid 2>/dev/null || true\" EXIT; sleep 2; ros2 bag play $segment_q --clock 100 --rate $rate_q --disable-keyboard-controls; }'"
MONITOR_CMD="docker exec -it $container_q bash -lc 'cd $ws_q && source /opt/ros/jazzy/setup.bash && source install/setup.bash && echo Ready in container $CONTAINER. Foxglove: ws://localhost:$FOXGLOVE_PORT && exec bash --noprofile --norc'"

tmux send-keys -t "$SESSION:replay" "$REPLAY_CMD" C-m
tmux send-keys -t "$SESSION:monitor" "$MONITOR_CMD" C-m

echo "Segment replay ready: tmux attach -t $SESSION"
echo "Foxglove: ws://localhost:$FOXGLOVE_PORT"
echo "Segment dir: $SEGMENT_DIR"
if [[ -n "$SEGMENT_RECOMMENDATION" ]]; then
  echo "Recommendation: $SEGMENT_RECOMMENDATION"
fi
if [[ -n "$SEGMENT_LABEL" ]]; then
  echo "Label: $SEGMENT_LABEL"
fi
if [[ -n "$SEGMENT_NOTES" ]]; then
  echo "Notes: $SEGMENT_NOTES"
fi

if [[ "$ATTACH" == "true" ]]; then
  exec tmux attach -t "$SESSION"
fi
