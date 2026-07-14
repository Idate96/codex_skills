#!/usr/bin/env bash
set -euo pipefail

duration_sec=45
min_radius=2.0
max_radius=10.0
voxel_size=0.01
tf_warmup_sec=8
tf_timeout_sec=0.1
out_root=/home/lorenzo/mcap
basename=colored_accum_1cm
target_frame=map
source_frame=livox_front_left
sync_host=
sync_root=/home/lorenzo/mcap

usage() {
  cat <<'EOF'
Usage: record_colored_map_fast.sh [options]

Options:
  --duration-sec SEC       Accumulation duration after TF warm-up; 0 means until stopped (default: 45)
  --min-radius M           Sensor-frame crop min radius (default: 2.0)
  --max-radius M           Sensor-frame crop max radius (default: 10.0)
  --voxel-size M           Voxel size; keep 0.01 unless explicitly requested
  --tf-warmup-sec SEC      Warm TF before accumulating (default: 8)
  --tf-timeout-sec SEC     Per-cloud TF lookup timeout (default: 0.1)
  --out-root DIR           Output root (default: /home/lorenzo/mcap)
  --basename NAME          Output basename (default: colored_accum_1cm)
  --sync-host HOST         Rsync finished run directory to HOST:/home/lorenzo/mcap
  --sync-perseverance      Shortcut for --sync-host perseverance
  --sync-root DIR          Remote sync root (default: /home/lorenzo/mcap)
  --help                   Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec) duration_sec="$2"; shift 2 ;;
    --min-radius) min_radius="$2"; shift 2 ;;
    --max-radius) max_radius="$2"; shift 2 ;;
    --voxel-size) voxel_size="$2"; shift 2 ;;
    --tf-warmup-sec) tf_warmup_sec="$2"; shift 2 ;;
    --tf-timeout-sec) tf_timeout_sec="$2"; shift 2 ;;
    --out-root) out_root="$2"; shift 2 ;;
    --basename) basename="$2"; shift 2 ;;
    --sync-host|--sync) sync_host="$2"; shift 2 ;;
    --sync-perseverance) sync_host=perseverance; shift ;;
    --sync-root) sync_root="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$sync_host" in
  perserverance|perservance) sync_host=perseverance ;;
esac

set +u
source /opt/ros/jazzy/setup.bash
workspace="${MOLEWORKS_ROS_WS:-}"
if [[ -z "$workspace" ]]; then
  for candidate in "$HOME/ros2_ws" "$HOME/moleworks/ros2_ws"; do
    if [[ -f "$candidate/install/setup.bash" ]]; then
      workspace="$candidate"
      break
    fi
  done
fi
if [[ -z "$workspace" || ! -f "$workspace/install/setup.bash" ]]; then
  echo "Could not find a built ROS workspace; set MOLEWORKS_ROS_WS." >&2
  exit 2
fi
source "$workspace/install/setup.bash"
set -u

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
accumulator="$script_dir/accumulate_pointcloud_topic.py"
timestamp="$(date -u +%Y%m%d_%H%M%S)"
out="${out_root%/}/colored_site_map_1cm_${timestamp}"
mkdir -p "$out"

topic_list="$out/topic_list.txt"
ros2 topic list > "$topic_list"
for topic in \
  /mole/livox_lidar_publisher/lidar_front_left \
  /camMainView/image_raw \
  /camMainView/camera_info \
  /tf \
  /tf_static; do
  if ! grep -qx "$topic" "$topic_list"; then
    echo "Missing required topic: $topic" >&2
    exit 1
  fi
done

ros2 pkg executables mole_lidar_backprojection >/dev/null

config_src="$workspace/src/open3d_slam/ros/open3d_slam_ros/param/param_robosense_rs16.yaml"
config_dst="$out/param_livox_dense_static.yaml"
if [[ -f "$config_src" ]]; then
  cp "$config_src" "$config_dst"
  python3 - "$config_dst" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text()
replacements = {
    "  is_build_dense_map: false": "  is_build_dense_map: true",
    "  is_attempt_loop_closures: true": "  is_attempt_loop_closures: false",
    "    map_voxel_size: 0.05": "    map_voxel_size: 0.01",
    "      cropping_radius_max: 16.0": "      cropping_radius_max: 40.0",
    "      cropping_radius_min: 2.0": "      cropping_radius_min: 0.0",
    "  save_dense_submaps: false": "  save_dense_submaps: true",
}
for old, new in replacements.items():
    text = text.replace(old, new, 1)
path.write_text(text)
PY
fi

backprojection_pid=""
existing_publishers="$(ros2 topic info /mole/colored_point_cloud 2>/dev/null | awk '/Publisher count:/ {print $3; exit}')"
if [[ "${existing_publishers:-0}" -gt 0 ]]; then
  echo "Reusing existing /mole/colored_point_cloud publisher." > "$out/backprojection.log"
else
  backprojection_log="$out/backprojection.log"
  setsid ros2 launch mole_lidar_backprojection mole_lidar_backprojection.launch.py \
    use_sim_time:=false \
    robot_namespace:=mole \
    mode:=color \
    camera_topic:=/camMainView/image_raw \
    camera_info_topic:=/camMainView/camera_info \
    lidar_topic:=livox_lidar_publisher/lidar_front_left \
    keep_uncolored_points:=true \
    > "$backprojection_log" 2>&1 &
  backprojection_pid=$!
fi

cleanup() {
  if [[ -n "$backprojection_pid" ]] && kill -0 "$backprojection_pid" 2>/dev/null; then
    kill -INT "-$backprojection_pid" 2>/dev/null || kill -INT "$backprojection_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      if ! kill -0 "$backprojection_pid" 2>/dev/null; then
        wait "$backprojection_pid" 2>/dev/null || true
        return
      fi
      sleep 0.25
    done
    kill -TERM "-$backprojection_pid" 2>/dev/null || kill -TERM "$backprojection_pid" 2>/dev/null || true
    wait "$backprojection_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

for _ in $(seq 1 30); do
  if ros2 topic info /mole/colored_point_cloud >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

ros2 topic info /mole/colored_point_cloud --verbose > "$out/colored_cloud_info.txt"
timeout 15 ros2 topic echo /mole/colored_point_cloud --once --field header \
  > "$out/colored_cloud_header.txt"
timeout 8 ros2 topic echo /mole/colored_point_cloud --once --field point_step \
  > "$out/colored_cloud_point_step.txt" || true

set +e
python3 "$accumulator" \
  /mole/colored_point_cloud \
  "$out" \
  --basename "$basename" \
  --target-frame "$target_frame" \
  --source-frame "$source_frame" \
  --tf-warmup-sec "$tf_warmup_sec" \
  --tf-timeout-sec "$tf_timeout_sec" \
  --duration-sec "$duration_sec" \
  --min-radius "$min_radius" \
  --max-radius "$max_radius" \
  --voxel-size "$voxel_size" \
  --reliability reliable \
  --formats ply,pcd \
  | tee "$out/accumulator.log"
accumulator_status=${PIPESTATUS[0]}
set -e
if [[ "$accumulator_status" -ne 0 && "$accumulator_status" -ne 130 ]]; then
  exit "$accumulator_status"
fi

notes="$out/RUN_NOTES.md"
{
  echo "# Colored Site Map 1 cm Run Notes"
  echo
  echo "- Date UTC: $(date -u '+%Y-%m-%d %H:%M:%S')"
  echo "- Mode: fast static Foxglove-style accumulator"
  echo "- Input cloud: /mole/colored_point_cloud"
  echo "- Raw LiDAR source: /mole/livox_lidar_publisher/lidar_front_left"
  echo "- Camera source: /camMainView/image_raw"
  echo "- Camera info: /camMainView/camera_info"
  echo "- Target frame: $target_frame"
  echo "- Source frame: $source_frame"
  echo "- Voxel size: $voxel_size m"
  echo "- Sensor-frame crop radius: $min_radius m to $max_radius m"
  echo "- TF warm-up: $tf_warmup_sec s"
  echo "- TF lookup timeout: $tf_timeout_sec s"
  echo "- Backprojection log: backprojection.log"
  echo "- Accumulator log: accumulator.log"
  echo
  echo "Accumulator summary:"
  echo
  while IFS= read -r line; do
    case "$line" in
      topic=*|target_frame=*|frames=*|frames_used=*|frames_tf_failed=*|raw_points=*|cropped_points=*|voxel_points=*|voxel_size=*|radius_min=*|radius_max=*|has_colors=*|aabb_min=*|aabb_max=*)
        echo "- \`$line\`"
        ;;
    esac
  done < "$out/accumulator.log"
} > "$notes"

(cd "$out" && find . -maxdepth 1 -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 -r sha256sum > SHA256SUMS)

if [[ -n "$sync_host" ]]; then
  remote_dir="${sync_root%/}/$(basename "$out")"
  ssh -o BatchMode=yes "$sync_host" "mkdir -p '$remote_dir'"
  rsync -avh "$out/" "$sync_host:$remote_dir/"
  ssh -o BatchMode=yes "$sync_host" "cd '$remote_dir' && sha256sum -c SHA256SUMS"
  echo "synced=$sync_host:$remote_dir/"
fi

echo "out=$out"
echo "ply=$out/$basename.ply"
echo "pcd=$out/$basename.pcd"
