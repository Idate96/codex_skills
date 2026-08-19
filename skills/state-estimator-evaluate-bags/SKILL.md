---
name: state-estimator-evaluate-bags
description: "Reprocess and evaluate Mole estimator rosbags. Use for Graph-MSF output generation, isolated ROS-domain replay, metric comparison, failure diagnosis, and tuning reports."
---

# State Estimator Evaluate Bags

## Quick start (workflow)

1) Build the estimator (fail fast on duplicate packages in this workspace):

```bash
WS="$HOME/ros2_ws"; [[ -f "$WS/install/setup.bash" ]] || WS="$HOME/moleworks/ros2_ws"
cd "$WS"
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to mole_estimator mole_bag_tools
source install/setup.bash
```

2) Pick a bag to evaluate.

Typical layout:
- sensor-only input: `~/mcap/<batch>/mole_estimator/<scenario>_sensors`
- reprocessed eval bag output: `~/mcap/<batch>/reproc/<scenario>_reproc_<timestamp>`

3) Reprocess `*_sensors` into eval bags (recommended automation).

Use the batch reprocessor (it orchestrates a dedicated tmux session so ROS output is inspectable):

```bash
WS="$HOME/ros2_ws"; [[ -f "$WS/install/setup.bash" ]] || WS="$HOME/moleworks/ros2_ws"
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Remove inherited robot/VPN discovery settings before choosing replay isolation.
unset ROS_DISCOVERY_SERVER DDS_DISCOVERY_SERVER_IP \
  FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE \
  CYCLONEDDS_URI RMW_FASTRTPS_USE_QOS_FROM_XML ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=80

# Fail closed if discovery errors or this domain is already occupied.
if ! REPLAY_NODES="$(ros2 node list --no-daemon --spin-time 1.0)"; then
  echo "ROS graph preflight failed" >&2
  exit 1
fi
if [[ -n "$REPLAY_NODES" ]]; then
  printf 'ROS domain %s is occupied:\n%s\n' "$ROS_DOMAIN_ID" "$REPLAY_NODES" >&2
  exit 1
fi
BATCH_DIR="$HOME/mcap/<batch>"

ros2 run mole_bag_tools reprocess_mole_estimator_sensor_bags \
  --batch-dir "$BATCH_DIR" \
  --ros-domain-id "$ROS_DOMAIN_ID"
```

Notes:
- The script hard-fails if required topics are missing (including the 4 leg IMUs).
- The script hard-fails if `ROS_DOMAIN_ID` already has any nodes (domain isolation preflight).
- Use `--no-default-trims` or an explicit `--trim ...` when a repository default trim does not match the selected batch.
- Reprocessed eval bags record `/mole/turn_joint_filtered` so the analyzer can use the estimator's filtered turn-joint omega.

4) Analyze + generate the summary table (automation).

Batch (writes JSON per bag + a Markdown report):

```bash
ros2 run mole_bag_tools evaluate_mole_estimator_eval_bags \
  --reproc-dir "$BATCH_DIR/reproc" \
  --json-dir "$BATCH_DIR/metrics_YYYYMMDD_HHMMSS" \
  --md-out "$BATCH_DIR/metrics_YYYYMMDD_HHMMSS/report.md"
```

Single bag (after reprocessing):

```bash
ros2 run mole_bag_tools analyze_mole_estimator_eval_bag \
  "$OUT_BAG" \
  --config src/moleworks_ros/mole_estimator/config/mole_estimator.yaml
```

5) Compare two runs (diff helper).

```bash
ros2 run mole_bag_tools compare_mole_estimator_metrics_runs \
  --a-json-dir "$BATCH_DIR/metrics_OLD" \
  --b-json-dir "$BATCH_DIR/metrics_NEW" \
  --label-a OLD \
  --label-b NEW \
  --md-out "$BATCH_DIR/metrics_NEW/metrics_diff.md"
```

The diff report includes control-relevant checks like `wz_err` (BASE yaw-rate consistency), plus the core smoothness/jitter metrics.

## Manual reprocessing (fallback)

If you want to do it manually, use 3 tmux windows: `est`, `rec`, `play`. Use a single `bash -lc '...'` per window (avoid multiline send-keys). Prefix every command with the workspace source, `unset`, and three replay-isolation exports from step 3 so each window explicitly uses Fast DDS, `LOCALHOST`, and the chosen domain. Run the `ros2 node list --no-daemon --spin-time 1.0` preflight once in `est`, before starting any process; abort on command failure or any listed node.

Important:
- Restart the estimator for each bag. Graph-MSF is not robust to `/clock` jumping backwards between separate bag plays (it can start rejecting measurements or crash on "measurement delay").

Estimator:

```bash
ros2 launch mole_estimator mole_estimator.launch.py \
  config:=src/moleworks_ros/mole_estimator/config/mole_estimator.yaml \
  use_sim_time:=true
```

Recorder (writes the eval bag that the analyzer needs):

```bash
export MOLE_EVAL_USE_SIM_TIME=0
"$(ros2 pkg prefix mole_bag_tools)/lib/mole_bag_tools/record_mole_estimator_eval_bag.sh" "$OUT_BAG"
```

Recorder wall time does not rewrite message header stamps; they retain the replayed bag time.

Player (exclude `/tf` and `/tf_static` from the sensors bag so the output bag has a single TF publisher):

```bash
ros2 bag play "$SENSORS_BAG" --clock --exclude-topics /tf /tf_static
```

Optional trimming (useful if the bag has a long static section at the start):

```bash
ros2 bag play "$SENSORS_BAG" \
  --clock \
  --exclude-topics /tf /tf_static \
  --start-offset 60.0 \
  --playback-duration 240.0
```

For metric semantics, frame interpretation, and the main velocity-quality knobs, read [references/metrics.md](references/metrics.md).

## Common pitfalls (high signal)

- Always replay sensor bags with `--clock` and run the estimator with `use_sim_time:=true`.
- Always exclude `/tf` and `/tf_static` during replay if the sensors bag contains them.
- If Python cannot import `mole_msgs`, you forgot to `source install/setup.bash`.
