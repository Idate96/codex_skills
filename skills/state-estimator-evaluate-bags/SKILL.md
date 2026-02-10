---
name: state-estimator-evaluate-bags
description: "Evaluate the Moleworks ROS2 `mole_estimator` on recorded MCAP/rosbag2 datasets: replay `*_sensors` bags through the estimator (use_sim_time), record a reprocessed eval bag with `/mole/state` + `/graph_msf/*`, run the offline analyzer, and generate a per-bag metrics markdown + summary table for tuning turn-joint filtering and base velocity smoothness."
---

# State Estimator Evaluate Bags

## Quick start (workflow)

1) Build the estimator (fail fast on duplicate packages in this workspace):

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to mole_estimator
source install/setup.bash
```

2) Pick a bag to evaluate.

Typical layout:
- sensor-only input: `~/mcap/<batch>/mole_estimator/<scenario>_sensors`
- reprocessed eval bag output: `~/mcap/<batch>/reproc/<scenario>_reproc_<timestamp>`

3) Reprocess `*_sensors` into eval bags (recommended automation).

Use the batch reprocessor (it orchestrates a dedicated tmux session so ROS output is inspectable):

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Important: pick a ROS domain id that is NOT already in use, otherwise you'll record mixed publishers.
python3 src/moleworks_ros/mole_estimator/scripts/reprocess_mole_estimator_sensor_bags.py \
  --batch-dir ~/mcap/mole_estimator_batch_2026-02-08 \
  --ros-domain-id 80
```

Notes:
- The script hard-fails if required topics are missing (including the 4 leg IMUs).
- The script hard-fails if `ROS_DOMAIN_ID` already has any nodes (domain isolation preflight).
- For iteration speed it trims `legs_updown_20260208_152409` to 240 s by default (override with `--no-default-trims` or `--trim ...`).
 - The reprocessed eval bags record `/mole/turn_joint_filtered` so the offline analyzer can use the same filtered turn-joint omega as the estimator.

4) Analyze + generate the summary table (automation).

Batch (writes JSON per bag + a Markdown report):

```bash
python3 src/moleworks_ros/mole_estimator/scripts/evaluate_mole_estimator_eval_bags.py \
  --reproc-dir ~/mcap/mole_estimator_batch_2026-02-08/reproc \
  --json-dir ~/mcap/mole_estimator_batch_2026-02-08/metrics_YYYYMMDD_HHMMSS \
  --md-out src/moleworks_ros/mole_estimator/docs/eval_batch_2026-02-08_metrics.md
```

Single bag (after reprocessing):

```bash
python3 src/moleworks_ros/mole_estimator/scripts/analyze_mole_estimator_eval_bag.py \
  "$OUT_BAG" \
  --config src/moleworks_ros/mole_estimator/config/mole_estimator.yaml
```

5) Compare two runs (diff helper).

```bash
python3 src/moleworks_ros/mole_estimator/scripts/compare_mole_estimator_metrics_runs.py \
  --a-json-dir ~/mcap/mole_estimator_batch_2026-02-08/metrics_OLD \
  --b-json-dir ~/mcap/mole_estimator_batch_2026-02-08/metrics_NEW \
  --label-a OLD \
  --label-b NEW \
  --md-out ~/mcap/mole_estimator_batch_2026-02-08/metrics_NEW/metrics_diff.md
```

The diff report includes control-relevant checks like `wz_err` (BASE yaw-rate consistency), plus the core smoothness/jitter metrics.

## Manual reprocessing (fallback)

If you want to do it manually, use 3 tmux windows: `est`, `rec`, `play`. Use a single `bash -lc '...'` per window (avoid multiline send-keys).

Important:
- Restart the estimator for each bag. Graph-MSF is not robust to `/clock` jumping backwards between separate bag plays (it can start rejecting measurements or crash on "measurement delay").

Estimator:

```bash
export ROS_DOMAIN_ID=77
ros2 launch mole_estimator mole_estimator.launch.py \
  config:=src/moleworks_ros/mole_estimator/config/mole_estimator.yaml \
  use_sim_time:=true
```

Recorder (writes the eval bag that the analyzer needs):

```bash
export ROS_DOMAIN_ID=77
export MOLE_EVAL_USE_SIM_TIME=1
src/moleworks_ros/mole_estimator/scripts/record_mole_estimator_eval_bag.sh "$OUT_BAG"
```

Player (exclude `/tf` and `/tf_static` from the sensors bag so the output bag has a single TF publisher):

```bash
export ROS_DOMAIN_ID=77
ros2 bag play "$SENSORS_BAG" --clock --exclude-topics /tf /tf_static
```

Optional trimming (useful if the bag has a long static section at the start):

```bash
export ROS_DOMAIN_ID=77
ros2 bag play "$SENSORS_BAG" \
  --clock \
  --exclude-topics /tf /tf_static \
  --start-offset 60.0 \
  --playback-duration 240.0
```

Notes on the report:
- The summary table highlights strict-threshold failures as `FAIL(xxx)`.
- It reports `J_TURN lag abs-med [ms]` (effective turn-joint lag; keep small since `J_TURN` is actively controlled).
- It reports pose/twist consistency in two frames:
  - `pose-twist map v_xy RMS`: includes map<->odom corrections (can be inflated by global corrections).
  - `pose-twist odom v_xy RMS`: preferred local consistency check (odom->base vs twist).
- Pose/twist consistency uses backward alignment (`*_bwd`) because the estimator publishes velocities derived from a
  backward finite difference.
- The analyzer drops initial `STATUS_OK` transients by default (`ok_warmup_s=1.0`).

Key estimator knob for velocity quality:
- `output_twist.source` in `mole_estimator/config/mole_estimator.yaml`
  - `graph`: derive BASE twist from Graph-MSF IMU twist + transforms
  - `pose_diff_odom`: finite-diff odom->BASE to get consistent v_xy (recommended baseline)
- `output_twist.pose_diff_lpf_tau_s`: small low-pass (<=0.10s) if v_xy is noisy.

## Common pitfalls (high signal)

- Always replay sensor bags with `--clock` and run the estimator with `use_sim_time:=true`.
- Always exclude `/tf` and `/tf_static` during replay if the sensors bag contains them.
- If Python cannot import `mole_msgs`, you forgot to `source install/setup.bash`.
