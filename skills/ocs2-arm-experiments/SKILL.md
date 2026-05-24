---
name: ocs2-arm-experiments
description: Start/restart Mole OCS2 arm controller experiments with real actuation commands in tmux, use configure-time auto-bootstrap (hold+reset+first-policy), activate safely, send cylindrical (r, theta, z, pitch) end-effector goals (Foxglove or CLI), record rosbag, and run staged validation while preventing competing /mole/actuator_commands publishers (e.g. dig_controller).
---

# Ocs2 Arm Experiments

## Workflow

### Delegated Startup / Configure Loop

When the user explicitly allows subagents, delegate the repetitive OCS2 launch,
configure, first-policy wait, activation, and publisher-exclusivity checks to a
worker so the main thread stays focused on the motion decision.

Delegation boundaries:
- The worker may manage the `ros:ocs2_arm` tmux launch pane and run lifecycle
  configure/activate checks.
- The worker must not send live motion goals, publish `/mole/ocs2/target`, or
  publish `/mole/actuator_commands`.
- The worker must return only a compact status: launch profile/command,
  lifecycle state, `bootstrap.done`, first-policy result, `/mole/actuator_commands`
  publisher count/names, machine status flags, and any map/SDF blocker.
- If `taskProfile:=real_collisions` fails with `No GridMap received yet`, the
  worker reports that blocker. It may only relaunch with `real_no_collisions`
  when the main agent or user has explicitly authorized the blind profile.

### 0) Segment Diagnostics Policy (Mandatory)

For **every** commanded motion segment, record full diagnostics. Do not run "naked" segments.

Required artifacts per segment:
- goal publish log
- pre/post inspection snapshots
- continuous per-sample benchmark CSV (errors, command/limit ratios, per-joint command diagnostics)

Recommended directory convention:

```bash
RUN_TAG=$(date +%Y%m%d_%H%M%S)_live
RUN_DIR=~/ocs2_benchmarks/ocs2_arm/live_${RUN_TAG}
mkdir -p "${RUN_DIR}"
```

Start continuous benchmark logger once per run (dedicated pane, keep running while you test segments):

```bash
python3 ~/ros2_ws/src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_tuning_cli.py \
  benchmark -- \
  --csv "${RUN_DIR}/segments.csv"
```

Preferred live goal-send entry point:
- Use `ros2 run mole_ocs2_arm_controller mole_m4_send_cyl_goal.py` for actual commanded motion segments.
- Treat `scripts/mole_m4_tuning_cli.py goal` only as a wrapper around that sender, not the primary operator path.

For each segment `SEG_TAG` (example: `seg3_z2p5_to_0p7`):

```bash
# 1) pre snapshot
python3 src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_tuning_cli.py \
  inspect -- --out-dir "${RUN_DIR}/${SEG_TAG}_pre"

# 2) send goal + keep full direct-target publish logs
ros2 run mole_ocs2_arm_controller mole_m4_send_cyl_goal.py \
  --r <R> --theta-deg <THETA> --z <Z> --pitch-deg <PITCH> \
  --timeout-sec 20 --result-timeout-sec 120 --no-quiet-feedback \
  2>&1 | tee "${RUN_DIR}/${SEG_TAG}_action.log"

# 3) post snapshot
python3 src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_tuning_cli.py \
  inspect -- --out-dir "${RUN_DIR}/${SEG_TAG}_post"
```

If the segment is theta-dominant, also capture predicted-trajectory consistency:

```bash
python3 src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_tuning_cli.py \
  predicted -- --goal-theta-deg <THETA> --goal-r-m <R> --json \
  > "${RUN_DIR}/${SEG_TAG}_predicted.json"
```

#### Scheduler-Driven Maneuvers

For maneuvers driven by a scheduler that republishes `/mole/ocs2/target` (for example dump/opening tests), do **not** lump multiple legs into one long benchmark run if you want clean command-vs-measured plots.

Preferred pattern:

```bash
# One run directory and one benchmark logger per maneuver leg.
RUN_DIR=~/ocs2_benchmarks/ocs2_arm/live_$(date +%Y%m%d_%H%M%S)_<LEG_TAG>
mkdir -p "${RUN_DIR}"

python3 ~/ros2_ws/src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_tuning_cli.py \
  benchmark -- --csv "${RUN_DIR}/segments.csv"
```

Also capture:
- current EE pose (`mole_m4_print_ee_cyl.py`)
- the scheduler-published target (`/mole/ocs2/target`)
- the scheduler start/stop service logs
- bucket/point references if using dump-local opening

For dump/open-close legs, the primary success metric is the calibrated
`bucket_angle` reported by `mole_m4_print_ee_cyl.py`, not `goal_pitch_deg` from
the benchmark CSV and not raw contact-frame Euler pitch. The scheduler may
publish a contact-frame quaternion whose equivalent Euler pitch looks
counterintuitive while the bucket-angle objective is still correct.

Mandatory rule for dump/open-close conclusions:
- Check the live `bucket_angle` before the leg.
- Check the live `bucket_angle` after the leg.
- Judge direction and endpoint on `bucket_angle` first.
- Use `goal_pitch_deg` / `cur_pitch_deg` only as secondary diagnostics.

If a leg appears "wrong" from `goal_pitch_deg` alone, do **not** conclude
failure until you verify the post-leg `bucket_angle`. This specifically avoids
misclassifying valid bucket-angle moves as failures just because the equivalent
`ENDEFFECTOR_CONTACT` Euler pitch took a non-intuitive value.

After the leg, write analysis artifacts explicitly:

```bash
python3 ~/ros2_ws/src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_tuning_assistant.py \
  analyze --csv "${RUN_DIR}/segments.csv" --segment-id <SEG_ID> --artifacts-dir "${RUN_DIR}/analysis"
```

When debugging oscillation, always inspect:
- `segments_seg<SEG_ID>_joint_velocity.png`
- `segments_seg<SEG_ID>_errors.png`
- `segments_seg<SEG_ID>_analysis.json`

If a remote GUI host is available, sync those artifacts immediately after each leg so command-vs-measured velocity can be inspected without rerunning the motion.

Important:
- `perseverance` is the SSH host alias for the remote machine with a screen/GUI, not a local directory.
- For remote inspection, sync artifacts with `scp` or `rsync` to `perseverance:~/Downloads/ocs2_debug/...`.
- Do not treat `/home/lorenzo/perseverance` as the remote host path unless it is explicitly mounted.

### 0.1) Workspace / Runtime Provenance (Mandatory)

Before trusting any review note, task file, or turn-model diagnosis, confirm you are looking at the same workspace that is actually running on hardware.

Minimum check:

```bash
pwd
readlink -f install/setup.bash
ros2 param get /mole/mole_arm_mpc_controller task.info_path
```

Rules:
- Record the exact task file path in the run notes.
- If a throwaway/live workspace is being used (for example `/home/lorenzo/tmp/mpc_machine_test_ws`), do **not** review or tune against stale defaults in another checkout.
- For every review bundle, copy the exact live task file and the exact live turn-model source files that produced the run.

Container overlay guard:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/moleworks/ros2_ws/install/setup.bash
readlink -f /workspace/moleworks/ros2_ws/install/setup.bash
ros2 pkg prefix mole_msgs
ros2 pkg prefix mole_ocs2_arm_controller
```

In the `moleworks_ros` Newton/Terra container, do **not** rely on a shell banner such as
`Sourcing ROS2 workspace at /home/lorenzo/ros2_ws`. That shell default can leave you with a graph that
looks alive while custom interfaces and OCS2 packages are missing in the current pane. For this workflow,
re-source `/workspace/moleworks/ros2_ws/install/setup.bash` and verify both package prefixes before
launching OCS2, benchmark scripts, or planner services.

### 0.2) True-Hold Rule (Mandatory)

Do **not** call a controller state "hold" just because the arm looks stationary.

Before diagnosing hold chatter or oscillation, prove all three:
- current target matches the current EE pose,
- the benchmark logger has started a fresh segment for that target,
- the controller is not still chasing an old large-error target.

Checks:

```bash
python3 ~/ros2_ws/src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_tuning_cli.py \
  ee -- --timeout 10
tail -n 5 "${RUN_DIR}/segments.csv"
ros2 topic echo --once /mole/ocs2/target
```

If the benchmark still shows an old target, do **not** analyze the motion as hold behavior. Clear it first by either:
- deactivating / reconfiguring / activating the controller, or
- publishing the current EE pose as a new target.

Never build a hold target from `/mole/ocs2/observation.state`; use TF / `mole_m4_print_ee_cyl.py`.

### 1. Safety / Preconditions (Hardware)

Do these checks before publishing to `/mole/actuator_commands`:

```bash
ros2 topic echo --once /machine_status | rg -n "is_hydraulilock_unlocked|is_autonomous_operation_unlocked|is_using_gravis_commands"
ros2 topic info /mole/actuator_commands -v
```

Ensure:
- `is_hydraulilock_unlocked=true`, `is_autonomous_operation_unlocked=true` only when you are ready to move.
- `/mole/actuator_commands` has **exactly one** command publisher (the OCS2 arm controller).
- Do **not** infer conflicts from tmux window state alone: `ros:foxglove` may be running while not publishing actuator commands.

If `dig_controller` is publishing actuator commands, stop it first:

```bash
ros2 lifecycle get /dig_controller
ros2 service call /dig_controller/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 6}}"
```

If `foxglove_bridge` appears as a publisher on `/mole/actuator_commands`, clear stale Foxglove publish sessions before activating OCS2:

```bash
ros2 topic info /mole/actuator_commands -v
# if you see "Node name: foxglove_bridge" under Publisher:
tmux send-keys -t ros:foxglove.1 C-c
tmux send-keys -t ros:foxglove.1 "cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml" C-m
sleep 2
ros2 topic info /mole/actuator_commands -v
```

### 2. tmux Layout (ros session)

Use/attach to the existing `ros` tmux session and create an `ocs2` window with 2 panes:
- left pane: launch OCS2
- right pane: rosbag + helper commands

If the rest of the Newton / ROS stack is already running in a container-local tmux session
(for example `newton_arm`), use that same in-container session for OCS2 windows. Do **not**
create a separate host-side tmux session that attaches into the container for a shared run.
Everyone inspecting the stack should see the same tmux namespace and window names.

Common tmux commands:

```bash
tmux ls
tmux list-windows -t ros
tmux new-window -t ros -n ocs2
tmux split-window -t ros:ocs2 -h
```

Use the same tmux pane family for launch + activate + goal publish. The preferred helpers now bootstrap ROS/workspace/FastDDS internally, so do not add manual `source /opt/ros/jazzy/setup.bash` or hand-picked DDS profiles unless you are debugging the bootstrap itself.

### 3. Launch (Real Actuation)

Fast-start path (preferred on machine; avoids duplicate-node/lifecycle drift). Run it unsourced; it resolves the workspace, sources Jazzy + the overlay, and adopts the running FastDDS profile itself:

```bash
TASK_FILE=~/ros2_ws/src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/config/mole_m4/task.pose_cyl_flat_nocollision_machine_fast_1p5_dt05_iter2_turnsmooth_e2.info \
COMMAND_LAG_COMP_SEC=0.0 AUTO_HOLD_ON_CONFIGURE=true AUTO_ACTIVATE=false \
bash ~/ros2_ws/src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_ocs2_start.sh
```

This script performs: stale-process cleanup, `ocs2_arm.launch.py`, state-aware lifecycle handover (`configure` when needed), hold/reset bootstrap, and `activate`.
It also enforces the safe startup pairing `delay_enable=true` with `command_lag_comp_sec=0.0`.

Realtime checks after launch:

```bash
# RT budget must be available in the login shell used to launch OCS2
sudo -iu lorenzo bash -lc 'ulimit -r'

# Verify effective scheduler at thread level (expect FF/99 on at least one thread)
PID=$(pgrep -f "mobile_manipulator_mpc_node" | head -n1)
ps -L -p "$PID" -o pid,tid,cls,rtprio,pri,psr,comm
```

Optional CPU pinning (active tuning only):

```bash
# Pin only RT MPC threads; keep all other nodes default/unpinned
for T in $(ps -L -p "$PID" -o tid=,cls=,rtprio= | awk '$2=="FF" && $3==99 {print $1}'); do
  taskset -cp 22,23 "$T"
done
```

Unified inspection snapshot (recommended pre/post each tuning change):

```bash
python3 ~/ros2_ws/src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_tuning_cli.py \
  inspect -- --goal-r-m 5.0 --goal-theta-deg 0.0 --goal-z-m 1.0 --goal-pitch-deg 65.0
```

Manual launch path (use only if you need custom handover/debug):

Launch OCS2 arm with real actuator commands:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/moleworks/ros2_ws/install/setup.bash
ros2 pkg prefix mole_ocs2_arm_controller
ros2 pkg prefix mole_msgs

ros2 launch mole_ocs2_arm_controller ocs2_arm.launch.py \
  use_sim_time:=false \
  actuator_commands_topic:=/mole/actuator_commands \
  command_chain_enable:=true \
  launch_policy_visualizer:=true \
  launch_target_bridge:=false \
  launch_dump_scheduler:=false \
  use_augmented_lagrangian:=false \
  command_lag_comp_sec:=0.0 \
  bootstrap_auto_hold_on_configure:=true \
  bootstrap_auto_activate:=false
```

For real-soil dump-leg runs, prefer the integrated startup path instead of a second
parallel dump-leg launch:

```bash
ros2 launch mole_ocs2_arm_controller ocs2_arm.launch.py \
  use_sim_time:=false \
  taskProfile:=real_collisions \
  elevation_map_topic:=excavation_mapping/grid_map \
  launch_policy_visualizer:=true \
  launch_dump_leg:=true \
  launch_dump_scheduler:=false \
  launch_target_bridge:=false \
  command_lag_comp_sec:=0.0 \
  sdf_max_age_sec:=0.0 \
  bootstrap_auto_hold_on_configure:=true \
  bootstrap_auto_activate:=false

ros2 lifecycle set /mole/mole_arm_mpc_controller configure
ros2 service call /mole/mobile_manipulator_mpc_node/terrain_collision/recompute_sdf std_srvs/srv/Trigger "{}"
timeout 12 ros2 topic echo --once /mole/ocs2/policy >/dev/null
ros2 lifecycle set /mole/mole_arm_mpc_controller activate

ros2 run mole_ocs2_arm_controller mole_m4_send_dump_leg_goal.py \
  --r 6.5 --theta-deg 90.0 --z 1.5 --wait-settle --settle-timeout-sec 180
```

`mole_m4_send_dump_leg_goal.py` already publishes the target and calls `/mole/mole_arm_dump_leg/start`.

Use `launch_target_bridge:=true` only when you explicitly want the action/GUI goal path.
The standard `ocs2_arm.launch.py` surface no longer exposes the legacy turn reversal knobs.
Tune the current turn model with `turn_servo_accel_decel_radps2`, `turn_servo_phase_gate_epsilon`, and `turn_servo_gate_min_abs_u_applied`.

Lifecycle handover:

```bash
ros2 lifecycle set /mole/mole_arm_mpc_controller configure
# configure triggers bootstrap: hold target -> reset -> wait first fresh policy
timeout 10 ros2 topic echo --once /mole/ocs2/policy >/dev/null
ros2 lifecycle set /mole/mole_arm_mpc_controller activate
```

Sanity checks:

```bash
timeout 5 ros2 topic hz /mole/ocs2/policy
ros2 topic echo --once /mole/ocs2/arm_controller/diagnostics
ros2 topic echo --once /mole/ocs2/arm_controller/diagnostics | rg -n "lifecycle_state|safe_stop_active|safe_stop_reason|policy_age_sec|cmd_vel.J_TURN"
```

Command-chain diagnostics:

```bash
ros2 topic echo --once /mole/ocs2/arm_controller/command_chain_diagnostics
ros2 run mole_ocs2_arm_controller mole_m4_command_chain_diag_summary.py
```

### 4. Goal Publish (Preferred: Direct Semantic Target)

Use `ros2 run mole_ocs2_arm_controller mole_m4_send_cyl_goal.py` for the real
send path. The `mole_m4_tuning_cli.py goal` subcommand is only a wrapper around
that sender and should not be treated as the primary live operator entry point.
On current `main`, the direct sender does **not** go through the old action
path by default. It directly publishes `MpcTargetTrajectories` to
`/mole/ocs2/target`, and `--pitch-deg` means the calibrated semantic
`bucket_angle`, not raw EE Euler pitch.

Use `predicted --` only as a dry-run policy/DDP check. It does not replace the
actual target send.

1) Read current cylindrical pose:

```bash
python3 ~/ros2_ws/src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_tuning_cli.py \
  ee -- --timeout 10
```

Important:
- Treat `mole_m4_print_ee_cyl.py` / TF as the source of truth for the current EE pose.
- Do **not** derive cylindrical hold targets from `/mole/ocs2/observation.state`; that topic is not a drop-in EE pose source.

2) Optionally dry-run the target first and check policy/DDP behavior:

```bash
python3 ~/ros2_ws/src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_tuning_cli.py \
  predicted -- \
  --goal-r-m 5.41 \
  --goal-theta-deg 30.0 \
  --goal-z-m 0.10 \
  --goal-pitch-deg 33.8 \
  --no-reset \
  --target-mode ramp \
  --goal-duration-sec 1.0 \
  --json
```

3) Send the real target:

```bash
ros2 run mole_ocs2_arm_controller mole_m4_send_cyl_goal.py \
  --r 5.41 \
  --theta-deg 30.0 \
  --z 0.10 \
  --pitch-deg 33.8 \
  --no-call-reset \
  --timeout-sec 20 \
  --result-timeout-sec 60
```

For a relative `+30 deg` step, keep `r/z/pitch` from the current printout and add `30` to current theta.

For large-theta tests, prefer absolute goals from the **just-read** current pose rather than chaining stale relative assumptions.

Quick verification:

```bash
python3 ~/ros2_ws/src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_tuning_cli.py \
  ee -- --timeout 10
ros2 topic echo --once /mole/ocs2/arm_controller/diagnostics | rg -n "safe_stop_active|cmd_vel.J_TURN|cmd_sat_max_ratio"
```

Optional (legacy action/GUI path, requires bridge):

Enable bridge in launch:

```bash
# set in launch command
launch_target_bridge:=true
```

CLI action goal:

```bash
ros2 run mole_ocs2_arm_controller mole_m4_send_cyl_goal.py \
  --dtheta-deg 30 \
  --timeout-sec 20 \
  --result-timeout-sec 60 \
  --no-call-reset \
  --quiet-feedback
```

Optional (Foxglove action client):

Use the OCS2 arm debug layout:
- `src/moleworks_ros/mole_bringup/foxglove/ocs2_arm_debug.json`

Send an action goal in Foxglove (with bridge enabled):
- action: `/mole/ocs2/send_cyl_goal`
- type: `mole_highlevel_msgs/action/SendCylindricalGoal`
- goal fields: `r_m`, `theta_deg`, `z_m`, `pitch_deg`, `call_reset`

Prefer publishing goals from the same `ros` tmux window used for OCS2.

Print current EE state in the same format (use as a baseline):

```bash
python3 ~/ros2_ws/src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_tuning_cli.py \
  ee -- --timeout 10
```

### 5. Command Scaling (Keep Defaults)

```bash
# Do not override command.max_velocity or command.max_accel.
# Keep defaults and tune response only via command.accel_scale in (0, 1].
ros2 param set /mole/mole_arm_mpc_controller command.accel_scale 1.0
```

Then kickstart and activate:

```bash
ros2 lifecycle set /mole/mole_arm_mpc_controller configure
timeout 10 ros2 topic echo --once /mole/ocs2/policy >/dev/null
ros2 lifecycle set /mole/mole_arm_mpc_controller activate
ros2 topic info /mole/actuator_commands -v
```

### 6. Recording (rosbag)

Record the minimum useful set:

```bash
mkdir -p ~/bags/ocs2_arm
ros2 bag record -o ~/bags/ocs2_arm/$(date +%Y%m%d_%H%M%S)_ocs2_arm \
  --topics \
  /mole/joint_states /tf /tf_static /machine_status \
  /mole/ocs2/arm_controller/diagnostics \
  /mole/ocs2/target /mole/ocs2/observation /mole/ocs2/policy \
  /mole/actuator_commands
```

### 7. Validation Phases (Recommended Order)

- Phase 0: HOLD at current pose for 30-60s, verify `safe_stop_active=0` and low saturation ratios.
- Phase 1: Single-axis tiny steps (return to baseline each time): `dr=0.02 m`, `dtheta=1 deg`, `dz=0.01 m`, `dpitch=1 deg`.
- Phase 2: Combined tiny steps (still conservative).
- Phase 3: Smooth sweeps (theta or pitch) and disturbance recovery.
- Only later: enable terrain/self-collision constraints (`use_augmented_lagrangian:=true`) and then integrate excavation mapping.

Large-theta policy:
- Do **not** jump straight to `45 deg` / `90 deg` on the short generic machine task.
- For `45 deg` / `90 deg`, use a dedicated long-horizon task and predicted-check first.
- Recommended sequence: `15 -> 30 -> 45`, then only try `90` if the predicted path and executed `45` are sane.
- If large-theta behavior is still bad, rerun on a helper-suppressed task before blaming the turn model or pitch model.

Monitor during any motion:

```bash
ros2 topic echo --once /mole/ocs2/arm_controller/diagnostics
ros2 topic echo --once /ocs2/mobile_manipulator/diagnostics
```

### 8. Troubleshooting: Planned Trajectory Visible But Robot Does Not Move

Most common cause: controller is active but latched in `SAFE_STOP` (often from `breakaway` timeout).

If CLI goal send fails with action-server timeout, verify bridge is enabled:

```bash
ros2 action list | rg -n "/mole/ocs2/send_cyl_goal"
# If listed but discovery is slow, retry with:
#   --timeout-sec 20 --result-timeout-sec 60
# If missing, restart launch with launch_target_bridge:=true
```

Inspect:

```bash
ros2 topic echo --once /mole/ocs2/arm_controller/diagnostics | rg -n "lifecycle_state|safe_stop_active|safe_stop_reason|cmd_vel.J_TURN|policy_age_sec|policy_time_to_end_sec"
```

Important non-moving fault check:

```bash
ros2 topic echo /mole/actuator_commands
ros2 topic echo --once /mole/joint_states
```

If the controller is active, the arm is not moving, and `/mole/actuator_commands`
stays non-zero for a couple of seconds, suspect machine communication rather
than MPC planning. In that fault mode, the machine often recovers only after it
sees a few seconds of zero actuator commands.

Recommended recovery:

```bash
# Stop scheduler / goal source first so commands can go to zero
ros2 service call /mole/mole_arm_dump_scheduler/stop std_srvs/srv/Trigger '{}'

# Or deactivate the controller briefly to force zero commands
ros2 lifecycle set /mole/mole_arm_mpc_controller deactivate
sleep 3
ros2 lifecycle set /mole/mole_arm_mpc_controller activate
```

If that does not clear it, restart the OCS2 controller node / stack and verify
`/mole/actuator_commands` is zero before sending the next goal.

If the live pane shows repeated
- `The requested currentTime is greater than the received plan`
- `policy_age` climbing into the seconds
- `SAFE_STOP: MPC policy horizon expired`

do **not** spend time toggling lifecycle transitions repeatedly. That is a stale-plan / late-solver fault; restart the full OCS2 launch window.

Fast OCS2 restart recipe in the shared tmux session:

```bash
tmux list-windows -t newton_sim
tmux kill-window -t newton_sim:ocs2 2>/dev/null || true
pkill -f "mobile_manipulator_mpc_node" || true
pkill -f "mole_arm_mpc_controller" || true
pkill -f "mole_arm_move_leg.py" || true
pkill -f "mole_arm_dump_leg.py" || true
pkill -f "mole_arm_dump_scheduler.py" || true
pkill -f "mole_m4_policy_visualizer" || true
tmux new-window -t newton_sim -n ocs2
tmux send-keys -t newton_sim:ocs2 "export ROS_DOMAIN_ID=24" C-m
tmux send-keys -t newton_sim:ocs2 "source /workspace/moleworks/ros2_ws/install/setup.bash" C-m
tmux send-keys -t newton_sim:ocs2 "ros2 launch mole_ocs2_arm_controller ocs2_arm.launch.py use_sim_time:=true robot_namespace:=mole auto_handover:=false taskProfile:=real_collisions elevation_map_topic:=/excavation_mapping/grid_map elevation_map_layer:=elevation command_lag_comp_sec:=0.0 delay_enable:=true delay_command_prefilter_enable:=true launch_move_leg:=true launch_dump_leg:=true launch_dump_scheduler:=true launch_policy_visualizer:=true bootstrap_auto_hold_on_configure:=true bootstrap_auto_activate:=false turn_servo_enable:=true boom_servo_enable:=true stick_servo_enable:=true tele_servo_enable:=true pitch_servo_enable:=true mpc_cpu_affinity:=22-23 arm_cpu_affinity:=22-23" C-m
```

Then re-bootstrap the controller:

```bash
ros2 lifecycle set /mole/mole_arm_mpc_controller configure
ros2 lifecycle set /mole/mole_arm_mpc_controller activate
ros2 lifecycle get /mole/mole_arm_mpc_controller
ros2 topic info /mole/actuator_commands -v
```

Rules:
- If `tmux capture-pane -pt newton_sim:ocs2` says `can't find window: ocs2`, recreate the window first. Do not keep sending commands to a dead pane.
- For long relaunch commands in shared tmux sessions, prefer `tmux new-window` followed by `tmux send-keys` over a single huge quoted `tmux new-window '...'` command. It fails less often and is easier to inspect.
- Restart only `ocs2` when Newton, TF, and the rest of the ROS stack are still healthy. If Newton was restarted, restart the full ROS-side stack instead.

If `safe_stop_active=1` with a breakaway timeout reason:

```bash
ros2 lifecycle set /mole/mole_arm_mpc_controller deactivate
ros2 lifecycle set /mole/mole_arm_mpc_controller activate
```

Then republish a small goal and confirm commands are non-zero:

```bash
python3 src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_send_cyl_goal.py --dtheta-deg 10 --no-call-reset --timeout-sec 20 --result-timeout-sec 60
ros2 topic echo --once /mole/ocs2/arm_controller/diagnostics | rg -n "safe_stop_active|cmd_vel.J_TURN"
ros2 topic echo --once /mole/actuator_commands
```

Bench-debug only (not final on-robot tuning):
- You can relax or disable breakaway to validate control-path plumbing, then re-enable it.
- Launch-time knobs: `breakaway_enable`, `breakaway.t_fail_pos[0]`, `breakaway.t_fail_neg[0]`.
