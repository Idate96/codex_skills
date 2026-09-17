# CAT323 tracking recording on integration x86

Use this profile for bounded CAT323 arm tracking tests in `gravis_ugep`, workspace
`/workspaces/gravis_ws`, tmux session `moleworks_ros`. Read the installed `gravis-cat323`
skill for the actual machine state and existing motion authorization. The recorder only
subscribes and writes bags; it does not configure a controller, send commands, unlock
hydraulics, set RPM, restart software, or access Orin.

## Start and stop

Use the already configured CAT323 ROS environment. The recorder uses the observer DDS
profile in its own process environment so Discovery Server exposes the full graph; do
not switch or restart running participants. Hardware uses wall-clock time, not `/clock`.

For the September 2026 tracking session, create a new recorder window with the following
command. Check that `record-tracking` is free first; choose another name if it exists.
Do not interrupt an existing recording or reuse an existing output directory.

```bash
mkdir -p /workspaces/gravis_ws/evidence/ik_tracking_20260910
tmux list-windows -t moleworks_ros -F '#{window_name}'
tmux new-window -d -t moleworks_ros -n record-tracking \
  'bash -lc "source /workspaces/gravis_ws/evidence/cat323-env.bash && export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/gravis_ws/evidence/dds-observer-lan-only.xml && exec python3 /workspaces/gravis_ws/codex_skills/skills/dig-bag-recording/scripts/cat323_tracking_recording.py record --camera --run-dir /workspaces/gravis_ws/evidence/ik_tracking_20260910/bag_tracking"'
```

The parent output directory must exist; `bag_tracking` must not exist. Choose a new
run name for each recording. The helper checks free space (default minimum 1 GiB), saves
a topic graph snapshot and its exact topic/command/environment manifest, and starts four
recorders, plus the main-camera recorder when `--camera` is selected. It refuses to overwrite an existing run. `plan --run-dir PATH` prints the
recording commands without creating files or subscribing.

Inspect `processes.json` and the selected `recorder-*.log` files. All selected recorders must reach
`Listening for topics`; check subscriptions to `/mole/measurements`, `/machine_status`,
`/joint_states`, and `/tf` before motion. `status: recording` confirms recorder startup,
not machine readiness or message freshness. The recorder continues discovery every
100 ms so command and UGEP topics can appear when the controller/gateway starts.
Retain the motion helper's own fresh-state and exclusive-publisher checks.

After the authorized test and native SHUTOFF observation, stop with:

```bash
source /workspaces/gravis_ws/evidence/cat323-env.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/gravis_ws/evidence/dds-observer-lan-only.xml
python3 /workspaces/gravis_ws/codex_skills/skills/dig-bag-recording/scripts/cat323_tracking_recording.py \
  stop --run-dir /workspaces/gravis_ws/evidence/ik_tracking_20260910/bag_tracking
```

`stop` keeps recording **eight additional seconds** to capture delayed/settling movement,
then signals only the saved supervisor whose PID, Linux process start tick, and command
line still match. The supervisor sends SIGINT to the process groups of its own selected
recorders and waits up to 25 seconds for finalization. It does not force-kill a stalled
recorder or touch another tmux window/process. Inspect `processes.json` on a finalization
timeout. Ctrl-C in the recorder pane also finalizes immediately, without the extra
settling interval. Stopping a recorder does not stop a machine command source.

A separate recording for each radial, vertical, and pitch attempt makes analysis simple;
one continuous recording is also supported. In either case save the authorized task
velocity, duration/travel bounds, controller parameter dump, exact deployed model/URDF
and helper result JSON alongside the bag. Record enough state before the pulse to define
a baseline. Keep the post-command observation even if immediate motion looks small.

## Recorded splits

| Split | Contents |
| --- | --- |
| `raw/state` | `/mole/measurements`, native `/machine_measurements`, `/machine_state`, `/machine_status`, `/joint_states`, both actuator-state aliases, native world cabin odometry, `/tf`, `/tf_static`, `/robot_description` |
| `raw/commands` | `/mole/actuator_commands_ugep`, native `/joint_commands`, LLC `/machine_lowlevel_controller/raw_commands_in` and `raw_commands_out` |
| `raw/telemetry` | Native joint/cylinder desired/measured velocity and LUT/PID terms for Boom, Dipper, EndeffectorPitch; `/mole/dig_ugep/*` observations, policy output, allocated joint command, depth shield/status; action feedback/status, lifecycle transitions, parameter events, ROS logs |
| `raw/elevation_map` | `/excavation_mapping/grid_map` and its upstream fusion event |
| `raw/camera` (`--camera`) | Native `/hal/perception/main/compressed_video` (`foxglove_msgs/msg/CompressedVideo`, H265) and `/hal/perception/main/camera_info` |

Native LLC scalar topics use `std_msgs/msg/Float32` and this exact pattern:
`/velocityCtrl/{term}/{joint}`, where joints are `Boom`, `Dipper`, `EndeffectorPitch`
and terms are `desiredJointVel`, `measuredJointVel`, `desiredCylinderVel`,
`measuredCylinderVel`, `lutCommand`, `pidCommand`, `pidProportional`, `pidIntegral`,
`pidDerivative`. `raw_commands_in/out` are `sensor_msgs/msg/Joy`; both actuator-state
aliases are `sensor_msgs/msg/JointState`.

The telemetry split also includes `/mole/dig_ugep/bucket_clearance`
(`mole_highlevel_msgs/msg/BucketClearance`) for automatic boom AIR/SOIL selection.
Compare its measurement/map stamps, validity and clearance with the native
boom interaction type in `/joint_commands`. The signal is available during
active UGEP control, not an independent manual pulse.
It also carries the vertical-extraction phase flag, target map height and
remaining lift. During that explicit phase verify AIR on all three arm slots;
outside it the geometry selector changes only boom. Policy inference pauses
during extraction, so absence of new policy-action samples in that phase is
expected; the native/internal executed command streams must continue.

For an isolated boom pulse with the UGEP controller inactive, the generic
scoop verifier reports missing UGEP commanded velocity and may report missing
desired/measured dipper and pitch velocities because their requests are zero.
Preserve that report. Validate direct-step completeness separately using native
and internal commands, state/status, all nine Boom LUT/PID/tracking channels,
selected camera/map streams, finalized bags and stopped owned recorders. Do
not label `raw_commands_out` as post-recovery valve/current evidence.

These native topics were present in the integration graph on 2026-09-10. Actual samples
are verified after recording. Controller telemetry is lifecycle-dependent;
`observations_raw` additionally needs its existing publish parameter enabled. Do not
change controller parameters just to satisfy an optional recording topic. There is no
separate allocator-internals topic currently identified; `commanded_joint_velocity`
provides the allocator output and the motion helper records the requested task velocity.
The manual override does not make the actor's published policy output the executed
command; use the override settings and actual command topics for tracking analysis.

The scalar Float32 telemetry has no source header. Use bag receive timestamps for it
and retain the stamped command/state streams; account for host clock alignment and
transport delay before calling a measured offset a physical response delay. Native
`raw_commands_out` demonstrates controller conversion before subsequent recovery/CAN
handling, not valve actuation. Measured joint response establishes movement.

## Keep large native map and sensor traffic out of these tests

The deployed mapper already subscribes to the native selected elevation layer. Record
its local `/excavation_mapping/grid_map`, retaining source stamps and target layers.
Do not add a native `/grid_map_postprocessed_interface/selected_layers` subscription for
tracking bags. Native map traffic has suffered fragmented UDP loss; each added native
reader can multiply large Orin-to-x86 transfers. The repository's
`recording_profile:=gravis_cat323_ugep` currently adds native map inputs automatically,
so this helper uses explicit topic lists instead.

Without `--camera`, this tracking profile omits cameras. Both modes omit LiDAR
and extra elevation maps. For learned-policy scoops or a user camera request,
select `--camera`; M445 JPEG/image topic names are not CAT323 camera topics. If the user
requests perception replay, extend the recording deliberately with the available
bandwidth in mind; this small profile is not a full sensor-reconstruction dataset.
All ordinary streams use BEST_EFFORT subscriptions, compatible with native and adapter
publishers. `/tf_static`, `/robot_description`, and the canonical map use RELIABLE,
TRANSIENT_LOCAL subscriptions to receive their latched state.

## Verify completeness

`stop` automatically verifies all selected finalized bags; verification can be repeated:

```bash
python3 /workspaces/gravis_ws/codex_skills/skills/dig-bag-recording/scripts/cat323_tracking_recording.py \
  verify --run-dir /workspaces/gravis_ws/evidence/ik_tracking_20260910/bag_tracking
```

`verification.json` lists every planned topic's recorded sample count, absent/empty
topics, split durations, and errors. A complete motion recording requires state/TF/map,
Mole and native commands, raw LLC output, UGEP allocated joint velocity, and native
joint desired/measured velocity samples. Each split must contain `metadata.yaml`, an
MCAP file, and pass `ros2 bag info`; no owned recorder may still be running. Optional
telemetry absence remains visible in the report. `--allow-no-commands` is only for a
recording deliberately made without a motion attempt. Do not use it to hide missing
command evidence from an attempted motion. Completeness verifies available evidence;
it does not prove tracking quality, sample rate, map freshness, or successful motion.

## Camera and durable setup

A wiped x86 image needs the camera message package before recording:

```bash
sudo apt-get update
sudo apt-get install ros-jazzy-foxglove-msgs
```

Omit `sudo` inside the root container. The helper fails before starting a camera
recording if the message type cannot be loaded. A discovered video topic or a
running native pipeline does not prove recorded frames. Verify camera message
counts and source stamps under the same DDS/recording load. The camera split is
managed by the same supervisor and finalized by the normal `stop` command.

H265 joining mid-GOP can record packets that are not independently decodable.
Before sending a goal, wait for complete VPS/SPS/PPS and an IDR to reach the bag:

```bash
python3 /workspaces/gravis_ws/codex_skills/skills/dig-bag-recording/scripts/check_cat323_camera.py \
  --bag-dir /path/to/run/raw/camera
```

This reads flushed MCAP data without another camera subscription. A nonzero exit
means the keyframe is not yet present; rerun after more data arrives. Also verify
fresh video continues. After an attempt, pass `--before-unix-s <goal-start>` to
prove the keyframe preceded it. This check does not certify lossless video; decode
representative footage and report missing-reference warnings separately. In the
2026-09-17 first attempt, video packets preceded the goal but the first complete
IDR followed its abort, so the attempt footage was not independently decodable.

Keep the exact controller and mapping parameter dumps, native image digest,
CAT323 runtime URDF/morphology/torque files, actor hash, target service request and
readback with each run. Save the action client's goal UUID, feedback, and **final
result**: feedback/status topics do not include the result-service response.
If rosbag reports missing action schemas, preserve installed `RunAction.action`/
`.idl`, `GoalStatusArray`, `GoalStatus`, `GoalInfo`, UUID and Time definitions.

Before native or x86 rebuilds, use the versioned CAT323
[map transport recovery procedure](../../gravis-cat323/references/map-transport-test.md).
Live topic inventories can change with Gravis updates: verify CAT323 joint names,
types, QoS and actual post-goal samples instead of copying M445 assumptions.
Idle policy/LLC topics can correctly have no samples before a goal.
