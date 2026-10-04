---
name: zed-camera-startup
description: Start, rebuild, inspect, or stop the Mole/Menzi ZED 2i manipulation sidecar and verify that its native colored PointCloud2 reaches the active moleworks_ros container through the host Fast DDS shared-memory path. Use for ZED RGB-depth or colored-cloud needs, not ordinary DIG video when the lightweight MainView gRPC camera is sufficient.
---

# ZED Camera Startup

Operate the versioned ZED 2i sidecar launcher and require an actual native RGB
cloud sample in the active Moleworks ROS container before reporting success.
The ZED cloud is already colored; do not launch LiDAR-camera backprojection just
to add color.

For ordinary DIG field video, use the MainView gRPC camera and the
`dig-bag-recording` workflow instead. It runs without another container and its
compressed topic is already the canonical recorder input. Choose this ZED
workflow only when the user needs registered depth, a native colored cloud, or
explicitly requests the ZED.

## Safety and ownership

- Run the launcher on the robot host, not inside an arbitrary ROS container.
- Do not start, stop, or restart LiDAR, the existing perception stack, robot
  controllers, or state estimation.
- Do not enable ZED positional tracking or its `map`/`odom` TF publishers.
- Keep stable ZED topic names while allowing the launcher to assign a unique
  internal composable-container identity on every start. This avoids stale
  discovery-server load-service endpoints after rapid restarts.
- Do not hand-write a replacement `docker run` command or patch ROS middleware
  packages in a running container.
- Operate only the managed container named `mole_zed2i_manipulation`. The
  launcher refuses to remove a container it does not own and retains its model
  cache volumes on stop.
- Treat discovery-only evidence as insufficient. A listed publisher or topic
  does not prove that PointCloud2 samples cross the container boundary.

## Canonical files

Use these repository-owned files:

- `/home/lorenzo/ros2_ws/src/moleworks_ros/docker/launch_zed2i_manipulation.sh`
- `/home/lorenzo/ros2_ws/src/moleworks_ros/docker/zed2i_aligned.Dockerfile`
- `/home/lorenzo/ros2_ws/src/moleworks_ros/perception/mole_perception_bringup/config/zed2i_manipulation.yaml`

Read the root and `docker/AGENTS.md` instructions before changing these files.
Do not duplicate the launcher into this skill.

## Workflow

1. Confirm that the active `mole_bringup robot.launch.py` owner and the ZED 2i
   hardware exist. Preserve unrelated containers and dirty repository changes.
2. Run the launcher's `status` action first when the managed sidecar exists.
3. Run `build-image` when the aligned image is missing or the launcher reports
   any Fast DDS package mismatch. This pins the full Fast DDS/RMW/ROSIDL package
   set to the active Moleworks container.
4. Run `start`. On the first start, allow up to ten minutes for neural-depth
   optimization; later starts reuse the named-volume cache.
5. Require the built-in health gate to receive a real message from
   `/zed/zed_node/point_cloud/cloud_registered` with an `rgb` field inside the
   Moleworks container.
6. Report the exact running container, topic, frame, point/rate/latency evidence,
   and whether a first-time optimization occurred.

Canonical commands:

```bash
cd /home/lorenzo/ros2_ws/src/moleworks_ros/docker
./launch_zed2i_manipulation.sh status
./launch_zed2i_manipulation.sh build-image
./launch_zed2i_manipulation.sh start
```

The launcher requires exactly one active `mole_bringup robot.launch.py` owner
and supports no alternate container, camera serial, middleware, or timeout
modes.

## Acceptance contract

Do not call the startup healthy unless all of these hold:

- the sidecar uses host network, host IPC, and host PID namespaces;
- camera serial `30868463` opens successfully;
- the active profile is HD720, compact `NEURAL_LIGHT`, target 15 Hz;
- positional tracking is false and ZED odometry/pose publication is false;
- the Moleworks container receives a PointCloud2 sample in
  `zed_left_camera_frame` with fields `x`, `y`, `z`, and `rgb`;
- the sample contains nonzero, varying RGB values;
- Fast DDS endpoints use the same host identity and map the shared
  `/dev/shm/fastrtps_port*` arena.

For a stronger performance check, subscribe from the Moleworks container with
`ROS2CLI_NO_DAEMON=1` and the live robot process's exact
`FASTRTPS_DEFAULT_PROFILES_FILE`. Measure at least 20 samples; report message
rate, point count, header age, and RGB diversity. Never rely on a ROS CLI daemon
whose discovery profile differs from the runtime client profile.

## Recovery

- If the image is absent or middleware differs, run `build-image`; do not use
  `apt` inside the live sidecar.
- If first-start optimization is in progress, wait and monitor logs. Do not
  repeatedly recreate the container because that discards progress unless the
  cache volume is retained.
- If `status` discovers a publisher but receives no sample, compare the
  runtime DDS profile inherited from the robot launch and confirm `--ipc host`.
- If an old unowned ZED container holds the USB device, stop that exact
  container but leave it intact as rollback unless the user asks to remove it.
- To stop the managed sidecar, run:

```bash
cd /home/lorenzo/ros2_ws/src/moleworks_ros/docker
./launch_zed2i_manipulation.sh stop
```

The stop action removes only the managed ephemeral container and keeps the ZED
calibration/model cache volumes.
