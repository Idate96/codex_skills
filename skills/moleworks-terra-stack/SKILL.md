---
name: moleworks-terra-stack
description: Bring up and test the Moleworks Terra ROS2 + IsaacLab stack in the correct Docker containers and tmux windows; use when starting/restarting isaac-terra + ros-terra, setting CycloneDDS/ROS_DOMAIN_ID, and validating /mole/state, /clock, TF, and basic node health.
---

# Moleworks Terra Stack (IsaacLab + ROS2)

## Non-negotiables
- Always run IsaacLab scripts via `/workspace/isaaclab/isaaclab.sh -p`.
- Always open a **new tmux window** for IsaacLab or ROS commands; capture the pane output after each command.
- Always set the **same** `ROS_DOMAIN_ID` on both sides (example: `24`).
- Only force the DDS implementation if needed:
  - `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (optional; otherwise use the distro default)
  - `export CYCLONEDDS_URI=...` (optional; only if you need a custom CycloneDDS config)
- For TF/topics checks, use **long timeouts (10-15s)**.
- Most general ROS launch entry points live under `mole_bringup/launch` (e.g., `terra.launch.py`, `robot.launch.py`, `nav2.launch.py`).

## Bring up Isaac container (isaac-terra window)
To allow **agents in the ROS container** to restart Isaac, start Isaac as a **restartable service** (not `docker compose run --rm`).

1) Host:
```bash
tmux new-window -n isaac-terra
```
2) Host: start the dev container as a service:
```bash
cd ~/moleworks/moleworks_ext/docker
docker compose --env-file .env.moleworks_ext-dev \
  -f docker-compose.yaml -f docker-compose.override.yaml \
  up -d isaac-lab-ext-dev
```
3) (Optional) Tail logs:
```bash
docker logs -f isaac-lab-moleworks_ext-dev
```

## Bring up ROS container (ros-terra window)
If you need agents inside ROS to control Isaac, you MUST mount the host Docker socket into the ROS container.

1) Host:
```bash
tmux new-window -n ros-terra
```
2) Host: launch the ROS container with the docker socket:
```bash
cd ~/moleworks/ros2_ws/src/moleworks_ros/docker
./docker_launch.sh moleworks_ros:latest mole.Dockerfile --docker-sock
```

3) Inside ROS container (match DDS + domain):
```bash
export ROS_DOMAIN_ID=24
source /opt/ros/jazzy/setup.bash
source "$HOME/moleworks/ros2_ws/install/setup.bash"
```

4) Capture output:
```bash
tmux capture-pane -p -S -200
```

## Start sim from ROS (sim-terra window)
This runs IsaacLab Terra **inside the Isaac container**, triggered from the ROS container via `/var/run/docker.sock`.

1) Host:
```bash
tmux new-window -n sim-terra
```
2) Inside ROS container:
```bash
export ROS_DOMAIN_ID=24
mole_sim_ctl terra
```
3) Capture output:
```bash
tmux capture-pane -p -S -200
```

## Health checks (ros-check window)
Run these **inside the ROS container** (new `ros-check` window recommended):

```bash
tmux new-window -n ros-check
```

```bash
# 1) Topic exists + message arrives
ros2 topic list | rg /mole/state

# Sim publishes /mole/state; on Jazzy you typically need best-effort QoS:
timeout 60 ros2 topic echo /mole/state --once --qos-reliability best_effort

# 2) Sim time is publishing
timeout 60 ros2 topic echo /clock --once

# 3) TF sanity (use a real frame pair if known)
# Example only; adjust frames to your setup
# timeout 15 bash -c 'ros2 run tf2_ros tf2_echo map base_link 2>&1' | head -20
```

Always capture output:
```bash
tmux capture-pane -p -S -200
```

## Restart workflow
- Restart Isaac dev container **from inside ROS container**:
  - `mole_sim_ctl restart`
  - then re-run `mole_sim_ctl terra`
  - then re-run `mole_state`

## Troubleshooting quick hits
- **No /mole/state**: verify both sides have the same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and Isaac is still running.
- **CycloneDDS/serdata errors**: if you've been connected to a robot via FastDDS discovery server, try `unset FASTRTPS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER` on both sides.
- **"Failed to find a free participant index"**: set `CYCLONEDDS_URI` to a CycloneDDS config in each container (examples: Isaac: `$HOME/moleworks/moleworks_ext/.ros/cyclonedds.xml`, ROS: `$HOME/moleworks/ros2_ws/src/moleworks_ros/.ros/cyclonedds.xml`).
- **No messages but topic exists**: likely QoS mismatch; use `--qos-reliability best_effort` when echoing `/mole/state`.
- **TF lookup fails**: use long timeouts and confirm frames exist; TF can take a few seconds to appear.
