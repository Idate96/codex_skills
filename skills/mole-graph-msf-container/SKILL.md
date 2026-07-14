---
name: mole-graph-msf-container
description: "Start or attach to the Mole Graph-MSF container in tmux, optionally launching `mole_estimator`. Use for estimator-container bringup, attachment, health checks, and shutdown."
---

# Mole Graph-MSF Container

Graph-MSF estimator work now uses the standard `moleworks_ros` container; do not rely on the retired `moleworks_ros_graph_msf` alias or `moleworks_ros:graph_msf` image.

## Start Or Attach

```bash
/home/lorenzo/codex_skills/skills/mole-graph-msf-container/scripts/start_graph_msf_container.sh [session] [window]
tmux attach -t <session>
```

The helper uses the preferred `moleworks_ros` shell function when no container exists and opens `docker exec -it moleworks_ros bash` when it is already running.

## Launch Estimator Inside The Container

Source the built workspace and use the current package default unless the user requested a named profile. Verify any explicit config exists before launch; `mole_estimator_robot.yaml` is not a current profile.

```bash
WS="$HOME/ros2_ws"
[[ -f "$WS/install/setup.bash" ]] || WS="$HOME/moleworks/ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
ros2 launch mole_estimator mole_estimator.launch.py
```

For an explicit profile, select a current YAML from `<workspace>/src/moleworks_ros/mole_estimator/config/` and pass it through the launch file's current config argument.

## Health And Cleanup

Use bounded checks:

```bash
ros2 topic list | rg 'graph_msf|/mole/state'
timeout 10 ros2 topic echo --once /mole/state
timeout 10 ros2 topic echo --once --qos-reliability best_effort /tf
```

Stop only the estimator process, tmux window/session, or `moleworks_ros` container started by this workflow. Do not stop an unrelated shared container.
