---
name: ros2-debugging
description: "Diagnose ROS 2 topics, nodes, services, actions, parameters, TF, DDS domains, and tmux-managed processes with bounded read-only checks. Use for missing graphs, publisher conflicts, transform failures, stale data, and ownership diagnosis before any restart."
---

# ROS 2 Debugging

Keep this skill portable and read-only by default. Route project-specific
bringup or recovery to its owning skill/runbook.

## Establish The Owning Environment

1. Identify the process/tmux session and sourced workspace that own the graph.
2. Check `ROS_DOMAIN_ID` and relevant discovery variables in the same shell as
   the failing process.
3. Run `ros2 <verb> -h` before relying on distribution-sensitive flags.
4. Bound every graph/data probe with `timeout`; do not wait indefinitely.
5. Treat one empty graph listing as weak evidence. Confirm the domain,
   discovery path, target endpoint, process logs, and direct data before a
   restart.

A supported project container may configure DDS automatically, while a robot
installation may use a host workspace. Do not declare a working owning shell
stale merely because it is not inside a particular image, and do not paste a
large DDS override in front of every command.

## Read-Only Graph And Data Checks

```bash
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
timeout 10 ros2 node list
timeout 10 ros2 node info /your_node

timeout 10 ros2 topic list
timeout 10 ros2 topic info /your_topic --verbose
timeout 10 ros2 topic echo /your_topic --once
timeout 10 ros2 topic hz /your_topic

timeout 10 ros2 service list
timeout 10 ros2 service type /your_service
timeout 10 ros2 action list -t

timeout 10 ros2 param list /your_node
timeout 10 ros2 param get /your_node parameter_name
```

`topic hz` commonly ends with timeout status 124 after producing useful
samples. Record the observed samples/rate rather than treating that bounded
exit alone as a failure.

Before calling a service, publishing, sending an action, setting a parameter,
or changing lifecycle state, establish the endpoint type and obtain the
operation-specific authorization. Those are not read-only debugging probes.

## TF Checks

Give `tf2_echo` at least 15-20 seconds to populate its buffer. Early “frame
does not exist” output can be transient:

```bash
timeout 20 bash -lc 'ros2 run tf2_ros tf2_echo map BASE 2>&1' | head -40
```

On normal ROS 2 stacks, TF transport remains global `/tf` and `/tf_static`;
putting a node in `/mole` does not move TF to `/mole/tf`. Join/remap a private
TF transport only when endpoint inspection proves that the stack explicitly
uses one.

Robot-local frame names may still carry a `tf_prefix`. Construct the effective
frame from the configured prefix and query that frame; keep global frames such
as `map` unprefixed. For an ad-hoc `TransformListener`, use the graph's actual
TF transport and time source, then allow the listener buffer to warm up.

## Publisher And Domain Conflicts

- Use `ros2 topic info <topic> --verbose` to enumerate endpoint node names,
  namespaces, GIDs, and QoS before assigning ownership.
- Compare the observed publisher count with the owning launch contract.
- Check `ROS_DOMAIN_ID` in every relevant process environment; use one named
  tmux session per domain when possible.
- Do not conclude that a topic vanished until a direct bounded read, process
  state, domain, and discovery path agree.
- Do not start a replacement component beside an unhealthy complete owner.

## Inspect tmux Without Mutating It

```bash
tmux list-sessions
tmux list-windows -a -F '#{session_name}:#{window_index} #{window_name} #{window_active}'
tmux list-panes -a -F '#{session_name}:#{window_name}.#{pane_index} #{pane_pid} #{pane_current_command} #{pane_dead}'
tmux capture-pane -p -S -200 -t session:window
```

Correlate pane commands/logs with ROS endpoint ownership. Creating, killing,
respawning, or sending keys to panes is a separate operational action.

## Moleworks Handoff

For a Moleworks robot, first use the maintained monitors under
`mole_utils/scripts/`, then the current
`docs/robot_agent/TROUBLESHOOTING_GUIDE.md`. Use `robot-ros` to select the
narrow recovery skill. Use `newton-ros-parity` when the failure is specifically
Newton-to-`moleworks_ros` clock/TF/topic parity.
