# Selective ROS probes

Choose only the probes needed for the current question, in the owning shell.
Check subcommand `--help` when a distro-sensitive flag is uncertain; do not
repeat help calls already verified in this environment. Save large output to a
task log and return relevant lines.

## Missing endpoint or data

```bash
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-unset}"
timeout 10 ros2 node info /expected_node
timeout 10 ros2 topic info /expected_topic --verbose
timeout 10 ros2 topic echo /expected_topic expected_msgs/msg/ExpectedType --once
```

Use the actual type from the interface or `ros2 topic type`. An explicit type
helps separate startup discovery from an unknown CLI message type. Check
ownership, QoS, source stamps and actual arrival. Use graph-wide lists only
when the endpoint is unknown. For large messages, select a known small field
if the installed CLI supports it, or save output to disk.

`timeout 10 ros2 topic hz /expected_topic` estimates subscriber-side rate, not
source-clock timing or transport latency. Streaming probes commonly exit 124
after useful samples. Report both the bounded timeout and observed evidence;
no samples remains inconclusive.

## TF

Query configured effective frames, including robot-local prefixes and the actual
tool name; do not assume `EE`. Keep global `map` unprefixed. Allow about 15–20
seconds for discovery and buffer warm-up:

```bash
timeout 20 ros2 run tf2_ros tf2_echo map configured_tool_frame
```

Capture with the helper or a task log. Do not pipe a live probe through `head`:
it can end observation before warm-up and hide the original exit status.
Initial “frame does not exist” messages do not establish persistent failure;
inspect later transforms, stamps and extrapolation errors.

TF normally uses global `/tf` and `/tf_static`; node namespaces alone do not
relocate transport. Use private/remapped TF only when the launch and endpoints
establish it. An ad-hoc listener needs that transport and the correct clock.

## Service, action, parameter or lifecycle

```bash
timeout 10 ros2 service type /expected_service
timeout 10 ros2 action info /expected_action
timeout 10 ros2 param get /expected_node parameter_name
timeout 10 ros2 lifecycle get /expected_node
```

These inspect state. Service calls, goals, publishing, parameter changes and
lifecycle transitions are separate operations requiring authorization within
the task. Stored parameters may differ from startup-cached runtime values;
lifecycle labels alone do not prove command ownership or freshness.

## Process ownership and domain conflict

```bash
tmux list-panes -a -F '#{session_name}:#{window_index}.#{pane_index} #{pane_pid} #{pane_current_command} #{pane_dead}'
tmux capture-pane -p -S -60 -t known_session:known_window
ps -p OWNED_PID -o pid,ppid,etime,pcpu,stat,args
```

Correlate the process, log and endpoints. Read only relevant domain/discovery
environment keys, never the full environment. Compare publisher GIDs/names and
counts with the owning launch contract. Do not kill processes, send tmux keys
or start replacement components as a probe.

## Build and test reports

Follow repository package selection and container instructions. For a configured
CMake test directory, inspect selection once with `ctest -N -R PATTERN`, then
use `ctest --output-on-failure -R PATTERN`. Where supported, `--no-tests=error`
rejects an unexpectedly empty selection; use it for an expected suite, not a
package intentionally lacking tests. Capture output with `run_logged.py`.

For colcon, use `--return-code-on-test-failure` and inspect fresh reports for
the tested packages. Keep test-run and result-check exits separate. An old
report is not evidence the new command ran those tests. Distinguish source
failures, dependency/environment failures, assertions and timeouts. Preserve
the first useful error before starting another build.
