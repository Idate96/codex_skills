---
name: ros2-debugging
description: "Diagnose ROS 2 failures and validate fixes with focused tests, compact log reports, and bounded topic, TF, DDS, and process checks. Use for missing or stale data, ownership conflicts, failing ROS tests, and inefficient debug loops. Route bringup and recovery to their owning skills."
---

# ROS 2 Debugging

Answer one diagnostic question at a time. Keep complete evidence on disk and
return only what the next decision needs. Diagnosis is read-only by default;
this skill does not authorize motion, restarts, or unrelated changes.

## Orient Once

- Read applicable `AGENTS.md`. Identify the exact checkout, dirty state,
  workspace overlay, container/host and process owner. Refresh when changed,
  not before every probe. Preserve unrelated work and active runtime installs.
- Use the failing process's ROS environment and time source. Check relevant
  domain/discovery keys, never dump entire environments. An empty host graph
  says little about a container using another domain.
- For Moleworks, run ROS builds, tests and probes in the owning ROS container:
  `docker exec` from the host, directly if already inside. Do not assume host
  ROS or another workspace is equivalent. Use `ros-worktree` for isolation and
  `supercluster-compute` when that host is requested.
- Verify affected package prefixes before using a rebuilt overlay. Check actual
  symlink targets before editing installed config; copy-install flags do not
  prove old symlinks disappeared.

## Investigate One Failure

State expected behavior, the first observed failure, and evidence that would
distinguish the leading explanations. Select one to three relevant probes
from [references/probes.md](references/probes.md); do not run it as a checklist.

Prefer existing diagnostics, an exact endpoint, or a known log over whole-graph
inventories. Bound each probe. One empty listing or early TF warning is weak
evidence; check the owning environment and direct data before recommending a
restart. Distinguish receipt, source, simulation and wall time. Preserve
freshness and safety thresholds while investigating.

Read the first causal error and nearby context. A timeout is a symptom until
the stalled operation is identified. A stored parameter value does not prove
that code rereads it after startup.

## Validate With A Small Evidence Funnel

Before editing, choose the behavioral claim and smallest check that could
disprove it. For authorized fixes:

1. Run focused deterministic tests during iteration. Use ROS integration only
   for boundaries that lower-level tests cannot exercise.
2. Once stable, build changed packages and affected consumers in the intended
   workspace, then test them. Include dependencies when needed. Incremental
   overlay success is not a clean dependency-closure build.
3. Run repository-required gates once before handoff. Repeat only for a relevant
   change, failure or unresolved concern. Report unrelated baseline failures
   separately; do not waive required checks.
4. If runtime evidence is necessary, run one bounded case through the existing
   public owner. Separate correctness from matched timing comparisons. Newton
   success does not establish machine speed, physical safety or soil retention.

Do not expand into repeated full suites, scenario banks or parameter sweeps
without a question requiring them. Successful process exit alone does not
prove tests were discovered or the requested behavior was checked.

## Compact Command Reports

Use [scripts/run_logged.py](scripts/run_logged.py) for finite, already-authorized
commands on Linux. It requires only Python's standard library. Run it inside
the owning environment, after sourcing ROS, with a new directory per invocation:

```bash
python3 /path/to/ros2-debugging/scripts/run_logged.py \
  --out /tmp/task-checks/build-01 --timeout-sec 900 -- \
  colcon build --packages-select affected_package affected_consumer

python3 /path/to/ros2-debugging/scripts/run_logged.py \
  --out /tmp/task-checks/test-01 --timeout-sec 300 -- \
  colcon test --packages-select affected_package affected_consumer \
  --return-code-on-test-failure
```

Adapt packages, paths and time budgets. If unavailable remotely, copy only the
helper to a task-owned location; do not create another container for it.

The helper saves combined stdout/stderr to `output.log`, writes terminal
`result.json`, propagates failure and prints a compact JSON receipt. Failures
include only a bounded log tail. Existing output directories are refused.
`exit_zero` means process exit zero, not a test or safety certificate.

Use fresh CTest/JUnit/colcon reports for test counts. Scope
`colcon test-result --test-result-base build/affected_package --verbose` to
packages just tested; capture verbose output the same way. Check expected
test discovery and freshness. Do not count stale reports or describe suite
wrapper counts as distinct underlying cases.
Colcon may keep detailed output in its package logs even when the wrapper log
is short; use those existing files when a failure needs further inspection.

Read more only when the receipt is insufficient: locate an error with `rg -n`,
then inspect a bounded line range. Preserve full logs; do not feed large logs,
JSON, parameter dumps or entire source files into context.

The helper controls only its own finite command group. Do not wrap long-lived
stacks, daemonizing commands or commands containing credentials. Use the existing
tmux owner for runtime. No receipt means incomplete/unknown, not success. If the
wrapper was hard-killed, inspect its owned process before retrying.

## Monitor And Coordinate Economically

- Keep one session/job per long command. Wait in bounded intervals, normally
  30–60 seconds; avoid repeated second-by-second polling and unchanged tails.
  Keep user updates concise and evidence-based.
- Read existing runtime phase/status output and new log lines. Report transitions,
  failures, stalled progress and completion. Refresh static parameters, graph
  ownership or resources when symptoms or new allocations warrant it.
- Do not build a supervisor, event framework or parallel orchestration path.
  For Moleworks, read bags only after recorder finalization.
- When delegation is authorized and useful, assign distinct bounded questions
  with exact files and evidence paths. Prefer a focused independent review of
  a stable patch over several agents repeating the investigation. A short task
  note usually suffices instead of a full conversation.

## Handoff And Recovery

For work spanning sessions, maintain one short current-state note: checkout and
revision, effective config, tested claim/result, active job/phase, remaining
blocker, next step and evidence paths. Link historical investigations instead
of copying them. Do not put task history in global guidance.

Distinguish build success, tests passed, runtime progressing, scenario complete
and hardware verified. State missing evidence and failures. Route recovery to
`robot-ros`, `terra-pipeline` or the relevant startup skill; use
`newton-ros-parity` for simulator/ROS data, clock and TF consistency.
Existing authorization persists, but diagnosis alone does not authorize service
calls, parameter changes, lifecycle transitions or motion.
