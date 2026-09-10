---
name: simple-research-code
description: Write, simplify, or review greenfield research code for maximum experiment iteration speed. Use when production hardening, API stability, legacy compatibility, broad configuration, and exhaustive testing are out of scope; prefer one direct supported path, loud failures, narrow APIs, and a small high-value test set. For single-operator ROS research launch refactors, assume one public launch starts and owns the complete application stack, including its sensor acquisition and perception, while only an exactly enumerated low-level robot prerequisite pre-exists; avoid attach/reuse modes and duplicate graph guards.
---

# Simple Research Code

Optimize for minimum time from idea to trustworthy result. Implement the shortest credible path that answers the current research question. Treat code structure as disposable, but keep experimental conclusions trustworthy.

Unless the task says otherwise, assume greenfield work with no legacy consumers. Do not optimize for production operations, hypothetical reuse, or future modes.

## Keep One Direct Path

- Cut scope before polishing code. Remove work that does not help answer the current question.
- Support one intended workflow. Do not build fallback paths or general frameworks for imagined use cases.
- Change APIs, schemas, and data formats directly. Update known call sites and delete the old path instead of adding migrations, deprecations, shims, or dual implementations.
- Prefer the least structure needed to make the code obvious. Do not minimize line count at the cost of dense or clever code.
- Keep logic local and direct. Inline one-off logic when a helper would only add indirection.
- Use early returns to flatten control flow.
- Minimize state, arguments, configuration, and object lifetime. Pass concrete values instead of config objects when only one mode exists.
- Make required arguments required. Do not add optional parameters, override plumbing, plugin points, or speculative extension hooks.
- Hardcode experiment-specific choices when they are fixed for the current work. Name non-obvious values and keep them near their use.
- Explain only non-obvious mathematics, assumptions, units, and coordinate conventions.

## Fail Loudly

- Handle tagged variants exhaustively and fail on unknown cases.
- Validate data and assumptions at the boundary, then rely on the validated shape internally.
- Raise a clear error for invalid inputs, missing data, unsupported modes, and violated runtime requirements.
- Use `assert` for internal developer invariants. Use explicit exceptions for runtime inputs and environment checks because Python can disable assertions.
- Do not silently substitute defaults, skip bad data, return partial results, retry, catch broad exceptions, or continue in a degraded mode unless the task explicitly requires that behavior.
- Do not add defensive code for states already ruled out by types or boundary validation.

Fail hard means stop and expose the problem. Do not interpret this rule as permission to weaken safety requirements imposed by the task or environment.

## Match Checks to the Operating Model

- State the real operating assumptions before adding ownership or startup machinery: single user or shared system, one full-stack owner or attachment to an existing stack, and which competing processes can actually exist.
- For every preflight check, name the concrete failure it prevents and the boundary where that failure matters. Remove checks justified only by hypothetical production actors outside the stated research setup.
- Prefer one cheap, deterministic mechanism per realistic failure. For example, a domain lease is enough to reject an accidental second launch in a single-user full-stack workflow; do not also add a discovery guard unless independent processes can bypass the lease in practice.
- Separate cheap startup checks from consumer-boundary checks. Validate config shape, required paths, and identities before startup; validate exact bytes, provenance, and live state once in the process that consumes them immediately before an irreversible mutation.
- Do not hash, parse, or copy the same large artifact in multiple layers to claim stronger preflight. If a consumer operates on captured immutable bytes, do not reread the mutable source unless its later state itself affects correctness.
- Treat fixed discovery windows, readiness monitors, graph inventories, and attestation layers as costs that require evidence. Keep a hardware interlock when it protects a realistic hazardous action; remove a production-style guard when the operating model rules its threat out.

### Single-Operator ROS Full-Stack Default

Use this default when one researcher invokes one supported public launch and that launch starts the full application stack.

Here, **full application stack** means every application component is launched together: enabled sensor acquisition, perception, mapping, planning, coordination, control/execution, and application workers. Nothing application-owned is expected to be pre-launched, discovered, reused, or attached to.

Do not use broad labels such as "hardware drivers" to define the external boundary. Enumerate the pre-existing layer exactly. A sensor driver or acquisition process belongs to the application launch when the application needs it and the named prerequisite does not provide it. For the current Terra operating model, only low-level actuator/vehicle control, safety interlocks, and state estimation are external; Terra owns its LiDAR/camera acquisition when enabled, perception, mapping, planning, coordination, controller/executor, and workers.

For Terra on the robot, define one canonical prerequisite procedure with the effective tool substituted consistently:

```bash
ros2 launch mole_low_level_bringup bringup.launch.py \
  use_sim_time:=false on_machine:=true \
  activate_trajectory_controller:=false endeffector_type:=<effective-tool> \
  robot_namespace:=<robot-namespace> tf_prefix:=<tf-prefix>
ros2 launch mole_estimator mole_estimator.launch.py \
  use_sim_time:=false urdf_xacro_endeffector_type:=<effective-tool> \
  robot_namespace:=<robot-namespace> tf_prefix:=<tf-prefix>
```

Use the same namespace, TF prefix, and effective tool for both prerequisites and the Terra launch. The readiness contract must cover the low-level control/safety endpoints, estimator TF from `map` to `<tf-prefix>/BASE`, and the matching effective tool frame before target mutation or machine autostart. Do not start a separate perception window for this workflow; the Terra public launch owns it.

**Invariant:** the one supported public launch starts every application-owned component on every run. “Full stack” never means starting only an orchestrator or coordinator that discovers, attaches to, or conditionally reuses application components launched elsewhere.

- Treat the public launch entry point as the sole application owner. Treat the external low-level layer as a dependency, not a competing stack.
- Record one canonical procedure that starts the external prerequisite and one short readiness contract for it. Do not invent several partial prerequisite procedures or leave operators to infer the ownership boundary from child-launch defaults.
- Audit every installed and documented entrypoint, not only the preferred launch file. A sole-owner contract is false while another supported launcher can start the same application graph. Delete superseded full-stack owners and attach-style launchers instead of merely hiding them, adding another lock, or leaving them installed for compatibility.
- When the application includes a generic robot/base launch, explicitly disable actuator/vehicle control, safety, or estimation that the operating model says already exists. Keep application-owned acquisition and perception enabled there. Do not inherit a child launch default that silently starts duplicate external infrastructure or disables part of the promised application stack.
- Launch required application components unconditionally. Do not add attach/reuse modes or `enable_*` launch arguments for components that are always part of the supported run.
- Keep topology in launch code and experiment values in configuration. Do not expose arguments that let one invocation reshape the owned stack into partially overlapping variants.
- If accidental double invocation is realistic, use one local lease or lock around the public entry point.
- Do not also inventory the DDS graph, wait through a discovery window, or prove that application nodes and topics are absent. Those checks duplicate ownership enforcement, slow every startup, and can race discovery.
- Add graph-conflict detection only after the operating model changes to support independently launched application components or a known actor that can bypass the owner lock.
- Keep startup checks deterministic and cheap: parse the selected config and verify required paths or static resources. Put live-state and exact-artifact checks in the component that performs the hazardous or irreversible action, immediately before it acts.

Do not generalize this default to a shared robot, multi-operator deployment, or independently composed graph. In those operating models, identify the actual competing actor first and add the smallest check that detects that concrete conflict.

## Test the Critical Surface

Write and run the smallest test set that makes the result trustworthy:

- Cover the main intended path.
- Cover mathematical, data-shape, unit, or coordinate assumptions that could invalidate the result.
- Prefer deterministic tests at the lowest practical layer over end-to-end smoke tests.
- Add an automated integration check only when one specific, stable invariant cannot be tested below the asynchronous runtime.
- Add a regression test when a bug was costly or is plausible to reintroduce.

Keep tests fast, deterministic, and targeted. Do not pursue coverage targets, compatibility matrices, duplicated test layers, exhaustive edge cases, or mocks for trivial glue. During iteration, run the narrowest relevant tests; run broader suites only when required by the repository or justified by integration risk.

Use a time-boxed evidence funnel:

- Keep focused unit and contract checks in the seconds-scale whenever practical.
- For a vectorized GPU simulator, test hundreds of environments together (default to roughly 512, adjusted within 128-1024 for memory) with the shortest useful horizon and a 2-5 minute hard timeout.
- Do not use a serial one-environment rollout as the default functional smoke; reserve it for a concrete trace, rendering, or numerical-debug question.
- Write detailed per-environment data to an artifact and print only compact aggregates.
- Stop after the first stage that answers the current research decision. Run large banks, repeated seeds, statistical intervals, and long horizons only when the claim or promotion gate requires them.
- Launch one bounded run and let it finish or time out; do not spend the iteration loop repeatedly polling an unchanged process.

## Do Not Build Smoke Frameworks

- Do not turn asynchronous infrastructure, process startup and cleanup, discovery, retries, and timing into a custom orchestration state machine merely to claim that a system works.
- Test precise behavioral claims at the smallest deterministic layer that can express them. Use offline replay only when recorded, ordered inputs are a natural fit for the problem.
- Treat a full-system run as a sanity or integration check: use the existing entry point, observe one useful external result, capture the normal output if helpful, then stop it normally.
- Do not infer exact internal behavior or deterministic timing from a wall-clock run through a distributed or asynchronous system.
- Do not add process supervisors, graph/GID monitors, provenance schemas, event journals, custom retry loops, or cleanup escalation to make a smoke test appear authoritative.
- When an existing smoke harness becomes flaky or larger than the path it checks, delete or reduce it. Do not harden the harness by default.

The rare automated smoke check should invoke the existing entry point, assert one externally visible invariant, and exit. It must not introduce new runtime modes, hidden state, fallback behavior, or a second implementation of orchestration.

### ROS-Specific Guidance

- Put exact controller or planner parity claims below ROS whenever the behavior can be expressed deterministically.
- Use direct component or lockstep tests for pure logic. Use offline traces when recorded, ordered boundary inputs are needed to reproduce stateful behavior.
- Use the normal launch and controller entry points for a ROS sanity run. Check that the graph starts and expected output appears; capture the ordinary log or bag only when it helps diagnosis.
- Do not make asynchronous ROS execution appear deterministic by adding graph/GID monitors, publisher-provenance schemas, lifecycle supervisors, retry machinery, or custom process cleanup.
- A ROS sanity run complements deterministic tests; it is not evidence of exact callback ordering, numerical equality, or timing parity.

## Use Simplicity to Experiment Faster

- Make the code easy to understand, change, run, and discard.
- Prefer a straightforward implementation that can be modified in minutes over a generalized or highly optimized one.
- Add complexity only when the current experiment cannot proceed without it.
- Keep the edit-run-understand loop short.

## While Reviewing

- Prioritize bugs and assumptions that could corrupt the experimental conclusion.
- Flag state spaces, APIs, configuration, and abstractions wider than the current problem.
- Flag compatibility layers, fallbacks, retries, defaults, and catch blocks that hide failures.
- Flag helpers or indirection that make the main path harder to follow.
- Flag tests whose maintenance or runtime cost exceeds the risk they protect.
- Do not recommend production hardening when it is outside the stated scope.
- Prefer one obvious simplification over incremental patchwork or a long wishlist.
