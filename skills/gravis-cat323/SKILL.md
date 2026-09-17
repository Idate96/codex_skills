---
name: gravis-cat323
description: Operate the Gravis CAT323 Moleworks high-level ROS workspace on integration x86. Use for CAT323 container and tmux setup, native Gravis state and elevation-map ingestion, Foxglove, hydraulic unlock, and bounded joint-command tests.
---

# Gravis CAT323

Run Moleworks code on `integration@10.27.0.13` (`rk-2609-507835-x86`). The native Gravis low-level controller, state estimator, TF, and elevation mapping already run on Orin A. The user established persistent `nvidia` SSH access with `ssh orin-a` from the x86 host and `gravis_ugep`; see the operations reference. Orin has proprietary code: keep diagnosis read-only unless the user authorizes a specific native change. Use the established account and keep Moleworks deployment on x86.

## Resolve the machine first

- Verify `hostname` and `id -un` over SSH, then inspect the existing container, tmux windows, and command publishers before starting software.
- Host workspace: `/home/integration/gravis_ws`; container: `gravis_ugep`; container workspace: `/workspaces/gravis_ws`. Tmux runs as root **inside** the container, session `moleworks_ros`; use `ssh -t integration@10.27.0.13 docker_attach` to enter it.
- Orin SSH uses a dedicated x86 key and boot-enabled host agent, independent of the laptop. Both `docker_attach` modes restore container SSH configuration and trusted host keys through the persistent catalog. Verify the agent service and actual Orin login when diagnosing access.
- Read [references/operations.md](references/operations.md) for shell entry, tmux, DDS profiles, map diagnostics, and the command-test path. Recheck current files and process state; addresses and deployment artifacts reflect the September 2026 setup.
- Read the active checkout's `docs/information/gravis_engineer_setup.md`, `gravis_interface.md`, and `gravis_cat323_ugep_deployment_plan.md` when setup or interface semantics matter. Source code and installed message definitions resolve drift.

## Bringup scope

Use the CAT323 launch `mole_bringup gravis_cat323_ugep.launch.py` and existing x86 wrappers. Keep `enable_command_output:=false`, `activate_controller:=false`, `run_action:=false`, and `record_bag:=false` for ordinary bringup. Configuring the lifecycle controller leaves it inactive; activation and motion require the requested test scope and live gates.

Versioned tmux/attach wrappers are in [scripts/](scripts/); deployment locations and their ROS-script dependencies are in the operations reference. They add missing windows and preserve existing work. The host wrapper rejects the legacy host tmux session to prevent competing stacks.

Machine skills persist as a full committed catalog at `/workspaces/gravis_ws/codex_skills/skills` on the workspace bind mount. Project discovery uses `/workspaces/gravis_ws/.agents/skills`; tmux startup also restores user discovery links through [scripts/install-cat323-skills.bash](scripts/install-cat323-skills.bash). Keep the full catalog to preserve cross-skill references, and keep the existing host skills checkout untouched.

Do not use the M4 startup wrapper, M4 low-level controller, Mole estimator, or another elevation mapper on this machine. The deployed excavation-mapping default is `gravis_selected_snapshot`: consume only the native postprocessed selected `elevation` layer, preserving filled cells without raw-map support, cloud gating, or an additional BASE-plane mask. Preserve source-stamp freshness, frame, resolution, and lattice checks. The older raw/cloud-supported mode is optional; inspect the active mode before diagnosing mapping or adding subscriptions.

For policy tests across the Orin-to-x86 link, restore and verify the small-packet,
paced selected-map profile before motion. Read
[references/map-transport-test.md](references/map-transport-test.md) for the
versioned patch generator/XML, disposable native-image rebuild, `gtask` startup,
and actual publisher-environment checks. A short fresh-map probe does not prove
continuous freshness; retain the configured source-age guards and check under
recording load. The mapping input gate remains 3 s; the operator-selected CAT323
controller/depth-shield map-age limit is 10 s. Joint-state expiry remains 0.15 s.
Reprovisioned hosts and updated AMG images must not rely on old `/tmp` files,
local image tags, unchecked Python bytecode, or an assumed installed patch.

## Machine actions

- Software bringup does not authorize hydraulic unlock or motion. Use the user's existing explicit authorization; do not ask again for the same established test scope.
- CAT323 status fields use numeric values; unsupported armrest/radio fields may be `-1`. Do not apply the Menzi M4 boolean prerequisites or interpret an unsupported field as unlocked.
- The CAT323 operator sets engine RPM manually. Read back native `measured_engine_rpm`; do not use the M4 RPM workflow or call the native `/engine_speed` service. A discovered service is not evidence that remote RPM control is appropriate for this machine.
- For motion require fresh `/machine_status` with `is_using_gravis_commands == 1` and `is_autonomous_operation_unlocked == 1`, operator readiness/E-stop control, and exclusive native/internal command ownership. A successful service response alone does not prove readiness.
- The verified native `/hydraulic_lock` interface is `std_srvs/srv/SetBool`: `data: true` releases the lock; `false` locks. Check the current interface/source before a user-authorized call, then verify fresh `is_hydraulilock_unlocked == 1` and `is_autonomous_operation_unlocked == 1`. See the reference for source evidence and contradictory free-text diagnostics.
- Use the repository's CAT323 bounded joint-step helper. Do not send the M4 five-joint zero-command latch sequence: CAT323 has different joints and command routing. Preserve the signed CAT323 boom direction; do not infer positive velocity means up.

## Adaptation

When a CAT323 task exposes a reusable difference, update this skill/reference and add a narrow routing note to the affected generic or M4 skill. Keep M4 behavior intact. Verify new operational commands against the integration checkout and record unresolved conditions explicitly rather than presenting a historical build or map receipt as current readiness.
