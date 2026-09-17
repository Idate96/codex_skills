# CAT323 integration operations

These paths were verified on the integration x86 on 2026-09-10. Re-read their contents when resuming; versioned tmux/attach wrappers are bundled with this skill, while ROS launch/environment helpers remain deployment artifacts.

## Container and workspace

`integration@10.27.0.13` is the x86 execution host. Its Tailscale address is `100.98.45.46`. Orin A is `10.27.0.10` / `100.93.196.125`; `10.27.0.10:41641` is a Tailscale transport endpoint, not SSH or DDS. The established Orin SSH account is `nvidia`; use `ssh orin-a` on x86 or inside the container for user-authorized diagnosis.

Inspect the existing `gravis_ugep` container before creating anything. The workspace's `src/moleworks_ros` used branch `gravis` and `deps/gravis.vcs`. Import the manifest into the dedicated workspace, preserving existing checkouts/dirt and checking exact revisions. Reuse the pulled `rslheap/moleworks_ros:latest` image after inspecting its digest and container configuration. The checkout provides `docker/gravis_cat323_container.sh`; read its options if creating a replacement is actually needed. It runs a CPU-only shell, with live ROS requiring host networking, and mounts only the dedicated workspace at `/workspaces/gravis_ws`.

Build evidence and source inventory are under `/home/integration/gravis_ws/evidence/`. They establish what was built then; use current source status and focused build checks for later edits. Do not replace the active installation while a controller is running.

## Native Gravis startup with gtask

When the operator authorizes starting the native stack, use `gtask` on **Orin A**,
not the integration x86. Normal startup is `gtask amg:run`; its installed owner is
`/etc/gravisrobotics/amg/Taskfile.yml`. Check `gtask --summary amg:run`, Docker
state, the active machine configuration, and the image of the previous deployment
before starting. Keep the foreground task in a persistent Orin tmux window.
`amg:run` starts its native dependencies and stops that Compose stack when it exits.

The package default can differ from the deployed image. On 2026-09-17,
`deploy.env` defaulted to `v2.3.5`, whereas the existing deployment used
`gravisrobotics/amg:debug-v3.1.0-rc1`. The authorized startup preserved the latter:

```bash
gtask amg:run AMG_BASE_IMAGE=gravisrobotics/amg:debug-v3.1.0-rc1
```

This is historical image evidence, not a permanent version pin. Verify the current
image on each startup. The active rack was `rk-2609-507835`, machine `armeno`.
The 2026-09-17 session is `cat323-native:amg` on Orin, with its log at
`/tmp/cat323-native-20260917.log`. Startup does not authorize hydraulic unlock or
controller activation. Recheck native status, command ownership and advancing
canonical maps afterward. The initial 20-second check passed without the override, but the subsequent
scoop aborted when a missed snapshot pushed source age to 3.039 s. Use the
[versioned paced-map recovery path](map-transport-test.md) before policy tests;
that short check was insufficient evidence of continuous freshness.

If the user requests a mapping scan before starting the controller, launch only
the state adapter (command gateway disabled) and the CAT323 excavation mapping
child in the integration `bringup` window, plus Foxglove. The generic CAT323 full
wrapper also starts the UGEP process even with activation disabled; do not use it
for a literal controller-not-started pause. After a checkout update, rebuild and
verify package prefixes; remove superseded historical worktree overlays from the
active environment after preserving the old environment file.

## Persistent Orin SSH

On 2026-09-10 the user requested permanent container SSH access independent of the laptop. The dedicated Ed25519 private key stays on x86 at `/home/integration/.ssh/cat323_orin_a` with mode 0600. Its fingerprint is `SHA256:rJcZhGwDJoHklsSchk5WjldMWHAcYlMC7sqXcNmn45M`. The public key was appended to `/home/nvidia/.ssh/authorized_keys` on Orin with `restrict,pty,from="10.27.0.13"`; existing keys were preserved. This permits interactive login from the x86 LAN address and disables SSH forwarding for that key. It does not authorize native service changes or motion.

The enabled system service `cat323-orin-ssh-agent.service` loads only this key at boot and exposes `/home/integration/gravis_ws/.ssh-agent/orin.sock` (0600, parent 0700). The container sees the same socket through its existing workspace mount. The service source is [scripts/cat323-orin-ssh-agent.service](../scripts/cat323-orin-ssh-agent.service). No private key is stored in the workspace or container. The earlier laptop tunnel was stopped; neither laptop connectivity nor `ssh -A` is needed for Orin authentication.

The host includes `~/.ssh/cat323_orin_a.conf` before its existing Gravis SSH configuration; that alias uses the dedicated key directly. Container root's configuration includes `/root/.ssh/cat323_orin_a.conf`, which selects the persistent agent and `/root/.ssh/cat323_orin_a_known_hosts`. Public configuration and trusted Orin host keys persist under the workspace's `.ssh-agent/` directory. [scripts/install-cat323-ssh.bash](../scripts/install-cat323-ssh.bash) restores them on both ordinary tmux entry and `docker_attach --shell`, preserving unrelated SSH entries. A recreated container needs the same workspace bind mount and this installer; it needs no copied home directory or private key.

From x86, check `systemctl is-enabled cat323-orin-ssh-agent.service` and `systemctl is-active cat323-orin-ssh-agent.service`, then verify `docker exec gravis_ugep ssh -o BatchMode=yes orin-a 'hostname; id -un'`. A scoped `sudo systemctl restart cat323-orin-ssh-agent.service` reloads the key if recovery is needed. Recheck service and login evidence rather than assuming a socket file proves a working agent. Operational notes and setup backups are at `/home/integration/gravis_ws/.ssh-agent/README.md` and `permanent-20260910/`.

Verification passed for host and running-container login with laptop forwarding disabled, agent service restart, and a fresh temporary container using only the workspace mount. Repeated installation preserved unrelated SSH entries and produced identical configuration. The live container and tmux pane PIDs were unchanged. Boot enablement was checked; the live x86 host was not rebooted for validation.

## GitHub access

The user-authorized GitHub identity is available through a separate persistent
host agent, `cat323-github-ssh-agent.service`, exposed in the container at
`/workspaces/gravis_ws/.ssh-agent/github.sock`. The existing SSH installer restores
the GitHub stanza from the shared public configuration during `docker_attach`.
Private key material stays on the host. Verify with `ssh -T git@github.com`;
the successful `Idate96` greeting deliberately returns status 1. Normal Git
fetch/push uses SSH remotes. This identity does not authorize unrelated publishing.

Root may need an exact `safe.directory` entry for the bind-mounted ROS checkout:
`git config --global --add safe.directory /workspaces/gravis_ws/src/moleworks_ros`.
Do not use a wildcard. Fetch and compare `origin/gravis` before publishing local
changes, preserve hydrated LFS assets, and retain the active overlay worktrees.
Git synchronization does not deploy a new controller or authorize a stack restart.

## Tmux and operator shell

From the laptop:

```bash
ssh -t integration@10.27.0.13 docker_attach
```

For an independent ROS shell:

```bash
ssh -t integration@10.27.0.13 docker_attach --shell
```

`docker_attach` is `/usr/local/bin/docker_attach` on the x86 host. With no arguments it runs `/home/integration/gravis_ws/start-cat323-tmux.sh`, then attaches through `docker exec -it gravis_ugep tmux attach-session -t moleworks_ros`. `--shell` opens an independent sourced shell in the running container; `--help` prints usage.

Tmux runs as container root on its default socket `/tmp/tmux-0/default`, session `moleworks_ros`. The legacy host session `gravis_ugep` was retired; do not recreate it. To inspect the correct server from the host, use `docker exec gravis_ugep tmux list-windows -t moleworks_ros` or equivalent `docker exec` commands, not host `tmux`.

The host startup wrapper starts the existing container if stopped, then runs `/workspaces/gravis_ws/start-cat323-tmux-inside.bash`. The inner wrapper uses `/workspaces/gravis_ws/tmux-cat323.conf` and adds missing managed windows while preserving existing windows/processes. Neither wrapper proves existing windows are healthy or restarts failed launches. Inspect panes and node ownership before any restart. The host wrapper refuses startup if the legacy host session still exists, preventing a duplicate stack.

Versioned sources and deployed locations:

| Skill source | Deployment |
| --- | --- |
| [scripts/docker-attach-cat323.sh](../scripts/docker-attach-cat323.sh) | Host `/usr/local/bin/docker_attach` |
| [scripts/start-cat323-tmux.sh](../scripts/start-cat323-tmux.sh) | Host `/home/integration/gravis_ws/start-cat323-tmux.sh` |
| [scripts/start-cat323-tmux-inside.bash](../scripts/start-cat323-tmux-inside.bash) | Workspace root, visible inside container as `/workspaces/gravis_ws/start-cat323-tmux-inside.bash` |
| [scripts/tmux-cat323.conf](../scripts/tmux-cat323.conf) | Workspace root, visible inside container as `/workspaces/gravis_ws/tmux-cat323.conf` |
| [scripts/install-cat323-skills.bash](../scripts/install-cat323-skills.bash) | Persistent catalog at `/workspaces/gravis_ws/codex_skills/skills/gravis-cat323/scripts/install-cat323-skills.bash` |
| [scripts/install-cat323-ssh.bash](../scripts/install-cat323-ssh.bash) | Persistent catalog at `/workspaces/gravis_ws/codex_skills/skills/gravis-cat323/scripts/install-cat323-ssh.bash` |
| [scripts/cat323-orin-ssh-agent.service](../scripts/cat323-orin-ssh-agent.service) | Host `/etc/systemd/system/cat323-orin-ssh-agent.service` |

Deploy these together after comparing current copies; the shell wrappers must be executable. They depend on the persistent skills catalog, `.ssh-agent/ssh_config` and `.ssh-agent/known_hosts`, and the workspace's `operator-shell.bash`, `run-cat323-bringup.bash`, `evidence/watch-machine-status.bash`, `evidence/foxglove-bridge.sh`, and `evidence/cat323-env.bash`. Verify these dependencies and the installed ROS workspace before invoking startup; the bundled wrappers alone do not build or configure ROS.

| Window | Purpose |
| --- | --- |
| `0:bringup` | CAT323 launch, commands disabled |
| `1:commands` | Sourced ROS operator shell |
| `2:machine-status` | Continuous native status |
| `3:checks` | Independent sourced ROS shell |
| `4:foxglove` | Visualization bridge |

Other windows may be preserved. Read `docker exec gravis_ugep tmux show-options -g prefix` on the host: the container configuration uses `C-a`, so `Ctrl-A` then a number selects a window and `Ctrl-A d` detaches. Do not infer the container's prefix or socket from the laptop or host tmux configuration.

The operator shell defines:

```bash
cat323-status --once
cat323-state --once
cat323-publishers
cat323-controller
```

These read `/machine_status`, `/mole/measurements`, publishers on `/joint_commands` and `/mole/actuator_commands_ugep`, and lifecycle state of `/mole/dig_ugep_controller`. Use bounded topic echoes for agent checks; the continuous status window is intentional.

Bringup is `/home/integration/gravis_ws/run-cat323-bringup.bash`, with current output at `evidence/bringup-current.log`. Its defaults are AIR (`interaction_type:=0`), controller activation disabled, no command output/action, and no recorder. Explicit configuration is needed to reach lifecycle INACTIVE. Check the running launch and lifecycle state instead of inferring state from static shell help.

## Persistent machine skills

Keep the **full committed skill catalog** in host `/home/integration/gravis_ws/codex_skills/skills`, visible in the container as `/workspaces/gravis_ws/codex_skills/skills`. This isolated catalog began as an export and is now maintained as the `gravis` checkout of `Idate96/codex_skills`; record the deployed SHA after synchronization. Do not replace or switch the separate existing host checkout at `/home/integration/git/codex_skills`. The catalog had 60 skills at initial deployment, including all relative cross-skill references. A subset risks missing dependencies.

The workspace's `.agents/skills` symlink targets `../codex_skills/skills` relative to `.agents`. Both the link and catalog live on the existing workspace bind mount, so project discovery survives container recreation without adding a mount or preserving the container's `/root` directory. Start Codex from `/workspaces/gravis_ws` or a descendant to use that project discovery path.

Before tmux operations, `start-cat323-tmux-inside.bash` runs the catalog's `scripts/install-cat323-skills.bash`. The installer links each skill into both `/root/.codex/skills` and `/root/.agents/skills` for the container's root user. It safely repeats, preserves real directories and unrelated links, and retargets only this deployment's older `/workspaces/gravis_ws/handoff/codex-skills/<name>` links. It leaves `.system` and other agent state intact. Retained collisions are printed; inspect them rather than claiming every catalog skill replaced an existing version. A fresh container can also run the installer directly without starting ROS.

The installer also creates compatibility links for `/home/lorenzo/codex_skills` and `/home/lorenzo/.codex/skills` to the mounted catalog, only when those paths are absent. Existing paths are preserved. These aliases support older absolute skill-helper references; they do not supply workstation-specific tools, credentials, plugins, or data.

The repository's generic `install.sh --codex` does not install `.agents/skills` and may retarget unrelated same-name links, so use the CAT323 installer here. Some catalog workflows remain M4-specific; select the CAT323 route for this machine and adapt other workflows only when requested. Restart Codex after a catalog update so it rescans the skills. Initial isolated-container verification, with no copied home and only the workspace mounted read-only, discovered all 60 skills enabled with no missing entries or discovery errors.

## DDS and native state

Source `/workspaces/gravis_ws/evidence/cat323-env.bash` inside the container. It sources Jazzy and the workspace, sets domain `0`, selects discovery server `10.27.0.10:11811`, and uses the repo `docker/listen_remote` daemon helper.

The working runtime and observer profiles are `evidence/dds-runtime-lan-only.xml` and `evidence/dds-observer-lan-only.xml`. Both explicitly select the integration LAN interface `10.27.0.13` and 16 MiB receive buffers. The runtime is a discovery CLIENT; graph observation uses the observer profile. A generic `ros2-listen 100.93.196.125` setup alone does not reproduce these receiver settings. Inspect the actual helper/environment rather than guessing its syntax or mixing discovery variables from other machines.

On the x86, temporary receiver tuning used `net.core.rmem_max=16777216` and `net.ipv4.ipfrag_high_thresh=134217728`. Verify current values and evidence before reapplying tuning. Do not turn it into persistent host/network configuration without task scope. Avoid switching the laptop's working connection.

Verify native `/machine_status`, `/machine_measurements`, `/joint_states`, TF, and adapter `/mole/measurements` with fresh messages. Small messages and topic discovery are separate evidence from successful large GridMap delivery.

After a native AMG stack restart, existing DDS clients may retain stale discovery state. Recheck fresh data in the actual running adapter, not only a newly started CLI subscriber; restart affected integration clients within the current software scope after stopping any active command owner. Never treat a healthy fresh probe as proof that an older controller connection recovered. This includes the persistent ROS CLI daemon: when direct fresh probes work but CLI discovery says `Node not found`, stop the stale daemon with `ros2 daemon stop`, then recreate it through `mole_dds_ensure_daemon "$MOLE_DDS_OBSERVER_PROFILE"` in the sourced CAT323 environment. Refresh on evidence of stale discovery, not automatically on every attach.

## Selected-map fragmentation and temporary pacing

For policy startup and recovery after native updates, use [CAT323 selected-map transport and recovery](map-transport-test.md). The maintained helper offers `prepare`, `status`, `check-maps` and guarded `restore`. The local test bundle adds an optional Compose overlay without editing permanent Orin files; it targets AMG's actual Python launch block. The details below are historical evidence from the initial transport diagnosis and temporary replacement; do not assume that supervisor still exists.

On 2026-09-10, packet headers showed the selected-map writer advancing at 1 Hz while x86 received incomplete maps. Approximately 65 kB UDP datagrams required IP fragmentation across the Orin MTU of 1466 and x86 MTU of 1500. Paired captures confirmed fragments present in Orin's outgoing capture but absent at x86. The 10 Gb/s sender to 1 Gb/s receiver transition suggested burst loss; it did not identify the exact dropping component. Discovery and fresh small messages did not establish complete map delivery.

After the user authorized a temporary native change, only the selected-layer republisher was replaced. The working process-specific profile uses `fastdds.max_message_size=1400`, asynchronous FIFO flow control at **8192 bytes per 5 ms**, and `RMW_FASTRTPS_USE_QOS_FROM_XML=1`. It retains incoming transport capacity, native BEST_EFFORT QoS, original executable/arguments, and the local discovery server. The outgoing message property avoids shrinking the receive transport needed for the native large input map. Both reader and writer use `PREALLOCATED_WITH_REALLOC` memory policy with XML QoS enabled.

The original permanent DDS profile was not edited. Original AMG launch, native elevation mapping and estimator processes remained running. A 60-second live check delivered 59 advancing canonical snapshots, zero stale publications, maximum published source age 1.3624 s, and zero IP fragmentation in 10,672 map datagrams. This is measured transport evidence, not a guarantee under all future loads or proof of new LiDAR integration.

The September temporary supervisor/profile/state are inside Orin `amg` at `/tmp/cat323-map-transport.8rmB3J`. Read current `replacement-state.json` and the workspace handoff before acting; recorded PIDs are historical identifiers. The supervisor retains the original publisher environment/arguments in memory and restores it on its restore marker or replacement failure. Keep its files while active. The replacement is outside original ROS-launch ownership and the override does not survive AMG/container restart.

Results, exact profile, scripts and evidence are at `/workspaces/gravis_ws/evidence/map_transport_20260910/temporary_fix/RESULTS.md`. The prepared rollback script in that directory requests only restoration of the original selected-map publisher; use `bash rollback.bash` from its directory when rollback is in scope, then verify supervisor phase `restored_original`, publisher identity and fresh accepted maps. Do not restart the complete native stack to roll back this transport test, or apply this recorded temporary profile to other native processes without inspecting their requirements.

Use canonical excavation-map receipt/source stamps and passive packet capture to check the existing consumer path. Additional native map subscribers can duplicate network traffic and change the result. Do not leave diagnostic publishers/subscribers running during control, or disable timestamp gates to conceal map loss. Orin inspection remains read-only unless a specific native change is authorized by the user.

Fast DDS 2.14 references: [outgoing message-size property](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/property_policies/non_consolidated_qos.html), [asynchronous flow control](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/use_cases/large_data/large_data.html), and [ROS 2 XML QoS](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/ros2/ros2_configure.html).

## Elevation map ingestion

Nominal native map publication is about **1 Hz**. The deployed default is `gravis_selected_snapshot`, consuming only `/grid_map_postprocessed_interface/selected_layers`. Confirm types, offered QoS, full message receipt, source stamps, and accepted mapper snapshots under the actual subscriber load.

The active config is `perception/mole_excavation_mapping/config/excavation_mapping_gravis_cat323.yaml`. It copies the native postprocessed `elevation` layer onto the fixed 0.1 m excavation-map lattice and authors local targets; it does not estimate terrain again. Native filled cells remain filled. The selected mode requires neither raw-map support, variance, nor filtered clouds and applies no extra BASE-plane mask. Each snapshot clears uncovered and nonfinite current terrain to unknown while preserving frozen targets and zone semantics.

Selected snapshots retain an advancing nonzero source stamp, frame `map`, identity pose rotation, zero pose height, matching resolution, aligned cell centers, and the configured source-age gate (currently 0–3 seconds). Periodic publication and local target edits preserve the last accepted source stamp. Gravis can restamp a map without integrating new sensor data; published-map freshness is not proof of new LiDAR integration. Freeze target geometry against TF at the accepted terrain source stamp as before.

The selected-mode deployment passed all 20 mapping test targets, including focused copy/NaN/target-preservation checks. Production logs reported `selected=54492`, `copied=54492`, `unsupported=0`, and `BASE-plane=0`; the operator confirmed that the artificial holes were fixed. An attempted simultaneous cell-parity probe received zero native maps and 24 canonical maps, so it did **not** establish a full live cell-by-cell comparison. Keep those verification limits distinct when describing the result.

The optional older `gravis_authoritative_snapshot` mode retains its strict contract: selected/raw stamps must match exactly, finite raw support is required, and that stamp must appear on a nonempty cloud from **any one** of the three filtered-cloud inputs. It also applies the BASE-plane exclusion. Raw input `/elevation_map/selected_layers` and cloud subscriptions belong to that mode, not the deployed selected-only default. Its explicit cloud-gate bypass remains confined to command-disabled simulated-time replay. Shared geometry/freshness parameters still use the `gravis_authoritative_snapshot.*` parameter namespace in both modes.

Earlier `read-only-02` pair-matching failures and sparse accepted pairs describe the old strict mode; do not treat them as the current mapper contract or current failure. For repeat diagnostics inspect current mode/configuration, mapper logs, and complete source/output messages first. Historical `dds-offline-map-gate.py` analysis exercises the older pairing contract. Run probes sequentially when simultaneous large subscriptions alter the load being measured. Report complete receipt, mapper acceptance, sustained rate, and cell equality separately; do not weaken freshness or validity checks to declare terrain usable for excavation.

## Foxglove on the laptop

The x86 helper `evidence/foxglove-bridge.sh` binds `127.0.0.1:8765`, uses the observer DDS profile, and exposes assets without command/service capabilities. It includes native and adapter state plus mapping topics. Seven CAT323 `mesh_library` assets were staged under `/workspaces/gravis_ws/visualization-assets`; check actual asset responses if the model is missing.

After checking whether a forward already owns local port 8765:

```bash
ssh -N -o ExitOnForwardFailure=yes -L 8765:127.0.0.1:8765 integration@10.27.0.13
```

Connect local Foxglove to `ws://localhost:8765`. Verify fresh state, TF, maps, and mesh loading; an open WebSocket alone is not visualization success. Keep the bridge bound to loopback and preserve its read-only capability selection.

## Hydraulic unlock

The native interface is `/hydraulic_lock`, `std_srvs/srv/SetBool`. Verified source evidence is in the existing Gravis checkout on integration, outside the Moleworks workspace:

- `/home/integration/git/core/machine/dev_ws/src/joint_actuator_model/README.md`, lines 59–68: service semantics.
- `/home/integration/git/core/machine/dev_ws/src/tools/machine_gui/src/machine_gui/engine_control.py`, lines 146–155: the unlock button sets request data true.

Recheck these sources and the installed interface before applying this to another version; this requires no Orin access.

Once the operator and user have explicitly authorized hydraulic release, from the sourced operator shell:

```bash
timeout 10 ros2 service call /hydraulic_lock std_srvs/srv/SetBool '{data: true}'
timeout 10 ros2 topic echo /machine_status machine_msgs/msg/MachineStatus --qos-reliability best_effort --once
```

Require a successful service result and fresh `is_hydraulilock_unlocked == 1` plus `is_autonomous_operation_unlocked == 1`; command execution also requires `is_using_gravis_commands == 1`. `data: false` locks. Do not repeatedly call services to work around a failed interlock.

The explicit unlock flags can be `1` while a free-text `machine_specific_fields` entry still says `Is machine locked: True`. That text comes from an unvalidated key-info CAN bit (`0x300004A`, byte 5, bit 1); the driver comments that it could not be tested without requesting key info, and it is not the command gate. Report the discrepancy without asserting it explains missing movement; the explicit native flags are the adapter contract. Armrest/radio fields were unsupported (`-1`) and must not be checked as M4 booleans.

## UGEP manual task-velocity tests and local target reference

The general runtime-profile API supports `reference_elevation_mode=neighborhood_mean`
with an independent reference disk: `reference_center_x_m`, `reference_center_y_m`
and positive `reference_radius_m` in the authoring frame. A caller can locate the
bucket using native TF and CAT323 FK, then author a full polar target relative to
nearby terrain without including distant hills in the reference mean. Terrain,
reference center and target are frozen at application; they do not follow the
bucket afterward. See the mapping package README/service for frame and one-shot
semantics. A 1 m neighborhood and a full 12 m, -1 m target were verified on
2026-09-10; these values are an operator choice, not universal defaults.

For Cartesian IK checks, use UGEP's configuration-time `manual_task_override`
in the three-axis `rotating_base` task frame. It keeps the production allocator
and three-joint CAT323 gateway. See the controller's `docs/ugep_controller.md` for
parameter semantics. The default pulse is 0.2 s; the validated maximum is 4 s,
preceded by 4 s zero task settling. Speeds are at most 0.1 m/s linear, 0.1 rad/s
pitch and 0.1 rad/s per commanded joint. Default tip/joint travel guards remain
0.05 m / 0.05 rad. Explicitly authorized longer tests can set
`manual_task_override.max_tip_travel_m` up to 0.6 m and
`manual_task_override.max_joint_travel_rad` up to 0.3 rad. Expiry or a guard
produces SHUTOFF with a safety-abort result; never mistake it for completed digging.
One pulse requires one configuration; there is no fallback to actor inference.

`manual_task_override.allow_stale_map` defaults false and is accepted only for
Gravis manual override. Its use requires the established operator-cleared in-air
scope. It relaxes terrain age expiry, retaining source/receipt timestamp integrity,
finite terrain queries, fresh measurements/TF and the depth shield. It cannot be
used for learned-policy configuration. Restore it false and restore default travel
bounds after the tests. Do not disable the depth shield to fix a target above the
bucket; correct the target reference first.

September 2026 tracking helpers are under
`/workspaces/gravis_ws/evidence/ik_tracking_20260910`. Inspect their current cases
and preflight before reuse. They record requested settings, state, native/internal
commands and native desired/measured velocities; the
[CAT323 recording reference](../../dig-bag-recording/references/cat323-tracking.md)
adds split rosbags without another native-map subscription. Recheck actual state,
readiness, ownership and operator scope; files do not authorize another test.
Use separate cases with settling observation. The explicit restore helper is
`configure_tracking_test.py --case restore`; older helpers do not reset the newer
travel parameters.

A short outward sample produced operator-visible motion but poor tracking:
about 1 s to 1 cm outward, about 5 cm outward plus 5 cm unintended upward, and
1.2 degrees pitch change. The measured boom barely responded while dipper/pitch
moved. This establishes physical response, not validated Cartesian tracking.
Measure joint response and the delayed stop interval before increasing speed or
transferring to learned control. The live map also showed a local elevation jump
near the bucket while distant terrain remained stable; self inclusion was suspected,
not established. If map clearance and operator observation disagree, resolve that
specific discrepancy before a larger motion. Never silently treat fresh publication
or historical clearance as proof of a clear current path.

## Native PID/LUT calibration

Gravis's native controller in Orin `amg` owns cylinder conversion, LUTs and PID.
The x86 CAT323 gateway currently accepts joint velocity or SHUTOFF only. Do not
start the M445 PID controller or send M445 current commands for this machine.
Read `/machine_lowlevel_controller`'s `gains_config_file` parameter to locate the
native configuration; a directory named `x86` in that path does not identify the
process host. The September file `boom_lowlevel_controllers.xml` contains AIR/SOIL
directional gains and steady-state/maximum LUTs. Arm values are cylinder m/s and
normalized raw output [-1, 1], not amperes. Preserve joint/cylinder signs.

The CAT323 gateway profile now supports automatic **boom-only** AIR selection
from `mole_highlevel_msgs/BucketClearance` on `/mole/dig_ugep/bucket_clearance`.
The maintained contract and thresholds are in
`low_level/mole_gravis_cat323_adapter/README.md` in `moleworks_ros`; verify the
installed profile/binary after reprovisioning. Whole padded bucket clearance,
source-stamp freshness, dynamic TF validity, dwell and hysteresis govern the
switch; dipper/pitch retain SOIL during the policy phase. Native bank changes reset both PID integrators,
and AIR/SOIL share the LUT. A fixed `interaction_type:=0` diagnostic bypasses
automatic selection. This feature selects existing banks; it is not calibrated
hardware acceptance or a LUT/gain retune. Record both clearance and native
`joint_commands` to verify actual bank selection.

The CAT323 launch defaults to the final V41 specialist,
`ugep_v41_cat323spec_s214_5999` (actor SHA256
`f1483fdcd7b554c2729991035f0c06737ddd1919146a540aff9de79121d02b10`).
The September 17 Multi-Ref comparison explicitly passes
`policy_manifest_path:=<package-share>/models/ugep_policies/manifests/ugep_v41_multiref_20260911_s214_5999_gravis_cat323.yaml`.
Its loaded policy ID is `ugep_v41_multiref_20260911_s214_5999`, actor SHA256
`b56fb648c68e9c530cc22162ceb09ef5f8fd17833368bf0950a829105038018c`.
Use `ros2 pkg prefix --share mole_highlevel_controller_cpp` for the package share.
Verify the loaded ID/hash. Both deployment manifests retain Armeno morphology,
kinematics, torque limits and nominal volume; do not substitute the similarly
named M445 manifest. Configure inactive before authorizing a scoop.

For this comparison, `gravis_pullup_distance_m:=3.43` and
`fill_aware_pullup.enabled: false` keep the traditional fixed distance.
The optional experimental shift has a workspace-wide +12.0 m ceiling, configured at 0.30 m/s,
capped against current bucket reach minus 0.3 m, and reset each goal. It is
not enabled in these trials. Check `controller_status` fields
`pullup_distance_m` and `policy_pullup_distance_m` separately. Source and
timing details belong to the controller's `docs/ugep_controller.md`.

UGEP's separate-file `VerticalExtractionController` takes over only 0.5 m
inside the requested pull-up distance (2.93 m for a 3.43 m request). The
policy owns normal curl and lift; neither fullness nor curl triggers handover.
It finishes curl while lifting and, if inside, moves outward beyond the requested
pull-up distance. Normal volume scaling stays unchanged. The lift uses the arm
plane; modest cabin inclination is not grounds for rejecting it. Remaining lift
comes from measured bucket/terrain clearance, not a fixed extra motion. The
CAT323 profile enables it. Success also requires restoring the requested reach.
The explicit `vertical_extraction_active` request selects native AIR on **boom,
dipper and pitch**, even while emerging from soil, independently of the normal
boom-only geometry selector. All ordinary command guards remain. The controller
adds measured-progress, maximum-lift and timeout bounds; see the owning
`high_level_controllers/mole_highlevel_controller_cpp/docs/ugep_controller.md`.
The former 0.15 m horizontal / 0.25 rad integrated-curl tracking abort was
removed after specialist scoop08 coasted 0.151 m inward in 0.14 s following
premature curl-triggered handover. Bounded feedback corrects those errors;
actual collision, joint, terrain and sensor-validity checks still apply.
Scoop09 (specialist) and scoop10 (Multi-Ref) were operator-observed good scoops,
but both action results were aborts after native `J_EE_ROLL` validity was lost
and the adapter stopped publishing measurements. Preserve that distinction
in comparisons. Full measured completion remains unvalidated. Do not relax
the 0.15 s TF/state limits to hide native sensor faults.
Do not start Gravis's separate native `PullUp` action in parallel with UGEP.

CAT323 RPM is set manually by the operator. Read native `measured_engine_rpm`
immediately before the goal (these trials used about 1650 RPM). Do not invoke
M4 RPM services or native `/engine_speed` automatically.

The September 17 SOIL baseline at roughly 900 RPM delivered a -0.02 rad/s,
2 s boom-up command but established no meaningful lift. Native recovery was
rate-limited for the first ~0.98 s. In the inspected deployed velocity controller,
initial-deadzone logic resets PID state each tick until measured joint speed
exceeds 0.005 rad/s; the recorded integral matched one tick, not accumulation.
Do not infer that longer duration or higher Ki alone fixes breakaway. Compare
at a held operating RPM after native recovery reaches Operational, with the
same authorized motion bounds and recorded response. Inspect the deployed
implementation again after a native image update.

For native tracking, use [the CAT323 bag recorder](../../dig-bag-recording/references/cat323-tracking.md).
Compare desired/measured joint and cylinder velocities, LUT and P/I/D terms, and
downstream recovery/output. `raw_commands_out` precedes recovery/CAN. Scalar
telemetry has no source header, so bag receipt timing is not actuator timing.
The unchanged `mole_sysid.analysis.bag_reader.read_joint_series_split` works on
the recorded CAT323 bags with explicit topics and joint names. Its standard
step metrics need constant joint steps; varying IK commands do not supply a
valid plateau. M445 PID/LUT hardware runners, gain writes and LUT export require
CAT323 adaptation. Inspect native interpolation/blending and units before
exporting a candidate; never relabel normalized output as current in amperes.

Evidence from the September three-case in-air run is in
`evidence/ik_tracking_20260910/CALIBRATION_REUSE.md` and `llc-analysis/`.
Recorded proportional terms match the AIR XML gains, including pitch positive
cylinder P=0.00364. Pitch overspeed and direction-dependent boom underspeed
justify native commissioning measurements, not a blind gain change. Read-only
config inspection does not authorize new native writes or calibration motions;
apply the user's established scope to each action.

## Bounded in-air command test

Read `mole_bringup/scripts/gravis_cat323_joint_step.py` and the active adapter/gateway config first. Existing deployment helpers are `evidence/air_readiness.py`, `configure_air_controller.py`, and `air-boom-up-once.bash`. Inspect before running: the composite helper starts a new CAT323 launch, and the regular bringup must not remain as a competing owner. Coordinate a scoped stop/restart and restore the ordinary command-disabled setup afterward.

`/joint_commands` is the native command **input**, with type `machine_msgs/msg/MachineActuatorCommands`; the native LLC subscriber was verified live. Receiving this topic proves forwarding to the interface, not physical actuation. The LLC `raw_commands_out` debug output proves conversion before recovery/CAN handling; it is not evidence that a valve actuated. Native recovery can zero commands in Locked/Recovering and rate-limit them at 0.3 normalized units/s over 3.33 seconds. Inspect the active recovery path and verify measured joint response separately; do not bypass it to obtain movement.

The original operator-approved tiny boom-up test used `J_BOOM`, **-0.02 rad/s for 0.10 s**, AIR mode. Short pulses can produce valid native velocity-mode `3` frames without an immediate physical response. After AMG restart, boom motion was operator-observed and bag-confirmed at about **0.215 degrees**, with delayed response; the helper's immediate position delta alone would miss that movement. Evaluate recorded state through the stop/settling interval and use the operator observation alongside it. Historical short-pulse nonmovement does not establish a current actuation failure. Test bounds are not standing authorization for another run: use fresh native status and publisher checks immediately before each authorized attempt, and do not retry motion automatically.

After the user separately requested a few seconds at 50 or 100 Hz, a scoped extension was created as `evidence/air_boom_up_2s.py`: fixed `J_BOOM`, -0.02 rad/s, **50 Hz**, at most 2 seconds. It checks live boom travel `-0.05 < delta < 0.005` rad and stays at least 0.01 rad inside the configured URDF joint limits. It retains adapter-state and native-readback freshness checks, inactive-controller ownership, and a verified native SHUTOFF tail. Inspect the helper and its result artifacts before reuse; its existence does not establish execution or movement. Authorization for this extension belongs to that session, not future runs, and it does not permit automatically increasing speed, travel, or duration.

The readiness helper requires two distinct fresh status stamps, `is_using_gravis_commands == 1`, and `is_autonomous_operation_unlocked == 1`, then rejects existing native/internal publishers and controller/gateway/test owners. Unsupported armrest/radio values of `-1` cannot substitute for live autonomous readiness. Do not publish commands to cure a lock or copy the M4 latch-recovery sequence.

Verify the helper's stop/zero behavior and scope before execution. A successful requested step needs actual state/command evidence and an end-state check; a prevented step is reported as no motion sent. Map receipt alone is insufficient authorization or readiness for soil interaction.

When wrapping a launch in a noninteractive Bash background job, preserve signal handling: the integration test used `env --default-signal=INT --default-signal=TERM ros2 launch ... &` so `kill -INT` reached ROS shutdown cleanly. Verify child exit and publisher disappearance before restoring bringup; stopping only the wrapper is insufficient. A short helper startup timeout previously missed otherwise healthy state, so wait for fresh adapter samples before invoking the existing bounded helper rather than weakening its freshness gate.

### Resume after the September 17 field session

The ROS branch's `docs/information/gravis_cat323_field_handoff.md` records the
final policy, configuration, recording paths, operator notes and unresolved
faults. The last controller used original V41 Sobol, explicitly selected through
`ugep_v41_sobol_20260911_s214_5999_gravis_cat323.yaml`; do not confuse it with the
specialist launch default. All five CAT deployment manifests now contain the
corrected workspace descriptor from side commit `53de959b`. Validate the loaded
47-value morphology descriptor against the installed `gravis_cat323/runtime_inputs.yaml`;
old recordings before corrected Multi-Ref attempt29 used the previous descriptor.

The session ended with recording finalized, controller inactive, native gateway
stopped and hydraulic lock verified. Recheck live state on resumption. Use a
clean environment when launching tmux commands: inherited worktree paths can
silently select old installed packages. Source the intended Jazzy/workspace
setup and DDS profile explicitly.
