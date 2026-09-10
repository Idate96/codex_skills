# Optional CAT323 map transport test

The preferred minimal upstream change is the optional XML profile plus the native Python launch hook. Once both are installed in the native image, use Gravis's existing `gtask amg:run -- env CAT323_SELECTED_MAP_DDS_PROFILE=/amg/install/share/autonomy_visualization/config/selected-map-fastdds.xml ros2 launch amg amg.launch.py` command override for a planned test. Normal Gravis startup keeps the opt-in unset. This needs no Compose pass-through or custom wrapper. The commit guide is `/workspaces/gravis_ws/handoff/CAT323_DDS_COMMIT_GUIDE.md`. The bundle workflow below is for the current image before that native change is integrated; its supporting files are excluded from the minimal native commit.

Normal Gravis operation uses the existing AMG deployment. The test is an explicit Compose overlay for the same native image and configuration; it applies the measured DDS settings only to the postprocessed selected-layer republisher. The native estimator, elevation mapping, low-level controller, topic names, elevation layer and map resolution are retained.

The bundle is prepared on integration x86. No permanent Orin configuration is edited. Starting the test is a separate operation, after an operator-controlled stop of AMG through the normal Gravis procedure. The helpers do not stop a running AMG, unlock hydraulics, set engine RPM or command motion.

## Prepare and inspect from gravis_ugep

New operator shells provide:

```bash
cat323-map-transport status
cat323-map-transport check-maps --seconds 30
cat323-map-transport prepare
```

In an existing shell, source `/workspaces/gravis_ws/operator-shell.bash` to load the helper, or call `python3 /workspaces/gravis_ws/codex_skills/skills/gravis-cat323/scripts/cat323_map_transport.py` with those arguments. `prepare` makes only scoped SSH reads and writes the local `/workspaces/gravis_ws/amg-map-transport-test` bundle. Repeating it preserves identical files and refuses to overwrite different content; after an AMG update, use `prepare --output <new-directory>` and review the new bundle.

The bundle contains the original and patched native Python launch block, a small diff, the tested XML profile, a manifest, the Compose overlay and launch/check helpers. It records the current image ID, native Compose files and bind paths. It does not copy private keys, secret contents or complete process environments.

Current AMG calls `get_gridmap_visualization_launch_actions` from `/amg/install/lib/python3.12/site-packages/autonomy_visualization/launch.py`. The older file under `share/autonomy_visualization/launch` is not the startup path used by this AMG entry point. The overlay must mount the Python launch block used by `amg.blocks`.

## Use the test at the next controlled startup

Copy the bundle to a fresh temporary directory on Orin. From integration x86 or gravis_ugep:

```bash
cat323_test_bundle=$(ssh orin-a 'mktemp -d /tmp/cat323-amg-test.XXXXXX')
scp -r /workspaces/gravis_ws/amg-map-transport-test/. "orin-a:${cat323_test_bundle}/"
ssh orin-a "python3 '${cat323_test_bundle}/compose_amg.py' check"
```

The copy example uses the container workspace path; the x86 host path is `/home/integration/gravis_ws/amg-map-transport-test`. Keep the directory while its test container is running. Do not copy over a bundle mounted by an active test.

`check` is safe while AMG is running. It resolves the captured native Compose file list using the original image and the same group/secret-path inputs as the installed Gravis Taskfile. It verifies native bind paths and the image ID, then requires the only configuration differences to be the test command and two read-only mounts. Secret contents and resolved container environments are not printed.

After AMG has been stopped through the established Gravis procedure, start the test **on the Orin host**, using the path returned above:

```bash
python3 /tmp/cat323-amg-test.XXXXXX/compose_amg.py start-test
```

Replace the example directory with the actual returned path. This starts only the AMG service, detached, with the existing image and native Compose settings. It does not build or pull an image or start other services. Native dependencies must already be available as in the normal Gravis setup. A running AMG container or Compose generation causes refusal. Keep Gravis's existing startup manager from independently starting another AMG during the test.

The normal entrypoint executes the bundle's `launch_amg.py`. It verifies the mounted block, original DDS profile and tested XML hashes, rejects an existing AMG/republisher process, sets `CAT323_SELECTED_MAP_DDS_PROFILE`, and executes the normal `ros2 launch amg amg.launch.py`. AMG then owns all its publisher processes from startup; no replacement supervisor is needed.

The native block responds to that opt-in variable by setting only the selected publisher's `FASTRTPS_DEFAULT_PROFILES_FILE` and `RMW_FASTRTPS_USE_QOS_FROM_XML=1`. With the opt-in unset, the patched block uses its original DDS environment. The ordinary native image and permanent Compose/DDS files remain untouched.

For normal operation afterward, stop the test through the usual Gravis procedure and return to the normal managed startup or `gtask amg:up` with the desired native image, without the test overlay. The normal Compose configuration recreates AMG without the two test mounts. The test uses Compose `up`, so a stopped test container may still occupy the name `amg`; the normal `amg:up` path handles recreation. Do not start a conflicting `amg:run --name amg` alongside that container.

## Verify map delivery

From the sourced gravis_ugep operator shell, run `cat323-map-transport status` and `cat323-map-transport check-maps --seconds 30`. The latter subscribes only to the local canonical excavation map. It requires advancing source timestamps, at least 0.8 fresh updates per second, no invalid/three-second-old publications or regressing stamps, and no gap over 2.5 seconds between advancing snapshots. A failure exits nonzero and prints the measured result; it does not relax controller checks.

The tested profile is byte-for-byte the successful temporary profile: outgoing message cap 1,400 bytes, asynchronous FIFO flow control at 8,192 bytes per 5 ms, and retained large incoming transport capacity. The September live test produced 59 fresh snapshots in 60 seconds with zero IP fragmentation. Fresh source stamps establish map transport freshness, not new LiDAR integration or motion readiness.

## Today's temporary replacement

The currently running temporary supervisor is independent of this prepared startup overlay. `cat323-map-transport status` discovers its live process and profile without relying on recorded PIDs. It also reports leftover diagnostic probe processes. Do not delete its `/tmp/cat323-map-transport.*` files while it is active.

When restoration is requested and control is idle, `cat323-map-transport restore` checks inactive UGEP and absence of native/internal command publishers, verifies the temporary supervisor's identity and child, then requests restoration of the original publisher. It starts no stack and sends no motion. This command is for the temporary replacement only; the future Compose test is removed by returning to normal Compose startup.

## Validation record

The bundle's native launch actions were constructed in memory inside the installed AMG environment, without executing ROS nodes. Normal mode added no DDS overrides; test mode added them to the selected interface publisher alone. The real Orin Compose comparison passed, with only the two read-only mounts and command changed. Offline tests cover opt-in scope, missing profiles, ambiguous/modified nodes, unchanged-file protection, profile drift, duplicate processes and freshness checks. A new canonical-only live check received 29 advancing snapshots in 30 seconds, with no stale publications and maximum source age 1.3581 seconds.

The test overlay has not been used to restart AMG in this session. Its first full startup remains to be verified at a planned test. Evidence is in `/workspaces/gravis_ws/evidence/map_transport_20260910/reusable_validation`; the original transport measurements remain under `temporary_fix` alongside it.
