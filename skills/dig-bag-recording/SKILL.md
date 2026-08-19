---
name: dig-bag-recording
description: "Record canonical split MCAP runs for Mole DIG/Newton controllers, including sensors, state, commands, LiDAR, compressed camera, excavation maps, actions, and Dig3D observations."
---

# DIG Bag Recording

This skill owns recording and finalized-artifact verification. It does not launch a controller (`dig-controllers`), start the base stack (`robot-startup`), replay data (`dig-bag-replay`), or upload it (`kleinkram-upload`).

## Quick start

```bash
SKILL_DIR="${CODEX_HOME:-$HOME/.codex}/skills/dig-bag-recording"
"$SKILL_DIR/scripts/dig_split_recording_tmux.sh" \
  --scenario dig_newton \
  --controller newton \
  --use-sim-time true \
  --attach
```

The helper calls the repository-owned `mole_bag_tools rosbag_record.launch.py` and creates one run root with `raw/sensors`, `raw/state`, `raw/commands`, `raw/lidar`, `raw/camera`, and `raw/elevation_map`. Dig3D also enables `raw/dig3d_special_obs`. The launch writes `recording_manifest.json`, `README.txt`, and `derived/`.

Pass `--controller` explicitly when the scenario tag is ambiguous. Recognized values are `dig3d`, `newton`, `dig`, `dig-ee`, and `ugep`; this selects the namespaced action transport to retain. Otherwise the helper makes a conservative inference from common scenario names. UGEP additionally records its namespaced observation topics and defaults command capture to its isolated `actuator_commands_ugep_dryrun` output; override `--actuator-commands-topic` only when the controller was deliberately launched with a different output.

## Runtime contract

- Use the published `rslheap/moleworks_ros:latest` image by default; retain an immutable `sha-<merge-sha>` tag when exact provenance matters.
- On a discovery-server robot shell, use normal plain `ros2 ...` inspection commands. The canonical launcher assigns the observer DDS profile to rosbag children while leaving normal processes on the runtime client profile.
- Do not replace the launcher with raw remote `ros2 bag record` or hand-copied DDS exports. If enabled splits stay empty, first verify the deployed image/container and start a fresh recorder window.
- Current robot-local state, command, LiDAR, and action topics derive from `--robot-namespace` (default `mole`). The primary DIG map is `/excavation_mapping/grid_map`; the canonical recorder also retains `/<robot_namespace>/elevation_map_filter` and the supported excavation-map aliases.
- Camera recording uses compressed image topics, preferring `/hal/grpc_image_client/Main/image_raw/compressed` while also retaining `/camMainView/image_raw/compressed` under the default camera configuration.

## Workflow

1. Confirm hardware versus simulation and set `--use-sim-time` accordingly.
2. Check required topics and free space under the output filesystem.
3. Start the helper before the DIG action.
4. Stop with `Ctrl-C` in the left `record` pane and wait for every rosbag process to finalize.
5. Verify each enabled split has `metadata.yaml`, at least one `.mcap`, and readable `ros2 bag info` output.

Read-only preflight example:

```bash
df -h ~/mcap/dig
timeout 10 ros2 topic echo /mole/state --once
timeout 10 ros2 topic echo /mole/actuator_commands --once
timeout 10 ros2 topic echo /excavation_mapping/grid_map --once
```

Defaults are tmux `ros:record`, output root `~/mcap/dig`, the first built workspace among `~/ros2_ws` and `~/moleworks/ros2_ws`, namespace `mole`, and hardware time (`false`). The helper refuses any busy pane by default; `--restart-window` replaces only the managed window and requires explicit restart authority.

## Verification

```bash
find ~/mcap/dig -maxdepth 4 -type f -name metadata.yaml | sort
ros2 bag info /path/to/run/raw/state
```

## Post-run scoop extraction

Use the repository-owned analyzer to turn one finalized campaign run into replayable scoop bags:

```bash
: "${RUN_ROOT:?Set RUN_ROOT to the finalized run directory}"
ros2 run mole_bag_tools analyze_dig_trenching_session \
  --run-root "$RUN_ROOT" \
  --write-splits \
  --split-excavation-map-snapshots-only
```

The default map topic is `/excavation_mapping/grid_map`. The analyzer resolves
`--command-topic` across the canonical raw splits. For isolated Dig3D tests where
other controllers keep `/mole/actuator_commands` continuously alive, add:

```bash
--command-topic /dig_3d/policy_action_stamped
```

This writes `derived/scoop_bags/scoop_XXX`. Run `ros2 bag info` on every child
bag before discarding the source. For command-versus-motion, bottom tracking,
current activity, and joint jitter diagnostics, run:

```bash
ros2 run mole_bag_tools analyze_dig_policy_motion \
  --input-root "$RUN_ROOT" \
  --force
```

If the source run must be discarded, first relocate the verified `derived/`
artifacts outside that run directory, preserve its manifest, and move the source
to recoverable trash. Never delete the only copy before all split metadata and
MCAP files pass `ros2 bag info`.

For OCS2 paper-evidence runs, use the OCS2 experiment skill and the repository's explicit `record_ocs2:=true` plus `validate_ocs2_recording` contract; this generic DIG profile intentionally does not claim OCS2 provenance completeness.
