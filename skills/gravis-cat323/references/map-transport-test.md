# CAT323 selected-map transport and recovery

Use this transport profile for CAT323 policy tests across the Orin-to-integration
link. The versioned owner is this skill in `Idate96/codex_skills`, branch `gravis`;
`amg-map-transport-test/`, images, containers, and files under `/tmp` are generated
artifacts. They can disappear when Gravis reprovisions either host. Commit and
push changes to the skill and its assets; local files alone are not recovery.

The profile caps outgoing DDS messages at **1400 bytes** and uses asynchronous
FIFO pacing of **8192 bytes per 5 ms**. Small packets avoid IP fragmentation;
pacing also bounds bursts across the sender/receiver link. Apply it only to
`grid_map_selected_layers_interface_republisher_node`. Keep normal incoming map
capacity, native QoS, source stamps, and the controller's 3 s freshness gate.
Never relax that gate to compensate for delivery loss.

## Recover after an image update or host wipe

First restore the x86 workspace and ROS dependencies using the repository's
`docs/information/gravis_engineer_setup.md`. Restore SSH access through the machine
owner; keys and proprietary Gravis source do not belong in the skill. Clone the
full skill catalog into the mounted workspace:

```bash
git clone --branch gravis git@github.com:Idate96/codex_skills.git \
  /workspaces/gravis_ws/codex_skills
```

On an existing checkout, fetch and inspect before updating; preserve local work.
Read the [native startup procedure](operations.md#native-gravis-startup-with-gtask).
Verify Orin identity, current rack/machine configuration, installed `gtask`, and
actual deployment image. Do not infer the image from `deploy.env`: its package
pin has differed from the deployed image. Keep command output disabled and the
controller inactive while preparing transport. Starting native AMG for inspection
requires the operator's software-startup scope.

Check whether the selected native image already contains both:

- `/amg/install/lib/python3.12/site-packages/autonomy_visualization/launch.py`
  with the optional `CAT323_SELECTED_MAP_DDS_PROFILE` hook;
- `/amg/install/share/autonomy_visualization/config/selected-map-fastdds.xml`
  matching `assets/cat323-selected-map-fastdds.xml` in this skill.

If it does, use that image and verify the effective process settings below.
Otherwise prepare a fresh build context against the current running native image,
from the sourced x86 container shell:

```bash
python3 /workspaces/gravis_ws/codex_skills/skills/gravis-cat323/scripts/prepare_cat323_map_transport.py \
  --host orin-a --container amg \
  --output /workspaces/gravis_ws/amg-map-transport-current
```

Choose another output directory if existing contents differ. The helper reads
only the relevant native launch/profile and image metadata. It validates the
selected Node's identity, rejects changed/customized launch structure, and writes
the original, minimal patch, patched launch, XML, manifest and Dockerfile. It
never modifies or restarts the native stack. A preexisting hook requires review
rather than applying the patch twice. No proprietary module is committed here;
the patch is generated from the machine's installed version.

The Dockerfile pins the inspected image **digest**, copies only the launch module
and XML, removes **only that module's `launch.*.pyc` cache**, and restores the
original image user. Cache removal is necessary: AMG has shipped unchecked-hash
Python bytecode that ignores source edits even when the new file is visible.
Checking the source file alone does not prove the hook executes.

Copy the build inputs to a new directory on Orin and build there. Substitute the
actual returned directory and a fresh local tag; no source rebuild or image pull
is required:

```bash
ssh orin-a 'mktemp -d /tmp/cat323-map-image.XXXXXX'
# Use the returned path below.
scp /workspaces/gravis_ws/amg-map-transport-current/{Dockerfile,patched-launch.py,selected-map-fastdds.xml} \
  orin-a:/tmp/cat323-map-image.ACTUAL/
ssh orin-a 'docker build --network=none --pull=false -t cat323-amg-map-transport:current /tmp/cat323-map-image.ACTUAL'
```

This local derivative is disposable. Rebuild from the then-current Gravis image
after reprovisioning; the versioned patch generator and XML are the recovery
source of truth. Do not distribute the proprietary derived image without the
machine owner's authorization.

## Start through gtask

Stop the prior native stack through its owning `gtask` session only when restart
is authorized, after the controller is inactive, command gateway stopped, and
machine locked. Preserve the existing x86 target and record its frame/pose before
restart. Verify no AMG Compose-project containers remain running. If the owning
foreground task exits but its discovery/task-service dependencies remain, use
`gtask amg:stop` to finish that authorized stop; the next `amg:run` correctly
refuses to start beside those survivors. Run the new stack in a persistent Orin tmux window:

```bash
gtask amg:run AMG_BASE_IMAGE=cat323-amg-map-transport:current -- env \
  CAT323_SELECTED_MAP_DDS_PROFILE=/amg/install/share/autonomy_visualization/config/selected-map-fastdds.xml \
  ros2 launch amg amg.launch.py
```

The XML path must be one shell argument. Keep the explicit image option: the
package's default image may not contain the patch. The original image and native
Compose files stay unchanged. Starting this software does not unlock hydraulics
or authorize a scoop.

## Prove that the restored configuration is active

From the sourced integration ROS environment:

```bash
python3 /workspaces/gravis_ws/codex_skills/skills/gravis-cat323/scripts/cat323_map_transport.py status
python3 /workspaces/gravis_ws/codex_skills/skills/gravis-cat323/scripts/cat323_map_transport.py check-maps --seconds 60
```

Require exactly one selected publisher, whose **effective process environment**
sets `FASTRTPS_DEFAULT_PROFILES_FILE` to the selected XML and
`RMW_FASTRTPS_USE_QOS_FROM_XML=1`. Check its XML hash, message size 1400,
asynchronous mode and flow values 8192/5. Merely seeing the parent
`CAT323_SELECTED_MAP_DDS_PROFILE` variable is insufficient. If source contains
the hook but effective overrides are absent, inspect the loaded function bytecode
and remove/rebuild that module's stale unchecked `.pyc` before restarting.

The canonical-map check adds no native map subscriber. Require advancing source
stamps, zero stale/regressing publications and bounded gaps. Also assess the
maximum age of the **last received map between updates**, as the controller runs
continuously; a publication-only freshness check can miss expiry before the next
publication. Repeat under intended recorder/camera load before another scoop.
After native restart verify the existing adapter/controller clients receive fresh
state and maps, and that native TF still agrees with the earth-fixed target.
Refresh stale clients only on evidence, preserving/reapplying the confirmed target
if a mapper restart is needed. Recheck full motion gates separately.

## Earlier temporary setup

The older generated Compose overlay and temporary publisher supervisor are not
the normal restart path. Inspect their actual ownership before cleanup; never
apply historical PIDs or stop unrelated native processes. The `restore` command
is only for that verified temporary supervisor. Existing helper tests retain
coverage of that older path, but new deployments should use the patched image
with native `gtask` ownership.

## September 17 recovery evidence

The disposable derivative of `gravisrobotics/amg:debug-v3.1.0-rc1` was started
through `gtask` with this exact XML. Removing the unchecked launch-module cache
made the effective publisher environment adopt the profile. The first 60-second
check then received 59 advancing canonical maps, zero stale or regressing
publications, maximum publication age 1.379 s and maximum held age 2.379 s.
These are measured results, not a substitute for checking the current deployment
and intended recorder load after every update.
