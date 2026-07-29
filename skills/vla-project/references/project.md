# VLA Pick-and-Place Reference

## Repository

Default checkout:

```text
/home/lorenzo/git/vla_picknplace
```

Use `uv run python` for repo scripts.

## Local SO-101 Hardware

Observed on Lorenzo's machine on 2026-05-15:

```text
follower serial: /dev/ttyACM0
camera:          /dev/video0
camera mode:     640x480, 30 fps, MJPEG/YUYV
leader serial:   not connected during the live check
```

Carmen's original recording command used `index_or_path: 4`; that was the path
on her recording machine. On this machine the working camera is `/dev/video0`.

If the local `uv` environment is missing LeRobot hardware deps, install:

```bash
uv pip install \
  'pyserial>=3.5,<4.0' \
  'feetech-servo-sdk>=1.0.0,<2.0.0' \
  'deepdiff>=8.0.0,<9.0.0'
```

If `/dev/ttyACM0` permission is denied, the temporary session fix is:

```bash
sudo setfacl -m u:lorenzo:rw /dev/ttyACM0
```

The persistent fix is to add the user to `dialout` and re-login:

```bash
sudo usermod -aG dialout $USER
```

Live camera preview:

```bash
tmux new-session -d -s vla_camera_preview \
  'ffplay -loglevel warning -fflags nobuffer -flags low_delay -f v4l2 -input_format mjpeg -video_size 640x480 -framerate 30 /dev/video0'
tmux attach -t vla_camera_preview
tmux kill-session -t vla_camera_preview
```

## Calibration Files

Calibration files were previously installed at these paths, but both were
absent during the 2026-07-13 audit:

```text
~/.cache/huggingface/lerobot/calibration/robots/so_follower/follower.json
~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/leader.json
```

Verify or restore them before hardware use. Treat the values below as the last
recorded calibration, not proof that the files are currently installed.

Follower calibration values:

```text
shoulder_pan   id=1 homing_offset= 1750 range= 699..3430
shoulder_lift  id=2 homing_offset=-1891 range= 772..3128
elbow_flex     id=3 homing_offset= 1033 range= 921..3159
wrist_flex     id=4 homing_offset= 1394 range= 904..3199
wrist_roll     id=5 homing_offset=-1741 range=   0..4095
gripper        id=6 homing_offset= 1924 range=2017..3471
```

Leader calibration values:

```text
shoulder_pan   id=1 homing_offset=-1702 range= 732..3426
shoulder_lift  id=2 homing_offset=-1340 range= 955..3311
elbow_flex     id=3 homing_offset= 1778 range= 835..3045
wrist_flex     id=4 homing_offset=  880 range= 891..3228
wrist_roll     id=5 homing_offset= -845 range=   0..4095
gripper        id=6 homing_offset= 1985 range=2019..3238
```

## Source Datasets

The banana source data is SO-101 follower LeRobot data:

```text
rslxcvg/banana_blue    target position: left
rslxcvg/banana_red1    target position: center
rslxcvg/banana_green   target position: right
```

Original physical bowl order:

```text
left: blue
center: red
right: green
```

The source LeRobot schema contains:

```text
observation.images.front  RGB video, 480x640
observation.state         float32[6]
action                    float32[6]
task                      source language text
```

## Main Augmented Dataset

Current final dataset target:

```text
repo id: rslxcvg/banana_act_position_targetcolor_v2
root:    outputs/lerobot/banana_act_position_targetcolor_v2
```

Build command:

```bash
uv run python data_processing/augment_banana_lerobot.py \
  --output-repo-id rslxcvg/banana_act_position_targetcolor_v2 \
  --root outputs/lerobot/banana_act_position_targetcolor_v2 \
  --mode position_target_colors \
  --prompt-mode eval_mixed \
  --auto-trim-static-start \
  --overwrite
```

Publish only after QA:

```bash
uv run python data_processing/augment_banana_lerobot.py \
  --output-repo-id rslxcvg/banana_act_position_targetcolor_v2 \
  --root outputs/lerobot/banana_act_position_targetcolor_v2 \
  --mode position_target_colors \
  --prompt-mode eval_mixed \
  --auto-trim-static-start \
  --overwrite \
  --push-to-hub
```

Expected episode count:

```text
60 source episodes * 3 rendered target colors * 4 prompt buckets = 720 episodes
```

Static-start trimming changes total frame count but should not change episode
count.

Do not train from `outputs/lerobot/banana_act_position_targetcolor_v1`; it was
observed stale/partial locally.

## Augmentation Semantics

`--mode position_target_colors` renders the demonstrated target-position bowl
as each possible color: red, green, and blue. It changes video pixels and task
text, while copying `observation.state` and `action` unchanged.

`--prompt-mode eval_mixed` writes one copy of each rendering for:

```text
direct_color
position_ordinal
relative_color
exclusion
```

`--auto-trim-static-start` removes per-episode table/init pre-roll using both
robot-state motion and image motion. This is preferred over fixed 2 second
cropping because some episodes start active.

## Required Checks

Prompt template logic:

```bash
uv run python data_processing/check_banana_prompt_templates.py
```

Color-swap regression, including real data:

```bash
uv run python data_processing/check_banana_color_swap.py --real-data
```

Compile touched Python files:

```bash
uv run python -m py_compile \
  data_processing/bowl_color_swap.py \
  data_processing/augment_banana_lerobot.py \
  data_processing/check_banana_color_swap.py \
  data_processing/check_banana_prompt_templates.py
```

ACT compatibility smoke:

```bash
uv run python act/check_act_dataset.py \
  --repo-id rslxcvg/banana_act_position_targetcolor_v2 \
  --root outputs/lerobot/banana_act_position_targetcolor_v2 \
  --episode 0 \
  --video-backend pyav
```

Metadata check:

```bash
uv run python - <<'PY'
import json
from pathlib import Path
import pandas as pd

root = Path("outputs/lerobot/banana_act_position_targetcolor_v2")
info = json.loads((root / "meta/info.json").read_text())
tasks = pd.read_parquet(root / "meta/tasks.parquet")

print(info["total_episodes"], info["total_frames"], info["total_tasks"])
print(tasks.head(20).to_string(index=False))
PY
```

Visual QA:

- Sample all source colors and rendered target colors.
- Check that direct-color, relative-color, and exclusion prompts resolve to the
  visible target bowl.
- Check that left/right/center bowls are not half-mixed after swapping.
- Check banana color artifacts are mild; bowl color errors are catastrophic.
- Check starts are trimmed near the reach, not static table/init pose.

## Training

ACT:

```bash
cluster/brev/submit_act_brev.sh \
  --dataset-repo-id rslxcvg/banana_act_position_targetcolor_v2 \
  --gpu-list 5 \
  --time 24h
```

Vanilla ACT ignores `task`, so `eval_mixed` prompt copies are duplicate
visual/action data for ACT. For ACT-only experiments, consider a position-only
or non-duplicated dataset.

MolmoAct2:

Use Ai2's official LeRobot `molmoact2-policy` branch for MolmoAct2 fine-tuning,
not the older local experimental script:

```text
historical local check clone: /tmp/molmoact2_recheck/lerobot (ephemeral; absent 2026-07-13)
Brev clone:       /home/nvidia/data/vla_picknplace/allenai_lerobot_molmoact2_policy
commit checked:   80633827176a0203064cb141383664fba024e050
```

For MolmoAct2 calibration compatibility, use the derived dataset:

```text
rslxcvg/banana_act_position_targetcolor_v2_molmo_compat
outputs/lerobot/banana_act_position_targetcolor_v2_molmo_compat
```

Build it with:

```bash
uv run python data_processing/convert_banana_molmo_compat.py \
  --source-root outputs/lerobot/banana_act_position_targetcolor_v2 \
  --output-root outputs/lerobot/banana_act_position_targetcolor_v2_molmo_compat \
  --output-repo-id rslxcvg/banana_act_position_targetcolor_v2_molmo_compat \
  --overwrite
```

Add `--push-to-hub` after local validation. The converter ignores `.cache/**`
during upload and syncs the LeRobot `v3.0` tag to `main`.

The converter changes only `observation.state` and `action`:

```text
shoulder_lift.pos = 90 - shoulder_lift.pos
elbow_flex.pos = elbow_flex.pos + 90
```

Videos, task text, joint order, FPS, and image shape stay unchanged.

Current full augmented Molmo-compatible dataset:

```text
repo:       rslxcvg/banana_act_position_targetcolor_v2_molmo_compat
hub sha:    ef14eec64bdbe42e9c3507a6de5c708556591aa3
episodes:   720
frames:     300168
source:     60 episodes * 3 rendered target colors * 4 prompt buckets
prompts:    direct_color, position_ordinal, relative_color, exclusion
```

Canonical official fine-tune setup:

```text
base checkpoint:     allenai/MolmoAct2-SO100_101
norm tag:            so100_so101_molmoact2
action mode:         continuous
train scope:         action expert only
precision:           bf16
flow timesteps:      8
image key:           observation.images.front
batch size:          16 per GPU, 2 GPUs
steps:               10000
save freq:           10000
freeze embedding:    true
gradient checkpoint: true
normalize gripper:   true
```

Bad checkpoint, do not deploy:

```text
run id: molmoact2_banana_compat_action_expert_2gpu_finalsave_20260515_044113
path:   /home/nvidia/code/vla_picknplace/outputs/molmoact2/banana_compat_action_expert_2gpu_finalsave
issue:  trained with --policy.normalize_gripper=false; offline validation
        showed gripper collapse from a 0..100 vs normalized-unit mismatch
```

Corrected retrain:

```text
run id: molmoact2_banana_compat_action_expert_normgripper_2gpu_finalsave_20260515_112148
log:    /home/nvidia/logs/vla_picknplace/brev-molmoact2_banana_compat_action_expert_normgripper_2gpu_finalsave_20260515_112148.log
path:   /home/nvidia/code/vla_picknplace/outputs/molmoact2/banana_compat_action_expert_normgripper_2gpu_finalsave
note:   final checkpoint is saved at step 10000; no intermediate checkpoint by default
```

The local deployment wrapper defaults should point at the corrected checkpoint:

```text
molmoact2/deploy_so101_cli.py
outputs/molmoact2/banana_compat_action_expert_normgripper_2gpu_finalsave/checkpoints/last/pretrained_model
```

## Molmo/Current LeRobot Convention

The Molmo-compatible dataset intentionally uses the old/backward-compatible
SO-100/SO-101 convention for shoulder and elbow:

```text
molmo_shoulder_lift = 90 - current_shoulder_lift
molmo_elbow_flex    = current_elbow_flex + 90
```

Invert this before commanding the current LeRobot SO follower:

```text
current_shoulder_lift = 90 - molmo_shoulder_lift
current_elbow_flex    = molmo_elbow_flex - 90
```

Do not compare Molmo-compatible shoulder/elbow values directly to live robot
readings, and do not send raw Molmo-compatible shoulder/elbow actions to current
hardware. Body joints use `robot.use_degrees=true`; gripper is still 0..100.

Dataset first-frame median from all 720 augmented episodes, in Molmo convention:

```text
shoulder_pan=-2.505, shoulder_lift=137.692, elbow_flex=84.989,
wrist_flex=97.582, wrist_roll=5.143, gripper=1.135
```

Equivalent current-robot command convention:

```text
shoulder_pan=-2.505, shoulder_lift=-47.692, elbow_flex=-5.011,
wrist_flex=97.582, wrist_roll=5.143, gripper=1.135
```

Central 10-90% Molmo start range:

```text
shoulder_pan   -7.3..  1.1
shoulder_lift 126.6..146.3
elbow_flex     75.1.. 97.7
wrist_flex     91.5.. 98.2
wrist_roll      0.8..  9.3
gripper         0.9..  2.4
```

Live read with the downloaded follower calibration before commanded motion:

```text
current convention:
shoulder_pan=-5.582, shoulder_lift=-103.736, elbow_flex=97.582,
wrist_flex=28.703, wrist_roll=3.033, gripper=1.651
```

Later live read before the bounded positioning test:

```text
current convention:
shoulder_pan=-16.044, shoulder_lift=15.209, elbow_flex=23.385,
wrist_flex=69.231, wrist_roll=12.967, gripper=21.871

Molmo equivalent:
shoulder_pan=-16.044, shoulder_lift=74.791, elbow_flex=113.385,
wrist_flex=69.231, wrist_roll=12.967, gripper=21.871
```

Bounded positioning target used for dataset start pose, in current convention:

```text
shoulder_pan=-2.505, shoulder_lift=-47.692, elbow_flex=-5.011,
wrist_flex=97.582, wrist_roll=5.143, gripper=1.135
```

Reusable start-pose controller:

```bash
uv run python molmoact2/move_so101_to_start_pose.py
```

It connects only the follower arm, uses no cameras, sends capped position
targets, and disconnects with torque still enabled so the arm keeps holding.
It derives the current-robot start target from the Molmo-compatible median using
`molmoact2/so101_compat.py`.

Do not deploy the banana MolmoAct2 checkpoint through stock `lerobot_rollout`
without custom robot-boundary processors. Stock rollout uses identity robot
processors, but this checkpoint needs:

```text
live observation -> policy: shoulder_lift = 90 - current, elbow_flex = current + 90
policy action -> robot:     shoulder_lift = 90 - molmo,   elbow_flex = molmo - 90
```

Safety notes:

- Confirm the workspace is clear and the arm is physically supported before any
  actuation.
- Move only in current robot convention and with bounded relative targets.
- Do not run a policy trained with the bad `normalize_gripper=false` checkpoint.
- If the arm must hold position, keep a hold loop alive or disconnect without
  disabling torque. A previous bounded move ended with `robot.disconnect()`,
  released torque, and the arm collapsed.

## Brev

Use `cluster/brev/README.md` and the `mw-newton-dev` SSH alias. Jobs write tail
and kill commands to:

```text
cluster/brev/submitted_jobs.txt
```
