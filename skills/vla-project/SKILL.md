---
name: vla-project
description: "Use for Lorenzo's vla_picknplace project: SO-101 banana datasets, LeRobot augmentation, ACT/Brev training, MolmoAct2 fine-tuning, and hardware workflows."
---

# VLA Project

Use this skill for the `vla_picknplace` repository and associated Hugging Face
banana pick-and-place datasets.

Keep inspection, local dataset builds, and local QA read/local-only by default. Publishing datasets,
starting paid remote training, pushing repository changes, or commanding hardware requires explicit
user intent for that action.

## First Steps

1. Work from `/home/lorenzo/git/vla_picknplace` unless the user gives a different checkout.
2. Check `git status --short` before editing. The tree often contains unrelated local work; do not revert it.
3. Read `references/project.md` when touching dataset builds, prompt generation, ACT training, MolmoAct2 preparation, or Brev launch commands.
4. Prefer repo scripts over ad hoc code:
   - `data_processing/augment_banana_lerobot.py`
   - `data_processing/check_banana_color_swap.py`
   - `data_processing/check_banana_prompt_templates.py`
   - `act/check_act_dataset.py`
   - `cluster/brev/submit_act_brev.sh`

## Dataset Workflow

For the main language-conditioned banana eval dataset, rebuild with:

```bash
uv run python data_processing/augment_banana_lerobot.py \
  --output-repo-id rslxcvg/banana_act_position_targetcolor_v2 \
  --root outputs/lerobot/banana_act_position_targetcolor_v2 \
  --mode position_target_colors \
  --prompt-mode eval_mixed \
  --auto-trim-static-start \
  --overwrite
```

Only push after local QA passes. Add `--push-to-hub` to publish.

## Required QA

Run these checks after changing augmentation code or rebuilding data:

```bash
uv run python data_processing/check_banana_prompt_templates.py
uv run python data_processing/check_banana_color_swap.py --real-data
uv run python act/check_act_dataset.py \
  --repo-id rslxcvg/banana_act_position_targetcolor_v2 \
  --root outputs/lerobot/banana_act_position_targetcolor_v2 \
  --episode 0 \
  --video-backend pyav
```

Also inspect contact sheets or sampled videos. Text correctness is not enough:
the rendered bowl colors must match the prompt, the banana should not acquire
strong color artifacts, and the first seconds should not contain long static
table/init pre-roll.

## Detailed Workflows

Read [references/project.md](references/project.md) before training, dataset
publication, Molmo/current-joint conversion, or hardware work. It owns the
current commands, checkpoint status, calibration values, and safety notes; do
not duplicate those details here.

Never deploy the known bad `normalize_gripper=false` MolmoAct2 checkpoint. For
hardware motion, verify the live device mapping and use bounded targets in the
current robot convention, with an operator present and an explicit motion request.
