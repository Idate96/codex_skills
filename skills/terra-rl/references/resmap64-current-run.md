# Current ResMap64 R1/R2 Run

Read this reference only when working on the paired ResMap64 R1/R2 experiment, its timeout semantics, validation, or monitoring.

## Contents

- [Launch contract](#current-resmap64-r1r2-launch)
- [Validation](#current-resmap64-r1r2-launch)
- [Monitoring signals](#current-resmap64-r1r2-launch)

## Current ResMap64 R1/R2 Launch

For the current combined-fix run, use the paired WIP trees and the prepared script:

- Local Terra tree: `/home/lorenzo/moleworks/terra_mask_wip`
- Local baselines tree: `/home/lorenzo/moleworks/terra-baselines_mask_wip`
- Euler workspace: `/cluster/home/lterenzi/codex_terra_edge_validation`
- Script: `scripts/euler/terra_train_resmap64_phase_4gpu.sbatch`

The intended architecture and semantics are:

- ResMap64 delayed-downsample encoder:
  `--map_encoder resnet_delayed --map_feature_dim 128 --use_map_derived_channels`
- Separate actor/critic trunks:
  `--separate_actor_critic_trunks`
- Action mask disabled:
  `--disable_action_mask`
- Critic-only affordances:
  `--use_critic_affordances --include_episode_progress --edge_features_dim 10`
- PPO shape:
  `--num_devices 4 --num_envs_per_device 1024 --num_steps 32 --num_minibatches 32`
- Time limit:
  current user request is to run this in `gpuhe.120h` with `5-00:00:00`.
- Timeout phase:
  randomize initial `env_steps`/`episode_progress` once after startup reset. This desynchronizes
  max-step timeouts while leaving normal reset behavior unchanged.

Do not add actor-visible affordances to this run. Anything actor-visible must be deployable on the
real robot with the same semantics; keep that as a later ablation.

Timeout handling for this run should be:

- Preserve `final_observation` through env reset.
- Bootstrap max-step truncations from the pre-reset final observation.
- Do not bootstrap true task terminals.
- Stop recursive GAE at any reset boundary so reset-episode advantages do not leak backward.
- Stagger initial episode ages so max-step timeouts do not arrive as one synchronized rollout step.
- Pay terminal success reward only on `task_done`, never on max-step timeout. A timeout with
  partial completion may have progress rewards, but must not receive the terminal bonus.

Before submitting this script from a fresh work session, verify the local gates include:

```bash
python3 -m py_compile \
  /home/lorenzo/moleworks/terra_mask_wip/terra/env.py \
  /home/lorenzo/moleworks/terra_mask_wip/terra/state.py \
  /home/lorenzo/moleworks/terra-baselines_mask_wip/train.py \
  /home/lorenzo/moleworks/terra-baselines_mask_wip/train_mixed.py \
  /home/lorenzo/moleworks/terra-baselines_mask_wip/utils/models.py \
  /home/lorenzo/moleworks/terra-baselines_mask_wip/utils/utils_ppo.py \
  /home/lorenzo/moleworks/terra-baselines_mask_wip/scripts/validation/validate_edge_mask_changes.py

JAX_PLATFORMS=cpu PYTHONPATH=/home/lorenzo/moleworks/terra_mask_wip:/home/lorenzo/moleworks/terra-baselines_mask_wip \
  /home/lorenzo/moleworks/.venv-terra-uv/bin/python \
  /home/lorenzo/moleworks/terra-baselines_mask_wip/scripts/validation/validate_edge_mask_changes.py \
  --case all --jax-platforms cpu \
  --dataset-path /home/lorenzo/moleworks/terra_data/train --dataset-size 1
```

The latest local reviewed gate on 2026-05-15 for the staggered-timeout patch passed a one-update
RTX 4090 smoke with this shape in `432.00s` at `79.36` steps/s. Do not skip the script's own Euler
W&B-disabled smoke; local smoke does not prove multi-GPU NCCL, allocation type, or cluster CUDA
paths.

When monitoring the first online run, check these before calling it healthy:

- Slurm allocation is exactly four RTX 3090/4090 GPUs.
- `check_jax_runtime.py --min-devices 4` passed.
- The script's W&B-disabled full-shape smoke completed update 1.
- The online W&B run id and URL are in `docs/EXPERIMENTS_RUNNING.md`.
- First online update completed, not just model/dataset initialization.

For the value-spike question, watch these together:

- `value_loss`, `entropy`, `explained_variance`, and `sched/entropy_coef`
- `train/done_rate`, `train/task_done_rate`, `train/timeout_rate`
- `train/mean_episode_progress`, `train/max_episode_progress`
- `train/value_loss_timeout_bucket_*`
- `train/explained_variance_timeout_bucket_*`
- `train/timeout_bucket_*_count`
- `affordance/do_valid_rate`, `affordance/valid_action_fraction`,
  `affordance/legal_edge_in_cone`, `affordance/blocked_edge_in_cone`, and
  `affordance/completion`

Do not interpret empty timeout buckets as stable value learning; check the bucket counts.
