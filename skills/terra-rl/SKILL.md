---
name: terra-rl
description: "Run Terra RL training across `terra` and `terra-baselines`. Use for validation, Euler Slurm, W&B comparison, PPO configuration, JAX/CUDA preflight, first-update smoke tests, checkpointed 24h/120h continuation, and failure diagnosis."
---

# Terra RL

Use this skill for Terra policy training work spanning:

- `/home/lorenzo/moleworks/terra`
- `/home/lorenzo/moleworks/terra-baselines`
- Euler account-selected workspaces under `/cluster/scratch/<account>/`
- W&B project `aless-weber-eth/mixed-agents`

## Core Rule

Do not treat `jax.devices()` as a sufficient GPU preflight. Before launching expensive training,
prove the CUDA runtime can execute the paths the trainer needs:

1. venv CUDA library paths include cuDNN, CUPTI, cuBLAS, NVRTC, and NCCL.
2. JAX sees the expected GPU count.
3. A tiny jitted conv backward pass completes on GPU.
4. For multi-GPU jobs, a pmap all-reduce completes through NCCL.
5. A training smoke either completes update 1 or is explicitly recorded as not a passed gate.

Use the bundled [scripts/check_jax_runtime.py](scripts/check_jax_runtime.py), or a verified copy in the active training checkout, inside the Slurm allocation after exporting the same environment as training. Test the selected path before submission; the canonical checkout does not always contain `scripts/euler/check_jax_runtime.py`.

## Environment Selection

Use these environments by default:

- Local CPU gates: `/home/lorenzo/moleworks/.venv-terra-uv`. This is the canonical uv/venv for
  syntax checks, CPU action-mask/state gates, and small non-GPU probes. Set `JAX_PLATFORMS=cpu`.
- Local 24 GB GPU tests: `/home/lorenzo/moleworks/.venv-terra-gpu-uv`. Use this for one-GPU
  first-update smoke tests, CUDA runtime checks, and env-capacity sweeps on the local RTX 4090.
  Do not use the CPU-only `.venv-terra-uv` to decide GPU memory fit.
- Euler training: select `TERRA_EULER_USER` explicitly. The current default is
  `alesweber` via SSH host `euler-alesweber`; `euler-lterenzi` is the legacy
  fallback. Derive `WORK=/cluster/scratch/$TERRA_EULER_USER/codex_terra_edge_validation`
  and `RUNS=/cluster/scratch/$TERRA_EULER_USER/codex_terra_edge_runs`. Use a
  pinned project-stored venv supplied by the campaign; the current V8 runtime
  `/cluster/project/rsl/lterenzi/terra_curriculum_20260730_c14bd7d_3ce0e84_py312_jax0426`
  is group-readable by `alesweber`. Record the account and exact venv in every
  run contract.

Always set:

```bash
export PYTHONPATH=/home/lorenzo/moleworks/terra:/home/lorenzo/moleworks/terra-baselines:${PYTHONPATH:-}
```

or the equivalent Euler `WORK` paths before invoking `train_mixed.py` or validation scripts.

## Euler GPU Selection

Use only verified NVIDIA GeForce RTX 3090 or RTX 4090 allocations unless the user explicitly approves another GPU family. A generic GPU count is not enough: constrain the node family, hard-fail on an unexpected `nvidia-smi` name or count before JAX/W&B starts, then verify `ReqTRES`, `AllocTRES`, and the first log lines after submission. Read [references/euler-runtime.md](references/euler-runtime.md) for the current Slurm shape, exports, guard, and monitoring commands.

## Launch Workflow

1. Check both repos with `git status --short --branch`; never revert unrelated dirty files.
2. If copying to Euler, sync only intended changes into the isolated workspace.
3. Run `python3 -m py_compile` plus the active checkout's task-relevant CPU validation cases. Test that the validator exists; the full `validate_edge_mask_changes.py` suite belongs to the paired ResMap64 WIP tree and is documented in [references/resmap64-current-run.md](references/resmap64-current-run.md).
4. Select only RTX 3090/4090 nodes and include the hard GPU-type guard from this skill in the Slurm
   job before any JAX or W&B training command.
5. Run the GPU runtime preflight from this skill in the Slurm job before training.
6. For new runtime/config changes, run a first-update smoke with W&B disabled before a full run.
   The smoke must assert finite losses, model parameters, optimizer state, and rollout/teacher
   tensors when applicable; a completed update is not enough if it wrote NaN parameters.
7. Launch a production Slurm/W&B job only when the user requested real training and the smoke evidence is real: update 1 completed, not only dataset/model initialization. A status, diagnosis, or config-review request does not authorize a new expensive run.
8. For A/B comparisons, verify each side independently past update 1. Do not count a Slurm
   `RUNNING` job as healthy until the log shows completed updates.
9. For performance/architecture work, obtain an independent review with the relevant diffs and
   measured logs, not only at final review. In Codex, use a credential-free Codex subagent by
   default. Invoke Claude, Opus, Oracle, or another model/provider only when the user explicitly
   requests it in the current task. Ask the reviewer to look for missed code hotspots, algorithmic
   options, profiling strategy, data layout changes, JAX/XLA architecture issues, and RL-level
   speed/learning tradeoffs; then verify recommendations locally before retaining changes. Never
   put passwords, tokens, private keys, or other credentials in a delegated prompt.
10. Keep `docs/EXPERIMENTS_RUNNING.md` and `docs/EXPERIMENTS_LOG.md` current with job ids, W&B ids,
   exact failure signatures, and whether W&B has real history.

## Checkpointed Training Duration

- Treat minute-scale/update-1 jobs as runtime smokes only. Unless explicitly
  named as a shorter diagnostic, give a behavioral screen at least one healthy
  24-hour allocation before making a negative or saturation claim.
- Configure an absolute update target safely beyond what one allocation can
  finish. Wall time may end a segment; it must not make a promising run stop
  early merely because the update target was too small.
- Save and validate a rolling checkpoint at least every 500 updates. A Slurm
  timeout with a finite, loadable checkpoint is `CONTINUABLE`, not a failed
  experiment. Evaluate the latest complete checkpoint in a separate job when
  the hard timeout prevents post-training commands from running.
- Continue with `--resume_from`, never `--warm_start_from`. Preserve model
  parameters, optimizer state/step, global `next_update`, schedule position,
  environment protocol, and adaptive sampler state. Set `total_timesteps` for
  the absolute final update target, not for an additional segment length.
- Reuse the W&B run ID with `resume=must` only when the checkpoint is at or
  beyond the last logged `train/update`. Otherwise create a linked continuation
  run so replay from an older checkpoint cannot create backward/duplicate
  history. Record every Slurm segment and checkpoint hash in the ledger.
- Promote a promising recipe to the 120-hour queue and continue while fixed
  held-out exact, macro, or worst-condition performance is still improving.
  Stop on a held-out plateau across multiple checkpoints, not on wall time or
  online training success alone. Report updates, transitions, and GPU-hours.

Current Terra resumes restart RNG, live environment state, and action history,
so continuation is statistically continuous but not bit-exact. Read
[references/euler-runtime.md](references/euler-runtime.md) before implementing
segmented jobs.

## Current ResMap64 R1/R2 Launch

For the paired ResMap64 WIP trees, timeout semantics, exact gates, and monitoring
signals, read `references/resmap64-current-run.md`.

## Current Default Solo Excavator Run

Use this as the baseline unless the user asks for another preset:

```bash
export DATASET_PATH=/cluster/project/rsl/alesweber/TerraProject/terra/data/terra/train
export DATASET_SIZE=600

python train_mixed.py \
  --config solo_excavator \
  --num_devices 4 \
  --num_envs_per_device 1024 \
  --num_steps 32 \
  --update_epochs 2 \
  --num_minibatches 16 \
  --total_timesteps 50000000000 \
  --log_train_interval 1 \
  --log_eval_interval 100 \
  --checkpoint_interval 100 \
  --eval_episodes 100
```

Effective defaults:

- Agent/action: `agent_types=[0]`, `action_types=[0]` tracked excavator.
- Map: `foundations_real_ring`, `DATASET_SIZE=600`, `max_steps=550`.
- Total envs: `4 * 1024 = 4096`.
- Env steps/update: `32 * 4096 = 131072`.
- Corrected update count for `50B` global timesteps: `381469`. Older code divided by
  `num_devices` twice and printed `95367`, which only executed about `12.5B` actual global env
  steps.
- PPO: `lr=3e-4`, `gamma=0.9984`, `gae_lambda=0.95`, `clip_eps=0.2`,
  `ent_coef=0.06`, `vf_coef=2.0`, `max_grad_norm=0.5`.
- Entropy schedule: `0.15 -> 0.005` over `9500` updates.
- Rewards: `dump_bonus_mult=0.5`, `excavator_relocate_dumped_mult=1.5`,
  `excavator_relocate_dug_dirt_mult=1.5`.
- PPO action masking is disabled by default; use `--enable_action_mask` only for explicit
  coarse action-availability mask experiments until the full-shape masked PPO path is stable.

## Reward Guidance

Treat the default reward setup as the baseline while validating runtime, action masking, and
PPO stability. Do not mix reward-weight changes into mask/no-mask or runtime A/B tests unless the
user explicitly asks; otherwise the comparison stops answering the original question.

Use rewards diagnostically, not as the only success criterion:

- `eval/rewards` rising while `eval/max_reward < 1` usually means dense-reward progress, not task
  completion.
- `eval/DO` rising without `eval/positive_terminations` can mean the policy learned to dig/dump
  activity but not finish the excavation objective.
- High `eval/DO_NOTHING %` means the policy is still dithering or avoiding committed actions.
- `progress/episode_completion_rate` can include timeouts/failures, so it is not a substitute for
  `eval/positive_terminations`.

Reward finetuning should come after a default-config policy shows repeatable terminal success. Then
start from the best checkpoint and change one reward knob at a time, logging the parent checkpoint,
diff, W&B run id, and expected behavioral effect in the experiment ledger. Candidate later
finetunes include terminal reward backfill, dump/relocation multipliers, and phase- or edge-specific
shaping, but do not use these to rescue an unvalidated training setup.

## W&B Success Calibration

For historical comparison runs, success thresholds, and legacy-checkpoint replay
compatibility, read `references/wandb-success-calibration.md`.

## Euler Runtime

Read [references/euler-runtime.md](references/euler-runtime.md) for the current workspace, module and library exports, verified runtime-check path, monitoring commands, and known failure signatures. Treat package pins and node families there as runtime-checked values, not timeless defaults.

### Storage targets

Resolve storage from the selected Unix account; do not embed a historical
username in a new launcher. All Euler homes have a 50 GB hard cap. On
2026-07-20 a Terra job died with `Disk quota exceeded` because `WANDB_DIR` and
checkpoints were written under home. Home is now for credentials and small
configuration only.

| Artifact | Path | Caveat |
|---|---|---|
| Reproducible code snapshots | `/cluster/scratch/$TERRA_EULER_USER/codex_terra_edge_validation/` | Keep out of home; rebuild or restage after scratch purge. |
| Checkpoints, `WANDB_DIR`, run logs | `/cluster/scratch/$TERRA_EULER_USER/codex_terra_edge_runs/` | NEVER write these to `/cluster/home`. Scratch files inactive for ~15 days are purged. |
| Final checkpoints, tars, venvs | Writable `/cluster/project/rsl/$TERRA_EULER_USER/` or account work storage | Verify live: `alesweber` has project storage but no `/cluster/work/rsl/alesweber` as of 2026-08-12. |
| Dataset (read-only) | `/cluster/project/rsl/alesweber/TerraProject/...` | Existing project space; do not copy into home. |

Before staging, verify the named SSH alias returns the selected `id -un`, its
`$HOME`, scratch writability, project/runtime readability, and current `lquota`.
Use a staging-only launcher mode when available; it may make read-only Slurm
association/partition/GPU-inventory queries, but must not call W&B or create a
job.
A new account needs its own finite update-1 smoke because private scratch makes
another account's smoke artifacts unavailable. Scratch is purged after
inactivity, so anything needed long-term must be copied to persistent storage
or be rebuildable. Do not put the training venv or only final artifact copy on
scratch.

## Local Performance And Failure History

For local CPU/GPU environments, capacity and throughput measurements, retained or
reverted optimizations, and known failure signatures, read
`references/performance-history.md`.

## Why Compile Is Slow

The first update compiles a large shape-specialized JAX/XLA rollout and PPO graph, so a quiet GPU before update 1 is not by itself a hang. Read [references/jax-compile.md](references/jax-compile.md) for diagnosis and mitigation before changing training code.

## References

- [references/resmap64-current-run.md](references/resmap64-current-run.md): paired ResMap64 R1/R2 launch, gates, timeout semantics, and monitoring.
- [references/wandb-success-calibration.md](references/wandb-success-calibration.md): historical policy baselines, success thresholds, and legacy replay.
- [references/performance-history.md](references/performance-history.md): local GPU setup, throughput studies, reverted probes, and failure history.
- [references/euler-runtime.md](references/euler-runtime.md): Slurm snippets, runtime checks, and failure signatures.
- [references/jax-compile.md](references/jax-compile.md): why first-update compile is slow and how to reduce test latency.
