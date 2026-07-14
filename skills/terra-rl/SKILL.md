---
name: terra-rl
description: "Run Terra RL training across `terra` and `terra-baselines`. Use for validation, Euler Slurm, W&B comparison, PPO configuration, JAX/CUDA preflight, first-update smoke tests, and failure diagnosis."
---

# Terra RL

Use this skill for Terra policy training work spanning:

- `/home/lorenzo/moleworks/terra`
- `/home/lorenzo/moleworks/terra-baselines`
- Euler workspace `/cluster/home/lterenzi/codex_terra_edge_validation`
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
- Euler training: `WORK=/cluster/home/lterenzi/codex_terra_edge_validation` with
  `VENV=/cluster/scratch/lterenzi/codex_terra_edge_venv`. Use this for real Slurm jobs and W&B
  training runs.

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
7. Launch a production Slurm/W&B job only when the user requested real training and the smoke evidence is real: update 1 completed, not only dataset/model initialization. A status, diagnosis, or config-review request does not authorize a new expensive run.
8. For A/B comparisons, verify each side independently past update 1. Do not count a Slurm
   `RUNNING` job as healthy until the log shows completed updates.
9. For performance/architecture work, ask Oracle with the relevant diffs and measured logs, not
   only at final review. Ask it to look for missed code hotspots, algorithmic options, profiling
   strategy, data layout changes, JAX/XLA architecture issues, and RL-level speed/learning
   tradeoffs; then verify recommendations locally before retaining changes.
10. Keep `docs/EXPERIMENTS_RUNNING.md` and `docs/EXPERIMENTS_LOG.md` current with job ids, W&B ids,
   exact failure signatures, and whether W&B has real history.

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
