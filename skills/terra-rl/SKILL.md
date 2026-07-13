---
name: terra-rl
description: "Run Terra RL training: validation, Euler Slurm, W&B comparison, PPO configs, JAX/CUDA preflight, smoke tests, and failure diagnosis."
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

Use `scripts/euler/check_jax_runtime.py` inside the Slurm allocation after exporting the same
environment that the training job will use.

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

Use only NVIDIA GeForce RTX 3090 or NVIDIA GeForce RTX 4090 nodes for Terra RL training unless the
user explicitly asks for a different GPU. Treat all other GPU types as invalid for baseline or
mask validation runs, including Quadro RTX 6000, Titan RTX, RTX 2080 Ti, A100, and RTX Pro 6000.

Known Euler node families:

- RTX 3090: `eu-g4-[001-032]`
- RTX 4090: `eu-g6-[001-080]`

Do not use generic GPU requests such as `#SBATCH --gpus=gpu:4`, `#SBATCH --gpus=4`, or the old
`train_cluster.sh` style request by itself. `gpuhe.*` contains multiple GPU families, and generic
requests have landed Terra jobs on Quadro RTX 6000 and RTX 2080 Ti nodes. If using a generic count,
combine it with a 3090/4090 node family restriction, for example:

```bash
#SBATCH --partition=gpuhe.120h
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=16
#SBATCH --gpus=4
#SBATCH --nodelist=eu-g4-[001-032],eu-g6-[001-080]
```

For a single family, use only that node list:

```bash
# 3090-only
#SBATCH --nodelist=eu-g4-[001-032]

# 4090-only
#SBATCH --nodelist=eu-g6-[001-080]
```

Every Slurm script must hard-fail before the JAX preflight and before W&B training if the actual
allocation is not 3090/4090-only. Put this guard after loading modules and before
`check_jax_runtime.py`:

```bash
EXPECTED_GPUS="${EXPECTED_GPUS:-4}"
GPU_NAMES="$(nvidia-smi --query-gpu=name --format=csv,noheader)"
echo "Allocated GPUs:"
echo "$GPU_NAMES"

GPU_COUNT="$(printf "%s\n" "$GPU_NAMES" | sed '/^$/d' | wc -l)"
if [ "$GPU_COUNT" -ne "$EXPECTED_GPUS" ]; then
  echo "Expected $EXPECTED_GPUS GPUs, got $GPU_COUNT" >&2
  exit 42
fi

if printf "%s\n" "$GPU_NAMES" | grep -Evq 'NVIDIA GeForce RTX (3090|4090)$'; then
  echo "Refusing to train on non-3090/4090 GPU allocation" >&2
  exit 43
fi
```

After submission, verify the allocation with both:

```bash
sacct -j "$JOB_ID" --format=JobID,State,Elapsed,NodeList%20,ReqTRES%80,AllocTRES%80 -P
tail -n 80 "$LOG_PATH"
```

If `AllocTRES` or `nvidia-smi` shows anything other than `nvidia_geforce_rtx_3090` or
`nvidia_geforce_rtx_4090`, cancel the job and relaunch. Do not count runtime preflight or W&B
history from a disallowed GPU allocation as validation evidence.

## Launch Workflow

1. Check both repos with `git status --short --branch`; never revert unrelated dirty files.
2. If copying to Euler, sync only intended changes into the isolated workspace.
3. Run fast isolated gates before any Slurm launch:
   - `python3 -m py_compile` for changed Python files.
   - `scripts/validation/validate_edge_mask_changes.py --case training-accounting --jax-platforms cpu`
   - `scripts/validation/validate_edge_mask_changes.py --case ppo-mask --jax-platforms cpu`
   - `--case model-policy --jax-platforms cpu`
   - `--case model-edge-no-mask --jax-platforms cpu`
   - `--case model-critic-affordance-shapes --jax-platforms cpu`
   - `--case checkpoint-config-restore --jax-platforms cpu`
   - `--case timeout-bootstrap-value --jax-platforms cpu`
   - `--case gae-timeout-bootstrap --jax-platforms cpu`
   - `--case state-action-mask --jax-platforms cpu --disable-jit`
   - `--case state-step-dispatch --jax-platforms cpu --disable-jit`
   - `--case synthetic-env-action-mask --jax-platforms cpu --disable-jit`
   - `--case env-action-mask --jax-platforms cpu --dataset-path /home/lorenzo/moleworks/terra_data/train --dataset-size 1`
   - `--case synthetic-step-fast-reset --jax-platforms cpu --disable-jit`
   - `--case synthetic-batch-step-fast-reset --jax-platforms cpu`
   - `--case env-episode-progress --jax-platforms cpu --disable-jit`
4. Select only RTX 3090/4090 nodes and include the hard GPU-type guard from this skill in the Slurm
   job before any JAX or W&B training command.
5. Run the GPU runtime preflight from this skill in the Slurm job before training.
6. For new runtime/config changes, run a first-update smoke with W&B disabled before a full run.
7. Launch the production job only after the smoke evidence is real: update 1 completed, not only
   dataset/model initialization.
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

Use the current isolated workspace unless the user explicitly requests another one:

```bash
WORK=/cluster/home/lterenzi/codex_terra_edge_validation
VENV=/cluster/scratch/lterenzi/codex_terra_edge_venv
module load stack/2024-06 cuda/12.1.1

SITE_PACKAGES=$("$VENV/bin/python" - <<'PY'
import site
print(site.getsitepackages()[0])
PY
)

export PYTHONPATH="$WORK/terra:$WORK/terra-baselines:${PYTHONPATH:-}"
export XLA_PYTHON_CLIENT_PREALLOCATE=false
export LD_LIBRARY_PATH="$SITE_PACKAGES/nvidia/cudnn/lib:$SITE_PACKAGES/nvidia/cuda_cupti/lib:$SITE_PACKAGES/nvidia/cublas/lib:$SITE_PACKAGES/nvidia/cuda_nvrtc/lib:$SITE_PACKAGES/nvidia/nccl/lib:${LD_LIBRARY_PATH:-}"
```

Known package pins for this stack:

```bash
python -m pip install --force-reinstall --no-cache-dir \
  numpy==1.26.4 scipy==1.12.0 ml-dtypes==0.5.4 opt-einsum==3.3.0 \
  chex==0.1.86 orbax-checkpoint==0.5.16 nest_asyncio
python -m pip install --no-cache-dir \
  nvidia-cudnn-cu12==8.9.7.29 nvidia-cuda-cupti-cu12==12.1.105 \
  nvidia-nccl-cu12==2.19.3
```

Run the preflight in the allocation:

```bash
"$VENV/bin/python" "$WORK/terra-baselines/scripts/euler/check_jax_runtime.py" --min-devices 4
```

For one-GPU comparison jobs, use `--min-devices 1`.

For a run that previously hit cuDNN autotune failures, export the mitigation before training:

```bash
export XLA_FLAGS="${XLA_FLAGS:+$XLA_FLAGS }--xla_gpu_autotune_level=0"
```

This can cost a few percent of steady throughput, so compare learning by W&B step/update rather
than wall-clock unless both sides use the same XLA flags.

## Local Performance And Failure History

For local CPU/GPU environments, capacity and throughput measurements, retained or
reverted optimizations, and known failure signatures, read
`references/performance-history.md`.

## Why Compile Is Slow

When asked why local or cluster tests are slow, use this explanation:

- The fast unit gates are quick because they avoid the full GPU training graph.
- The first `train_mixed.py` update compiles a large shape-specialized JAX/XLA program: vectorized
  Terra env reset/rollout, 32 steps, PPO update epochs/minibatches, conv model forward/backward,
  optimizer, and pmap collectives.
- Large static shapes such as `num_envs_per_device=1024`, map tensors around `64x64`, and rollout
  batches dominate compile before GPU utilization appears.
- The logs already showed XLA constant-folding work inside Terra state code and cuDNN convolution
  autotune before any update completed.
- Each fresh Slurm process can pay this compile cost again unless a persistent XLA cache is set up.

See `references/jax-compile.md` for mitigation ideas before changing training code.

## References

- `references/resmap64-current-run.md`: paired ResMap64 R1/R2 launch, gates, timeout semantics, and monitoring.
- `references/wandb-success-calibration.md`: historical policy baselines, success thresholds, and legacy replay.
- `references/performance-history.md`: local GPU setup, throughput studies, reverted probes, and failure history.
- `references/euler-runtime.md`: Slurm snippets, runtime checks, and failure signatures.
- `references/jax-compile.md`: why first-update compile is slow and how to reduce test latency.
