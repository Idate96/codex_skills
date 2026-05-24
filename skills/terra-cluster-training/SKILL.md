---
name: terra-cluster-training
description: Launch, validate, compare, and monitor Terra / terra-baselines PPO training jobs on the Euler cluster. Use when Codex is asked to start Terra policy training, compare W&B hyperparameters, run single-excavator or mixed-agent `train_mixed.py` jobs, prepare Slurm scripts, check `squeue`/`sacct` logs, or document/default Terra cluster training parameters.
---

# Terra Cluster Training

## Core Workflow

1. Work from the active local checkouts, usually:
   - `/home/lorenzo/moleworks/terra`
   - `/home/lorenzo/moleworks/terra-baselines`
2. Confirm branches and dirtiness with `git status --short --branch` in both repos. Never overwrite unrelated dirty files.
3. If matching a W&B run, fetch `run.config`, `run.summary`, and `wandb-metadata.json`; trust config fields over display names.
4. Run fast gates before submitting training:
   - `python3 -m py_compile` for changed Python files.
   - `scripts/validation/validate_edge_mask_changes.py --case ppo-mask`
   - `--case terminal-backfill`
   - `--case model-policy`
   - `--case state-action-mask --disable-jit`
   - `--case synthetic-env-action-mask --agent-types "(0,)" --action-types "(0,)" --disable-jit`
5. Sync only the intended files to an isolated Euler workspace unless the user explicitly wants to mutate a shared project checkout.
6. For GPU jobs, verify the Slurm script runs a GPU preflight after validation and before `train_mixed.py`.
7. Submit with `sbatch`, then report job id, log path, exact command, W&B run identity if available, and first log evidence.
8. Monitor with `squeue`, `sacct`, `tail`, and `seff`; distinguish queued, compiling, running updates, failed, completed, and timed out.

## Default Single-Excavator Policy Run

Use this as the current default unless the user asks for another preset:

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

- Agent/action: `agent_types=[0]`, `action_types=[0]` (tracked excavator).
- Map: `foundations_real_ring`, `max_steps=550`.
- Total envs: `4 * 1024 = 4096`.
- PPO: `lr=3e-4`, `gamma=0.9984`, `gae_lambda=0.95`, `clip_eps=0.2`, `ent_coef=0.06`, `vf_coef=2.0`, `max_grad_norm=0.5`.
- Entropy schedule: `0.15 -> 0.005` over `9500` updates.
- Rewards: `dump_bonus_mult=0.5`, `excavator_relocate_dumped_mult=1.5`, `excavator_relocate_dug_dirt_mult=1.5`.
- Terminal backfill: disabled by default; use `--backfill_terminal_reward` only when intentionally testing multi-agent terminal-credit behavior.

## Slurm Template For Current Default

Use this resource shape for a full run matching W&B `54tk3k5x`:

```bash
#SBATCH -n 1
#SBATCH --cpus-per-task=4
#SBATCH --gpus=gpu:4
#SBATCH --partition=gpu.24h
#SBATCH --time=24:00:00
#SBATCH --mem-per-cpu=8G
#SBATCH --job-name=terra-edge-solo
#SBATCH --output=/cluster/home/lterenzi/codex_terra_edge_validation/logs/%j_edge_mask_solo.out
```

Load CUDA and expose the venv-installed CUDA libraries before importing JAX:

```bash
module load stack/2024-06 cuda/12.1.1

WORK=/cluster/home/lterenzi/codex_terra_edge_validation
VENV=/cluster/scratch/lterenzi/codex_terra_edge_venv
SITE_PACKAGES=$("$VENV/bin/python" - <<'PY'
import site
print(site.getsitepackages()[0])
PY
)

export PYTHONPATH="$WORK/terra:$WORK/terra-baselines:${PYTHONPATH:-}"
export XLA_PYTHON_CLIENT_PREALLOCATE=false
export LD_LIBRARY_PATH="$SITE_PACKAGES/nvidia/cudnn/lib:$SITE_PACKAGES/nvidia/cuda_cupti/lib:$SITE_PACKAGES/nvidia/cublas/lib:$SITE_PACKAGES/nvidia/cuda_nvrtc/lib:${LD_LIBRARY_PATH:-}"
export WANDB_ENTITY=aless-weber-eth
export WANDB_PROJECT=mixed-agents
export WANDB_DIR="$WORK/wandb"
export DATASET_PATH=/cluster/project/rsl/alesweber/TerraProject/terra/data/terra/train
export DATASET_SIZE=600
```

After the CPU fast gates, require this GPU preflight before training:

```bash
"$VENV/bin/python" - <<'PY'
import jax
print("Training JAX devices:", jax.devices(), flush=True)
if jax.local_device_count() < 4:
    raise SystemExit(f"Expected 4 JAX devices, got {jax.local_device_count()}")
PY
```

Do not set `XLA_FLAGS=--xla_force_host_platform_device_count=4` for real GPU training. Use that
only for CPU-only checks.

## W&B Comparison Notes

For W&B URLs like `https://wandb.ai/aless-weber-eth/mixed-agents/runs/<id>`:

```bash
curl -n -sS 'https://api.wandb.ai/graphql' \
  -H 'Content-Type: application/json' \
  --data-binary @- <<'JSON'
{"query":"query Run($entity:String!, $project:String!, $name:String!) { project(name:$project, entityName:$entity) { run(name:$name) { name displayName state config createdAt updatedAt historyLineCount summaryMetrics } } }","variables":{"entity":"aless-weber-eth","project":"mixed-agents","name":"RUN_ID"}}
JSON
```

Also fetch metadata:

```bash
curl -n -sS -L \
  "https://api.wandb.ai/files/aless-weber-eth/mixed-agents/RUN_ID/wandb-metadata.json"
```

Important: legacy run names and tags may mention skid-steer even when the actual config is
`agent_types_override=[0]`. Treat `run.config`, `summary.agents/num_agents`, and metadata GPU
fields as evidence.

## Euler Environment

Known working isolated validation workspace:

- `WORK=/cluster/home/lterenzi/codex_terra_edge_validation`
- `VENV=/cluster/scratch/lterenzi/codex_terra_edge_venv`

For CPU-only fast gates:

```bash
module load stack/2024-06 python/3.12.8
export PYTHONPATH="$WORK/terra:$WORK/terra-baselines:${PYTHONPATH:-}"
export JAX_PLATFORMS=cpu
```

For 4 CPU devices during non-GPU checks:

```bash
export XLA_FLAGS=--xla_force_host_platform_device_count=4
```

For GPU training, load CUDA and verify `jax.devices()` sees GPUs before launching a full run. If
the log says `Unable to load cuDNN`, `Unable to load cuPTI`, or `Training JAX devices:
[CpuDevice(id=0)]`, install the CUDA runtime packages and export `LD_LIBRARY_PATH` as shown above.

Keep NumPy pinned below 2 for the current Flax/JAX stack; the known import-clean package set in the
Euler venv is:

```bash
python -m pip install --force-reinstall --no-cache-dir \
  numpy==1.26.4 scipy==1.12.0 ml-dtypes==0.5.4 opt-einsum==3.3.0 \
  chex==0.1.86 orbax-checkpoint==0.5.16 nest_asyncio
python -m pip install --no-cache-dir \
  nvidia-cudnn-cu12==8.9.7.29 nvidia-cuda-cupti-cu12==12.1.105
```

Known successful GPU startup markers:

- all fast validation lines print `PASS`
- `Training JAX devices: [cuda(id=0), cuda(id=1), cuda(id=2), cuda(id=3)]`
- `Mixed Agent Training - Devices: 4, Updates: 95367`
- W&B run URL appears under `aless-weber-eth/mixed-agents`
- dataset loader reads `600` maps from `foundations_real_ring`
- model initializes with `472,457` parameters
- configuration prints `Environments per device: 1024`, `Total environments: 4096`, `Training steps: 32`, `Total timesteps: 50,000,000,000`

## Reporting

When reporting a launch, include:

- W&B run compared against, if any.
- Hyperparameter match/mismatch.
- Validation commands that passed.
- Slurm job id, state, partition/resources, and output file.
- Whether the log reached dataset loading, model init, JAX compile, first update, or W&B init.
