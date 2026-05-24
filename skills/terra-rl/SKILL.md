---
name: terra-rl
description: Run, validate, debug, and document Terra RL training in the terra and terra-baselines repositories. Use for Euler Slurm jobs, W&B run comparison, PPO/default solo-excavator configs, JAX/CUDA runtime preflight, first-update smoke tests, experiment ledgers, and diagnosis of slow compilation or GPU training failures.
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

Use `aless-weber-eth/mixed-agents` as the historical comparison project. Prefer
`solo_excavator` runs as the apples-to-apples baseline; `trench_excavator` and
`trench_masked_excavator` are useful upper-bound references but use different task configs.

Closest successful `solo_excavator` references:

- `o9aewzsx`: 4 GPUs, 1024 envs/device, `eval/positive_terminations=9.56`,
  `eval/rewards=0.174`, `eval/max_reward=6.87`, `eval/DO=0.343`, `DO_NOTHING ~= 0`.
- `hcwvorkm`: 4 GPUs, 1024 envs/device, `eval/positive_terminations=3.45`,
  `eval/rewards=0.157`, `eval/max_reward=6.87`, `eval/DO=0.311`, `DO_NOTHING=0.018`.
- `o8bpdoex`: 4 GPUs, 128 envs/device, `eval/positive_terminations=4.01`,
  `eval/rewards=0.180`, `eval/max_reward=6.87`, `eval/DO=0.316`, `DO_NOTHING=0.011`.
  This run started with nonzero success, so treat it as a performance reference, not a clean
  from-scratch learning curve.

Strong non-solo references: `xjncfmr6`, `jnwj5tnj`, and `rov40bmt` reached
`eval/positive_terminations ~= 26-31`, `eval/rewards ~= 0.37-0.43`,
`eval/max_reward ~= 6.87`, and `eval/DO ~= 0.33-0.35`.

Historical replay compatibility:

- Do not judge old healthy policies only in the current `multi-agent` worktrees. `hcwvorkm`
  fails under current Terra replay (`0` positive terminations in a 64-env local check) even with
  action masking disabled, but replays healthily with training-era code.
- For `hcwvorkm`, use Terra commit `de698b7f` and terra-baselines commit `8091d3e` when trying to
  reproduce W&B behavior locally. A local training-era eval on 2026-05-11 with 64 envs, 550 steps,
  seed 123 produced `positive_terminations_per_env=7.5625`, `reward=0.1226`,
  `max_reward=6.8679`, `DO=0.2706`, and `DO_NOTHING=0.0762`.
- The single-env training-era GIF
  `/home/lorenzo/moleworks/terra-baselines/logs/local_policy_play/hcwvorkm_training_era_seed123.gif`
  terminated successfully in 61 steps with return about `8.41`.
- Old checkpoints do not store `use_action_mask`. When unpickled with current classes, plain
  `getattr(config, "use_action_mask", True)` silently inherits the new class default. For legacy
  replay, infer whether the field was saved on the instance; if it was missing, default to
  `False` to match the historical no-mask policy/eval path.

Interpret W&B signals as follows:

- First terminal signal: `eval/max_reward >= 6.8` and `eval/positive_terminations > 0`.
- A tiny one-off positive termination can be luck. Call it real only if it persists across
  several evals and has plausible `eval/avg_positive_episode_length`.
- Healthy learning: `eval/rewards > 0.01`, then trending toward `0.05+`; `eval/DO` climbing
  toward `0.25-0.35`; `eval/DO_NOTHING %` falling below about `0.05`.
- Strong solo target: `eval/positive_terminations >= 3`, `eval/rewards ~= 0.15-0.18`,
  `eval/DO ~= 0.31-0.34`, and `eval/DO_NOTHING % <= 0.02`.
- No-success signature: `eval/positive_terminations == 0`, `eval/max_reward < 1`,
  `eval/DO ~= 0.05-0.10`, and `eval/DO_NOTHING % ~= 0.20+`.

`eval/positive_terminations` is normalized by `num_envs_per_device * num_devices`, so it is
comparable across GPU counts; it can exceed 1 because multiple successful episodes may happen per
eval environment over the eval horizon. `progress/episode_completion_rate` is not the same as task
success because it includes ordinary done/timeouts; use `eval/positive_terminations` plus
`eval/max_reward` for clean success calls.

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

## Local Environments

Use this local CPU-only environment for quick gates:

```bash
source /home/lorenzo/moleworks/.venv-terra-uv/bin/activate
export JAX_PLATFORMS=cpu
export PYTHONPATH=/home/lorenzo/moleworks/terra:/home/lorenzo/moleworks/terra-baselines
```

For local RTX 4090 / 24 GB GPU tests, use the separate CUDA-enabled environment:

```bash
VENV=/home/lorenzo/moleworks/.venv-terra-gpu-uv
SITE_PACKAGES=$("$VENV/bin/python" - <<'PY'
import site
print(site.getsitepackages()[0])
PY
)
export PYTHONPATH=/home/lorenzo/moleworks/terra:/home/lorenzo/moleworks/terra-baselines
export XLA_PYTHON_CLIENT_PREALLOCATE=false
export XLA_PYTHON_CLIENT_MEM_FRACTION=0.95
export LD_LIBRARY_PATH="$SITE_PACKAGES/nvidia/cudnn/lib:$SITE_PACKAGES/nvidia/cuda_cupti/lib:$SITE_PACKAGES/nvidia/cublas/lib:$SITE_PACKAGES/nvidia/cuda_nvrtc/lib:$SITE_PACKAGES/nvidia/nccl/lib:${LD_LIBRARY_PATH:-}"
```

As of 2026-05-11, the local GPU venv's `activate` script may still point
`VIRTUAL_ENV` and console-script shebangs at `.venv-terra-uv` because the env was copied from the
CPU venv. For GPU checks, invoke `"$VENV/bin/python"` directly and verify `jax.default_backend()`
is `gpu`; do not trust `source "$VENV/bin/activate"` alone.

If the GPU env is missing, create it from the known-good CPU env and install the matching CUDA JAX
wheel before running capacity or training checks:

```bash
cp -a /home/lorenzo/moleworks/.venv-terra-uv /home/lorenzo/moleworks/.venv-terra-gpu-uv
/home/lorenzo/moleworks/.venv-terra-gpu-uv/bin/python -m pip install -U \
  "jax[cuda12]==0.4.26" \
  -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html
```

Verify the selected env explicitly:

```bash
"$VENV/bin/python" - <<'PY'
import jax
print(jax.devices())
assert jax.default_backend() == "gpu" or jax.default_backend() == "cuda"
PY
```

Local throughput settings after the 2026-05-11 fast-reset and pmap-donation optimizations:

- Use `--num_envs_per_device 1536` for peak local RTX 4090 / 24 GB throughput tests. It measured
  about 60.8k steady steps/s after pmap runner-state donation.
- Use `--num_envs_per_device 1280` when you want nearly peak local throughput with less memory
  pressure. It measured about 60.0k steady steps/s after donation.
- `1024` remains the safer default for Euler 24 GB RTX 3090/4090 runs and for comparability with
  existing W&B runs.
- `2048` is not the local throughput peak; it measured about 58.4k steady steps/s after donation.
  Do not increase env count unless the measured steps/s increases.
- `4096` and above are slower for training throughput on the local 4090; `5120` OOMed after
  fast-reset.

## Performance Triage After Fast-Reset

After the 2026-05-11 fast-reset optimization, do not assume the environment wrapper is still the
main limiter. The current local 1280-env evidence is:

- `wrap_state_only`, `state_to_obs_only`, and `action_mask_edge_only` are about 255k-279k steps/s.
- Random env-step hot repeats are about 64k-79k steps/s.
- Policy-forward-plus-env-step is about 76k steps/s.
- Full PPO default training was about 51k steps/s before pmap donation and about 60k steps/s after
  donation at the local peak.

First inspect PPO update/backprop/minibatching before changing more env code. Check the
`train_mixed.py` path that swaps `[steps, envs]` to `[envs, steps]`, shuffles envs with
`jnp.take`, reshapes into minibatches, and runs `ppo_update_networks`.

Keep the multi-GPU accounting fix:

- `config.num_envs` already equals `num_envs_per_device * num_devices`.
- `env_steps_per_update` should be `num_steps * config.num_envs`.
- `performance/steps_per_second` should multiply iterations/s by `env_steps_per_update`, not by
  `env_steps_per_update * num_devices`.
- `num_updates` should be `total_timesteps // env_steps_per_update` when `total_timesteps` is
  global env steps.
- Old 4-GPU W&B `performance/steps_per_second` values from this code path should be divided by 4;
  local 1-GPU values are unaffected.

Keep pmap runner-state donation unless it fails a real training smoke:

- `_update_step` should use `jax.pmap(..., donate_argnums=(0,))` for `runner_state`.
- This is valid because the loop overwrites `runner_state` after each update and does not reuse the
  donated input.
- Local validation on 2026-05-11 passed a 16-env GPU smoke, 1024/1280/1536/2048 env sweeps, and the
  CPU edge-mask gates.

Do not retry the manual categorical replacement unless new profiling specifically implicates TFP.
On 2026-05-11, a manual JAX categorical matched TFP log-prob/entropy but measured only about 61.1k
steps/s at 1536 envs/GPU versus about 60.8k with TFP after donation. That is within noise and would
change seeded sample streams, so it was reverted.

Do not retry flat env/time PPO minibatches as a generic cleanup. The 2026-05-11 probe passed a
synthetic old-vs-flat PPO math parity check and a tiny GPU smoke, but measured only about 60.25k
steps/s at 1536 envs/GPU, below the retained donation baseline, so it was reverted.

Do not retry a simple `jnp.any(timestep.done)` guard around `curriculum_manager.update_cfgs` as a
generic env optimization. It passed selected no-done parity but measured only about 60.06k steps/s
at 1536 envs/GPU, below the retained donation baseline, so it was reverted.

Oracle's 2026-05-11 PPO review concluded that no obvious large correctness-preserving PPO-side win
remains after fast-reset and pmap donation. The next safe probes, if more optimization is needed,
are packed model inputs/preclipped action maps, lean transition storage, host-transfer gating, and
solo-agent model specialization. Treat `num_minibatches=8`, `num_steps=64`, and `update_epochs=1`
as semantic speed/learning experiments, not default config changes.

Oracle's 2026-05-11 architecture review agreed that easy correctness-preserving gains are mostly
exhausted. The next ranked probes are: strengthen fast-reset partial/all-done parity, remove
duplicated `_is_done` work between reward and step, add a value-only bootstrap path, sweep
`lax.scan` unroll factors, profile forced action classes, and treat `pmap -> shard_map/pjit` as a
larger architecture experiment.

Host-transfer gating is retained in `train.py` and `train_mixed.py`: unreplicate
`loss_info`/`runner_state` only when train logging, checkpointing, eval, or final return needs host
values. This mainly cleans up logging-disabled local sweeps; it is not a compiled-update steps/s
win.

Do not retry packed model-input transition storage as a generic PPO update optimization. It passed
exact raw-vs-packed PPO update parity and GPU smokes, but measured about 60.59k/59.71k/58.43k
steps/s at 1536/1792/2048 envs/GPU, so it did not beat the retained 1536-env donation baseline and
was reverted.

Do not retry lean transition storage as a generic cleanup. Setting unused transition leaves to
`None` passed gates and a tiny GPU smoke, but measured about 60.41k steps/s at 1536 envs/GPU, below
the retained donation baseline, so it was reverted.

Do not retry static solo-agent model specialization without a new profile. It produced tiny
generic-vs-specialized output differences around 1e-10, passed a smoke, but measured about 60.60k
steps/s at 1536 envs/GPU, below the retained donation baseline, so it was reverted.

Do not retry scalar RNG split relocation without a new profile. Letting `TerraEnvBatch.step` accept
a scalar key and splitting only in reset/curriculum branches passed scalar-vs-batched no-done parity
and a GPU smoke, but measured about 60.18k steps/s at 1536 envs/GPU, below the retained donation
baseline, so it was reverted. Trainer/profile call sites should split per-env reset keys before
`env.step`.

Do not retry value-only bootstrap as a generic cleanup. Replacing the post-rollout
`select_action_ppo` bootstrap with a value-only model apply passed CPU gates and a tiny GPU smoke,
but measured about 60.62k steps/s at 1536 envs/GPU, below the retained donation baseline, so it was
reverted.

Do not retry reward/done reuse as a generic cleanup. Threading `_get_reward`'s `done`/`task_done`
result into `step_no_reset` passed CPU gates and a tiny GPU smoke, but measured about 60.54k
steps/s at 1536 envs/GPU, below the retained donation baseline, so it was reverted.

Do not retry rollout `jax.lax.scan(unroll=2)` without a new reason. A 16-env local GPU smoke
jumped to about 21 GB peak memory and slowed to about 12.4 steps/s, so the probe was reverted
without a 1536-env run.

Use `scripts/profile_rollout_components.py --sections forced_action_env_step_only` when deciding
whether to optimize action-specific env code. On 2026-05-11, forced-action profiling at 1280
envs/GPU showed hot repeats around cabin ~130k steps/s, do-nothing ~110k, and DO ~92k. DO is the
slow branch, but still faster than full PPO, so only keep simple DO-specific changes. A cone-reuse
probe in `_handle_dig`/`_handle_dump` did not improve forced DO and was reverted.

Use `scripts/profile_rollout_components.py --sections build_update_inputs_only,ppo_update_only,full_update_step`
to split hot full PPO update time. On 2026-05-11 at 1280 envs/GPU, build-update-inputs/rollout was
about 0.56s per update (~73k steps/s), PPO update-only was about 0.136s (~301k steps/s), and full
update was about 0.685s (~60k steps/s). This means the remaining correctness-preserving bottleneck
is mostly rollout/model/env, not the PPO minibatch/backprop loop.

Do not retry a foundation-border workspace-touch guard without a new profile. Skipping
foundation-border alignment when the current workspace did not touch a border tile slightly
improved forced DO microprofiles, but full PPO at 1536 envs/GPU measured about 60.45k steps/s,
below the retained pmap-donation baseline of about 60.8k, so the probe was reverted.

Treat these as controlled learning-speed tradeoffs, not defaults:

- `update_epochs=1` measured about 66.5k steps/s at 1280 envs/GPU.
- `num_minibatches=8` measured about 61.1k steps/s.
- `num_steps=64` measured about 61.0k steps/s.

Only keep such changes after comparing W&B learning curves by update count and env steps. Watch
`eval/positive_terminations`, `eval/rewards`, `progress/episode_completion_rate`,
`explained_variance`, `value_loss`, `entropy`, and action percentages.

## Failure Lessons

- `65844986` reached `Training: 0/95367` and then failed because NCCL was missing:
  `Unable to load NCCL library. Multi-GPU collectives will not work.` W&B had no summary/history.
- `65846373` reached `Training: 0/1525878` and then failed with
  `CUDNN_STATUS_INTERNAL_ERROR` in convolution backward input. W&B had no summary/history.
- `66138248` was the no-mask action-mask A/B run (`f43doigo`). Slurm still showed `RUNNING`,
  but the log had `CUDNN_STATUS_INTERNAL_ERROR`, the Python process was sleeping, and GPU
  utilization was 0%. Treat this as a failed runtime allocation, not a no-mask learning result.
  Cancel stuck jobs in this state.
- `66145980` proved the no-mask 1024-env shape after `XLA_FLAGS=--xla_gpu_autotune_level=0`:
  one PPO update completed with `--disable_action_mask` and W&B disabled. Use this smoke pattern
  before relaunching any full job after a cuDNN autotune failure.
- `66148171` replaced the failed no-mask run (`ho3duz3w`) with autotune disabled, completed update
  1, wrote a checkpoint, and continued at about 13.6k-13.9k steps/s on an RTX 2080 Ti. The masked
  comparison `66138246` (`9wlth93n`) ran on another RTX 2080 Ti at about 14.2k steps/s. Treat these
  as historical runtime evidence only; new Terra RL launches must use 3090/4090 allocations.
- `66129078` was cancelled after 31 minutes stuck at first-update compile with 128 envs/device;
  this is not a passed smoke. It showed that dataset/model startup is still far from proving a
  train update.
- Local compile isolation on 2026-05-11 showed one synthetic 32x32 dynamic-action `TerraEnv.step`
  compiled in about 115 s on CPU on `origin/multi-agent`, while closing over constant `DO_NOTHING`
  compiled the no-reset body in about 4.4 s. The real step compiles every action branch.
- Refactoring `State._step` from duplicated tracked/wheeled 16-way dispatch to one 8-way primitive
  dispatch removed duplicate `DO`/dig/dump compilation and reduced local one-env step compile to
  about 73.7 s. Keep the `state-step-dispatch` gate when touching action routing.
- The trainer used to run `jax.clear_caches()` at update `0`, which forced update `1` to retrace
  immediately. Keep cache clearing on completed intervals only: `(i + 1) % cache_clear_interval == 0`.
  On the local 4090, this reduced 4-update wall time by about 1.6x at 1024/2048/4096 envs/GPU
  without changing steady post-compile throughput.
- `TerraEnvBatch.step` used to sample reset maps and build reset candidates every step even when no
  env was done. The fast-reset path now runs `step_no_reset` first and only enters reset-map
  sampling under `jax.lax.cond(jnp.any(done), ...)`. On the local 4090 at 2048 envs/GPU, direct
  batch env-step throughput improved from about 36.7k to 60.6k steps/s when `any_done_rate=0.0`;
  full PPO 2048-env throughput improved from about 39.3k to 47.6k steps/s. Keep
  `step_unconditional_reset_candidates` as a reference path until mixed/forced-reset parity is cheap.
- W&B state `crashed` plus empty history means no policy result. Report it as environment/runtime
  failure, not RL behavior.

If cuDNN autotune fails, try a tiny runtime check first, then a first-update smoke with:

```bash
export XLA_FLAGS=--xla_gpu_autotune_level=0
```

Mask/no-mask can produce slightly different JAX/XLA graphs, which can make cuDNN autotune choose
different convolution algorithms. If one side fails and the other runs, do not infer that action
masking fixed cuDNN or that the failed side learned worse; prove the failed side with a smoke and
relaunch.

On Euler, generic GPU jobs may allocate disallowed nodes. Observed failures include one-GPU jobs on
RTX 2080 Ti and a clean four-GPU baseline on Quadro RTX 6000. Always report GPU type from
`sacct AllocTRES` and `nvidia-smi`, not from the job name or assumptions. New Terra RL validation
runs must hard-fail unless all allocated GPUs are NVIDIA GeForce RTX 3090 or RTX 4090.

Do not use `XLA_FLAGS=--xla_force_host_platform_device_count=4` for real GPU training; it is only
for CPU-only checks.

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

- `references/euler-runtime.md`: Slurm snippets, runtime checks, and failure signatures.
- `references/jax-compile.md`: why first-update compile is slow and how to reduce test latency.
