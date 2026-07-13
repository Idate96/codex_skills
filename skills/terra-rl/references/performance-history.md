# Terra RL Local Performance And Failure History

Read this reference for local CUDA environment setup, throughput/capacity tuning, optimization history, or known failure signatures.

## Contents

- [Local environments](#local-environments)
- [Performance triage](#performance-triage-after-fast-reset)
- [Failure lessons](#failure-lessons)

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
