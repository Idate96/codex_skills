# JAX Compile Reference

## What Is Slow

The slow part is not the isolated validation tests. The slow part is the first compiled
`train_mixed.py` update.

That first call asks XLA to lower and optimize a large program at fixed shapes:

- vectorized Terra env reset/step;
- rollout scan over `num_steps=32`;
- PPO advantage/return computation;
- model forward/backward with convolution layers;
- minibatch/update-epoch loop;
- optimizer update;
- pmap collectives for multi-GPU.

For the default 4-GPU run, each device handles `1024` envs and the rollout batch per device is
`1024 * 32`. The map tensors are around `64x64`, and several masks and reductions are compiled as
part of the state transition. Logs from the failed runs showed XLA constant folding around Terra
state reductions before any update completed.

GPU utilization can remain `0%` while compilation is CPU-bound. Seeing GPU memory allocated only
means XLA/JAX initialized CUDA, not that a training update is running.

## Why Small Smokes Can Still Be Slow

Reducing `num_envs_per_device` helps execution size but does not remove most of the control-flow,
state-update, model, optimizer, and shape-specialized compile work. A 128-env one-update smoke can
still spend tens of minutes compiling if it includes the full trainer graph.

## Better Test Ladder

Use this ladder instead of jumping straight to a full run:

1. CPU unit gates with JIT disabled where possible.
2. `check_jax_runtime.py` to prove CUDA/cuDNN/NCCL can execute tiny compiled programs.
3. A very small training smoke with W&B disabled and explicit timeout.
4. Production launch only after update 1 completes.

If step 3 is too slow, do not keep shrinking blindly. Add compile diagnostics or isolate which
part of the trainer graph is compiling slowly.

## Mitigations To Consider

- Use a persistent XLA cache for repeated Slurm jobs if compatible with the current JAX version.
- Keep smoke shapes separate from production shapes and record them clearly.
- Disable cuDNN autotune for diagnosis: `XLA_FLAGS=--xla_gpu_autotune_level=0`.
- Move static map/mask preprocessing out of jitted training where possible.
- Split very large jitted functions if compile time dominates iteration.
- Avoid eval/checkpoint/logging paths in smoke jobs: `--log_eval_interval 0 --checkpoint_interval 0 --eval_episodes 0`.
- Time and log each phase: dataset load, model init, first compile, first update, first W&B metric.

## 2026-05-11 Terra Step Finding

On local CPU, a single synthetic 32x32 `TerraEnv.step` compiled in about 115 s when the action was
dynamic, but the same no-reset body compiled in about 4.4 s when `DO_NOTHING` was closed over as a
constant. The dynamic step compiles all primitive action branches.

The old dispatcher had 16 tracked/wheeled branches and duplicated the expensive `DO` handler. The
8-way primitive dispatcher removed that duplicate and reduced the one-env step compile to about
73.7 s. When step compile regresses, inspect action dispatch duplication before blaming CUDA.
