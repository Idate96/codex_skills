---
name: terra-cluster-training
description: "Compatibility route for Terra PPO on Euler: validation, Slurm launch/monitoring, W&B comparison, and training documentation."
---

# Terra Cluster Training Compatibility Route

Use `terra-rl` for the current Terra/terra-baselines training workflow, including
validation gates, GPU selection, CUDA/JAX preflight, first-update proof, W&B
comparison, and experiment ledgers.

This compatibility skill remains so older prompts about "Terra cluster
training" still route correctly. Do not copy resource requests, package pins,
or PPO defaults from historical runs without checking the active repositories
and `terra-rl`; those details change independently.

## Minimum Cluster Contract

1. Inspect both active repositories and preserve unrelated changes.
2. Load `terra-rl` and use its current validation and GPU-allocation rules.
3. Compare W&B runs by saved config and summary, not display name.
4. Run CPU gates and a GPU runtime preflight before training.
5. Require a completed first update before calling a job healthy.
6. Record the exact command, Slurm job, W&B identity, and terminal state in the
   current experiment ledgers.

Never use a generic GPU request by itself. The current `terra-rl` contract
restricts training to verified RTX 3090/4090 allocations unless the user
explicitly approves another GPU family.
