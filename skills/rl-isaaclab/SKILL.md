---
name: rl-isaaclab
description: "Route Moleworks IsaacLab RL work. Use as the default entry point for smoke tests, Euler or Vast training, sync, playback, benchmarking, and experiment tracking."
---

# RL + IsaacLab Router

Use this as the default entry point for `moleworks_ext` RL work. Keep it thin and load narrower skills for the specific workflow step you need.

## Repo Root

- Prefer the active `moleworks_ext` checkout the user is already using.
- If you need a default on this machine, start from `/home/lorenzo/moleworks/moleworks_ext`.
- Read repo `AGENTS.md` plus `docs/AI_RESEARCHER_WORKFLOW.md` before changing workflow or commands.

## Always-Loaded Rules

- Any script that touches Isaac Sim or IsaacLab must run via `/workspace/isaaclab/isaaclab.sh -p`.
- Local smoke first, with W&B disabled.
- Real training runs use W&B and should be tracked in the experiment docs.
- Do not recommend checkpoints from sync alone. Require benchmark evidence.
- For UGEP generated/randomized excavator training, benchmark against the
  same-contract real-machine-trained baseline on `real_machines`; matching that
  baseline is the critical evidence that generation is working.
- Prefer repo scripts and checked-in helpers over ad-hoc command reconstruction.
- Do not run long noisy training jobs yourself unless the user explicitly wants live execution and the output is tightly bounded.

## Route To Narrower Skills

- For cluster submit, `squeue`/`sacct`, failed-job debugging, sync, and `docs/EXPERIMENTS_*` hygiene, also use `rl-isaaclab-cluster-ops`.
- For billable Vast.ai instance lifecycle and IsaacLab runtime hydration, also use `vast-isaaclab-training`.
- For checkpoint benchmarking, policy playback, TensorBoard temporal plots, and run-to-run comparisons, also use `rl-isaaclab-benchmark`.
- If IsaacLab debugging also involves ROS parity stacks, TF, Dig3D replay, or controller-side checks, also use `newton-ros-parity` and `ros2-debugging`.
- If the task needs many code/doc searches, W&B queries, log inspection, or benchmark diffs, also use `moleworks-subagent-orchestrator`.

## Branch References

Resolve the active checkout first and read only docs that exist and match the current task; branch-specific ledgers may not exist in the canonical checkout:

- `docs/AI_RESEARCHER_WORKFLOW.md`
- `docs/EXPERIMENTS_ONGOING.md`
- `docs/EXPERIMENTS_RUN.md`
- `docs/MULTI_GPU_TRAINING.md`
- `docs/MULTI_GPU_BALANCED_TRAINING.md`

## Default Starting Points

- Train: `scripts/rsl_rl/train.py`
- Play: `scripts/rsl_rl/play.py`
- Cluster submit: `docker/cluster/cluster_interface.sh`
- Cluster sync: `docker/cluster/sync_experiments.sh`
- Live run report: `scripts/utils/cluster_run_report.sh`
- Config diff: `scripts/utils/compare_run_configs.py`
- Excavation benchmark: `scripts/mole_environments/excavation3D/benchmark_excavation.py`
- Temporal plots: `/home/lorenzo/codex_skills/skills/rl-isaaclab/scripts/plot_tb_scalars.py`

If the task starts to sprawl across cluster ops, benchmarks, ROS parity, and repo archaeology at the same time, stop and load the narrower skills rather than expanding this one.
