---
name: rl-newton
description: "Entry point for Moleworks Newton RL work. Use when working on Newton training, shared-turn experiments, local smoke tests, cluster launches, benchmark interpretation, or experiment ledgers. Routes to narrower Newton skills for cluster ops, benchmarking, ROS parity, and long-horizon orchestration."
---

# RL + Newton Router

Use this as the default entry point for `moleworks_newton` work. Keep this skill thin. Pull in narrower skills only for the part of the workflow you actually need.

## Repo Root

- If the user refers to the current shared-turn branch worktree, prefer `/home/lorenzo/moleworks/.worktrees/dev/analytic`.
- Otherwise locate the active Newton repo with `git rev-parse --show-toplevel`.
- Treat `docs/README.md` as the documentation index for the active branch.

## Always-Loaded Rules

- Read repo `AGENTS.md` before changing code or proposing commands.
- Prefer repo docs and checked-in helper scripts over ad-hoc command reconstruction.
- Do not run long noisy `uv run python ...` commands yourself unless the user explicitly wants live execution and the output is tightly bounded. Prefer preparing exact commands for the user or using narrow shell helpers.
- Local smoke first. Real training runs use W&B. Quick local smoke and debug runs disable W&B.
- For shared-turn excavation tasks:
  - use the analytic stack, not the MPM excavation env
  - keep empirical normalization disabled
  - use the torch soil path only
  - prefer Euler `1x4090`, then `3090` only if the run stays pending too long
- Benchmark before drawing conclusions about a checkpoint or training recipe.
- Keep `docs/experiments/README.md`, `docs/experiments/running.md`, and `docs/experiments/done.md` current for real runs.
- If runtime-sensitive shared-turn behavior changes, update `docs/SHARED_TURN_SPEED_BENCHMARKS.md`.

## Route To Narrower Skills

- For cluster submit, monitor, sync, and experiment-ledger work, also use `rl-newton-cluster-ops`.
- For benchmark, terrain-bank, replay, and result-analysis work, also use `rl-newton-benchmark`.
- For ROS parity, Dig3D replay, TF, controller-facing terrain topics, or ROS cleanup, also use `newton-ros-parity` and `ros2-debugging`.
- For long-horizon tasks with lots of code search, docs, logs, W&B inspection, or repeated shell exploration, also use `moleworks-subagent-orchestrator`.

## Branch References

Read only the branch docs that match the current task:

- `docs/README.md`
- `docs/ResearcherWorkflow.md`
- `docs/SHARED_TURN_DEFAULT_TRAINING.md`
- `docs/PARTIALLY_DUG_TERRAIN_WORKFLOW.md`
- `docs/experiments/README.md`
- `docs/SHARED_TURN_SPEED_BENCHMARKS.md`

## Default Starting Points

- Shared-turn default launcher: `cluster/submit_shared_turn_w_cabin_default.sh`
- Generic train entrypoint: `scripts/rsl_rl/train.py`
- Shared-turn benchmark entrypoint: `scripts/benchmark/benchmark_excavation_w_cabin_analytic.py`
- Terrain-bank verifier: `scripts/benchmark/verify_success_terrain_bank.py`
- Play entrypoint: `scripts/rsl_rl/play.py`

If the task starts to sprawl across cluster ops, benchmarks, ROS parity, and repo archaeology all at once, stop and load the narrower skills rather than expanding this one.
