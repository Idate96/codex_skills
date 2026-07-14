---
name: rl-newton
description: "Route Moleworks Newton RL work. Use as the default entry point for training, shared-turn or FEE experiments, smoke tests, cluster runs, benchmarks, and experiment ledgers."
---

# RL + Newton Router

Keep this entry point thin. Resolve the task-specific checkout, enforce the common evidence rules, then load the narrower cluster or benchmark skill.

## Resolve The Checkout

- For current analytic shared-turn work, prefer `/home/lorenzo/moleworks/.worktrees/dev/analytic` when it contains the requested entrypoint.
- For FEE work, prefer the active checkout containing `scripts/mole_environments/fee_excavation/`.
- Otherwise locate the user's active `moleworks_newton` repository with `git rev-parse --show-toplevel`.

No single checkout is guaranteed to contain every analytic and FEE tool. Before composing or running a command, `test -e` the exact entrypoint and read that checkout's `AGENTS.md` plus task-relevant docs. Do not silently fall back to another branch.

## Common Rules

- Preserve unrelated dirty work and prefer checked-in helpers over reconstructed commands.
- Run launchers from the selected worktree. A shared `CLUSTER_ENV_FILE` does not change the code source unless `LOCAL_MOLEWORKS_DIR` is explicitly set.
- Keep local execution to static checks, dry runs, smoke tests, and bounded diagnostics. Do not run full RL/transformer training on Lorenzo's workstation.
- Disable W&B for smoke/debug. Use W&B and the experiment ledgers for real training.
- Treat tiny PPO runs as startup evidence only. Judge behavior from meaningful rollout scale plus pinned checkpoint benchmarks.
- A production cluster launch requires a user request for real training; status, diagnosis, config review, or benchmark requests do not authorize a new expensive run.
- Use `sacct` for scheduler truth, W&B for learning history, and synced run directories for checkpoints and metadata. Reconcile all three before reporting terminal state.
- Benchmark before promoting a checkpoint or training recipe. Compare a common pinned checkpoint and contract, never each run's unrelated “latest”.
- Keep `docs/experiments/README.md`, `latest.md`, `running.md`, and `done.md` current when they exist in the selected checkout.
- Append dated audits to old launch notes instead of rewriting original context.
- For live Newton/ROS/Terra failures, save a checkpoint/map snapshot and logs before changing code or config, record the hypothesis/change/test, then resume from the saved state.
- For live excavation, keep the per-scoop and per-workspace evidence ledger required by the active runbook.

## Route

- Use `rl-newton-cluster-ops` for smoke gates, Euler/Brev/Vast submit, startup verification, monitoring, sync, and ledgers.
- Use `rl-newton-benchmark` for FEE or analytic benchmarks, terrain banks, replay, leaderboards, qualitative clips, and result analysis.
- Use `newton-ros-parity` plus `ros2-debugging` for ROS, TF, terrain-topic, Dig3D, or cleanup issues.
- Use `moleworks-subagent-orchestrator` only when the task has independent evidence-gathering lanes and delegation is authorized.

## Starting Points

Verify each path in the selected checkout before use:

- generic train/play: `scripts/rsl_rl/train.py`, `scripts/rsl_rl/play.py`
- shared-turn submit: `cluster/submit_shared_turn_w_cabin_default.sh`
- targeted sync: `cluster/sync_logs.sh`
- analytic benchmark: `scripts/benchmark/benchmark_excavation_w_cabin_analytic.py`
- terrain-bank verifier: `scripts/benchmark/verify_success_terrain_bank.py`
- FEE benchmark/play: `scripts/mole_environments/fee_excavation/benchmark/benchmark_fee_excavation.py`, `scripts/mole_environments/fee_excavation/play.py`
- FEE qualitative video: `scripts/mole_environments/fee_excavation/benchmark/record_fee_leaderboard_videos.py`

Read only the branch docs needed for the task, starting from `docs/README.md` when present.

## Fast Status Triage

1. Query `sacct` for the exact job IDs.
2. Query W&B for configured runs and scan scalar history when the question is about learning evolution.
3. Sync only the exact finished run directories needed for pinned checkpoints, events, or missing metadata.
4. Report scheduler state and scientific/artifact quality separately; W&B may label a wall-time-ended run as `crashed` even when checkpoints are usable.
