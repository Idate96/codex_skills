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
- For experiment status checks after a run has been live for a while:
  - use `sacct` first for scheduler truth
  - do not trust stale `RUNNING` notes in `docs/experiments/running.md` until you reconcile them against `sacct`
  - if the run used `logger=wandb`, prefer W&B API for learning-curve and end-state reads
  - use cluster log sync only when you need checkpoints, TensorBoard fallback, or missing metadata
- For Euler single-GPU OOMs, first check the job stdout for `CUDA_VISIBLE_DEVICES` and `SLURM_JOB_GPUS`. Concurrent jobs on the same node must preserve Slurm's GPU assignment; forcing every job to `CUDA_VISIBLE_DEVICES=0` can make unrelated runs collide on one physical GPU.
- For shared-turn excavation tasks:
  - use the analytic stack, not the MPM excavation env
  - keep empirical normalization disabled
  - use the torch soil path only
  - prefer Euler `1x4090`, then `3090` only if the run stays pending too long
- Benchmark before drawing conclusions about a checkpoint or training recipe.
- For active sweep comparisons, sync the targeted run dirs first, then compare a common pinned `model_<N>.pt` across runs. Do not compare “latest available” checkpoints across different jobs.
- Keep `docs/experiments/README.md`, `docs/experiments/running.md`, and `docs/experiments/done.md` current for real runs.
- When updating `docs/experiments/running.md` after a few days have passed, prefer appending a dated audit section over rewriting the original launch note. Preserve the original launch context and add the corrected final read below it.
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
- Targeted run sync helper: `cluster/sync_logs.sh`
- Generic train entrypoint: `scripts/rsl_rl/train.py`
- Shared-turn benchmark entrypoint: `scripts/benchmark/benchmark_excavation_w_cabin_analytic.py`
- Shared-turn sweep smoothness evaluator: `moleworks_newton/tasks/m445_excavation_shared_turn_w_cabin_no_aoa_actor/sim_to_real/measure_action_penalty_effect.py`
- Terrain-bank verifier: `scripts/benchmark/verify_success_terrain_bank.py`
- Play entrypoint: `scripts/rsl_rl/play.py`
- FEE-specific play entrypoint: `scripts/mole_environments/fee_excavation/play.py`

## Fast Experiment Triage

For "what happened to these runs?" or "how are the trainings going now?" requests:

1. Use `sacct` on the exact job ids first.
2. If the run logged to W&B, query W&B next for scalar history and final summaries.
3. Only then sync the exact finished run dirs you need with `cluster/sync_logs.sh --remote-subpath ...`.

Practical defaults:

- `sacct` is the source of truth for `RUNNING` vs `TIMEOUT` vs `FAILED`.
- W&B is the source of truth for training evolution and the final scalar picture.
- Synced run dirs are the source of truth for pinned checkpoints, event files, and any run metadata not captured in the ledger.
- W&B may label a wall-time-ended run as `crashed`; combine W&B with `sacct` instead of trusting that state string in isolation.

## FEE Hard-Soil Robustness

- Before adding hard-soil-specific rewards, first add low-probability hard-soil contamination to the normal random mix and benchmark whether the policy still learns fast full-bucket behavior on normal soil.
- Keep the hard-soil mixture explicit in `docs/experiments/running.md`: list each fixed preset and probability, plus the remaining default random-soil probability.
- For hard-soil policy diagnosis, benchmark finish gates and load together:
  - fill ratio versus sparse close threshold
  - close height and curl-angle gates
  - penetration depth
  - per-joint torque ratio/saturation
  - negative termination breakdown
- If hard-soil runs mostly time out with real load and good target-depth tracking, treat it as a finish-discovery/gating problem first, not as evidence of catastrophic dynamics failure.

## Play Workflow

- Prefer `scripts/rsl_rl/play.py` directly for Newton policy inspection.
- Exception: for `fee_excavation`, prefer `scripts/mole_environments/fee_excavation/play.py`. It restores the task-local replay contract that generic play does not own:
  - private `dev/analytic` compat overlay inference from the run dir
  - latest logged curriculum level, otherwise hardest level
  - task-local FEE viewer overlays
  - Newton-native framebuffer snapshots
- Use `--run-dir <absolute-or-log-relative-run-dir>` and a pinned `--model-number <N>` for reproducible review. Do not rely on "latest" when comparing runs.
- Use `--num-worlds 1` for GUI/debug inspection and `--debug-vis` when checking task overlays.
- By default, play should run at the hardest curriculum level for tasks that expose a curriculum, including the analytic excavation envs that only have task-local `curriculum_level` instead of a generic `curriculum_manager`. Use `--curriculum-level <k>` only when you explicitly want an easier level.
- `model_0.pt` is usually just a smoke or very early checkpoint. For behavior review, prefer a trained checkpoint from the synced run dir.
- For benchmark-faithful `fee_excavation` replay:
  - keep reset cache enabled unless the task is explicitly a reset-cache debug session
  - pin `--seed` to the benchmark seed before comparing behavior to benchmark numbers
  - treat `--no-reset-cache-enable` as a startup/debug convenience only, not as a performance or success-rate comparison mode
  - if the user says "play this FEE policy", default to the benchmark-faithful path first, then only suggest the cache-off fast path as an opt-in debugging shortcut

## Shared-Turn Sweep Comparison

When comparing active shared-turn sweep runs:

1. Use `cluster/sync_logs.sh --run-dir ... --slurm-job ...` to sync the exact run dirs and slurm logs locally.
2. Pick one common pinned `model_<N>.pt` that every run has already written.
3. Use `scripts/benchmark/benchmark_excavation_w_cabin_analytic.py` for task-success comparison under the standard fresh-RBF `1024 x 300` contract.
4. Use `.../sim_to_real/measure_action_penalty_effect.py` when the question is specifically about smoothness/action-rate effects. Keep one shared reset-cache namespace across the compared checkpoints so they reuse the same runtime cache.

For action-penalty sweeps, prefer reporting both:
- task outcome: success/full/close/negative rates
- rollout smoothness: weighted action-rate cost, per-joint step p95, and high-frequency ratio above actuator cutoff

If the task starts to sprawl across cluster ops, benchmarks, ROS parity, and repo archaeology all at once, stop and load the narrower skills rather than expanding this one.
