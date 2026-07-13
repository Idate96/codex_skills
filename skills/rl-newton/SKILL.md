---
name: rl-newton
description: "Route Moleworks Newton RL work: training, shared-turn experiments, smoke tests, cluster runs, benchmarks, and ledgers."
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
- For Euler/Brev launches from a worktree, run the launcher from that
  worktree's `cluster/` directory. `CLUSTER_ENV_FILE` may point at a shared env
  file, but the code source should be the launcher checkout unless
  `LOCAL_MOLEWORKS_DIR` is intentionally set in the command environment.
- Do not run long noisy `uv run python ...` commands yourself unless the user explicitly wants live execution and the output is tightly bounded. Prefer preparing exact commands for the user or using narrow shell helpers.
- Do not run full transformer/RL training locally on Lorenzo's workstation; keep local execution to smoke tests,
  static checks, dry-run manifests, and bounded diagnostics so the PC stays responsive. Use Euler for real training
  runs when training is needed.
- Local smoke first. Real training runs use W&B. Quick local smoke and debug runs disable W&B.
- For real Euler training, prefer longer walltimes. Default to `JOB_TIME=24h` when composing, reviewing, or launching training commands unless the user explicitly asks for a shorter run.
- Use `4h` or shorter only for smoke gates, startup validation, queue/launcher probes, or intentionally bounded debugging runs. Do not use short walltimes for runs meant to judge learning quality.
- PPO behavior judgments require real rollout scale. Treat tiny local PPO runs as startup/smoke only; do not use them to decide whether a reward, termination, or policy behavior is working. For Newton RL, expect meaningful PPO evidence to come from multi-hour runs with at least tens of thousands of environment rollouts/episodes, with `40000+` as the practical minimum target before judging learning quality, plus pinned checkpoint benchmarks.
- When checking an active Newton training run, expect meaningful learning readouts to come from long runs, usually 24h-scale jobs, plus pinned checkpoint benchmarks rather than very early W&B curves.
- For `fee_workspace_planner` PPO macro budgets, compute the ideal full-bucket lower bound before picking a cap: `ceil(target_volume_m3 / deployed_bucket_volume_m3)`. Treat `3x` that value as the minimum efficient-planner lower bound, not as proof the cap is high enough for early PPO. If online or benchmark rollouts hit the macro cap without completing, estimate the empirical need from `cap / final_completion_ratio` and relaunch with slack, normally the larger of `3x ideal` and `~1.3-1.5x` that empirical estimate. For the 30 cm shallow fan with the teeth600 bucket, `2.264 / 0.250661 -> 10` ideal scoops, but `H=32` only reached about `0.36` final completion, implying `32 / 0.36 ~= 90`; use about `128` macros for initial training.
- For experiment status checks after a run has been live for a while:
  - use `sacct` first for scheduler truth
  - do not trust stale `RUNNING` notes in `docs/experiments/running.md` until you reconcile them against `sacct`
  - if the run used `logger=wandb`, prefer W&B API for learning-curve and end-state reads
  - use cluster log sync only when you need checkpoints, TensorBoard fallback, or missing metadata
- For Euler single-GPU OOMs, first check the job stdout for `CUDA_VISIBLE_DEVICES` and `SLURM_JOB_GPUS`. Concurrent jobs on the same node must preserve Slurm's GPU assignment; forcing every job to `CUDA_VISIBLE_DEVICES=0` can make unrelated runs collide on one physical GPU.
- For Daint/CSCS queue triage, a single-GPU GH job showing `72` CPUs is one GPU slice: current Daint GH nodes expose `288` CPUs and `4` GPUs. Current GPU partitions are `debug` max `00:30:00`, `normal` default/max `01:00:00`/`1-00:00:00`, and `low` default/max `01:00:00`/`1-00:00:00`; `xfer` has no GPU GRES.
- For shared-turn excavation tasks:
  - use the analytic stack, not the MPM excavation env
  - keep empirical normalization disabled
  - use the torch soil path only
  - prefer Euler `1x4090`, then `3090` only if the run stays pending too long
- Benchmark before drawing conclusions about a checkpoint or training recipe.
- For FEE leaderboard/report work from a worktree, route to `rl-newton-benchmark`; use the current fast FEE benchmark contract there (`128` envs, `180` steps) unless the user explicitly asks for a high-confidence rerun. After local benchmark or video generation, archive the batch into `/home/lorenzo/moleworks/moleworks_newton/outputs/fee_benchmarks`, rebuild `global_latest`, and verify the new policy appears there before giving the user the full leaderboard URL.
- For FEE specialist, leaderboard, or confusing checkpoint work, generate at least one short qualitative GL clip by default after the pinned benchmark unless the user asks not to. Use `record_fee_leaderboard_videos.py`, normally one `precision_profile_10cm` episode at `160` steps with `--soil-wireframe-mode full`, archive it under the canonical FEE benchmark tree, rebuild `global_latest`, and give the browser `index.html` or global latest link.
- For new real `fee_excavation` W&B training launches, include the async W&B video hook by default when the code supports it and a spare GPU or clear training headroom is available. Baseline flags: `--wandb-video-interval 1000 --wandb-video-cases precision_profile_10cm --wandb-video-max-steps 160 --wandb-video-episodes 1 --wandb-video-soil-wireframe-mode full`. If using a spare GPU, add `--wandb-video-cuda-visible-devices <spare_gpu_id> --wandb-video-device cuda:0`. Do not enable this on the same 24GB GPU as a large 30k/40k-world FEE job unless the user explicitly accepts the slowdown/OOM risk; for fully occupied Vast/Euler machines, leave it disabled and generate post-hoc clips after sync.
- For active sweep comparisons, sync the targeted run dirs first, then compare a common pinned `model_<N>.pt` across runs. Do not compare “latest available” checkpoints across different jobs.
- Keep `docs/experiments/README.md`, `docs/experiments/latest.md`, `docs/experiments/running.md`, and `docs/experiments/done.md` current for real runs.
- When a pinned benchmark produces a new best-known checkpoint for a named condition, update `docs/experiments/latest.md` in the same turn instead of leaving the promotion buried only in `running.md`.
- When updating `docs/experiments/running.md` after a few days have passed, prefer appending a dated audit section over rewriting the original launch note. Preserve the original launch context and add the corrected final read below it.
- If runtime-sensitive shared-turn behavior changes, update `docs/SHARED_TURN_SPEED_BENCHMARKS.md`.
- For live Newton/ROS/Terra execution issues, use a checkpoint-first loop:
  save the current Terra checkpoint or excavation-map snapshot and relevant logs before changing code or config, apply the narrow fix, record the symptom/hypothesis/change/test/resume result in the experiment notebook, then resume from the saved checkpoint instead of restarting from scratch.
- For live Terra excavation runs, keep a per-scoop notebook ledger. After every scoop or failed
  scoop attempt, add a row before resuming that records the attempt id, target, predicted useful
  soil, measured scooped/removed soil, remaining soil before/after, coverage, precision/error
  notes, checkpoint/map artifact, and outcome. At the end of each workspace, save and record the
  final surface/map artifact, remaining soil, precision against the target surface, and whether the
  result is acceptable before moving on.

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
- FEE-specific benchmark entrypoint: `scripts/mole_environments/fee_excavation/benchmark/benchmark_fee_excavation.py`
- FEE qualitative video recorder: `scripts/mole_environments/fee_excavation/benchmark/record_fee_leaderboard_videos.py`
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

## FEE W&B Termination History

For FEE policy comparisons, use W&B `scan_history` before syncing logs when the
question is about discovery order or live training curves. The key termination
history scalars are:

- positive finish history: `Episode Termination/last_full`, `Episode Termination/last_close`, `Episode Termination/last_partial`
- total negative history: `Episode Termination/last_total_negative`
- torque-limit negative history: `Episode Termination/last_negative_torque_limits`
- bucket-velocity negative history: `Episode Termination/last_negative_bucket_velocity`
- supporting counts: `Episode Count/full`, `Episode Count/close`, `Episode Count/partial`, `Episode Count/tot_negative`

Minimal comparison script from the active Newton worktree:

```bash
uv run python - <<'PY'
import math
import wandb

api = wandb.Api(timeout=60)
runs = [
    ("label_a", "wandb_run_id_a"),
    ("label_b", "wandb_run_id_b"),
]
keys = [
    "_step",
    "Episode Termination/last_full",
    "Episode Termination/last_close",
    "Episode Termination/last_partial",
    "Episode Termination/last_total_negative",
    "Episode Termination/last_negative_torque_limits",
    "Episode Termination/last_negative_bucket_velocity",
    "Episode Count/full",
    "Episode Count/close",
    "Episode Count/partial",
    "Episode Count/tot_negative",
]

def finite(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except Exception:
        return None

for label, run_id in runs:
    run = api.run(f"idate96/moleworks_newton/{run_id}")
    latest = {}
    first_close = first_full = first_partial = None
    first_close_005 = None
    for row in run.scan_history(keys=keys, page_size=1000):
        step = int(row.get("_step", 0))
        vals = {key: finite(row.get(key)) for key in keys if key != "_step"}
        latest = {"_step": step, **vals}
        close = vals.get("Episode Termination/last_close")
        full = vals.get("Episode Termination/last_full")
        partial = vals.get("Episode Termination/last_partial")
        if first_close is None and close is not None and close > 0.0:
            first_close = (step, close)
        if first_close_005 is None and close is not None and close >= 0.05:
            first_close_005 = (step, close)
        if first_full is None and full is not None and full > 0.0:
            first_full = (step, full)
        if first_partial is None and partial is not None and partial > 0.0:
            first_partial = (step, partial)
    print(label, run_id, run.state)
    print("  first_close_any:", first_close, "first_close>=0.05:", first_close_005)
    print("  first_full:", first_full, "first_partial:", first_partial)
    print("  latest:", latest)
PY
```

Treat tiny nonzero `last_close` values near `1e-5` as logging/counting blips
unless the user explicitly asks for first nonzero. For policy discovery order,
prefer threshold crossings such as `last_close >= 0.05` plus latest/max close
and full rates.

## FEE Hard-Soil Robustness

- Before adding hard-soil-specific rewards, first add low-probability hard-soil contamination to the normal random mix and benchmark whether the policy still learns fast full-bucket behavior on normal soil.
- When sweeping `fee_excavation` policy frequency or `simulation.decimation`, normalize true per-step dense reward terms by `policy_dt / reference_policy_dt` before interpreting results. Keep terminal rewards and delta/potential-style progress terms unchanged, and log `Task/policy_dt` plus `Task/reward_time_scale` so the sweep is auditable.
- Keep the hard-soil mixture explicit in `docs/experiments/running.md`: list each fixed preset and probability, plus the remaining default random-soil probability.
- When the user asks how a FEE checkpoint is doing, separate live training scalars from pinned benchmark results. Do not answer hard-soil robustness or tracking-precision questions from training ratios alone.
- The default FEE benchmark bundle must include:
  - termination outcome breakdown: `desired_full`, `desired_close`, `desired_partial`, and total negative
  - target-depth precision from the benchmark report or JSON, not just raw success: in-soil target-depth dwell, best-from-above or closest approach, overshoot, and penetration
  - hard-soil preset performance on the current suite: `default_hard_soil_mix`, `fixed_soil_random_hard_cap`, `fixed_soil_hard_interp_25`, `fixed_soil_hard_interp_50`, `fixed_soil_hard_interp_75`, and `asphalt_like`
  - load usage under hard soil: global and per-joint torque saturation, torque-stage clipping (`preclip` vs `applied` over-limit fractions), plus resisting force or power
- On fixed hard-soil presets, `desired_full` is often unattainable and should not be treated as the primary metric. Prefer target-depth precision, penetration, load usage, negative-termination breakdown, and whether the policy stays controlled under resistance.
- For hard-soil policy diagnosis, benchmark finish gates and load together:
  - fill ratio versus sparse close threshold
  - close height and curl-angle gates
  - penetration depth
  - per-joint torque ratio/saturation
  - negative termination breakdown
- For hard-soil failure analysis, derive failure phase from the per-episode benchmark JSON before proposing fixes:
  - `pre_entry`: `in_soil_step_count == 0`
  - `entry_instability`: negative `bucket_velocity` or `bucket_angle_of_attack` within the first few in-soil steps, with tiny fill
  - `in_soil_pre_finish`: entered soil but never reached close/high/curl gates
  - `late_finish_or_lift`: failed only after reaching close/high/curl gates
- If the dominant hard-soil negatives are `pre_entry` or `entry_instability`, treat it as an entry-control or hardness-inference problem first. Do not jump straight to post-capture fill or closing rewards.
- If `bucket_velocity` is the dominant hard-soil negative, verify whether the policy actually over-commanded speed before recommending a fix. Use the task-local hard-entry trace to compare `qd_desired` arm targets against measured `arm_joint_vel` over the last few steps before termination. If commanded fractions stay modest but measured arm velocity spikes or reverses sign after contact, treat it as contact/control instability under load rather than policy speed saturation.
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
- For GL FEE videos, use the real local display when available. If the Codex shell has `DISPLAY` unset, check `/tmp/.X11-unix/X*` and validate with `DISPLAY=:<n> xdpyinfo`; use `env DISPLAY=:<n> ... --viewer gl --headless` before falling back to `xvfb-run`. Do not silently switch to USD when the user asks for GL.
- For torque-compensated FEE policies, always distinguish preclip policy/servo torque from applied torque including compensation. If the user says the machine must not go above `1.0`, treat applied torque saturation/over-limit as the hard safety check unless they explicitly mean policy preclip torque.

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
