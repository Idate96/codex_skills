---
name: rl-newton-cluster-ops
description: "Submit, monitor, sync, and ledger Moleworks Newton RL runs on Euler or Brev. Use when preparing a smoke test, launching shared-turn jobs, checking `squeue`/`sacct`/Slurm logs, syncing one run, or updating `docs/experiments`."
---

# Newton Cluster Ops

Use this skill for cluster-side RL operations on the active `moleworks_newton` branch.

## Scope

- local smoke gate before cluster launch
- choose the right submit helper
- monitor `squeue`, `sacct`, and Slurm logs
- targeted sync of one run
- keep `docs/experiments` accurate

## Source Of Truth

Read the branch docs before improvising:

- `docs/ResearcherWorkflow.md`
- `docs/experiments/README.md`
- `docs/experiments/latest.md`
- `docs/experiments/running.md`
- `docs/experiments/done.md`
- `docs/SHARED_TURN_DEFAULT_TRAINING.md`
- `cluster/README.md`

For the current shared-turn `dev/analytic` work, prefer the Euler helpers and ledgers over older generic Daint examples in legacy docs.

## Launcher Selection

- Shared-turn default recipe: `cluster/submit_shared_turn_w_cabin_default.sh`
- Shared-turn partially dug recovery or replay finetune: `cluster/submit_shared_turn_w_cabin_partially_dug.sh`
- Shared-turn no-AoA actor branch family: `cluster/submit_shared_turn_w_cabin_no_aoa_actor.sh`
- Generic training: `cluster/submit_job.sh`
- Brev only when explicitly requested: `cluster/submit_job_brev.sh`

## Hard Rules

- Local smoke first, with W&B disabled.
- Real cluster runs use W&B.
- After submitting any real training run, verify that the run actually gets
  through startup before reporting it as successfully launched. `sbatch`
  success, a Brev service pid, or a stale `RUNNING` ledger row is not enough.
  Keep checking until the run either enters the training loop or fails with a
  concrete reason.
- Worktree launches are source-sensitive. `cd` into the intended worktree's
  `cluster/` directory before `submit_job.sh`, `submit_job_brev.sh`,
  `sync_code.sh`, or `sync_code_brev.sh`. A shared `CLUSTER_ENV_FILE` is fine;
  do not set `LOCAL_MOLEWORKS_DIR` unless intentionally submitting a different
  tree.
- For Brev/Euler worktree smoke tests, verify the remote snapshot when code
  identity matters: compare `sha256sum` for one or two changed source files
  against the local worktree, then inspect the run log for the expected
  experiment alias and diagnostic scalars.
- Real Euler training runs should usually request `JOB_TIME=24h`. When composing, reviewing, or launching a real training command, set `JOB_TIME=24h` explicitly by default unless the user asks for a shorter run.
- Use `4h` or shorter only for smoke gates, startup validation, queue/launcher probes, or intentionally bounded debugging runs. Do not use short walltimes for runs meant to judge learning quality.
- If a 24h run has not saturated at its last evaluated checkpoint, continue from the best/latest checkpoint with another long run instead of treating the partial curve as converged.
- For performance claims, prefer 24h-scale runs with pinned checkpoint benchmarks over early W&B curves from short jobs.
- Shared-turn analytic tasks keep `--disable-empirical-normalization`.
- Prefer Euler `1x4090`. Fall back to `3090` only if the run stays pending too long.
- For Euler `fee_excavation`, use the normal `submit_job.sh` path: sync code and create a per-run snapshot. Do not set `MOLEWORKS_SKIP_CODE_SYNC=1` or `MOLEWORKS_SKIP_CODE_SNAPSHOT=1`.
- For Euler `fee_excavation` on `rtx_3090`, cap runs at `40000` worlds. For profile sweeps, prefer a single `40000`-world contract across 3090 and 4090 plus a matching RBF control; use 4090 for larger runs.
- Do not launch new real runs without updating `docs/experiments/running.md`.
- For exploratory reward/env sweeps, prefer diversity over duplicate seeds.
  Do not spend parallel GPUs on multiple seeds of the same config unless the
  user explicitly asks for seed robustness or the design has already narrowed
  to a small set of finalists. Repeated runs are not a substitute for
  ablations.

## Smoke Gate

Prefer preparing an exact local smoke command for the user instead of running a noisy training job yourself.

Shared-turn smoke template:

```bash
export WANDB_MODE=disabled
uv run python scripts/rsl_rl/train.py \
  --task m445_excavation_shared_turn_w_cabin \
  --num-worlds 8 \
  --max-iterations 3 \
  --disable-empirical-normalization
unset WANDB_MODE
```

## Monitoring

Prefer narrow status commands and summarize the result instead of dumping full logs into context.

```bash
ssh euler 'squeue -u $USER'
ssh euler 'sacct -j <jobid> --format=JobID,State,ExitCode,Elapsed,Start,End -P'
ssh euler 'tail -n 200 /cluster/scratch/$USER/moleworks_logs/slurm-<jobid>.out'
```

## Post-Submit Startup Gate

For every real Euler/Brev training launch, run a startup gate before ending the
turn or calling the run "launched":

1. Confirm scheduler/process state:
   - Euler: `squeue` immediately after submit, then `sacct` if the job leaves
     the queue.
   - Brev: confirm the service/python process is alive and tied to the expected
     GPU.
2. Inspect stdout/stderr for the expected startup contract:
   - expected GPU assignment (`CUDA_VISIBLE_DEVICES`, `SLURM_JOB_GPUS`, or Brev
     GPU list)
   - expected task, experiment alias, seed, world count, and diagnostic flags
   - expected code snapshot path for worktree launches
3. Keep watching past slow world/reset-cache construction until one of these
   terminal startup states is reached:
   - **startup verified**: W&B run id/URL exists, or the log reaches
     `Creating PPO runner`, `PPO runner created`, `Starting training`, or a
     first checkpoint/run directory with training artifacts.
   - **startup failed**: Slurm/Brev exits, stderr has an exception, OOM,
     reset-cache failure, invalid config, import error, or missing resource.
4. If startup fails before W&B initializes, say explicitly that there is no W&B
   run, capture the failing phase and stack trace headline, and update the
   ledger with a terminal label such as `FAILED_RESOURCE`,
   `FAILED_INVALID`, or `FAILED_RESET_PREFILTER`.
5. If the user requested a short-queue/short-walltime launch, still perform this
   gate. Short jobs are especially easy to lose in reset-cache or world
   construction before training begins.

Useful Euler probe:

```bash
ssh euler 'squeue -j <jobid> -o "%i|%T|%M|%D|%R|%b"; \
  echo ---OUT---; tail -n 220 /cluster/scratch/$USER/moleworks_logs/slurm-<jobid>.out; \
  echo ---ERR---; tail -n 160 /cluster/scratch/$USER/moleworks_logs/slurm-<jobid>.err'
```

If W&B is expected but no W&B URL appears yet, do not assume the run is healthy;
check whether it is still constructing the world/reset cache or has failed
before logger initialization.

When waiting for long training runs to reach curriculum/checkpoint gates, sleep in long intervals by default:

```bash
sleep 6000
```

Use shorter sleeps only for startup validation, benchmark completion, debugging failures, or when the user explicitly asks for a tighter cadence. While sleeping, avoid repeated manual polling; resume with a compact status table sourced from W&B, Slurm, checkpoint files, and benchmark markers.

If the run is pending too long, check the requested GPU type and decide whether to resubmit on `3090`.

## Delegated Metric And Benchmark Checks

For multi-run sweeps and any local benchmark execution, delegate noisy inspection work to sub-agents by default. Lorenzo's standing preference is that benchmark runs and benchmark-log parsing happen in a worker agent so the parent context stays clean.

Preferred sequence:

1. Ask a metrics worker to read W&B/TensorBoard/Slurm summaries first and return a compact table with latest iteration, curriculum level, success proxies, termination proxies, torque-over-limit metrics, and checkpoint availability.
2. Decide from that table which checkpoints are worth syncing and benchmarking.
3. Ask a benchmark worker to sync only selected artifacts and run local benchmarks sequentially, because the local workstation normally has one GPU. The worker must not start concurrent benchmark processes.
4. Have the worker write raw benchmark output under `/tmp` or the run output directory and return only report paths plus a compact result table.

Do not stream full benchmark logs into the parent conversation unless debugging a crash. The parent should keep only the decision-relevant numbers: success, close/full/partial, major terminations, `tau_applied_over_limit`, `tau_preclip_over_limit`, and any force/depth diagnostics requested by the user. If delegation tools are unavailable, run the smallest necessary local check and summarize only from generated report files.

## Sync

Prefer targeted sync for one run:

```bash
cd cluster
./sync_logs.sh \
  --experiment <experiment> \
  --run-name <run_name> \
  --slurm-job <jobid>
```

Expected local destinations:

- `logs/rsl_rl/<experiment>/<run_name>/`
- `logs/slurm/slurm-<jobid>.out`

Treat the final folder name under `logs/rsl_rl/<experiment>/` as the canonical `run_name`.
Example:
- `/cluster/scratch/$USER/moleworks_logs/rsl_rl/fee_excavation_runtime_cache_ab/2026-03-30_22-10-27_fee_excavation-prepr_runtime_cache_on_s201_4090`
- canonical `run_name`: `2026-03-30_22-10-27_fee_excavation-prepr_runtime_cache_on_s201_4090`

Use the broad `./sync_logs.sh` only when the user wants a full refresh.

## Ledger Rules

Treat these as mandatory for real runs:

1. Add a row to `docs/experiments/running.md` immediately after submit.
2. Include `run_name`, `wandb_run`, `wandb_url`, job id, and key artifact paths.
3. Reconcile `running.md` against live cluster state before reporting status.
4. When a completed pinned benchmark clearly changes the best-known checkpoint for a named condition, update `docs/experiments/latest.md`.
5. When a run is no longer live, preserve the latest detailed state in `docs/experiments/archive/`.
6. Then move the compact summary row to `docs/experiments/done.md`.
7. A non-live job must not remain `RUNNING`; mark it with one terminal label such as `FAILED_INVALID`, `FAILED_RESOURCE`, `FAILED_RESET_PREFILTER`, `FAILED_SIGNAL`, `CANCELLED_PRUNED`, or `TIMEOUT_COMPLETE`.

## Reporting Format

Use one-line snapshots:

```text
<job_id> | <run_name> | <task> | wandb_run=<...> | wandb_url=<...> | timeout=<...> | full=<...> | close=<...>
```

Keep the report compact and source every field from either the synced Slurm log, W&B, or the experiment ledger.
