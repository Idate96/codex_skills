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
- Shared-turn analytic tasks keep `--disable-empirical-normalization`.
- Prefer Euler `1x4090`. Fall back to `3090` only if the run stays pending too long.
- Do not launch new real runs without updating `docs/experiments/running.md`.

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

If the run is pending too long, check the requested GPU type and decide whether to resubmit on `3090`.

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

Use the broad `./sync_logs.sh` only when the user wants a full refresh.

## Ledger Rules

Treat these as mandatory for real runs:

1. Add a row to `docs/experiments/running.md` immediately after submit.
2. Include `run_name`, `wandb_run`, `wandb_url`, job id, and key artifact paths.
3. Reconcile `running.md` against live cluster state before reporting status.
4. When a run is no longer live, preserve the latest detailed state in `docs/experiments/archive/`.
5. Then move the compact summary row to `docs/experiments/done.md`.

## Reporting Format

Use one-line snapshots:

```text
<job_id> | <run_name> | <task> | wandb_run=<...> | wandb_url=<...> | timeout=<...> | full=<...> | close=<...>
```

Keep the report compact and source every field from either the synced Slurm log, W&B, or the experiment ledger.
