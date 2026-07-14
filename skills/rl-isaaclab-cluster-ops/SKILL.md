---
name: rl-isaaclab-cluster-ops
description: "Operate Moleworks IsaacLab runs on Euler. Use for local smoke gates, Slurm submission and monitoring, failed-job diagnosis, targeted sync, config comparison, and experiment ledgers."
---

# IsaacLab Cluster Ops

Use this skill for cluster-side RL operations in `moleworks_ext`.

## Source Of Truth

Read the repo workflow/docs before improvising:

- `docs/AI_RESEARCHER_WORKFLOW.md`
- `docs/EXPERIMENTS_ONGOING.md`
- `docs/EXPERIMENTS_RUN.md`
- `docs/MULTI_GPU_TRAINING.md`
- `docker/cluster/cluster_interface.sh`
- `docker/cluster/sync_experiments.sh`
- `scripts/utils/cluster_run_report.sh`
- `scripts/utils/compare_run_configs.py`

## Hard Rules

- Local smoke first, with W&B disabled.
- Real training runs use W&B.
- Real Euler training runs should usually request `JOB_TIME=24h`. Use `30m`/`4h` only for smoke gates, startup validation, queue probes, or intentionally bounded debugging.
- If a 24h run has not saturated by its last evaluated checkpoint, continue from the best/latest checkpoint with another long run instead of treating the partial curve as converged. On Euler, chained 24h continuations are preferred over one fragile oversized job.
- Do not talk about "pulling a policy" unless the job actually trained one.
- Keep experiment docs current in the same work session as submit/sync/closeout.
- Prefer targeted live diagnostics and targeted sync before broad full-log sync.

## Local Smoke Gate

Prefer preparing or running a tiny bounded smoke command before cluster launch:

```bash
export WANDB_MODE=disabled
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --task <TASK> \
  --num_envs 4 \
  --max_iterations 3 \
  --headless
unset WANDB_MODE
```

## Submit

Default cluster entrypoint:

```bash
JOB_TIME=24h NUM_GPUS=2 GPU_TYPE=rtx_3090 \
./docker/cluster/cluster_interface.sh job \
  --task <TASK> \
  --num_envs 64000 \
  --max_iterations 10000
```

Notes:

- In multi-GPU mode, `--num_envs` is the total across GPUs.
- With resume, `--max_iterations` is additional learning iterations beyond the loaded checkpoint.
- If using a git worktree, launch from that worktree root and prove the staged code copy matches the worktree.

## Monitor And Debug

Use narrow checks first:

```bash
ssh euler 'squeue -u $USER'
ssh euler 'sacct -j <jobid> --format=JobID,State,Elapsed,Timelimit,Partition%12,ExitCode -P'
ssh euler 'tail -100 /cluster/scratch/<user>/moleworks_ext_<timestamp>/slurm-<jobid>.out'
ssh euler 'tail -50 /cluster/scratch/<user>/moleworks_ext_<timestamp>/slurm-<jobid>.err'
```

Preferred live diagnostic helper:

```bash
scripts/utils/cluster_run_report.sh
scripts/utils/cluster_run_report.sh --job-ids <jobA> <jobB> --sync-params
```

Use `compare_run_configs.py` before manual YAML archaeology:

```bash
python3 scripts/utils/compare_run_configs.py --run-a <runA> --run-b <runB>
```

## Sync

Full sync when needed:

```bash
./docker/cluster/sync_experiments.sh
```

Use `--remove` only after inspecting the helper's current semantics and obtaining explicit intent to delete remote artifacts; prefer a dry run or targeted sync first.

Targeted-first workflow:

1. `cluster_run_report.sh --job-ids ... --sync-params`
2. compare configs if needed
3. full sync only when you need checkpoints or broader artifacts

## Experiment Docs

Treat these as mandatory:

- `docs/EXPERIMENTS_ONGOING.md` for live runs
- `docs/EXPERIMENTS_RUN.md` for benchmarked/completed/stopped runs

For every new real run, record:

- `name`
- `run_name`
- `wandb_run`
- `wandb_url`
- environment
- date in UTC
- short intention
- job id and key artifact path when known

Before each monitoring report:

- reconcile `EXPERIMENTS_ONGOING.md` against live `squeue`
- if a listed job is no longer in `squeue`, query `sacct` and record its actual terminal state (`COMPLETED`, `FAILED`, `TIMEOUT`, or `CANCELLED`)
- benchmark/archive usable artifacts only after that classification, then update the experiment ledger

## Reporting Format

Use one-line snapshots:

```text
<job_id> | <run_name> | <task> | wandb_run=<...> | wandb_url=<...> | timeout=<...> | full=<...> | close=<...>
```

Always include `run_name`, task, and W&B identity, not just job ids.
