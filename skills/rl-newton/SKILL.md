---
name: rl-newton
description: "End-to-end Newton RL workflow in moleworks_newton: local smoke, cluster submit/monitor/debug, sync logs, benchmark checkpoints, and run post-benchmark analysis with strict experiment ledgers."
---

# RL + Newton Workflow (moleworks_newton)

Single-skill workflow for Newton RL: local smoke test -> cluster submit -> monitor/debug -> sync -> benchmark -> analysis.

## 1) Always Run Newton Python via `uv`

Use `uv` from repo root:

```bash
uv run python scripts/rsl_rl/train.py --task <TASK> --num-worlds <N>
```

Do not use IsaacLab wrappers here.

## 2) Local Smoke Test (ALWAYS first)

Disable W&B for smoke runs to avoid junk runs:

```bash
export WANDB_MODE=disabled
uv run python scripts/rsl_rl/train.py \
  --task m445_excavation_w_cabin_analytic \
  --num-worlds 8 \
  --max-iterations 3
unset WANDB_MODE
```

## 3) Cluster Build/Push (Euler)

Run from `moleworks_newton/cluster`:

```bash
./test_cluster_setup.sh
./build_container.sh
./push_container.sh
```

Rebuild/push whenever container dependencies or cluster scripts change.

## 4) Submit Training Jobs

### Euler

```bash
./submit_job.sh \
  --task m445_excavation_w_cabin_analytic \
  --num-worlds 32000 \
  --gpus 1 \
  --time 24h \
  --partition gpuhe.24h \
  -- --max-iterations 10000
```

Multi-GPU:

```bash
./submit_job.sh \
  --task m445_excavation_w_cabin_analytic \
  --num-worlds 64000 \
  --gpus 4 \
  --time 24h \
  --partition gpuhe.24h \
  -- --max-iterations 10000
```

### Brev

```bash
./submit_job_brev.sh \
  --task m445_excavation_w_cabin_analytic \
  --num-worlds 32000 \
  --gpus 1 \
  --time 24h \
  -- --max-iterations 10000
```

## 5) Monitor and Debug Jobs

```bash
ssh euler 'squeue -u $USER'
ssh euler 'sacct -j <jobid> --format=JobID,State,ExitCode,Elapsed,Start,End -P'
ssh euler 'tail -n 200 /cluster/scratch/$USER/moleworks_logs/slurm-<jobid>.out'
```

Time-limit diagnosis:

```bash
ssh euler 'sacct -j <jobid> --format=JobID,State,Elapsed,Timelimit,Partition%12,ExitCode -P'
```

## 6) Sync Logs and Checkpoints

### Full sync

```bash
cd cluster
./sync_logs.sh
# optional cleanup after sync
./sync_logs.sh --remove
```

### Targeted sync (single run dir)

```bash
RUN_DIR=/cluster/scratch/$USER/moleworks_logs/rsl_rl/<experiment>/<run_name>
rsync -azP euler:${RUN_DIR}/ logs/rsl_rl/<experiment>/<run_name>/
```

### One-command sync + benchmark + analysis (cabin task)

Use the helper script when you want a strict end-to-end pass:

```bash
scripts/benchmark/sync_benchmark_analyze_w_cabin.sh \
  --run-name <run_name> \
  --checkpoint model_<N>.pt \
  --num-envs 512 \
  --benchmark-steps 300 \
  --device cuda:0 \
  --output-tag <tag>
```

## 7) Benchmark Protocol (Mandatory before conclusions)

Policy recommendations must include benchmark evidence.

### 7.1 Benchmark trained excavation checkpoints

```bash
uv run python scripts/benchmark/benchmark_excavation.py \
  --run-dir logs/rsl_rl/m445_excavation/<run_name> \
  --checkpoint model_<N>.pt \
  --num-envs 128 \
  --num-cohorts 5 \
  --benchmark-steps 400 \
  --device cuda:0
```

Outputs default to:

```text
logs/rsl_rl/m445_excavation/<run_name>/benchmark_results/
```

### 7.2 Benchmark cabin-analytic parity/ext policy in Newton

```bash
uv run python scripts/benchmark/benchmark_excavation_w_cabin_analytic.py \
  --checkpoint <checkpoint.pt> \
  --num_envs 2048 \
  --benchmark_steps 300 \
  --seed 0 \
  --device cuda:0 \
  --output_dir artifacts/benchmark_w_cabin/<run_tag>
```

Note:
- `benchmark_excavation_w_cabin_analytic.py` accepts both ext-style checkpoints (`model_state_dict`) and Newton RSL-RL checkpoints (`actor_state_dict`).

### 7.3 Throughput scaling benchmark on cluster

```bash
cd cluster
./submit_excavation_scaling_benchmark.sh \
  --checkpoint /cluster/scratch/$USER/moleworks_logs/rsl_rl/m445_excavation/<run_name>/model_<N>.pt \
  --num-envs-per-gpu 8 \
  --benchmark-steps 300 \
  --warmup-steps 50 \
  --time 45m
```

## 8) Post-Benchmark Analysis

For JSON produced by benchmark scripts:

```bash
uv run python scripts/benchmark/analyze_benchmark.py \
  logs/rsl_rl/m445_excavation/<run_name>/benchmark_results/benchmark_<timestamp>.json \
  --output-dir outputs/benchmark_analysis/<run_name>
```

Optional threshold re-analysis:

```bash
uv run python scripts/benchmark/analyze_benchmark.py <benchmark_json> \
  --filter-min-coverage-ratio 0.95 \
  --filter-max-height-error 0.05
```

If `pandas` is unavailable, the script still prints a basic summary and termination breakdown.

## 9) Play Policy Locally

```bash
uv run python scripts/rsl_rl/play.py \
  --task m445_excavation_w_cabin_analytic \
  --run-dir logs/rsl_rl/m445_excavation_w_cabin_analytic/<run_name> \
  --model-number <N> \
  --num-worlds 1
```

## 10) Experiment Ledgers (Mandatory)

Maintain these files in `moleworks_newton/docs`:
- `EXPERIMENTS_ONGOING.md`
- `EXPERIMENTS_RUN.md`

Hard rules:
1. Add a row to `EXPERIMENTS_ONGOING.md` immediately after submit.
2. Include `run_name`, `wandb_run`, and `wandb_url` (`NA` if unavailable).
3. Do not launch additional runs before updating the ongoing ledger.
4. When a run finishes/fails/is benchmarked, move it to `EXPERIMENTS_RUN.md` with result summary and artifacts.
5. Reconcile `EXPERIMENTS_ONGOING.md` against `squeue` before status reports.
6. If a row is not in `squeue` anymore, treat it as finished, benchmark/archive it, and remove it from ongoing in the same session.

## 11) Reporting Format (Mandatory)

Use one-line snapshots with run identity:

```text
<job_id> | <run_name> | <task> | wandb_run=<...> | wandb_url=<...> | timeout=<...> | full=<...> | close=<...>
```

Extractors:

```bash
# run_name
grep -m1 'Run name:' /cluster/scratch/$USER/moleworks_logs/slurm-<jobid>.out

# W&B URL (stdout/stderr)
grep -Eo 'https://wandb.ai/[^ ]+/runs/[^ ]+' \
  /cluster/scratch/$USER/moleworks_logs/slurm-<jobid>.out \
  /cluster/scratch/$USER/moleworks_logs/slurm-<jobid>.err | tail -n1
```
