---
name: rl-newton-cluster-ops
description: "Operate Newton RL runs on Euler, Brev, or Vast.ai. Use for smoke gates, submission, startup verification, scheduler and W&B monitoring, targeted sync, and experiment ledgers."
---

# Newton Cluster Ops

Use this skill for cluster-side RL operations on the active `moleworks_newton` branch.

## Scope

- local smoke gate before cluster launch
- choose the right submit helper
- monitor `squeue`, `sacct`, and Slurm logs
- monitor Vast `nohup` processes, GPU state, and remote logs
- targeted sync of one run
- keep `docs/experiments` accurate

## Source Of Truth

Read the branch docs before improvising:

- `docs/ResearcherWorkflow.md`
- `docs/experiments/README.md`
- `docs/experiments/latest.md`
- `docs/experiments/running.md`
- `docs/experiments/done.md`
- `docs/research/README.md`
- `cluster/README.md`
- `cluster/docs/workflows.md`

For the current shared-turn `dev/analytic` work, prefer the Euler helpers and ledgers over older generic Daint examples in legacy docs.

## Launcher Selection

- Shared-turn default recipe: `cluster/submit_shared_turn_w_cabin_default.sh`
- Shared-turn partially dug recovery or replay finetune: `cluster/submit_shared_turn_w_cabin_partially_dug.sh`
- Shared-turn no-AoA actor branch family: `cluster/submit_shared_turn_w_cabin_no_aoa_actor.sh`
- Generic training: `cluster/submit_job.sh`
- Bounded one-GPU Euler benchmark/diagnostic: `cluster/submit_job.sh quick`
- Brev only when explicitly requested: `cluster/submit_job_brev.sh`
- Vast.ai: `cluster/sync_code_vast.sh`, `cluster/setup_vast_env.sh`,
  `cluster/submit_job_vast.sh`, then `cluster/sync_logs.sh` with
  `CLUSTER_TYPE=vast`

For a short Euler command that is not `scripts/rsl_rl/train.py`, use
`submit_job.sh quick`; `MOLEWORKS_CLUSTER_COMMAND` is unsupported. Pass each
command token after `--`, use `@LOG_ROOT@` for persistent inputs, and use
`@OUTPUT_DIR@` for outputs. The launcher writes exact argv to a private
NUL-delimited command file, forces normal code sync plus an immutable snapshot,
records the command in the submit manifest, and disables W&B unless explicitly
configured otherwise:

```bash
cd /path/to/intended/worktree/cluster
CLUSTER_ENV_FILE=/path/to/shared/.env.euler ./submit_job.sh quick \
  --run-id <unique-id> \
  --task <task> \
  --num-worlds <N> \
  -- \
  python scripts/<benchmark>.py \
  --checkpoint @LOG_ROOT@/<checkpoint-subpath> \
  --output @OUTPUT_DIR@
```

Sync and checksum-verify any checkpoint under `@LOG_ROOT@` before submitting.
Never reuse a quick-job run id: its source snapshot and command file are
immutable.

## Hard Rules

- When collaboration/sub-agent tools are available, always delegate the
  mechanical cluster task to a worker agent. This includes submission,
  scheduler polling, remote log and W&B parsing, checkpoint inventory, log
  sync, and benchmark execution. Give the worker the exact run identifiers,
  expected contract, and compact output fields; keep raw logs out of the
  primary context. The primary agent retains scientific interpretation,
  state-changing decisions, and the user-facing conclusion. Fall back to
  direct execution only when delegation is unavailable or the worker cannot
  access the required system.
- Local smoke first, with W&B disabled.
- Smoke gates must check numerical validity, not only process survival: after the first update,
  assert finite loss scalars, model parameters, optimizer state, and any teacher/distillation
  tensors that affect the update before calling the gate passed.
- Real cluster runs use W&B.
- After submitting any real training run, verify that the run actually gets
  through startup before reporting it as successfully launched. `sbatch`
  success, a Brev service pid, a Vast `nohup` pid, or a stale `RUNNING` ledger
  row is not enough. Keep checking until the run either enters the training
  loop or fails with a concrete reason.
- After startup is verified, stop tight polling. For long training, collection,
  or convergence runs, wait long enough for meaningful evidence to change
  before checking again. Short few-minute polling is only for startup,
  suspected failure, imminent completion, or active debugging.
- A completed process is not automatically a successful run. After completion,
  sync the relevant artifacts and verify the expected outputs, summaries,
  checkpoints, command manifests, and metrics before reporting success.
- Worktree launches are source-sensitive. `cd` into the intended worktree's
  `cluster/` directory before `submit_job.sh`, `submit_job_brev.sh`,
  `sync_code.sh`, or `sync_code_brev.sh`. For Vast, prefer the intended
  worktree's `sync_code_vast.sh`/`submit_job_vast.sh`; if that worktree does
  not yet contain the Vast helpers, run the helper from a checkout that does
  and explicitly set `LOCAL_MOLEWORKS_DIR`, `LOCAL_NEWTON_DIR`,
  `LOCAL_NEWTON_ACTUATORS_DIR`, and `LOCAL_RSL_RL_DIR` to the intended
  worktree siblings. A shared `CLUSTER_ENV_FILE` is fine; do not set
  `LOCAL_MOLEWORKS_DIR` unless intentionally submitting a different tree.
- For Brev/Euler/Vast worktree smoke tests, verify the remote snapshot when
  code identity matters: compare `sha256sum` for one or two changed source
  files against the local worktree, then inspect the run log for the expected
  experiment alias and diagnostic scalars.
- Real Euler training runs should usually request `JOB_TIME=24h`. When composing, reviewing, or launching a real training command, set `JOB_TIME=24h` explicitly by default unless the user asks for a shorter run.
- Use `4h` or shorter only for smoke gates, startup validation, queue/launcher probes, or intentionally bounded debugging runs. Do not use short walltimes for runs meant to judge learning quality.
- If a 24h run has not saturated at its last evaluated checkpoint, continue from the best/latest checkpoint with another long run instead of treating the partial curve as converged.
- For performance claims, prefer 24h-scale runs with pinned checkpoint benchmarks over early W&B curves from short jobs.
- Shared-turn analytic tasks keep `--disable-empirical-normalization`.
- Prefer Euler `1x4090`. Fall back to `3090` only if the run stays pending too long.
- For Euler `fee_excavation`, use the normal `submit_job.sh` path: sync code and create a per-run snapshot. Do not set `MOLEWORKS_SKIP_CODE_SYNC=1` or `MOLEWORKS_SKIP_CODE_SNAPSHOT=1`.
- For Euler `fee_excavation` on `rtx_3090`, cap runs at `40000` worlds. For profile sweeps, prefer a single `40000`-world contract across 3090 and 4090 plus a matching RBF control; use 4090 for larger runs.
- For Vast, use the port mapped to container port 22 from the instance
  terminal (`$VAST_TCP_PORT_22`), not the Jupyter/portal port. Vast instances
  are already Docker containers, so use `uv` inside the instance rather than
  Docker-in-Docker. Read `/etc/vast-agents-guide.md` before acting on a Vast
  instance.
- For real Vast W&B runs, set the W&B entity explicitly, normally
  `WANDB_USERNAME=idate96` and `WANDB_ENTITY=idate96`, and verify those values
  in the remote Python process before W&B initializes. Do not rely on the
  rented container's default W&B login/entity.
- For new real `fee_excavation` W&B launches, add the async video hook by
  default when the code supports it and there is a spare GPU or known headroom:
  `--wandb-video-interval 1000 --wandb-video-cases precision_profile_10cm
  --wandb-video-max-steps 160 --wandb-video-episodes 1
  --wandb-video-soil-wireframe-mode full`. Prefer isolating it with
  `--wandb-video-cuda-visible-devices <spare_gpu_id> --wandb-video-device
  cuda:0`. Do not add the hook when all GPUs are occupied, or on the same 24GB
  GPU as a large 30k/40k-world FEE job, unless the user explicitly accepts the
  slowdown/OOM risk. For active runs that lack the hook, do not restart solely
  for video; sync checkpoints and generate post-hoc clips locally.
- Vast instances with `workspace_is_volume=false` lose `/workspace` on destroy
  or recycle. Sync logs/checkpoints before terminating the instance.
- Do not launch new real runs without updating `docs/experiments/running.md`.
- For exploratory reward/env sweeps, prefer diversity over duplicate seeds.
  Do not spend parallel GPUs on multiple seeds of the same config unless the
  user explicitly asks for seed robustness or the design has already narrowed
  to a small set of finalists. Repeated runs are not a substitute for
  ablations.

## Euler Storage Contract

Treat file placement as a launch invariant, not a cleanup afterthought:

- `/cluster/scratch/$USER`: synchronized Newton/Newton-Actuators/RSL-RL source,
  per-run code snapshots, logs, W&B state, and disposable caches. Scratch is
  purged, so sync required outputs and checkpoints before they age out.
- `/cluster/project/rsl/$USER`: persistent expanded datasets, generated asset
  roots, and software trees with many small files.
- `/cluster/work/rsl/$USER`: large `.sif` images, checkpoints, and compressed
  archives only. Never rsync an expanded source, environment, cache, or asset
  tree into work; its inode allowance is much smaller.

Before a broad sync or large launch, check:

```bash
ssh euler 'lquota; lfs quota -u "$USER" /cluster/work/rsl; \
  (head -n 5 && grep -w "$USER") < /cluster/project/rsl/.rsl_user_data_usage.txt'
```

Use the checked-in `.env.cluster.template` layout. If a persistent small-file
tree is found in work, migrate it to
project storage, verify the copy, and leave a compatibility symlink only when
existing launch contracts still reference the old path.

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

Useful Vast probe:

```bash
ssh -p <VAST_TCP_PORT_22> root@<VAST_PUBLIC_IP> 'ps -p <pid> -o pid,stat,etime,cmd; \
  nvidia-smi --query-gpu=index,name,memory.used,utilization.gpu --format=csv,noheader; \
  tail -n 220 /workspace/logs/vast-<run-id>.log'
```

## Post-Submit Startup Gate

For every real Euler/Brev/Vast training launch, run a startup gate before ending the
turn or calling the run "launched":

1. Confirm scheduler/process state:
   - Euler: `squeue` immediately after submit, then `sacct` if the job leaves
     the queue.
   - Brev: confirm the service/python process is alive and tied to the expected
     GPU.
   - Vast: confirm the `nohup`/`timeout` process and underlying Python process
     are alive, tied to the expected GPU, and using the intended W&B env.
2. Inspect stdout/stderr for the expected startup contract:
   - expected GPU assignment (`CUDA_VISIBLE_DEVICES`, `SLURM_JOB_GPUS`, Brev
     GPU list, or Vast GPU list)
   - expected task, experiment alias, seed, world count, and diagnostic flags
   - expected code snapshot path for worktree launches, or expected synced
     Vast checkout/source hashes
   - if the FEE W&B video hook is enabled, expected `[WANDB_VIDEO]` setup lines,
     no recorder launched at iteration `0`, and a staging/publish location under
     the run directory such as `wandb_video_jobs/` and `wandb_videos/`
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

For long training runs, use the product's recurring monitor or wait mechanism with a cadence matched to the next expected checkpoint or completion boundary. Do not block an agent process with multi-hour `sleep` commands. Each resumed check should produce one compact status table from W&B, Slurm, checkpoint files, benchmark markers, and run artifacts.

When a long run exits, do a completion gate before calling it successful:

1. Confirm the scheduler/process reached a normal terminal state.
2. Sync logs and run artifacts if they are on a non-persistent Vast instance or
   remote scratch.
3. Inspect expected outputs: checkpoints, train summaries, benchmark reports,
   command manifests, and comparison artifacts.
4. For convergence claims, inspect the final loss/metric window. If it is still
   materially improving, continue with another long leg instead of calling the
   run converged.

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
For Vast, `sync_logs.sh` mirrors `VAST_LOGS_DIR` and pulls top-level
`vast-<run-id>.log` files plus `rsl_rl/`, `wandb/`, `submit_manifests/`, and
`py_spy_recordings/`. Always sync before destroying a Vast instance that does
not have a persistent workspace volume.

## Ledger Rules

Treat these as mandatory for real runs:

1. Add a row to `docs/experiments/running.md` immediately after submit.
2. Include `run_name`, `wandb_run`, `wandb_url`, job id, and key artifact paths.
3. Reconcile `running.md` against live cluster state before reporting status.
4. When a completed pinned benchmark clearly changes the best-known checkpoint for a named condition, update `docs/experiments/latest.md`.
5. When a run is no longer live, preserve the latest detailed state in `docs/experiments/archive/`.
6. Then move the compact summary row to `docs/experiments/done.md`.
7. A non-live job must not remain `RUNNING`. Record the scheduler terminal state (`COMPLETED`, `FAILED`, `TIMEOUT`, or `CANCELLED`) separately from the scientific/artifact assessment such as `usable`, `invalid`, `resource failure`, or `pruned`; do not encode a timeout as completion.
8. For Vast.ai work, also update `docs/experiments/vast_current_jobs.md`
   whenever launching, stopping, syncing, or status-checking instances. Keep it
   compact: instance id, endpoint, hardware, live job names, and whether the
   instance is running, idle, exited, or needs sync before destroy.

## Reporting Format

Use one-line snapshots:

```text
<job_id> | <run_name> | <task> | wandb_run=<...> | wandb_url=<...> | timeout=<...> | full=<...> | close=<...>
```

For Vast, replace `<job_id>` with `vast pid=<pid>` or the Vast run id and
include the remote top-level log path.

Keep the report compact and source every field from either the synced Slurm/Vast
log, W&B, or the experiment ledger.
