---
name: rl-isaaclab
description: "End-to-end IsaacLab RL workflow in moleworks_ext: run with isaaclab.sh -p, local smoke tests, cluster submit/monitor/debug on Euler, sync results, benchmark pulled policies, and generate temporal training plots."
---

# RL + IsaacLab Workflow (moleworks_ext)

Single-skill workflow for IsaacLab-based RL: local smoke test → cluster submit → monitor/debug → sync → playback.

## 0) If IsaacLab Debugging Also Uses The ROS Stack

When an IsaacLab investigation includes a ROS parity stack, Dig3D replay, or
Newton-via-ROS comparison, clean that ROS stack up before the next IsaacLab
launch. Do not leave a previous container-side tmux session or ROS launch tree
running in parallel with IsaacLab.

Preferred teardown sequence in the ROS container:

```bash
tmux kill-session -t <session_name> 2>/dev/null || true

ps -eo pid,cmd | rg 'newton_bridge.launch.py|standalone_dig_newton_env.py|dig_3d_controller_cpp.launch.py|robot.launch.py|mole_state_publisher.launch.py|compare_dig3d_live_obs.py|publish_flat_excavation_map|ros2 action send_goal'

kill <pid1> <pid2> ... 2>/dev/null || true
sleep 2
kill -9 <pid1> <pid2> ... 2>/dev/null || true
```

Then verify host/container resources before starting the next GPU workload:

```bash
docker stats --no-stream --format '{{.Name}}\t{{.MemUsage}}\t{{.CPUPerc}}' <container>
nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader,nounits
```

Hard rule:

- If the tmux session is gone but the ROS/Newton processes are still alive,
  cleanup is not done yet.

## 1) Always Run IsaacLab Scripts via Wrapper

**IMPORTANT**: Any script that touches Isaac Sim/Lab must use:

```bash
/workspace/isaaclab/isaaclab.sh -p <script.py> [args]
```

Use this for:
- Isaac Sim/Lab imports
- USD/PhysX
- GPU/RTX
- training/testing scripts

### pxr Import Errors

If you hit `ImportError: No module named 'pxr'`, load the app launcher **inside the file**:

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom
```

## 2) Local Smoke Test (ALWAYS first)

Disable W&B for local smoke tests to avoid junk runs.

```bash
export WANDB_MODE=disabled

/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --task <TASK> --num_envs 4 --max_iterations 3 --headless

unset WANDB_MODE
```

Example (excavation3d_w_cabin):

```bash
export WANDB_MODE=disabled
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --task Moleworks-Isaac-m445-digging-3D-w-cabin --num_envs 4 --max_iterations 3 --headless
unset WANDB_MODE
```

## 3) Submit to Euler Cluster

```bash
JOB_TIME=30m NUM_GPUS=2 GPU_TYPE=rtx_3090 ./docker/cluster/cluster_interface.sh job \
  --task <TASK> --num_envs 64000 --max_iterations 10000
```

Example (excavation3d_w_cabin, 24h, 1 GPU):

```bash
JOB_TIME=24h NUM_GPUS=1 GPU_TYPE=rtx_3090 ./docker/cluster/cluster_interface.sh job \
  --task Moleworks-Isaac-m445-digging-3D-w-cabin --num_envs 64000 --max_iterations 10000
```

Custom entrypoint (optional):

```bash
CLUSTER_EXECUTABLE=scripts/mole_environments/grading/train_grading.py \
  JOB_TIME=2h NUM_GPUS=2 GPU_TYPE=rtx_3090 ./docker/cluster/cluster_interface.sh job \
  --task Moleworks-Isaac-m445-grading --num_envs 64000 --max_iterations 10000
```

Important semantics:
- In multi-GPU mode, `--num_envs` is interpreted as total envs and split across GPUs (e.g., `32000` with `NUM_GPUS=2` becomes `16000/GPU`).
- With `--resume`, `--max_iterations` is treated as additional learning iterations beyond the loaded checkpoint iteration (not a strict total cap).

## 3.2) Reward Sweeps and Direct Checkpoint Resume

Prefer CLI overrides for quick reward sweeps instead of cloning many task IDs.

Current useful train-time overrides in `scripts/rsl_rl/train.py`:
- `--close_reward_weight`
- `--max_depth_tracking_weight`
- `--pullup_barrier_weight`
- `--terminal_penalty`
- `--base_vel_target`
- `--base_vel_sigma`
- `--base_vel_weight`

Example: from-scratch reward sweep arm

```bash
JOB_TIME=8h NUM_GPUS=1 GPU_TYPE=rtx_3090 ./docker/cluster/cluster_interface.sh job \
  --task Moleworks-Isaac-m445-digging-3D-shared-turn-w-cabin \
  --num_envs 48000 \
  --max_iterations 400 \
  --seed 201 \
  --experiment_name Isaac-m445-excavation3D_sharedturn_wcabin_rewards_v1 \
  --run_name wcabrs1_combo_c25_d020_p020_it400_s201 \
  --close_reward_weight 0.25 \
  --max_depth_tracking_weight 0.02 \
  --pullup_barrier_weight 0.02
```

If you want a short fine-tune from an existing checkpoint, use direct resume:

```bash
JOB_TIME=8h NUM_GPUS=1 GPU_TYPE=rtx_3090 ./docker/cluster/cluster_interface.sh job \
  --task Moleworks-Isaac-m445-digging-3D-shared-turn-w-cabin \
  --num_envs 48000 \
  --max_iterations 300 \
  --seed 201 \
  --resume True \
  --resume_path /cluster/project/.../model_250.pt \
  --run_name wcabrs1_ft_combo \
  --close_reward_weight 0.25 \
  --max_depth_tracking_weight 0.02 \
  --pullup_barrier_weight 0.02
```

Notes:
- `--resume_path` bypasses `experiment_name/load_run/checkpoint` lookup and loads the exact checkpoint path you provide.
- Use from-scratch sweeps when you want clean reward comparisons.
- Use short fine-tunes only for fast screening after a strong baseline already exists.

## 3.1) Worktrees on Cluster (Mandatory when using a worktree)

If you are working from a git worktree instead of the main `moleworks_ext` checkout:

- Launch cluster commands from the worktree root, not the main repo root.
- Keep the in-container path canonical. The staged worktree should still mount at the normal `moleworks_ext` path inside the container; do not invent worktree-specific in-container paths.
- Expect gitignored env files such as `docker/cluster/.env.cluster` and `docker/.env.moleworks_ext` to be missing in secondary worktrees. Materialize/copy them locally before submit, or use the repo helper if present.
- Prove the cluster is using the staged worktree code, not just the main checkout. Preferred proof:
  1. stage a worktree-only file or task into the submit
  2. run a cluster smoke script through `CLUSTER_EXECUTABLE`
  3. verify the smoke output or marker file from cluster logs
- Run sync commands from the same worktree you submitted from, so pulled logs/checkpoints land in the matching checkout.

Recommended worktree smoke submit:

```bash
CLUSTER_EXECUTABLE=scripts/utils/cluster_worktree_submit_smoke.py \
  JOB_TIME=30m NUM_GPUS=1 GPU_TYPE=rtx_2080_ti ./docker/cluster/cluster_interface.sh job \
  --tasks <TASK_A> <TASK_B>
```

## 4) Monitor Jobs (Euler)

```bash
ssh euler 'squeue -u $USER'
```

Job states:
- RUNNING: executing
- PENDING: waiting on resources
- FAILED/NODE_FAIL: crashed

## 5) Debug Failed Jobs

```bash
ssh euler 'find /cluster/scratch/$USER -name "slurm-*.out" -mmin -60'
ssh euler 'tail -100 /cluster/scratch/<user>/moleworks_ext_<timestamp>/slurm-<jobid>.out'
ssh euler 'cat /cluster/scratch/<user>/moleworks_ext_<timestamp>/slurm-<jobid>.err'
ssh euler 'tail -50 /cluster/scratch/<user>/moleworks_ext_<timestamp>/slurm-<jobid>.err'
ssh euler 'grep -n "bind mount detected" /cluster/scratch/<user>/moleworks_ext_<timestamp>/slurm-<jobid>.out | head'
```

**Note**: `.err` is where Python tracebacks are.

### Detect Time-Limit Kills Quickly

Use `sacct` to verify if jobs were cancelled by partition wall time:

```bash
ssh euler 'sacct -j <jobid> --format=JobID,State,Elapsed,Timelimit,Partition%12,ExitCode -P'
```

Typical symptom:
- top-level job state is `TIMEOUT`
- batch step state is `CANCELLED` with exit `0:15`

If that happens, rerun on a longer partition/time (for example `PARTITION=gpuhe.24h JOB_TIME=24h`).

### Detect Stale-File-Handle Startup Failures

If a job fails in the first minute with messages like:

```text
Stale file handle
.../docker/cluster/run_singularity.sh: error reading input file
```

then the staged `/cluster/project/.../moleworks_ext_*` code copy is being removed while
the shell is still executing files from it. Treat this as a cluster staging/runtime bug,
not an RL config bug.

Checks:

```bash
ssh euler 'sacct -j <jobid> --format=JobID,State,Elapsed,ExitCode -P'
ssh euler 'tail -120 /cluster/project/.../slurm-<jobid>.out'
ssh euler 'tail -120 /cluster/project/.../slurm-<jobid>.err'
```

Fix:
- Do not early-delete the staged code copy while `run_singularity.sh` is still executing from it.
- Defer removal until job exit, or execute the launcher from a stable path outside the staged tree.

## 6) Sync Results

```bash
./docker/cluster/sync_experiments.sh
# or to save space
./docker/cluster/sync_experiments.sh --remove
```

### Preferred: Live Report + Targeted Sync

Use the local helper to avoid full log sync when you only need active run diagnostics:

```bash
# Report active runs with run_name/task/wandb/timeout/full/close + resolved run_dir
scripts/utils/cluster_run_report.sh

# Restrict to specific jobs
scripts/utils/cluster_run_report.sh --job-ids 58383883 58334585

# Report + sync only params/ for those runs
scripts/utils/cluster_run_report.sh --job-ids 58383883 58334585 --sync-params
```

Then compare configs directly from synced runs:

```bash
python3 scripts/utils/compare_run_configs.py \
  --run-a <run_name_or_run_dir_A> \
  --run-b <run_name_or_run_dir_B>
```

Filter to keys you care about:

```bash
python3 scripts/utils/compare_run_configs.py \
  --run-a fresh24h_32k_ros_limit_margins_s43 \
  --run-b 4gpu64k_24h_it10k_decim3_rtx3090 \
  --contains decimation \
  --contains action_noise \
  --contains curriculum_end_height_above_soil
```

Use this tool first for "why run A vs B differs?" before manual YAML inspection.

### Smoke Jobs vs Training Jobs

Do not talk about "syncing the policy" unless the submitted job actually trains a policy.

- Real training jobs (`scripts/rsl_rl/train.py`) produce checkpoints and must be synced, benchmarked, and tracked in the experiment docs.
- Smoke or diagnostic jobs launched via `CLUSTER_EXECUTABLE` usually do not produce meaningful checkpoints. For those jobs, sync logs only if needed for proof/debugging; do not pretend there is a policy artifact to recommend.
- If a smoke/diagnostic job is used to prove worktree staging, prefer writing a marker file into the shared cluster logs path so the evidence survives workdir cleanup.

## 6.1) Policy Pull Protocol (Mandatory)

When new policies are pulled from cluster logs, do all of the following in order:

1. Identify active cluster runs and map them to experiment/run directories.
2. Sync logs/checkpoints with `sync_experiments.sh`.
3. Benchmark the pulled checkpoints with fixed benchmark settings.
4. Plot temporal training progression for core metrics before recommending a checkpoint.

Do not recommend a new checkpoint from sync alone; always include benchmark evidence.

## 6.2) Experiment Tracking Docs (Mandatory)

Maintain two repo docs in `moleworks_ext/docs`:
- `EXPERIMENTS_ONGOING.md`: only live `RUNNING` / `PENDING` experiments
- `EXPERIMENTS_RUN.md`: archive of completed/stopped/benchmarked experiments

Hard rule:
- Every time you launch a new training/benchmark run (local or cluster), update `EXPERIMENTS_ONGOING.md` in the same work session.
- Do not launch additional runs until docs are updated (name, run_name, date UTC, status, intention, path/job id).
- When a run finishes/stops/is benchmarked, move it to `EXPERIMENTS_RUN.md` before reporting final conclusions.
- Reconcile `EXPERIMENTS_ONGOING.md` against live cluster state (`squeue`) before each monitoring report.
- If a row exists in `EXPERIMENTS_ONGOING.md` but its job id is no longer in `squeue`, treat it as finished by default: benchmark it, archive it in `EXPERIMENTS_RUN.md`, and remove it from `EXPERIMENTS_ONGOING.md` in the same session.

For every submit:
1. Immediately add a row to `EXPERIMENTS_ONGOING.md` with:
   - name
   - training `run_name`
   - W&B run reference (`wandb_run` and URL, or `NA` if unavailable)
   - environment
   - date (UTC)
   - short note / intention
   - job id + workdir/log path if available
2. During monitoring, update status and notes in place.
3. When the run is benchmarked or stopped/completed, move the row to `EXPERIMENTS_RUN.md`.
4. In `EXPERIMENTS_RUN.md`, keep at least:
   - name
   - environment
   - date (UTC)
   - short note / intention
   - concise result summary
   - artifact path(s) (run dir / report / checkpoint)
5. For cluster runs, include a benchmark summary before archiving:
   - checkpoint used
   - success/close/full or termination breakdown
   - dominant failure mode (if any)

Rules:
- Keep timestamps absolute (UTC), not relative.
- If job args are unknown while pending, mark `TBD` and resolve once stdout exists.
- If duplicate submissions happen, track each job id separately and note duplication explicitly.

## 6.3) Reporting Format: Run Name + W&B (Mandatory)

When reporting active runs in chat or docs, always include:
- `run_name`
- `wandb_run` (preferred: W&B run id/name)
- `wandb_url`

If W&B is not initialized or not printed in logs, report explicit fallback:
- `wandb_run=NA`
- `wandb_url=NA`

### Timeout-Dominance Snapshot Format (Mandatory)

When reporting termination-health snapshots, always include these fields per run:
- `job_id`
- `run_name`
- `task` (environment id)
- `wandb_run`
- `wandb_url`
- `last_timeout`
- `last_full`
- `last_close`

Preferred one-line format:

```text
<job_id> | <run_name> | <task> | wandb_run=<...> | wandb_url=<...> | timeout=<...> | full=<...> | close=<...>
```

Never report only raw job ids with metrics; include run name and task on every line.

Quick extraction from SLURM logs:

```bash
# run_name
grep -m1 -oE -- '--run_name [^ ]+' <slurm.out> | awk '{print $2}'

# W&B run URL (if present in stdout or stderr)
grep -Eo 'https://wandb.ai/[^ ]+/runs/[^ ]+' <slurm.out> <slurm.err> | tail -n1
```

## 6.4) What Is Logged Per Run (Current Ground Truth)

From `scripts/rsl_rl/train.py`, every run directory stores:
- `params/env.yaml`
- `params/agent.yaml`
- `params/env.pkl`
- `params/agent.pkl`

Notes:
- Prefer `env.yaml` / `agent.yaml` for fast diffs (`compare_run_configs.py`).
- `env.pkl` / `agent.pkl` are best when exact Python object reconstruction is needed.
- Git snapshot logging is currently disabled in train script (`runner.add_git_repo_to_log` is commented).

## 7) Play Policy Locally

```bash
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task <TASK> \
  --checkpoint logs/rsl_rl/<exp>/<run_dir>/model_150.pt \
  --num_envs 1
```

### Benchmark Latest Checkpoints (Excavation3D / w-cabin)

Prefer the benchmark script for quantitative comparisons:

```bash
/workspace/isaaclab/isaaclab.sh -p scripts/mole_environments/excavation3D/benchmark_excavation.py \
  --task Moleworks-Isaac-m445-digging-3D-w-cabin \
  --checkpoint logs/rsl_rl/<exp>/<run_dir>/model_<N>.pt \
  --num_envs 2048 \
  --benchmark_steps 300
```

Notes:
- Keep `--auto_task_from_checkpoint` and `--sync_eval_from_checkpoint` enabled (defaults) to avoid task/config mismatches.
- Reports are written under `<run_dir>/play_model_<N>/<timestamp>_benchmarking/benchmark_report_*.txt`.
- For fair cross-run comparison, keep `num_envs` and `benchmark_steps` fixed.

### Benchmark Multiple Checkpoints (sweep)

Use a fixed benchmark setup (same `num_envs`, `benchmark_steps`, `seed`) when comparing models:

```bash
RUN_DIR=logs/rsl_rl/<exp>/<run_dir>
for m in 500 750 1000 1250 1500 1750 2000 2250 2500 2750 3000 3250 3500 3750 4000 4250 4500; do
  /workspace/isaaclab/isaaclab.sh -p scripts/mole_environments/excavation3D/benchmark_excavation.py \
    --task Moleworks-Isaac-m445-digging-3D-w-cabin \
    --checkpoint "${RUN_DIR}/model_${m}.pt" \
    --num_envs 1024 \
    --benchmark_steps 400 \
    --seed 0
done
```

Use the generated `benchmark_report_*.txt` files under `play_model_<N>/...` for ranking.

## 8) Temporal Plots (Mandatory after Pull)

After benchmarking pulled policies, generate temporal plots from TensorBoard event files:

- core termination progression: `full`, `close`, `partial`, `timeout`, `negative`
- stability progression: `Train/mean_reward`, `Train/mean_episode_length`

Use:

```bash
python3 scripts/plot_tb_scalars.py \
  --run-dir logs/rsl_rl/<exp>/<run_dir> \
  --plot-core \
  --out-dir outputs/analysis/training_curves
```

This writes:
- `core_progression.png` (fractions + counts + reward/episode length)
- `core_progression.csv` (step-wise fractions/counts/reward)

### On-Demand Plotting (Custom Terms)

To inspect arbitrary temporal metrics (for example specific termination modes), first list tags:

```bash
python3 scripts/plot_tb_scalars.py --run-dir logs/rsl_rl/<exp>/<run_dir> --list-tags
```

Then plot exact tags:

```bash
python3 scripts/plot_tb_scalars.py \
  --run-dir logs/rsl_rl/<exp>/<run_dir> \
  --tags Episode_Termination/goal_reached_full Episode_Termination/goal_reached_close Episode_Termination/time_out
```

Or use regex selection:

```bash
python3 scripts/plot_tb_scalars.py \
  --run-dir logs/rsl_rl/<exp>/<run_dir> \
  --regex '^Episode_Termination/(goal_reached_full|goal_reached_close|time_out|bucket_velocity)$'
```

Temporal progression plots are the default diagnostic source when close/full benchmark numbers look inconsistent with W&B summaries.

## 8.1) ROS Stack Cleanup After Parity / Playback Tests

If an IsaacLab debugging session also uses a ROS stack for parity, playback, or
controller-side policy checks, always tear that ROS side down before finishing or
before relaunching.

Minimum cleanup:

1. Stop the tmux session that launched the ROS stack inside the container.
2. Kill ad hoc background helpers such as flat map publishers or custom bag
   recorders.
3. Check for orphaned `ros2 launch`, controller, state-publisher, and
   `robot_state_publisher` processes.
4. Verify the container is back near idle RAM / GPU VRAM before the next test.

Example inside the ROS container:

```bash
tmux kill-session -t <session_name> || true
ps -eo pid,cmd | rg 'ros2 launch|dig_3d_controller|robot_state_publisher|mole_joint_state_publisher|publish_flat_excavation_map' || true
```

Example host-side checks:

```bash
docker stats --no-stream --format '{{.Name}}\t{{.MemUsage}}\t{{.CPUPerc}}' <container>
nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader,nounits
```

Hard rule:

- Do not leave a previous ROS parity stack running in the background while
  launching another IsaacLab playback or ROS-side parity session.

## RL Debugging Tips

- Init order matters: buffers must exist before ObservationManager uses them.
- CurriculumManager uses `compute()` (EventManager uses `apply()`).
- Keep rewards bounded: `exp(-penalty)` keeps in (0,1].
- Reference `excavation3d_w_cabin` for established API patterns.
- If behavior is odd: check reward farming, velocity distribution, and speed penalties.

## tmux Requirement

When running IsaacLab scripts or ROS commands, open a **new tmux window** so output persists:

```bash
tmux new-window -n rl-isaaclab
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py --task <TASK> --num_envs 4

tmux capture-pane -p
```
