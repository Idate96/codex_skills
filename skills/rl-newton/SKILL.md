---
name: rl-newton
description: "End-to-end Newton RL workflow in moleworks_newton: local smoke, cluster submit/monitor/debug, sync logs, benchmark checkpoints, and run post-benchmark analysis with strict experiment ledgers."
---

# RL + Newton Workflow (moleworks_newton)

Single-skill workflow for Newton RL: local smoke test -> cluster submit -> monitor/debug -> sync -> benchmark -> analysis.

## 0) Shared-Turn Excavation Notes

For the shared-turn excavation tasks:

- `m445_excavation_shared_turn`
- `m445_excavation_shared_turn_w_cabin`

use the analytic Newton stack, not the Newton MPM excavation env.

Hard rules for those tasks:

- Keep empirical normalization disabled with `--disable-empirical-normalization`.
- Use the torch soil path only for parity/training/recovery on shared-turn analytic tasks.
- Do not use the warp soil path here; it is experimental and unsupported for shared-turn analytic training, terrain-bank replay, and reliability work.
- Treat Euler `4090` as the default target GPU. Only fall back to `3090` if the run stays pending too long.
- Use W&B logging for real Newton RL training runs by default, especially cluster jobs and any run you want to compare later.
- Do not use TensorBoard as the training logger unless the user explicitly asks for it.
- Pass `--logger wandb --wandb-project moleworks_newton` unless the task already defaults to W&B.
- For Newton-side Python helpers, benchmarks, and W&B API queries, use `uv run` from the repo root instead of relying on host-installed packages.
- Do not install host `wandb` or other Python deps if `uv run` in the repo environment is sufficient.
- For local quick smoke / debug training runs, disable W&B so we do not spam the project with garbage runs.
- Maintain the canonical run ledgers in `docs/experiments/README.md`, `docs/experiments/running.md`, and `docs/experiments/done.md`.
- Treat `docs/EXPERIMENTS_ONGOING.md` and `docs/EXPERIMENTS_RUN.md` as compatibility wrappers only.
- If you change runtime speed-sensitive code, update the shared-turn speed ledger in `docs/SHARED_TURN_SPEED_BENCHMARKS.md`.

Partially dug analytic terrain workflow:

- Use the terrain-bank path only for the analytic shared-turn cabin task.
- The saved bank contains analytic terrain/task state, not particle state.
- Successful bank entries are terminal `desired_full` and `desired_close` episodes only.
- Terrain-bank collection is stricter than the termination reason alone: require a nonzero applied excavation footprint from `Soil3D.export_excavation_trace()` before saving an entry.
- Terrain-bank collection is only valid when soil-height updates are enabled during collection.
- For progressively carved banks, use:
  - `--save-success-terrain-repeat-count N` to require `N` successful digs on the same terrain before saving
  - `--save-success-terrain-until-exhausted` to stop early once remaining diggable volume is exhausted
  - `--save-success-terrain-repeat-count 0 --save-success-terrain-until-exhausted` for pure exhaustion-mode collection
- Before trusting a bank, run the same-episode verifier and require:
  - `saved_bank_to_exported_max_abs_change == 0`
  - `exported_to_terminal_selected_surface_max_abs_change == 0`
  - nonzero `initial_to_terminal_*_max_abs_change`
  - plots based on the actual applied excavation footprint, not a guessed bucket-path rectangle
- Benchmark collection reports now expose `Skipped Uncarved Successes`; treat nonzero values as expected when the policy can terminate successfully without producing a real carved terrain snapshot.
- The primary flow is:
  1. benchmark on fresh RBF terrain
  2. save successful carved terrains with `--save-success-terrain-bank --enable-soil-height-updates`
  3. verify a same-episode saved entry with `scripts/benchmark/verify_success_terrain_bank.py`
  4. replay benchmark with `--terrain-bank-source terrain_bank`
  5. only then try mixed finetuning with `--terrain-bank-source mixed`
- If a harsher repeated-carve bank still underperforms after mixed finetuning, test a pure carved-terrain adaptation run with `--terrain-bank-source terrain_bank` instead of more low-probability mixes.
- For repeated-carve recovery on the current shared-turn branch, prefer the helper:
  - `cluster/submit_shared_turn_w_cabin_partially_dug.sh --terrain-bank-path <BANK_PT> --terrain-bank-probability <P>`
  - Current first recovery family mixes a `repeat-count=2` carved bank at low probability (`0.05`, `0.10`, `0.20`) while resuming from `aoasafe@6750`.

## 0.1) Kinematics Contract Notes

When debugging policy parity, bucket/EE observations, or Jacobian-based quantities in Newton:

- `State.body_q` is the body-frame/body-origin pose.
- `State.body_qd` stores a COM-referenced twist in world coordinates.
- On Newton `main` and on the FK-fix branch, public `eval_jacobian()` behaves like a world-origin screw Jacobian in world coordinates; it is not transported to the row body's origin or COM.
- Therefore `eval_jacobian(...) @ joint_qd` is not, in general, equal to `State.body_qd`.
- If you need body-origin, COM, or tool-point velocity parity, transport the screw twist to the desired point or compare against an independent kinematics implementation (for example Pinocchio) plus finite-difference sanity checks.
- For translated-joint descendant velocity bugs, the upstream FK correctness fix is tracked separately from the public Jacobian contract discussion. Do not assume a Jacobian mismatch implies FK is wrong, or vice versa.
- Policies trained before the translated-joint FK fix may have learned against wrong descendant linear velocity and derived AOA terms. Re-benchmark those checkpoints on the fixed Newton branch before trusting parity or sim-to-real conclusions.

## 0.2) ROS Stack Cleanup After Parity / Benchmark Tests

If a Newton RL debugging session uses the ROS stack for parity checks,
TorchScript benchmarking, or Dig3D controller replay, always clean it up before
finishing or before starting a fresh run.

Minimum cleanup:

1. Stop the tmux session that launched the ROS stack inside the container.
2. Kill ad hoc background helpers such as flat map publishers or custom bag
   recorders.
3. Check for orphaned `ros2 launch`, controller, state-publisher, and
   `robot_state_publisher` processes.
4. Verify the container is back near idle RAM / GPU VRAM before the next test.

Preferred teardown sequence:

1. Kill the tmux session first.
2. Immediately scan for orphaned ROS/Newton processes.
3. Terminate the exact PIDs that remain.
4. Re-run the process scan and resource checks.

Example inside the ROS container:

```bash
tmux kill-session -t <session_name> 2>/dev/null || true

ps -eo pid,cmd | rg 'newton_bridge.launch.py|standalone_dig_newton_env.py|dig_3d_controller_cpp.launch.py|robot.launch.py|mole_state_publisher.launch.py|compare_dig3d_live_obs.py|publish_flat_excavation_map|ros2 action send_goal'

kill <pid1> <pid2> ... 2>/dev/null || true
sleep 2
kill -9 <pid1> <pid2> ... 2>/dev/null || true

ps -eo pid,cmd | rg 'newton_bridge.launch.py|standalone_dig_newton_env.py|dig_3d_controller_cpp.launch.py|robot.launch.py|mole_state_publisher.launch.py|compare_dig3d_live_obs.py|publish_flat_excavation_map|ros2 action send_goal' || true
```

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
  launching another Newton RL parity or benchmark session.
- If tmux is gone but the launched processes remain, treat that as a failed
  cleanup and kill the orphaned PIDs explicitly before the next run.

## 0.3) Preferred ROS Stack Bringup For Parity

If you are validating Newton observations or replaying TorchScript policies
through `moleworks_ros`, prefer launching the ROS stack from an interactive
shell inside the running `moleworks_ros` container, then managing the stack via
`tmux` inside that same shell.

Why:

- it avoids brittle nested host-side quoting for long `ros2 launch ... extra_args:=...` commands
- it makes it easy to inspect pane output live and restart only one part of the stack
- it reduces accidental overlap between old and new sim sessions

Recommended order inside the container:

1. Attach interactively:

```bash
docker exec -it <container> bash --noprofile --norc
```

2. Create a fresh tmux session:

```bash
tmux kill-session -t <session_name> 2>/dev/null || true
tmux new-session -d -s <session_name> -n newton
tmux new-window -t <session_name> -n robot
tmux new-window -t <session_name> -n state_pub
tmux new-window -t <session_name> -n dig
tmux new-window -t <session_name> -n compare
```

3. Launch in this order:
   - `newton`
   - `robot`
   - `state_pub`
   - wait for `/clock`, the terrain topic, and `map -> CABIN`
   - only then start `dig` and `compare`

4. Use `tmux capture-pane -p -t <session>:<window>` between steps instead of
   starting all windows at once blindly.

Minimum health checks before starting Dig3D:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/moleworks/ros2_ws/install/setup.bash
timeout 15 bash -lc "ros2 topic echo /clock --once"
timeout 15 bash -lc "ros2 topic echo /excavation_mapping/grid_map --once"
timeout 15 bash -lc "ros2 run tf2_ros tf2_echo map CABIN 2>&1" | head -40
```

If direct analytical terrain publishing is enabled, keep the standard
perception stack off unless you explicitly want to compare both paths. Do not
leave another publisher on the controller-facing terrain topic by accident.

## 1) Always Run Newton Python via `uv`

Use `uv` from repo root:

```bash
uv run python scripts/rsl_rl/train.py --task <TASK> --num-worlds <N>
```

Do not use IsaacLab wrappers here.
Do not assume host Python has the needed packages.

For W&B inspection from the command line, use the repo env too:

```bash
uv run python - <<'PY'
import wandb
api = wandb.Api()
run = api.run("idate96/moleworks_newton/<run_id>")
print(run.summary.get("Episode Termination/last_full"))
PY
```

## 2) Local Smoke Test (ALWAYS first)

Disable W&B for local smoke runs so quick checks do not pollute the project:

```bash
export WANDB_MODE=disabled
uv run python scripts/rsl_rl/train.py \
  --task m445_excavation_w_cabin_analytic \
  --num-worlds 8 \
  --max-iterations 3
unset WANDB_MODE
```

For shared-turn excavation tasks, use a smoke command that matches the real training contract:

```bash
export WANDB_MODE=disabled
uv run python scripts/rsl_rl/train.py \
  --task m445_excavation_shared_turn_w_cabin \
  --num-worlds 8 \
  --max-iterations 3 \
  --disable-empirical-normalization
unset WANDB_MODE
```

Mixed terrain-bank smoke:

```bash
export WANDB_MODE=disabled
uv run python scripts/rsl_rl/train.py \
  --task m445_excavation_shared_turn_w_cabin \
  --num-worlds 8 \
  --max-iterations 1 \
  --disable-empirical-normalization \
  --debug-reset-cache \
  --terrain-bank-source mixed \
  --terrain-bank-path <BANK_PT> \
  --terrain-bank-mixed-probability 0.25 \
  --checkpoint <CHECKPOINT>
unset WANDB_MODE
```

## 2.1) Newton Visual Sanity Check

When a local Newton task looks suspicious, capture a host-side snapshot before changing training code. This is especially useful after robot/terrain/controller edits.

From `moleworks_newton/`:

```bash
PYTHONPATH=$PWD:$HOME/moleworks/newton uv run python scripts/debug/capture_terra_snapshot.py \
  --mode full \
  --viewer-mode visible \
  --device cuda:0 \
  --steps-per-phase 80
```

For rigid obstacle / barrier sanity checks:

```bash
PYTHONPATH=$PWD:$HOME/moleworks/newton uv run python scripts/debug/capture_terra_snapshot.py \
  --mode navigation-only \
  --viewer-mode headless \
  --device cuda:0 \
  --steps-per-phase 20 \
  --rigid-box-preset workspace_walls \
  --camera-pos 6.5 6.5 4.0 \
  --camera-pitch -20 \
  --camera-yaw 225
```

Notes:

- Use `--viewer-mode headless` when no display is available.
- Outputs land in `outputs/debug_snapshots/` as paired `.png` and `.json` files.
- `--rigid-box-preset` currently supports `workspace_walls`, `front_barrier`, and `corridor`. Terra now defaults to `workspace_walls`; pass `none` when you explicitly want an open arena.
- Terra now resolves the checked-in `meshes/*.dae` assets correctly again, so the robot body is visible in snapshots. Treat the capture as a fast geometry/pose sanity check, not a polished render.

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

Prefer `4090` first:

```bash
./submit_job.sh \
  --task m445_excavation_w_cabin_analytic \
  --num-worlds 32000 \
  --gpus 1 \
  --time 24h \
  --gpu-type rtx_4090 \
  --partition gpuhe.24h \
  -- --max-iterations 10000 --logger wandb --wandb-project moleworks_newton
```

If a run sits pending too long, resubmit on `3090`:

```bash
./submit_job.sh \
  --task m445_excavation_w_cabin_analytic \
  --num-worlds 32000 \
  --gpus 1 \
  --time 24h \
  --gpu-type rtx_3090 \
  --partition gpuhe.24h \
  -- --max-iterations 10000 --logger wandb --wandb-project moleworks_newton
```

Multi-GPU:

```bash
./submit_job.sh \
  --task m445_excavation_w_cabin_analytic \
  --num-worlds 64000 \
  --gpus 4 \
  --time 24h \
  --partition gpuhe.24h \
  -- --max-iterations 10000 --logger wandb --wandb-project moleworks_newton
```

### Shared-turn partially dug terrain collection / replay

Collect a success-only bank:

```bash
uv run python scripts/benchmark/benchmark_excavation_w_cabin_analytic.py \
  --task m445_excavation_shared_turn_w_cabin \
  --checkpoint <CHECKPOINT> \
  --num-envs 1024 \
  --benchmark-steps 500 \
  --device cuda:0 \
  --soil-backend torch \
  --curriculum-level 100 \
  --enable-soil-height-updates \
  --save-success-terrain-bank outputs/terrain_banks/<BANK_NAME> \
  --save-success-terrain-limit 512 \
  --output-dir outputs/benchmark_terrain_bank/<RUN_NAME>
```

Collect terrains after two successful digs on the same terrain:

```bash
uv run python scripts/benchmark/benchmark_excavation_w_cabin_analytic.py \
  --task m445_excavation_shared_turn_w_cabin \
  --checkpoint <CHECKPOINT> \
  --num-envs 1024 \
  --benchmark-steps 500 \
  --device cuda:0 \
  --soil-backend torch \
  --curriculum-level 100 \
  --enable-soil-height-updates \
  --save-success-terrain-bank outputs/terrain_banks/<BANK_NAME> \
  --save-success-terrain-limit 256 \
  --save-success-terrain-repeat-count 2 \
  --output-dir outputs/benchmark_terrain_bank/<RUN_NAME>
```

Collect terrains until they are effectively exhausted:

```bash
uv run python scripts/benchmark/benchmark_excavation_w_cabin_analytic.py \
  --task m445_excavation_shared_turn_w_cabin \
  --checkpoint <CHECKPOINT> \
  --num-envs 1024 \
  --benchmark-steps 500 \
  --device cuda:0 \
  --soil-backend torch \
  --curriculum-level 100 \
  --enable-soil-height-updates \
  --save-success-terrain-bank outputs/terrain_banks/<BANK_NAME> \
  --save-success-terrain-limit 128 \
  --save-success-terrain-repeat-count 0 \
  --save-success-terrain-until-exhausted \
  --save-success-terrain-exhausted-relative-threshold 0.01 \
  --output-dir outputs/benchmark_terrain_bank/<RUN_NAME>
```

Verify one saved entry against the same episode before reset:

```bash
uv run python scripts/benchmark/verify_success_terrain_bank.py \
  --task m445_excavation_shared_turn_w_cabin \
  --checkpoint <CHECKPOINT> \
  --device cuda:0 \
  --soil-backend torch \
  --num-envs 16 \
  --max-policy-steps 180 \
  --output-dir outputs/terrain_bank_verification/<RUN_NAME>
```

Replay-benchmark on the saved carved terrains:

```bash
uv run python scripts/benchmark/benchmark_excavation_w_cabin_analytic.py \
  --task m445_excavation_shared_turn_w_cabin \
  --checkpoint <CHECKPOINT> \
  --num-envs 1024 \
  --benchmark-steps 500 \
  --device cuda:0 \
  --soil-backend torch \
  --curriculum-level 100 \
  --disable-empirical-normalization \
  --terrain-bank-source terrain_bank \
  --terrain-bank-path <BANK_PT> \
  --output-dir outputs/benchmark_terrain_bank/<REPLAY_RUN>
```

If replay is worse than fresh-RBF performance, finetune on a mixture:

```bash
uv run python scripts/rsl_rl/train.py \
  --task m445_excavation_shared_turn_w_cabin \
  --checkpoint <CHECKPOINT> \
  --num-worlds 48000 \
  --disable-empirical-normalization \
  --terrain-bank-source mixed \
  --terrain-bank-path <BANK_PT> \
  --terrain-bank-mixed-probability 0.25
```

If the mixed run still loses on the carved-bank replay, probe pure bank-only finetuning:

```bash
uv run python scripts/rsl_rl/train.py \
  --task m445_excavation_shared_turn_w_cabin \
  --checkpoint <CHECKPOINT> \
  --num-worlds 48000 \
  --disable-empirical-normalization \
  --terrain-bank-source terrain_bank \
  --terrain-bank-path <BANK_PT>
```

### Shared-turn excavation reward sweeps

The shared-turn Newton excavation tasks accept explicit reward-schedule overrides from `scripts/rsl_rl/train.py`.

Time-gated example:

```bash
./submit_job.sh \
  --task m445_excavation_shared_turn_w_cabin \
  --num-worlds 48000 \
  --gpus 1 \
  --time 24h \
  --gpu-type rtx_4090 \
  --partition gpuhe.24h \
  -- \
  --seed 201 \
  --max-iterations 400 \
  --logger wandb \
  --wandb-project moleworks_newton \
  --disable-empirical-normalization \
  --reward-curriculum-mode timesteps \
  --close-reward-weight 0.2 \
  --max-depth-tracking-weight 0.0 \
  --pullup-barrier-weight 0.0 \
  --close-reward-curriculum-end-weight 0.35 \
  --max-depth-tracking-curriculum-end-weight 0.005 \
  --pullup-barrier-curriculum-end-weight 0.01 \
  --reward-curriculum-start-timesteps 160000000 \
  --reward-curriculum-end-timesteps 320000000
```

Performance-gated example:

```bash
./submit_job.sh \
  --task m445_excavation_shared_turn_w_cabin \
  --num-worlds 48000 \
  --gpus 1 \
  --time 24h \
  --gpu-type rtx_4090 \
  --partition gpuhe.24h \
  -- \
  --seed 201 \
  --max-iterations 400 \
  --logger wandb \
  --wandb-project moleworks_newton \
  --disable-empirical-normalization \
  --reward-curriculum-mode performance \
  --close-reward-curriculum-end-weight 0.35 \
  --max-depth-tracking-curriculum-end-weight 0.005 \
  --pullup-barrier-curriculum-end-weight 0.01 \
  --reward-curriculum-start-metric 0.22 \
  --reward-curriculum-end-metric 0.50
```

Notes:

- For single-GPU runs, the timestep schedule uses total timesteps = `num_worlds * env_step_count`.
- Fresh runs do not need anything special, but resumed time-gated runs rely on the launcher restoring the env step counter from the checkpoint.
- Invalid reward schedule ranges should fail at startup; do not ignore those errors.

### Current branch-default `w_cabin` finetune entrypoint

On the floating-base shared-turn experiment branch, prefer the branch-local helper when you want the current default training recipe:

```bash
cd cluster
./submit_shared_turn_w_cabin_default.sh
```

That helper currently encodes the default `aoasafe@6750` resume recipe for:

- `m445_excavation_shared_turn_w_cabin`
- Euler `1x4090`
- `48k` worlds
- `24h`
- `wandb`
- empirical normalization disabled

Use `--help` on that helper to override seed, checkpoint, GPU type, time, run name, or to pass extra `train.py` args through.

### Brev

```bash
./submit_job_brev.sh \
  --task m445_excavation_w_cabin_analytic \
  --num-worlds 32000 \
  --gpus 1 \
  --time 24h \
  -- --max-iterations 10000 --logger wandb --wandb-project moleworks_newton
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

### Targeted sync (single run / benchmark prep)

Prefer the main sync entrypoint when the branch has the updated cluster tooling:

```bash
cd cluster
./sync_logs.sh \
  --experiment <experiment> \
  --run-name <run_name> \
  --slurm-job <jobid>
```

This syncs exactly one run into:

```bash
logs/rsl_rl/<experiment>/<run_name>/
```

and, when requested, also syncs:

```bash
logs/slurm/slurm-<jobid>.out
```

Useful shortcuts:

```bash
# whole experiment subtree
./sync_logs.sh --experiment <experiment>

# shorthand
./sync_logs.sh --run-dir <experiment>/<run_name>
```

`cluster/sync_experiment.sh` may exist as a thin wrapper, but `sync_logs.sh` should be the default path.

Fallback if the branch does not have the targeted `sync_logs.sh` options yet:

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
  --task m445_excavation_shared_turn_w_cabin \
  --checkpoint <checkpoint.pt> \
  --num_envs 2048 \
  --benchmark_steps 300 \
  --seed 0 \
  --device cuda:0 \
  --output_dir artifacts/benchmark_w_cabin/<run_tag>
```

Note:
- `benchmark_excavation_w_cabin_analytic.py` accepts both ext-style checkpoints (`model_state_dict`) and Newton RSL-RL checkpoints (`actor_state_dict`).
- Supported analytic parity tasks currently include `m445_excavation_w_cabin_analytic`, `m445_excavation_shared_turn`, and `m445_excavation_shared_turn_w_cabin`.

For shared-turn parity work, benchmark the policy in the matching Newton shared-turn env and record the result in the run ledger before drawing conclusions.

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

Maintain these canonical files in `moleworks_newton/docs/experiments`:
- `README.md`
- `running.md`
- `done.md`
- `archive/` (append-only snapshots for finished batches/runs)

For shared-turn runtime work, also maintain:
- `SHARED_TURN_SPEED_BENCHMARKS.md`
- `SHARED_TURN_ALLOWED_MISMATCHES.md`

Hard rules:
1. Add a row to `docs/experiments/running.md` immediately after submit.
2. Include `run_name`, `wandb_run`, and `wandb_url` (`NA` if unavailable).
3. Do not launch additional runs before updating the ongoing ledger.
4. When a run finishes/fails/is benchmarked, preserve the detailed state in `docs/experiments/archive/` and move the compact summary row to `docs/experiments/done.md`.
5. Reconcile `docs/experiments/running.md` against `squeue` before status reports.
6. If a row is not in `squeue` anymore, treat it as finished, benchmark/archive it, and remove it from `running.md` in the same session.
7. For shared-turn parity/debugging, record whether the run used `4090` or `3090`, whether empirical normalization was disabled, and which reward schedule mode was used.

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
