---
name: rl-isaaclab-benchmark
description: "Benchmark Moleworks IsaacLab checkpoints: playback, excavation evaluation, fair ranking, TensorBoard plots, and run comparison."
---

# IsaacLab Benchmark Workflow

Use this skill for checkpoint evaluation, playback, and run-to-run comparison in `moleworks_ext`.

## Source Of Truth

Read only the files needed for the current task:

- `docs/AI_RESEARCHER_WORKFLOW.md`
- `docs/EXPERIMENTS_ONGOING.md`
- `docs/EXPERIMENTS_RUN.md`
- `scripts/rsl_rl/play.py`
- `scripts/mole_environments/excavation3D/benchmark_excavation.py`
- `scripts/utils/compare_run_configs.py`
- `scripts/plot_tb_scalars.py`

## Hard Rules

- Do not recommend a checkpoint from sync alone.
- Keep benchmark settings fixed when comparing checkpoints.
- Use temporal plots after pull when benchmark numbers and live W&B summaries disagree.
- If the job was only a smoke or diagnostic run, do not pretend there is a production policy artifact to rank.
- For UGEP generated/randomized-machine policies, always compare real-machine eval
  performance against a same-contract policy trained on the real machines
  themselves. The critical claim is not only high generated-distribution score;
  it is that randomized/generated excavator training obtains similar
  real-machine performance to real-machine training.
- Treat bucket-velocity and bucket-AoA terminations as possible downstream symptoms, not always primary causes. In excavation policies they are often triggered by torque-limit saturation or compensation errors that make the bucket jerk; check torque utilization, torque-limit events, actuator/compensation settings, and the frames around the termination before attributing the failure to velocity or AoA behavior.

## Main Entry Points

- Play: `scripts/rsl_rl/play.py`
- Excavation benchmark: `scripts/mole_environments/excavation3D/benchmark_excavation.py`
- Config diff: `scripts/utils/compare_run_configs.py`
- Temporal plots: `scripts/plot_tb_scalars.py`

## Playback

```bash
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task <TASK> \
  --checkpoint logs/rsl_rl/<exp>/<run_dir>/model_<N>.pt \
  --num_envs 1
```

Use `--num_envs 1` for visual debugging unless the user explicitly wants a larger headless check.

## UGEP Fixed Replay

For UGEP checkpoint diagnosis, do not wait only on a slow cluster replay. If
the checkpoint and asset root can be synced locally, launch a local headless
fixed replay in the `isaac-lab-moleworks_ext-dev` container and keep the
cluster replay as redundant evidence.

Use the same contract as the training/eval run:

- `--runtime-mode eval`
- the exact `--ppo_preset`
- the exact `--ugep_finish_profile`
- the exact actuator, observation-history, morphology, and asset arguments
- the same `--num_steps` and output diagnostics when comparing against
  cluster replay

For fast triage, first run a smaller local shard or shorter rollout with a
clearly marked output prefix. For final evidence, run the full fixed replay
and write per-asset diagnostics plus any shadow/trajectory diagnostics needed
for the investigation.

When evaluating whether generated or randomized excavator training generalizes,
run the same real-machine fixed replay for:

- the generated/randomized-machine checkpoint under review
- the best available same-contract real-machine-trained checkpoint

Report the generated-vs-real transfer gap and the generated-vs-real-trained
baseline gap. Include full/close split, timeout rate, negative rate, and
per-real-asset spread. A generated/randomized policy that matches strict
full+close but has a much worse full/close split or negative rate is not yet
equivalent evidence of generation quality.

For UGEP, treat desired-full scoop rate as a separate promotion gate, not just
an explanatory split. Always report the absolute desired-full percentage and
the percentage-point delta versus the same-contract real-machine-trained
baseline. Do not rank a checkpoint from strict full+close alone: a policy can
game that metric by barely scraping the surface and terminating through close.

Launch local UGEP replays through tmux inside the dev container so long runs
remain inspectable:

```bash
tmux new-window -n mwext-ugep-replay
tmux send-keys -t mwext-ugep-replay \
  "docker exec -it isaac-lab-moleworks_ext-dev bash -lc 'cd /workspace/moleworks_ext && /workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py <ARGS>'" C-m
tmux capture-pane -pt mwext-ugep-replay -S -200
```

## Benchmark

Excavation3D / w-cabin template:

```bash
/workspace/isaaclab/isaaclab.sh -p scripts/mole_environments/excavation3D/benchmark_excavation.py \
  --task Moleworks-Isaac-m445-digging-3D-w-cabin \
  --checkpoint logs/rsl_rl/<exp>/<run_dir>/model_<N>.pt \
  --num_envs 2048 \
  --benchmark_steps 300
```

For fair cross-run comparison:

- keep `num_envs` fixed
- keep `benchmark_steps` fixed
- keep `seed` fixed
- avoid task/config mismatches between checkpoints

## Multi-Checkpoint Ranking

When sweeping checkpoints, use one fixed benchmark contract:

```bash
RUN_DIR=logs/rsl_rl/<exp>/<run_dir>
for m in 500 1000 1500 2000; do
  /workspace/isaaclab/isaaclab.sh -p scripts/mole_environments/excavation3D/benchmark_excavation.py \
    --task Moleworks-Isaac-m445-digging-3D-w-cabin \
    --checkpoint "${RUN_DIR}/model_${m}.pt" \
    --num_envs 1024 \
    --benchmark_steps 400 \
    --seed 0
done
```

Rank from the generated benchmark reports, not from "latest checkpoint wins".

## Temporal Plots

After pulling or benchmarking a run, generate progression plots:

```bash
python3 scripts/plot_tb_scalars.py \
  --run-dir logs/rsl_rl/<exp>/<run_dir> \
  --plot-core \
  --out-dir outputs/analysis/training_curves
```

Use exact tags or regex when the default core plot is not enough:

```bash
python3 scripts/plot_tb_scalars.py --run-dir logs/rsl_rl/<exp>/<run_dir> --list-tags
python3 scripts/plot_tb_scalars.py --run-dir logs/rsl_rl/<exp>/<run_dir> --regex '^Episode_Termination/'
```

## Comparison Hygiene

- Use `compare_run_configs.py` before manual config diffs.
- If the task spans many benchmark reports, TensorBoard runs, or W&B records, also use `moleworks-subagent-orchestrator`.
- If playback or debugging also uses a ROS parity stack, also use `newton-ros-parity` and `ros2-debugging` and make sure the ROS side is cleaned up before the next IsaacLab launch.
