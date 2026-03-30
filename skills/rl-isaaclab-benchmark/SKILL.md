---
name: rl-isaaclab-benchmark
description: "Benchmark and analyze Moleworks IsaacLab RL checkpoints. Use when playing a policy locally, benchmarking excavation checkpoints, ranking multiple checkpoints fairly, plotting TensorBoard progression, or comparing synced run artifacts before recommending a checkpoint."
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
