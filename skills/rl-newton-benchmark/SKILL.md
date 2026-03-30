---
name: rl-newton-benchmark
description: "Benchmark and analyze Moleworks Newton RL checkpoints. Use when benchmarking analytic cabin or shared-turn checkpoints, collecting or replaying terrain banks, verifying a saved bank, comparing fresh vs carved terrain, or summarizing benchmark JSON outputs."
---

# Newton Benchmark Workflow

Use this skill for checkpoint evaluation, terrain-bank workflows, and benchmark interpretation.

## Source Of Truth

Read only what matches the task:

- `docs/PARTIALLY_DUG_TERRAIN_WORKFLOW.md`
- `docs/SHARED_TURN_DEFAULT_TRAINING.md`
- `docs/benchmarks/README.md`
- `docs/experiments/README.md`
- `scripts/benchmark/benchmark_excavation_w_cabin_analytic.py`
- `scripts/benchmark/verify_success_terrain_bank.py`
- `scripts/benchmark/analyze_benchmark.py`

## Hard Rules

- Benchmark before promoting a checkpoint or recipe.
- For shared-turn analytic tasks:
  - use the analytic env
  - keep empirical normalization disabled
  - use the torch soil backend only
- Compare fresh-RBF performance before reasoning about carved-terrain replay.
- Do not trust a saved terrain bank until same-episode verification passes.

## Main Entry Points

- Generic excavation benchmark: `scripts/benchmark/benchmark_excavation.py`
- Shared-turn / analytic cabin benchmark: `scripts/benchmark/benchmark_excavation_w_cabin_analytic.py`
- Terrain-bank verifier: `scripts/benchmark/verify_success_terrain_bank.py`
- Result summarizer: `scripts/benchmark/analyze_benchmark.py`
- Visual sanity checks: `scripts/debug/capture_terra_snapshot.py`, `scripts/debug/capture_shared_turn_snapshot.py`

## Shared-Turn Terrain-Bank Contract

For partially dug shared-turn work:

- use `torch`, not `warp`
- collect only terminal `desired_full` and `desired_close` successes
- require a nonzero applied excavation footprint before saving a bank entry
- require `--enable-soil-height-updates` during collection
- verify the bank against the terminal same-episode terrain before trusting replay

Verification must require:

- `saved_bank_to_exported_max_abs_change == 0`
- `exported_to_terminal_selected_surface_max_abs_change == 0`
- nonzero `initial_to_terminal_*_max_abs_change`

## Evaluation Order

Use this order for carved-terrain work:

1. Fresh-RBF benchmark on the checkpoint.
2. Collect a success-only carved bank if needed.
3. Verify one saved bank entry with `verify_success_terrain_bank.py`.
4. Replay-benchmark with `--terrain-bank-source terrain_bank`.
5. Only if replay is materially worse, consider mixed finetuning.
6. If mixed finetuning still loses on the carved benchmark, probe pure `terrain_bank` finetuning.

## Command Templates

Shared-turn replay benchmark:

```bash
export WANDB_MODE=disabled
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
  --output-dir outputs/benchmark_terrain_bank/<RUN_NAME>
unset WANDB_MODE
```

Same-episode verifier:

```bash
export WANDB_MODE=disabled
uv run python scripts/benchmark/verify_success_terrain_bank.py \
  --task m445_excavation_shared_turn_w_cabin \
  --checkpoint <CHECKPOINT> \
  --device cuda:0 \
  --soil-backend torch \
  --num-envs 16 \
  --max-policy-steps 180 \
  --output-dir outputs/terrain_bank_verification/<RUN_NAME>
unset WANDB_MODE
```

## Context Hygiene

- Prefer preparing exact benchmark commands for the user over running long Python jobs yourself.
- If the task is mainly comparison or summarization, inspect the generated JSON or synced artifacts instead of re-running the benchmark.
- If the task spans many result files or W&B runs, also use `moleworks-subagent-orchestrator`.
