---
name: rl-newton-benchmark
description: "Benchmark Newton RL checkpoints. Use for FEE policies, analytic shared-turn runs, pinned comparisons, category leaderboards, qualitative clips, and fresh or replayed terrain banks."
---

# Newton Benchmark Workflow

Use this skill for checkpoint evaluation, terrain-bank workflows, and benchmark interpretation.

## Source Of Truth

Read only what matches the task:

- `docs/PARTIALLY_DUG_TERRAIN_WORKFLOW.md`
- `docs/SHARED_TURN_DEFAULT_TRAINING.md`
- `docs/benchmarks/README.md`
- `docs/experiments/README.md`
- `scripts/mole_environments/fee_excavation/README.md`
- `scripts/mole_environments/fee_excavation/benchmark/README.md`
- `scripts/mole_environments/fee_excavation/benchmark/benchmark_fee_excavation.py`
- `scripts/benchmark/benchmark_excavation_w_cabin_analytic.py`
- `scripts/benchmark/verify_success_terrain_bank.py`
- `scripts/benchmark/analyze_benchmark.py`

## Hard Rules

- Benchmark before promoting a checkpoint or recipe.
- Run benchmark executions and noisy benchmark-log parsing in a worker agent by default. The parent thread should keep only the command contract, report paths, and compact metrics.
- For shared-turn analytic tasks:
  - use the analytic env
  - keep empirical normalization disabled
  - use the torch soil backend only
- Compare fresh-RBF performance before reasoning about carved-terrain replay.
- Do not trust a saved terrain bank until same-episode verification passes.

## Fast Evidence Funnel

Use the cheapest stage that can answer the current question:

1. Run focused import, shape, coordinate, and contract tests; target seconds, not minutes.
2. Run one bounded vectorized CUDA smoke. For Newton simulators designed for batched execution, start at 512 worlds (normally stay within 128-1024), use one macro or the smallest useful step horizon, disable W&B, and impose a 2-5 minute wall-clock limit.
3. Scale to a development bank only after the smoke passes and the next decision needs behavior across geometries.
4. Run the full pinned promotion bank, repeats, confidence intervals, videos, and tail analysis only for a promotion or paper claim that needs them.

Never benchmark a batched GPU environment with `num_envs=1` merely because it
looks cheaper; simulator setup and synchronized rollout overhead can make that
much slower. Reserve one-world runs for trace inspection, rendering, or
reproducing a specific numerical failure. Emit aggregate counts, rates, timing,
and artifact paths to the console; keep per-world rows in files. Start one
bounded command, wait for completion or its hard timeout, and avoid repeated
status polling.

## Main Entry Points

- Generic excavation benchmark: `scripts/benchmark/benchmark_excavation.py`
- Shared-turn / analytic cabin benchmark: `scripts/benchmark/benchmark_excavation_w_cabin_analytic.py`
- FEE excavation benchmark: `scripts/mole_environments/fee_excavation/benchmark/benchmark_fee_excavation.py`
- FEE category leaderboard: `scripts/mole_environments/fee_excavation/benchmark/benchmark_fee_leaderboard.py`
- Terrain-bank verifier: `scripts/benchmark/verify_success_terrain_bank.py`
- Result summarizer: `scripts/benchmark/analyze_benchmark.py`
- Visual sanity checks: `scripts/debug/capture_terra_snapshot.py`, `scripts/debug/capture_shared_turn_snapshot.py`

## Delegated Benchmark Execution

Use a worker subagent for benchmark runs, benchmark sweeps, large JSON/report comparisons, or any task likely to print long Newton/Warp/IsaacLab logs. This is Lorenzo's standing preference for this skill unless the active turn explicitly asks to keep execution in the parent thread or delegation tools are unavailable.

Parent responsibilities:

- choose the benchmark contract: checkpoint(s), seed, curriculum level, terrain source, env count, step budget, output directory, and any reset-cache namespace
- pass only the exact commands and expected summary fields to the worker
- keep the main context free of stdout/stderr; do not paste full benchmark logs into the parent conversation
- after the worker returns, read only the final report JSON/Markdown if a specific number needs verification

Worker instructions should include:

```text
You are the Newton benchmark worker. Run the provided benchmark commands sequentially on the local GPU; do not start concurrent benchmark processes. Keep raw stdout/stderr out of your final answer unless a command fails. Write outputs under the requested output dirs. Return only: command status, report paths, success rate, desired_full/close/partial counts, major negative terminations, key tracking precision, torque/load metrics when present, and any failure traceback summary.
```

If a benchmark fails, the worker should return the shortest useful failure diagnosis plus the command that failed. The parent can then decide whether to inspect logs or patch code.

## FEE Benchmark Bundle

Use this as the default contract for `fee_excavation` checkpoint evaluation, especially for hard-soil questions.

- Do not treat live training ratios as a substitute for benchmark results. If the user asks about hard-soil robustness, tracking precision, or load behavior, read or run the pinned benchmark.
- The default readout must include:
  - overall success rate plus per-type counts or rates for `desired_full`, `desired_close`, `desired_partial`, and negative terminations
  - target-depth precision from the benchmark report or JSON: in-soil target-depth dwell, closest approach or best-from-above, overshoot, and penetration
  - hard-soil preset performance on the current suite: `default_hard_soil_mix`, `fixed_soil_random_hard_cap`, `fixed_soil_hard_interp_25`, `fixed_soil_hard_interp_50`, `fixed_soil_hard_interp_75`, and `asphalt_like`
  - load usage under hard soil: `in_soil_tau_sat_step_mean`, per-joint torque saturation, torque-stage clipping (`preclip` vs `applied` over-limit fractions), and resisting force or power
- For hard-soil failure diagnosis, use the episode JSON to cut negatives by phase, not just by reason:
  - `in_soil_step_count == 0`
  - `in_soil_step_count in [1, 3]`
  - `in_soil_step_count in [4, 10]`
  - `in_soil_step_count >= 11`
  - cross-check those bins against `episode_ever_close_gate`, `episode_ever_high_gate`, `episode_ever_curl_gate`, and `episode_best_fill_ratio`
- If most negatives happen before the fill thresholds used by finish shaping and before any high/curl/close gate, treat the problem as entry/contact handling, not as finish discovery.
- If most negatives happen after the episode has already hit high/curl/close, treat the problem as lift-out / finish / spill control instead.
- Negative attribution is ordered in the task runtime. `bucket_velocity` is currently attributed before `bucket_angle_of_attack`, so benchmark JSON alone does not tell you how often those two raw masks co-fired on the same step.
- If `bucket_velocity` dominates and the user asks whether the policy is simply commanding too much speed, do one more check before proposing changes: compare the last few `qd_desired` arm targets against measured `arm_joint_vel` near failure. If commanded fractions are still modest but measured arm velocity spikes or reverses sign after contact, classify that as entry/contact-control instability under load, not simple policy over-commanding.
- Use that phase split before recommending interventions:
  - mostly early bins suggest entry control, near-soil speed/AoA shaping, or hardness inference
  - mostly mid bins suggest in-soil progress / finish discovery
  - mostly late bins suggest curl / lift / pullup gating or reward issues
- When comparing checkpoints, keep the benchmark contract pinned: same checkpoint number, seed, curriculum level, env count, step budget, and hard-soil preset list.
- When summarizing a hard-soil result, report both finish behavior and tracking behavior. A checkpoint that reaches `desired_close` with shallow penetration is not equivalent to one that reliably reaches `desired_full`.
- On fixed hard-soil presets, `desired_full` is often impossible and is not the main score. Use it as a secondary signal only; the main score is whether the policy tracks the target depth, manages load, avoids unstable negatives, and stays productive under resistance.

## FEE Category Leaderboard

For FEE profile-v2 policy comparisons, prefer the checked-in wrapper instead of manually replaying ad hoc `/tmp/fee_geo14_latest_bench` commands:

```bash
export WANDB_MODE=disabled
uv run python scripts/mole_environments/fee_excavation/benchmark/benchmark_fee_leaderboard.py \
  --source-results-root /tmp/fee_geo14_latest_bench \
  --output-root /tmp/fee_category_leaderboard_bench \
  --num-envs 128 \
  --benchmark-steps 180 \
  --seed 0 \
  --device cuda:0 \
  --curriculum-level 100
unset WANDB_MODE
```

Delegate this runner to a benchmark worker when possible. It runs child benchmarks serially, writes noisy logs to per-run files, reuses compatible reset caches across FEE namespaces, and emits `fee_category_leaderboard.html`. The fast default contract is `128` envs, `180` benchmark steps, and a shared reset cache of `10000` generated / `8000` required samples. Interpret the report by terrain category and tracking quality, not only aggregate success: RBF/profile, entry/exit/continuing, partial-state, 5 cm dwell, shallow fraction, overshoot fraction, best in-soil error, and torque over-limit are all part of the promotion decision.

The default FEE leaderboard suite should stay small and in-distribution for the current training geometry: `default`, `hard_random_scrape_05cm`, `hard_scrape_50cm`, `precision_profile_10cm`, `precision_profile_entry45deg_10cm`, `precision_profile_exit45deg_10cm`, `frontier_largefront`, and `frontier_largefront45`. The flat hard-soil scrape core case is 50 cm below the current soil and uses `fixed_soil_random_hard_cap`, which is in the random-mix soil support; `hard_scrape_05cm` remains an optional shallow stress probe. The frontier cases use profile-v2 front-partial cleanup geometry with default-random soil; `frontier_largefront` uses a 35-65 deg smooth front and `frontier_largefront45` uses a 25-45 deg front. Benchmark-only soils such as interp-25/50/75/asphalt are OOD stress probes, not default promotion cases. Broader 5cm depth/profile or 30/45/60deg sweeps are optional follow-ups, with 60deg treated as an out-of-distribution stress case, not the default promotion view.

Use the canonical aggregate latest page as the browser default:

```text
outputs/fee_benchmarks/fee_category_leaderboard_latest.html
```

Refresh it without running benchmarks:

```bash
uv run python scripts/mole_environments/fee_excavation/benchmark/benchmark_fee_leaderboard.py \
  --global-latest-only
```

This page is the global view across saved `outputs/fee_benchmarks/**/benchmark_results_*.json` files and should include the current table tabs, sortable columns, policy-click filtering, and video links discovered under archived `videos*` folders. Per-batch leaderboards remain useful for provenance, but do not present them as the default "latest" view. Normal leaderboard runs refresh the global latest page unless `--skip-global-latest` is passed.

When evaluating FEE specialists or experts, update the curated expert benchmark
tracker in addition to the raw leaderboard:

- `docs/experiments/fee_specialist_expert_benchmarks.md`
- `docs/experiments/fee_specialist_expert_benchmarks.csv`
- `docs/experiments/fee_specialist_expert_benchmarks.html`

Do this in the active Newton worktree that produced the benchmark, and mirror to
the canonical checkout if the user is looking at the canonical report. The HTML
is the browser-facing dispatch view; do not leave it stale after adding new
expert rows or changing the promotion interpretation. For hard-soil experts,
state explicitly that full scoops may be physically unrealistic, but close
success is only acceptable when depth tracking, stable pullup/lift-out, and
load/torque evidence show productive scraping rather than a cheap shallow-close
solution.

Keep the curated HTML tracker compact. Put detailed interpretation in the
Markdown tracker and use only short table-cell text or links in the HTML. After
manual HTML edits, validate with an HTML parser, `git diff --check`, and a
browser/render sanity check when possible; do not report the page ready if the
dashboard layout is visually broken.

For expert benchmark dashboards, do not expose intermediate checkpoint sweeps
or per-checkpoint rows in the browser-facing HTML. Lorenzo tracks checkpoint
progression in W&B and the raw/Markdown artifacts. The HTML should preserve the
useful high-level view: current best decision per task and a compact
generalist-vs-best-expert delta table showing success/full/close/negative
differences.

When running benchmarks or videos from a worktree, treat the worktree `outputs/fee_benchmarks/<batch>` directory as staging only unless `--output-root` already points inside the canonical archive tree. The global builder scans the canonical repo root, not arbitrary worktree output roots. Before telling the user to open the full leaderboard:

```bash
CANONICAL_BENCH_ROOT=/home/lorenzo/moleworks/moleworks_newton/outputs/fee_benchmarks
rsync -a outputs/fee_benchmarks/<BENCH_ROOT>/ "$CANONICAL_BENCH_ROOT/<BENCH_ROOT>/"
uv run python scripts/mole_environments/fee_excavation/benchmark/benchmark_fee_leaderboard.py \
  --global-latest-only
rg -n "<POLICY_LABEL>|<RUN_ALIAS_OR_UNIQUE_TOKEN>" \
  "$CANONICAL_BENCH_ROOT/global_latest/fee_category_leaderboard.html" | head
```

Only call the global leaderboard ready after the `rg` check finds the new policy rows in `global_latest/fee_category_leaderboard.html`. If videos were recorded, verify that the video links in the global HTML point under `$CANONICAL_BENCH_ROOT/<BENCH_ROOT>/videos*`, not the worktree staging path. Give the browser URL for the canonical global page by default; give the per-batch HTML only when the user explicitly asks for that batch.

For qualitative FEE videos, use `record_fee_leaderboard_videos.py`. Whenever a FEE specialist/leaderboard benchmark is run, or a checkpoint is promoted/pruned based on non-obvious behavior, generate short qualitative clips by default unless the user asks not to. Default behavior should record all raw Newton MP4s first and then batch-transcode browser-safe `_h264.mp4` / `_vp9.webm` variants in parallel across CPU workers. Do not reintroduce per-rollout inline transcoding unless explicitly requested; it leaves the GPU idle while FFmpeg runs. Useful flags:

```bash
uv run python scripts/mole_environments/fee_excavation/benchmark/record_fee_leaderboard_videos.py \
  --benchmark-output-root outputs/fee_benchmarks/<BENCH_ROOT> \
  --policy-label <POLICY_LABEL> \
  --episodes 1 \
  --cases default,hard_random_scrape_05cm,hard_scrape_50cm,precision_profile_10cm,precision_profile_entry45deg_10cm,precision_profile_exit45deg_10cm,frontier_largefront,frontier_largefront45 \
  --viewer gl \
  --soil-wireframe-mode full \
  --run
```

Use `--skip-browser-video-variants` for raw-only recording, `--transcode-workers <n>` to cap CPU parallelism, `--ffmpeg-threads-per-worker <n>` to control FFmpeg threading, and `--inline-browser-video-variants` only for the old slower inline transcode behavior. After videos finish, archive the whole batch into the canonical benchmark tree, rebuild the global latest leaderboard, and verify the policy/video links there before reporting the full leaderboard path.

If a benchmark batch has only a manifest or the result root is not available locally, use manifest-only recording instead of rerunning the benchmark:

```bash
uv run python scripts/mole_environments/fee_excavation/benchmark/record_fee_leaderboard_videos.py \
  --manifest <manifest.json> \
  --benchmark-output-root /tmp/missing-results-ok-with-manifest \
  --top-policies <N> \
  --episodes 1 \
  --cases precision_profile_10cm \
  --max-steps 160 \
  --viewer gl \
  --soil-wireframe-mode full \
  --video-root /home/lorenzo/moleworks/moleworks_newton/outputs/fee_benchmarks/<CLIP_BATCH>/videos_wireframe_full_headless \
  --run
```

For normal benchmark batches, prefer `--benchmark-output-root outputs/fee_benchmarks/<BENCH_ROOT>` and record at least one `precision_profile_10cm` clip per policy/checkpoint under the same archived batch. Validate the finished output with `ffprobe`, an HTML parser count of video/source tags, and one extracted or rendered representative frame when possible. Keep recorder logs out of the main answer unless a command fails.

For GL recording on Lorenzo's workstation, prefer the real X display. The Codex shell may have `DISPLAY` unset even though a display exists; check `/tmp/.X11-unix/X*`, validate with `DISPLAY=:<n> xdpyinfo`, and run the recorder/play command as `env DISPLAY=:<n> ... --viewer gl --headless`. Use `xvfb-run` only when there is no usable real display, and do not switch to USD unless the user explicitly asks.

For torque/load readouts, always report both torque stages when present:
- `in_soil_tau_preclip_*`: policy/servo torque before applied compensation
- `in_soil_tau_sat_*` and `in_soil_tau_applied_over_limit_*`: actual applied torque including compensation

If the user says "do not go above 1", assume they mean applied torque until clarified. A policy can have near-zero preclip over-limit while still violating applied torque because compensation is added after clipping.

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

- For existing small result files, the parent may inspect JSON directly.
- For new benchmarks, repeated comparisons, or noisy log parsing, delegate to the benchmark worker and keep only compact result tables in the parent.
- Prefer exact benchmark commands with pinned contracts over ad-hoc reruns.
- If delegation tools are unavailable, run the minimum benchmark locally, keep output bounded, and summarize from the generated report files rather than from streamed logs.
