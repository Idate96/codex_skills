---
name: workspace-planner-benchmark
description: "Benchmark workspace-planner heuristics with standard HTML bundles, leaderboards, CSVs, trench/fan visuals, and selector comparisons."
---

# Workspace Planner Benchmark

Use this skill when the user asks for workspace planner benchmark runs, standardized HTML bundles, leaderboards, trench/fan visualization indexes, or comparisons between planner selector policies.

## Canonical Rule

Use only the standardized bundle generator:

```text
high_level_planning/workspace_planner/workspace_planner/benchmark_html_bundle.py
```

Do not create ad hoc benchmark HTML folders or one-off direct simulator scripts as the final artifact. If a direct simulator script is needed for diagnosis, label it as temporary and follow up with a standardized bundle run before reporting benchmark results.

A complete standardized bundle has:

```text
index.html
leaderboard.md
manifest.csv
progress.csv
run_metadata.json
```

Per-case Plotly animation HTML files are present only when `--write-animations` is used.

## Repo Setup

Run commands from:

```bash
cd /home/lorenzo/moleworks/ros2_ws/src/moleworks_ros
```

Use:

```bash
PYTHONPATH=high_level_planning/workspace_planner
```

Default output root:

```bash
/home/lorenzo/tmp/workspace_planner_benchmarks
```

## Default Focused Bundle

For the standard trench + fan bundle with `free_lane` `pointwise_score` and `max_strip_volume`, run:

```bash
RUN_NAME="$(date -u +%Y%m%d_%H%M%S)_workspace_planner_focused_standard"
PYTHONPATH=high_level_planning/workspace_planner \
python3 high_level_planning/workspace_planner/workspace_planner/benchmark_html_bundle.py \
  --output-root /home/lorenzo/tmp/workspace_planner_benchmarks \
  --run-name "$RUN_NAME" \
  --scope focused \
  --soil-mode standard \
  --trench-case centered_trench \
  --trench-case deep_centered_trench \
  --geometry-case center_fan \
  --write-animations
```

This should produce:

```text
/home/lorenzo/tmp/workspace_planner_benchmarks/runs/$RUN_NAME/index.html
/home/lorenzo/tmp/workspace_planner_benchmarks/runs/$RUN_NAME/leaderboard.md
/home/lorenzo/tmp/workspace_planner_benchmarks/runs/$RUN_NAME/manifest.csv
```

## Variants

For a faster metrics-only run, omit `--write-animations`. Do this only when the user does not need visual inspection.

For both focused fan geometries, omit `--geometry-case center_fan`, or pass both:

```bash
--geometry-case center_fan --geometry-case deep_center_fan
```

For all focused cases and default focused selectors, use:

```bash
--scope focused
```

with no case filters.

For the larger planner comparison suite, use:

```bash
--scope full
```

Expect this to take substantially longer. Keep the same output contract.

## Reporting

Before reporting that a benchmark is complete, inspect:

```bash
RUN_DIR=/home/lorenzo/tmp/workspace_planner_benchmarks/runs/<run_name>
sed -n '1,120p' "$RUN_DIR/leaderboard.md"
sed -n '1,120p' "$RUN_DIR/manifest.csv"
tail -20 "$RUN_DIR/progress.csv"
```

Report these paths:

- `index.html` for visual inspection
- `leaderboard.md` for ranked summary
- `manifest.csv` for machine-readable rows
- failed or incomplete rows from `manifest.csv`

If a run is killed or interrupted, use `progress.csv` only as partial progress. Do not call the bundle complete unless `index.html`, `leaderboard.md`, `manifest.csv`, and `run_metadata.json` all exist.

## Interpretation Checks

For fan-like geometry bugs, review both animation HTML and `manifest.csv`. Specifically call out:

- whether `center_fan` uses the expected angular range and radial limits
- whether `free_lane__pointwise_score.html` and `free_lane__max_strip_volume.html` both exist
- whether `finish_step`, `executed_steps`, `final_remaining_m3`, and `residual_tolerance_m3` are comparable between selectors
- whether planner success and residual completion disagree

When comparing runs, prefer the newest standardized bundle with matching scope, soil mode, cases, and selectors. Do not compare a standardized bundle against an ad hoc HTML-only folder without saying that the artifact contracts differ.
