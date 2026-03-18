#!/usr/bin/env python3
"""Plot TensorBoard scalar temporal progressions for IsaacLab RL runs.

Core mode (recommended) plots:
- Episode outcome fractions from Episode Count/{full,close,partial,timeout,tot_negative}
- Episode outcome raw counts
- Train/mean_reward and Train/mean_episode_length

Custom mode supports plotting arbitrary tags and regex-selected tags.
"""

from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from tensorboard.backend.event_processing import event_accumulator


CORE_COUNT_TAGS = [
    "Episode Count/full",
    "Episode Count/close",
    "Episode Count/partial",
    "Episode Count/timeout",
    "Episode Count/tot_negative",
]

CORE_STABILITY_TAGS = [
    "Train/mean_reward",
    "Train/mean_episode_length",
]


def _resolve_event_file(run_dir: Path | None, event_file: Path | None) -> Path:
    if event_file is not None:
        if not event_file.is_file():
            raise FileNotFoundError(f"Event file not found: {event_file}")
        return event_file

    if run_dir is None:
        raise ValueError("Either --run-dir or --event-file must be provided.")

    event_files = sorted(run_dir.glob("events.out.tfevents.*"))
    if not event_files:
        raise FileNotFoundError(f"No TensorBoard event files found in: {run_dir}")
    return event_files[-1]


def _load_event_accumulator(event_path: Path) -> event_accumulator.EventAccumulator:
    ea = event_accumulator.EventAccumulator(str(event_path), size_guidance={"scalars": 0})
    ea.Reload()
    return ea


def _scalar_series(ea: event_accumulator.EventAccumulator, tag: str) -> tuple[np.ndarray, np.ndarray]:
    vals = ea.Scalars(tag)
    if not vals:
        raise KeyError(f"Missing scalar tag: {tag}")
    steps = np.array([v.step for v in vals], dtype=np.int64)
    data = np.array([v.value for v in vals], dtype=np.float64)
    return steps, data


def _dict_series(ea: event_accumulator.EventAccumulator, tag: str) -> dict[int, float]:
    steps, values = _scalar_series(ea, tag)
    return {int(s): float(v) for s, v in zip(steps, values, strict=True)}


def _intersect_steps(series_maps: list[dict[int, float]]) -> np.ndarray:
    if not series_maps:
        return np.array([], dtype=np.int64)
    shared = set(series_maps[0].keys())
    for s in series_maps[1:]:
        shared &= set(s.keys())
    return np.array(sorted(shared), dtype=np.int64)


def _plot_core(
    ea: event_accumulator.EventAccumulator,
    out_dir: Path,
    title: str | None = None,
) -> tuple[Path, Path]:
    missing = [t for t in CORE_COUNT_TAGS + CORE_STABILITY_TAGS if t not in ea.Tags().get("scalars", [])]
    if missing:
        raise KeyError(f"Missing required core tags: {missing}")

    count_maps = {tag: _dict_series(ea, tag) for tag in CORE_COUNT_TAGS}
    stab_maps = {tag: _dict_series(ea, tag) for tag in CORE_STABILITY_TAGS}

    count_steps = _intersect_steps(list(count_maps.values()))
    if count_steps.size == 0:
        raise RuntimeError("No shared steps found across core Episode Count tags.")

    full = np.array([count_maps["Episode Count/full"][int(s)] for s in count_steps], dtype=np.float64)
    close = np.array([count_maps["Episode Count/close"][int(s)] for s in count_steps], dtype=np.float64)
    partial = np.array([count_maps["Episode Count/partial"][int(s)] for s in count_steps], dtype=np.float64)
    timeout = np.array([count_maps["Episode Count/timeout"][int(s)] for s in count_steps], dtype=np.float64)
    negative = np.array([count_maps["Episode Count/tot_negative"][int(s)] for s in count_steps], dtype=np.float64)
    total = full + close + partial + timeout + negative

    full_frac = np.divide(full, total, out=np.zeros_like(full), where=total > 0.0)
    close_frac = np.divide(close, total, out=np.zeros_like(close), where=total > 0.0)
    partial_frac = np.divide(partial, total, out=np.zeros_like(partial), where=total > 0.0)
    timeout_frac = np.divide(timeout, total, out=np.zeros_like(timeout), where=total > 0.0)
    negative_frac = np.divide(negative, total, out=np.zeros_like(negative), where=total > 0.0)

    stab_steps = _intersect_steps(list(stab_maps.values()))
    if stab_steps.size == 0:
        raise RuntimeError("No shared steps found across core stability tags.")

    mean_reward = np.array([stab_maps["Train/mean_reward"][int(s)] for s in stab_steps], dtype=np.float64)
    mean_ep_len = np.array([stab_maps["Train/mean_episode_length"][int(s)] for s in stab_steps], dtype=np.float64)

    fig, axs = plt.subplots(3, 1, figsize=(12, 12), constrained_layout=True)

    axs[0].plot(count_steps, full_frac, label="full", linewidth=1.6)
    axs[0].plot(count_steps, close_frac, label="close", linewidth=1.6)
    axs[0].plot(count_steps, partial_frac, label="partial", linewidth=1.3)
    axs[0].plot(count_steps, timeout_frac, label="timeout", linewidth=1.3)
    axs[0].plot(count_steps, negative_frac, label="negative", linewidth=1.3)
    axs[0].set_title("Episode Outcome Fractions (temporal)")
    axs[0].set_xlabel("Training iteration")
    axs[0].set_ylabel("Fraction")
    axs[0].set_ylim(0.0, 1.0)
    axs[0].grid(alpha=0.3)
    axs[0].legend(ncols=5, fontsize=9)

    axs[1].plot(count_steps, full, label="full count", linewidth=1.5)
    axs[1].plot(count_steps, close, label="close count", linewidth=1.5)
    axs[1].plot(count_steps, partial, label="partial count", linewidth=1.2)
    axs[1].plot(count_steps, timeout, label="timeout count", linewidth=1.2)
    axs[1].plot(count_steps, negative, label="negative count", linewidth=1.2)
    axs[1].set_title("Episode Outcome Counts (temporal)")
    axs[1].set_xlabel("Training iteration")
    axs[1].set_ylabel("Count")
    axs[1].grid(alpha=0.3)
    axs[1].legend(ncols=3, fontsize=9)

    axs[2].plot(stab_steps, mean_reward, label="Train/mean_reward", linewidth=1.6)
    axs[2].plot(stab_steps, mean_ep_len, label="Train/mean_episode_length", linewidth=1.6)
    axs[2].set_title("Training Stability")
    axs[2].set_xlabel("Training iteration")
    axs[2].grid(alpha=0.3)
    axs[2].legend(fontsize=9)

    if title:
        fig.suptitle(title, fontsize=13)

    out_png = out_dir / "core_progression.png"
    fig.savefig(out_png, dpi=160)
    plt.close(fig)

    out_csv = out_dir / "core_progression.csv"
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "step",
                "full_count",
                "close_count",
                "partial_count",
                "timeout_count",
                "negative_count",
                "full_frac",
                "close_frac",
                "partial_frac",
                "timeout_frac",
                "negative_frac",
            ]
        )
        for i, step in enumerate(count_steps):
            writer.writerow(
                [
                    int(step),
                    float(full[i]),
                    float(close[i]),
                    float(partial[i]),
                    float(timeout[i]),
                    float(negative[i]),
                    float(full_frac[i]),
                    float(close_frac[i]),
                    float(partial_frac[i]),
                    float(timeout_frac[i]),
                    float(negative_frac[i]),
                ]
            )

    return out_png, out_csv


def _plot_custom(
    ea: event_accumulator.EventAccumulator,
    out_dir: Path,
    tags: list[str],
    regex_patterns: list[str],
    title: str | None = None,
) -> Path | None:
    scalar_tags = ea.Tags().get("scalars", [])
    selected: list[str] = []

    for tag in tags:
        if tag not in scalar_tags:
            raise KeyError(f"Requested tag missing from event file: {tag}")
        selected.append(tag)

    for pattern in regex_patterns:
        rx = re.compile(pattern)
        matched = [t for t in scalar_tags if rx.search(t)]
        if not matched:
            raise ValueError(f"No scalar tags matched regex: {pattern}")
        selected.extend(matched)

    # Deduplicate while preserving order
    deduped: list[str] = []
    seen: set[str] = set()
    for tag in selected:
        if tag not in seen:
            seen.add(tag)
            deduped.append(tag)

    if not deduped:
        return None

    fig, ax = plt.subplots(figsize=(12, 6), constrained_layout=True)
    for tag in deduped:
        steps, values = _scalar_series(ea, tag)
        ax.plot(steps, values, label=tag, linewidth=1.3)

    ax.set_title(title or "Custom Scalars (temporal)")
    ax.set_xlabel("Training iteration")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncols=1)

    out_png = out_dir / "custom_progression.png"
    fig.savefig(out_png, dpi=160)
    plt.close(fig)
    return out_png


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot TensorBoard scalar temporal progressions.")
    parser.add_argument("--run-dir", type=Path, default=None, help="Run directory containing events.out.tfevents.*")
    parser.add_argument("--event-file", type=Path, default=None, help="Explicit TensorBoard event file path.")
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/training_curves"), help="Output dir.")
    parser.add_argument("--plot-core", action=argparse.BooleanOptionalAction, default=True, help="Plot core panels.")
    parser.add_argument("--tags", nargs="*", default=[], help="Exact scalar tags to plot.")
    parser.add_argument("--regex", nargs="*", default=[], help="Regex patterns for scalar tags to plot.")
    parser.add_argument("--list-tags", action="store_true", help="List all scalar tags and exit.")
    parser.add_argument("--title", type=str, default=None, help="Optional plot title.")
    args = parser.parse_args()

    event_path = _resolve_event_file(args.run_dir, args.event_file)
    ea = _load_event_accumulator(event_path)
    scalar_tags = sorted(ea.Tags().get("scalars", []))

    if args.list_tags:
        for tag in scalar_tags:
            print(tag)
        if not args.plot_core and not args.tags and not args.regex:
            return

    args.out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] event_file={event_path}")
    print(f"[INFO] out_dir={args.out_dir}")

    if args.plot_core:
        core_png, core_csv = _plot_core(ea, args.out_dir, title=args.title)
        print(f"[INFO] core_plot={core_png}")
        print(f"[INFO] core_csv={core_csv}")

    custom_png = _plot_custom(ea, args.out_dir, args.tags, args.regex, title=args.title)
    if custom_png is not None:
        print(f"[INFO] custom_plot={custom_png}")


if __name__ == "__main__":
    main()
