#!/usr/bin/env python3
"""Reconstruct split rosbag roots from a flattened Kleinkram mission payload.

Kleinkram uploads flatten files into one directory and store provenance in
``upload_name_map.yaml``. This helper rebuilds one run root per original source
directory so replay launchers that expect ``sensors/``, ``state/``,
``commands/``, ``lidar/``, and ``elevation_map/`` can run unchanged.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path

import yaml


REQUIRED_TOP_LEVELS = ("sensors", "state", "commands", "lidar", "elevation_map")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Rebuild one split-bag root per original run from a flattened "
            "Kleinkram mission directory that contains upload_name_map.yaml."
        )
    )
    parser.add_argument(
        "mission_root",
        help="Flattened mission directory containing upload_name_map.yaml",
    )
    parser.add_argument(
        "--output-root",
        help=(
            "Destination directory for reconstructed runs. "
            "Defaults to <mission_root>/reconstructed_runs."
        ),
    )
    parser.add_argument(
        "--link-mode",
        choices=("hardlink", "symlink", "copy"),
        default="hardlink",
        help="How to materialize files in the reconstructed runs.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace conflicting existing files in the output tree.",
    )
    parser.add_argument(
        "--skip-missing",
        action="store_true",
        help=(
            "Skip mapping rows whose flattened payload file is missing locally. "
            "Useful when the download intentionally skipped a corrupt remote file."
        ),
    )
    return parser.parse_args()


def ensure_file(path: Path, description: str) -> None:
    if not path.is_file():
        raise SystemExit(f"{description} not found: {path}")


def load_mapping(mapping_path: Path) -> dict:
    ensure_file(mapping_path, "mapping file")
    try:
        data = yaml.safe_load(mapping_path.read_text())
    except yaml.YAMLError as exc:
        raise SystemExit(f"failed to parse {mapping_path}: {exc}") from exc
    if not isinstance(data, dict) or "files" not in data or not isinstance(data["files"], list):
        raise SystemExit(f"unexpected mapping format in {mapping_path}")
    return data


def _existing_matches(dest: Path, src: Path, link_mode: str) -> bool:
    if not dest.exists():
        return False
    if link_mode == "symlink":
        return dest.is_symlink() and Path(os.readlink(dest)) == src
    try:
        return dest.samefile(src)
    except FileNotFoundError:
        return False


def _remove_existing(dest: Path) -> None:
    if dest.is_dir() and not dest.is_symlink():
        raise SystemExit(f"refusing to replace directory with file: {dest}")
    dest.unlink()


def _materialize(src: Path, dest: Path, link_mode: str, overwrite: bool) -> None:
    if dest.exists() or dest.is_symlink():
        if _existing_matches(dest, src, link_mode):
            return
        if not overwrite:
            raise SystemExit(f"destination exists and differs: {dest}")
        _remove_existing(dest)

    dest.parent.mkdir(parents=True, exist_ok=True)
    if link_mode == "hardlink":
        os.link(src, dest)
    elif link_mode == "symlink":
        os.symlink(src, dest)
    elif link_mode == "copy":
        shutil.copy2(src, dest)
    else:
        raise AssertionError(f"unsupported link mode: {link_mode}")


def _run_name_from_source(source_abs: str) -> str:
    source_path = Path(source_abs)
    try:
        return source_path.parents[1].name
    except IndexError as exc:
        raise SystemExit(f"cannot infer run name from source_abs={source_abs!r}") from exc


def reconstruct_runs(
    mission_root: Path,
    output_root: Path,
    mapping: dict,
    *,
    link_mode: str,
    overwrite: bool,
    skip_missing: bool,
) -> tuple[dict[str, set[str]], list[Path]]:
    discovered: dict[str, set[str]] = {}
    missing_files: list[Path] = []
    for row in mapping["files"]:
        if not isinstance(row, dict):
            raise SystemExit("mapping row is not a dict")
        upload_name = row.get("upload_name")
        source_rel = row.get("source_rel")
        source_abs = row.get("source_abs")
        if not all(isinstance(v, str) for v in (upload_name, source_rel, source_abs)):
            raise SystemExit(f"malformed mapping row: {row}")

        src_file = mission_root / upload_name
        if not src_file.is_file():
            if skip_missing:
                missing_files.append(src_file)
                continue
            ensure_file(src_file, "flattened payload file")
        run_name = _run_name_from_source(source_abs)
        rel_path = Path(source_rel)
        if rel_path.is_absolute():
            raise SystemExit(f"expected relative source_rel, got: {source_rel}")

        dest_file = output_root / run_name / rel_path
        _materialize(src_file, dest_file, link_mode, overwrite)
        discovered.setdefault(run_name, set()).add(rel_path.parts[0])
    return discovered, missing_files


def print_summary(output_root: Path, discovered: dict[str, set[str]], missing_files: list[Path]) -> None:
    print(f"Reconstructed runs under {output_root}")
    for run_name in sorted(discovered):
        present = discovered[run_name]
        missing = [name for name in REQUIRED_TOP_LEVELS if name not in present]
        status = "OK" if not missing else f"missing: {', '.join(missing)}"
        print(f"- {run_name}: {status}")
    if missing_files:
        print(f"Skipped {len(missing_files)} missing payload files:")
        for path in missing_files[:20]:
            print(f"  - {path}")
        if len(missing_files) > 20:
            print(f"  - ... and {len(missing_files) - 20} more")


def main() -> int:
    args = parse_args()
    mission_root = Path(args.mission_root).expanduser().resolve()
    if not mission_root.is_dir():
        raise SystemExit(f"mission root not found: {mission_root}")

    mapping_path = mission_root / "upload_name_map.yaml"
    output_root = (
        Path(args.output_root).expanduser().resolve()
        if args.output_root
        else mission_root / "reconstructed_runs"
    )
    output_root.mkdir(parents=True, exist_ok=True)

    mapping = load_mapping(mapping_path)
    discovered, missing_files = reconstruct_runs(
        mission_root,
        output_root,
        mapping,
        link_mode=args.link_mode,
        overwrite=args.overwrite,
        skip_missing=args.skip_missing,
    )
    print_summary(output_root, discovered, missing_files)
    return 0


if __name__ == "__main__":
    sys.exit(main())
