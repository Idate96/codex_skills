#!/usr/bin/env python3
"""Build a Kleinkram-safe payload directory from files and directories."""

from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

SUPPORTED_EXTENSIONS = (".bag", ".mcap", ".db3", ".svo2", ".tum", ".yaml", ".yml")
SAFE_STEM_RE = re.compile(r"[^A-Za-z0-9_-]+")
MAX_STEM_LEN = 50


@dataclass(frozen=True)
class SourceFile:
    path: Path
    source_root: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        dest="inputs",
        action="append",
        required=True,
        help="Input file or directory. Repeat to include multiple sources.",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output payload directory. Files are written to <output>/files.",
    )
    parser.add_argument(
        "--include-ext",
        action="append",
        default=None,
        help="Allowed extension filter (example: .mcap). Repeatable. Default: all supported.",
    )
    parser.add_argument(
        "--link-mode",
        choices=("copy", "hardlink", "symlink"),
        default="hardlink",
        help="How to materialize output files. Falls back to copy on link failure.",
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Remove output directory first if it exists.",
    )
    return parser.parse_args()


def normalize_extensions(include_ext: list[str] | None) -> set[str]:
    if not include_ext:
        return set(SUPPORTED_EXTENSIONS)

    allowed = set()
    for ext in include_ext:
        normalized = ext.strip().lower()
        if not normalized:
            continue
        if not normalized.startswith("."):
            normalized = f".{normalized}"
        allowed.add(normalized)

    unsupported = sorted(allowed.difference(SUPPORTED_EXTENSIONS))
    if unsupported:
        supported = ", ".join(SUPPORTED_EXTENSIONS)
        raise ValueError(
            f"Unsupported extension(s): {', '.join(unsupported)}. Supported: {supported}"
        )
    return allowed


def resolve_inputs(paths: Iterable[str]) -> list[Path]:
    resolved: list[Path] = []
    for raw in paths:
        path = Path(raw).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(f"Input does not exist: {path}")
        resolved.append(path)
    return resolved


def collect_files(inputs: list[Path], allowed_ext: set[str]) -> tuple[list[SourceFile], list[Path]]:
    files: list[SourceFile] = []
    skipped: list[Path] = []

    for input_path in inputs:
        if input_path.is_file():
            if input_path.suffix.lower() in allowed_ext:
                files.append(SourceFile(path=input_path, source_root=input_path.parent))
            else:
                skipped.append(input_path)
            continue

        for candidate in sorted(input_path.rglob("*")):
            if not candidate.is_file():
                continue
            if candidate.suffix.lower() in allowed_ext:
                files.append(SourceFile(path=candidate.resolve(), source_root=input_path))
            else:
                skipped.append(candidate.resolve())

    dedup: dict[Path, SourceFile] = {}
    for entry in files:
        dedup.setdefault(entry.path, entry)
    return list(dedup.values()), skipped


def normalize_stem(stem: str) -> str:
    normalized = SAFE_STEM_RE.sub("_", stem).strip("_")
    if not normalized:
        normalized = "file"
    return normalized[:MAX_STEM_LEN]


def make_unique_filename(stem: str, extension: str, used: set[str]) -> str:
    candidate = f"{stem}{extension}"
    if candidate not in used:
        used.add(candidate)
        return candidate

    suffix_index = 1
    while True:
        suffix = f"__{suffix_index:02d}"
        max_base_len = MAX_STEM_LEN - len(suffix)
        base = stem[: max(1, max_base_len)]
        candidate = f"{base}{suffix}{extension}"
        if candidate not in used:
            used.add(candidate)
            return candidate
        suffix_index += 1


def yaml_quote(text: str) -> str:
    return "'" + text.replace("'", "''") + "'"


def write_mapping_yaml(
    output_path: Path,
    payload_name: str,
    extensions: set[str],
    mappings: list[dict[str, object]],
) -> None:
    with output_path.open("w", encoding="utf-8") as handle:
        handle.write(f"payload: {yaml_quote(payload_name)}\n")
        now_utc = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        handle.write(f"generated_at_utc: {yaml_quote(now_utc)}\n")
        handle.write("allowed_extensions:\n")
        for ext in sorted(extensions):
            handle.write(f"  - {yaml_quote(ext)}\n")
        handle.write("files:\n")
        for entry in mappings:
            handle.write(f"  - upload_name: {yaml_quote(str(entry['upload_name']))}\n")
            handle.write(f"    source_abs: {yaml_quote(str(entry['source_abs']))}\n")
            handle.write(f"    source_rel: {yaml_quote(str(entry['source_rel']))}\n")
            handle.write(f"    source_ext: {yaml_quote(str(entry['source_ext']))}\n")
            handle.write(f"    size_bytes: {int(entry['size_bytes'])}\n")


def materialize_file(source: Path, destination: Path, link_mode: str) -> None:
    if link_mode == "copy":
        shutil.copy2(source, destination)
        return

    try:
        if link_mode == "hardlink":
            os.link(source, destination)
        elif link_mode == "symlink":
            os.symlink(source, destination)
        else:
            raise ValueError(f"Unknown link mode: {link_mode}")
    except OSError:
        shutil.copy2(source, destination)


def main() -> int:
    args = parse_args()
    allowed_ext = normalize_extensions(args.include_ext)
    input_paths = resolve_inputs(args.inputs)

    output = Path(args.output).expanduser().resolve()
    if output.exists():
        if args.clean:
            shutil.rmtree(output)
        else:
            print(f"Output directory exists: {output}", file=sys.stderr)
            print("Use --clean to overwrite.", file=sys.stderr)
            return 1

    payload_files_dir = output / "files"
    payload_files_dir.mkdir(parents=True, exist_ok=True)

    included, skipped = collect_files(input_paths, allowed_ext)
    if not included:
        print("No files matched the selected extensions.", file=sys.stderr)
        return 1

    used_names: set[str] = set()
    mappings: list[dict[str, object]] = []
    included_ext_counter: Counter[str] = Counter()

    for entry in included:
        extension = entry.path.suffix.lower()
        stem = normalize_stem(entry.path.stem)
        upload_name = make_unique_filename(stem, extension, used_names)
        destination = payload_files_dir / upload_name
        materialize_file(entry.path, destination, args.link_mode)
        mappings.append(
            {
                "upload_name": upload_name,
                "source_abs": str(entry.path),
                "source_rel": str(entry.path.relative_to(entry.source_root)),
                "source_ext": extension,
                "size_bytes": entry.path.stat().st_size,
            }
        )
        included_ext_counter[extension] += 1

    map_path = output / "upload_name_map.yaml"
    write_mapping_yaml(map_path, output.name, allowed_ext, mappings)

    map_upload_name = make_unique_filename("upload_name_map", ".yaml", used_names)
    shutil.copy2(map_path, payload_files_dir / map_upload_name)

    print(f"payload: {output}")
    print(f"files_dir: {payload_files_dir}")
    print(f"included_files: {len(mappings)}")
    print(f"mapping_file: {map_path}")
    print(f"mapping_file_in_payload: {map_upload_name}")
    print("included_extensions:")
    for ext, count in sorted(included_ext_counter.items()):
        print(f"  {ext}: {count}")
    print(f"skipped_files: {len(skipped)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
