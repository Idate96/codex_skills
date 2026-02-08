#!/usr/bin/env python3
"""
Upload one or more ROS/ROS2 bag directories to Kleinkram using the `klein` CLI.

Key behaviors:
- Resolves project name -> project UUID (upload is more reliable with UUIDs).
- Splits nested bag folders into separate missions to avoid filename collisions
  (most commonly repeated `metadata.yaml`).
"""

from __future__ import annotations

import argparse
import os
import re
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Iterable

UUID_RE = re.compile(
    r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}",
    re.IGNORECASE,
)

DATA_EXTS = {".mcap", ".bag", ".db3"}


def _sh(cmd: list[str]) -> str:
    return " ".join(shlex.quote(c) for c in cmd)


def _run(cmd: list[str], *, capture: bool = False, check: bool = True) -> subprocess.CompletedProcess[str]:
    if capture:
        return subprocess.run(
            cmd,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=check,
        )
    return subprocess.run(cmd, check=check)


def _ensure_authenticated() -> None:
    p = subprocess.run(
        ["klein", "list", "projects"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if p.returncode != 0 or "Not Authenticated" in (p.stdout or ""):
        sys.stderr.write(p.stdout or "")
        raise SystemExit("Not authenticated. Run `klein login` and retry.")


def _is_uuid(s: str) -> bool:
    return bool(UUID_RE.fullmatch(s.strip()))


def _resolve_project_id(project: str) -> str:
    if _is_uuid(project):
        return project

    # Preferred: project info prints the UUID.
    p = subprocess.run(
        ["klein", "project", "info", "-p", project],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if p.returncode == 0:
        m = UUID_RE.search(p.stdout or "")
        if m:
            return m.group(0)

    # Fallback: scan list output.
    p = subprocess.run(
        ["klein", "--max-lines", "-1", "list", "projects"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if p.returncode == 0:
        for line in (p.stdout or "").splitlines():
            if project.lower() in line.lower():
                m = UUID_RE.search(line)
                if m:
                    return m.group(0)

    raise SystemExit(f"Could not resolve project id for {project!r}.")


def _iter_bag_groups(root: Path, *, max_depth: int) -> Iterable[tuple[Path, list[Path], Path | None]]:
    """
    Yield (group_dir, data_files, metadata_yaml_or_none) for each directory under root
    that contains at least one supported bag file.
    """
    root = root.resolve()
    for dirpath, dirnames, filenames in os.walk(root):
        d = Path(dirpath)
        rel = d.relative_to(root)
        if len(rel.parts) > max_depth:
            dirnames[:] = []
            continue

        data_files = sorted(
            (d / f for f in filenames),
            key=lambda p: p.name,
        )
        data_files = [p for p in data_files if p.suffix.lower() in DATA_EXTS]
        if not data_files:
            continue

        meta = d / "metadata.yaml"
        yield d, data_files, meta if meta.is_file() else None


def _mission_for_group(
    root: Path,
    group_dir: Path,
    *,
    mission_prefix: str | None,
) -> str:
    base = root.name
    if mission_prefix:
        base = f"{mission_prefix}_{base}"
    rel = group_dir.relative_to(root)
    if rel.parts:
        suffix = "_".join(rel.parts)
        return f"{base}_{suffix}"
    return base


def main() -> int:
    ap = argparse.ArgumentParser(description="Upload ROS/ROS2 bag folders to Kleinkram via `klein`.")
    ap.add_argument("--project", required=True, help="Kleinkram project name or UUID.")
    ap.add_argument(
        "--endpoint",
        choices=["local", "dev", "prod"],
        help="Switch klein endpoint before uploading.",
    )
    ap.add_argument(
        "--mission-prefix",
        help="Prefix added to every derived mission name (example: estimator_eval).",
    )
    ap.add_argument(
        "--max-depth",
        type=int,
        default=3,
        help="Max directory depth to search for nested bag folders (default: 3).",
    )
    ap.add_argument("--no-create", action="store_true", help="Do not pass --create to `klein upload`.")
    ap.add_argument("--dry-run", action="store_true", help="Print commands without uploading.")
    ap.add_argument("paths", nargs="+", help="Bag directories (or individual bag files) to upload.")
    args = ap.parse_args()

    if args.endpoint:
        _run(["klein", "endpoint", args.endpoint], check=True)

    _ensure_authenticated()
    project_id = _resolve_project_id(args.project)

    missions_seen: set[str] = set()
    plan: list[tuple[str, list[Path]]] = []

    for raw in args.paths:
        p = Path(raw).expanduser()
        if not p.exists():
            raise SystemExit(f"Path not found: {raw}")

        if p.is_file():
            if p.suffix.lower() not in DATA_EXTS:
                raise SystemExit(f"Unsupported file type: {p}")
            root = p.parent.resolve()
            mission = (f"{args.mission_prefix}_" if args.mission_prefix else "") + p.stem
            files: list[Path] = []
            meta = root / "metadata.yaml"
            if meta.is_file():
                files.append(meta)
            files.append(p.resolve())
            if mission in missions_seen:
                raise SystemExit(f"Mission name collision: {mission}")
            missions_seen.add(mission)
            plan.append((mission, files))
            continue

        root = p.resolve()
        groups = list(_iter_bag_groups(root, max_depth=args.max_depth))
        if not groups:
            raise SystemExit(f"No bag files found under: {root}")

        for group_dir, data_files, meta in groups:
            mission = _mission_for_group(root, group_dir, mission_prefix=args.mission_prefix)
            if mission in missions_seen:
                raise SystemExit(f"Mission name collision: {mission}")
            missions_seen.add(mission)
            files: list[Path] = []
            if meta is not None:
                files.append(meta)
            files.extend(data_files)
            plan.append((mission, files))

    for mission, files in plan:
        needs_experimental = any(f.suffix.lower() == ".db3" for f in files) or any(
            f.name == "metadata.yaml" for f in files
        )
        cmd = ["klein", "upload", "-p", project_id, "-m", mission]
        if not args.no_create:
            cmd.append("--create")
        if needs_experimental:
            cmd.append("--experimental-datatypes")
        cmd.extend(str(f) for f in files)

        if args.dry_run:
            print(_sh(cmd))
        else:
            print(f"[upload] mission={mission} files={len(files)}", file=sys.stderr)
            _run(cmd, check=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

