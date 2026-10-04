#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import urlparse
from urllib.parse import unquote


FENCE_RE = re.compile(r"^\s{0,3}(`{3,}|~{3,})")
REF_DEF_RE = re.compile(
    r"^\s{0,3}\[(?P<label>[^\]]+)\]:\s*(?P<dest><[^>]+>|[^\s]+)(?:\s+(?P<title>.+))?\s*$"
)
NON_LOCAL_SCHEMES = {
    "data",
    "file",
    "ftp",
    "ftps",
    "http",
    "https",
    "irc",
    "ircs",
    "mailto",
    "sms",
    "tel",
    "xmpp",
}
LOCAL_FILE_SCHEMES = {"notes"}


@dataclass
class PendingReference:
    path: Path
    line_no: int
    label: str
    is_shortcut: bool = False


def normalize_label(label: str) -> str:
    return " ".join(label.strip().lower().split())


def strip_inline_code(line: str, active_fence: str = "") -> tuple[str, str]:
    out: list[str] = []
    i = 0

    if active_fence:
        end = line.find(active_fence)
        if end == -1:
            return " " * len(line), active_fence
        out.append(" " * (end + len(active_fence)))
        i = end + len(active_fence)
        active_fence = ""

    while i < len(line):
        if line[i] != "`":
            out.append(line[i])
            i += 1
            continue

        run_len = 1
        while i + run_len < len(line) and line[i + run_len] == "`":
            run_len += 1
        fence = "`" * run_len
        end = line.find(fence, i + run_len)
        if end == -1:
            out.append(" " * (len(line) - i))
            return "".join(out), fence

        out.append(" " * (end + run_len - i))
        i = end + run_len
    return "".join(out), active_fence


def strip_html_comments(line: str, in_comment: bool) -> tuple[str, bool]:
    out: list[str] = []
    i = 0
    while i < len(line):
        if in_comment:
            end = line.find("-->", i)
            if end == -1:
                out.append(" " * (len(line) - i))
                return "".join(out), True
            out.append(" " * (end + 3 - i))
            i = end + 3
            in_comment = False
            continue

        start = line.find("<!--", i)
        if start == -1:
            out.append(line[i:])
            return "".join(out), False
        out.append(line[i:start])
        out.append(" " * 4)
        i = start + 4
        in_comment = True

    return "".join(out), in_comment


def strip_blockquote_prefixes(line: str) -> str:
    stripped = line
    while True:
        candidate = stripped.lstrip(" ")
        if not candidate.startswith(">"):
            return stripped
        candidate = candidate[1:]
        if candidate.startswith(" "):
            candidate = candidate[1:]
        stripped = candidate


def iter_markdown_files(paths: list[str]) -> list[Path]:
    seen: set[Path] = set()
    files: list[Path] = []
    for raw in paths:
        path = Path(raw).expanduser().resolve()
        if path.is_dir():
            for candidate in sorted(p for p in path.rglob("*") if p.is_file() and (p.suffix == ".md" or p.name == "README")):
                if candidate not in seen:
                    seen.add(candidate)
                    files.append(candidate)
        elif path.is_file():
            if path not in seen:
                seen.add(path)
                files.append(path)
        else:
            raise FileNotFoundError(f"path not found: {raw}")
    return files


def parse_destination(raw: str) -> str | None:
    dest = raw.strip()
    if not dest:
        return None

    if dest.startswith("<"):
        end = dest.find(">")
        if end == -1:
            return None
        dest = dest[1:end].strip()
    else:
        dest = dest.split()[0]

    dest = re.sub(r"\\(.)", r"\1", dest)
    dest = dest.split("#", 1)[0]
    dest = dest.split("?", 1)[0]
    return unquote(dest)


def should_skip_target(source: Path, target: str) -> bool:
    parsed = urlparse(target)
    if parsed.scheme.lower() in LOCAL_FILE_SCHEMES:
        return False
    return (
        not target
        or target.startswith("#")
        or target.startswith("//")
        or parsed.scheme.lower() in NON_LOCAL_SCHEMES
        or (bool(parsed.scheme) and not resolve_target(source, target).exists())
    )


def resolve_target(source: Path, target: str) -> Path:
    if Path(target).is_absolute():
        return Path(target).resolve()
    return (source.parent / target).resolve()


def find_closing_bracket(line: str, start: int) -> int:
    depth = 1
    i = start + 1
    while i < len(line):
        if line[i] == "\\":
            i += 2
            continue
        if line[i] == "[":
            depth += 1
            i += 1
            continue
        if line[i] == "]":
            depth -= 1
            if depth == 0:
                return i
        i += 1
    return -1


def parse_parenthesized(line: str, start: int) -> tuple[str | None, int]:
    if start >= len(line) or line[start] != "(":
        return None, -1

    depth = 0
    quote_char = ""
    in_angle = False
    i = start
    while i < len(line):
        ch = line[i]
        if ch == "\\":
            i += 2
            continue
        if quote_char:
            if ch == quote_char:
                quote_char = ""
            i += 1
            continue
        if in_angle:
            if ch == ">":
                in_angle = False
            i += 1
            continue
        if ch in {'"', "'"}:
            quote_char = ch
            i += 1
            continue
        if ch == "<":
            in_angle = True
            i += 1
            continue
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                return line[start + 1 : i], i
        i += 1
    return None, -1


def parse_links_in_line(line: str) -> tuple[list[str], list[str], list[str]]:
    inline_targets: list[str] = []
    explicit_reference_labels: list[str] = []
    shortcut_reference_labels: list[str] = []

    i = 0
    while i < len(line):
        if line[i] == "!" and i + 1 < len(line) and line[i + 1] == "[":
            start = i + 1
        elif line[i] == "[":
            start = i
        else:
            i += 1
            continue

        close = find_closing_bracket(line, start)
        if close == -1:
            i = start + 1
            continue

        text = line[start + 1 : close]
        next_char = line[close + 1] if close + 1 < len(line) else ""

        if next_char == "(":
            dest_block, end = parse_parenthesized(line, close + 1)
            if dest_block is not None:
                dest = parse_destination(dest_block)
                if dest:
                    inline_targets.append(dest)
                i = end + 1
                continue

        if next_char == "[":
            label_end = find_closing_bracket(line, close + 1)
            if label_end != -1:
                label = line[close + 2 : label_end]
                explicit_reference_labels.append(normalize_label(label or text))
                i = label_end + 1
                continue

        shortcut_reference_labels.append(normalize_label(text))
        i = close + 1

    return inline_targets, explicit_reference_labels, shortcut_reference_labels


def collect_links(path: Path) -> tuple[dict[str, str], list[tuple[int, str]], list[PendingReference]]:
    references: dict[str, str] = {}
    inline_links: list[tuple[int, str]] = []
    pending_refs: list[PendingReference] = []

    in_fence = False
    active_fence = ""
    in_html_comment = False
    inline_code_fence = ""

    for line_no, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        raw_is_indented_code = raw_line.startswith("    ") or raw_line.startswith("\t")
        markdown_line = strip_blockquote_prefixes(raw_line)
        markdown_is_indented_code = markdown_line.startswith("    ") or markdown_line.startswith("\t")
        fence_match = FENCE_RE.match(markdown_line)
        if in_fence:
            if fence_match and fence_match.group(1).startswith(active_fence[0]) and len(fence_match.group(1)) >= len(
                active_fence
            ):
                in_fence = False
                active_fence = ""
            continue

        if raw_is_indented_code or markdown_is_indented_code:
            continue

        if fence_match:
            in_fence = True
            active_fence = fence_match.group(1)
            continue

        if in_html_comment:
            line, in_html_comment = strip_html_comments(markdown_line, in_html_comment)
            line, _ = strip_inline_code(line, "")
            inline_code_fence = ""
        else:
            line, inline_code_fence = strip_inline_code(markdown_line, inline_code_fence)
            line, in_html_comment = strip_html_comments(line, in_html_comment)

        ref_def_match = REF_DEF_RE.match(line)
        if ref_def_match:
            label = normalize_label(ref_def_match.group("label"))
            dest = parse_destination(ref_def_match.group("dest"))
            if dest and label not in references:
                references[label] = dest
            continue

        inline_targets, explicit_reference_labels, shortcut_reference_labels = parse_links_in_line(line)
        for dest in inline_targets:
            inline_links.append((line_no, dest))
        for label in explicit_reference_labels:
            if label:
                pending_refs.append(PendingReference(path=path, line_no=line_no, label=label))
        for label in shortcut_reference_labels:
            if label:
                pending_refs.append(PendingReference(path=path, line_no=line_no, label=label, is_shortcut=True))

    return references, inline_links, pending_refs


def check_file(path: Path) -> list[str]:
    errors: list[str] = []
    references, inline_links, pending_refs = collect_links(path)

    for line_no, target in inline_links:
        if should_skip_target(path, target):
            continue
        resolved = resolve_target(path, target)
        if not resolved.exists():
            errors.append(f"{path}:{line_no}: broken link -> {target} => {resolved}")

    for pending in pending_refs:
        target = references.get(pending.label)
        if target is None:
            if pending.is_shortcut:
                continue
            errors.append(f"{pending.path}:{pending.line_no}: missing reference definition -> [{pending.label}]")
            continue
        if should_skip_target(pending.path, target):
            continue
        resolved = resolve_target(pending.path, target)
        if not resolved.exists():
            errors.append(f"{pending.path}:{pending.line_no}: broken reference -> {target} => {resolved}")

    return errors


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Check local markdown links in files or directories.")
    parser.add_argument("paths", nargs="+", help="Markdown files or directories to scan")
    return parser


def main(argv: list[str]) -> int:
    parser = build_parser()
    args = parser.parse_args(argv[1:])

    try:
        files = iter_markdown_files(args.paths)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    errors: list[str] = []
    for path in files:
        errors.extend(check_file(path))

    if errors:
        print("\n".join(errors))
        return 1

    print(f"all markdown links resolved across {len(files)} file(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
