#!/usr/bin/env python3
"""Extract text covered by PDF highlight annotations."""

from __future__ import annotations

import argparse
import html
import json
import re
import subprocess
import sys
from pathlib import Path

try:
    from pypdf import PdfReader
    from pypdf.generic import IndirectObject
except ImportError:
    from PyPDF2 import PdfReader
    from PyPDF2.generic import IndirectObject


WORD_RE = re.compile(
    r'<word xMin="([\d.]+)" yMin="([\d.]+)" '
    r'xMax="([\d.]+)" yMax="([\d.]+)">(.*?)</word>'
)


def dereference(value):
    return value.get_object() if isinstance(value, IndirectObject) else value


def words_on_page(pdf: Path, page_number: int):
    try:
        result = subprocess.run(
            [
                "pdftotext",
                "-f",
                str(page_number),
                "-l",
                str(page_number),
                "-bbox",
                str(pdf),
                "-",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as exc:
        raise RuntimeError("pdftotext is required (Ubuntu package: poppler-utils)") from exc
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(exc.stderr.strip() or "pdftotext failed") from exc

    words = []
    for match in WORD_RE.finditer(result.stdout):
        x1, y1, x2, y2 = map(float, match.group(1, 2, 3, 4))
        text = html.unescape(re.sub(r"<[^>]+>", "", match.group(5)))
        words.append((x1, y1, x2, y2, text))
    return words


def annotation_quads(annotation, page_height: float):
    points = annotation.get("/QuadPoints")
    if points:
        values = [float(value) for value in points]
        for start in range(0, len(values), 8):
            quad = values[start : start + 8]
            if len(quad) < 8:
                continue
            xs = quad[0::2]
            ys = quad[1::2]
            yield min(xs), page_height - max(ys), max(xs), page_height - min(ys)
        return

    rect = annotation.get("/Rect")
    if rect:
        x1, y1, x2, y2 = map(float, rect)
        yield min(x1, x2), page_height - max(y1, y2), max(x1, x2), page_height - min(y1, y2)


def overlaps(word, box):
    wx1, wy1, wx2, wy2, _ = word
    bx1, by1, bx2, by2 = box
    return wx2 >= bx1 and wx1 <= bx2 and wy2 >= by1 and wy1 <= by2


def highlighted_text(words, boxes):
    selected = []
    seen = set()
    for box in boxes:
        for index, word in enumerate(words):
            if index not in seen and overlaps(word, box):
                selected.append((word[1], word[0], index, word[4]))
                seen.add(index)
    selected.sort(key=lambda item: (round(item[0] / 3) * 3, item[1]))
    return " ".join(item[3] for item in selected).strip()


def extract(pdf: Path):
    reader = PdfReader(str(pdf))
    page_words = {}
    highlights = []

    for page_number, page in enumerate(reader.pages, 1):
        annotations = dereference(page.get("/Annots", []))
        for reference in annotations:
            annotation = dereference(reference)
            if str(annotation.get("/Subtype")) != "/Highlight":
                continue

            if page_number not in page_words:
                page_words[page_number] = words_on_page(pdf, page_number)
            page_height = float(page.mediabox.height)
            boxes = list(annotation_quads(annotation, page_height))
            text = highlighted_text(page_words[page_number], boxes)
            highlights.append(
                {
                    "page": page_number,
                    "modified": str(annotation.get("/M", "")),
                    "created": str(annotation.get("/CreationDate", "")),
                    "comment": str(annotation.get("/Contents", "")),
                    "text": text,
                    "boxes": boxes,
                }
            )

    highlights.sort(key=lambda item: item["modified"] or item["created"], reverse=True)
    return highlights


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pdf", type=Path)
    parser.add_argument("--latest", action="store_true", help="show only the newest highlight")
    parser.add_argument("--json", action="store_true", help="emit JSON")
    args = parser.parse_args()

    if not args.pdf.is_file():
        parser.error(f"file not found: {args.pdf}")

    try:
        highlights = extract(args.pdf)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    if args.latest:
        highlights = highlights[:1]

    if args.json:
        print(json.dumps(highlights, ensure_ascii=False, indent=2))
    elif not highlights:
        print("No saved highlight annotations found.")
    else:
        for item in highlights:
            print(f"Page {item['page']} | {item['modified'] or item['created']}")
            print(item["text"] or "[No overlapping text recovered]")
            if item["comment"]:
                print(f"Comment: {item['comment']}")
            print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
