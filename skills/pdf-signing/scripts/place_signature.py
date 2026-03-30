#!/usr/bin/env python3
"""Place a handwritten signature image onto explicit rectangles in a PDF page."""

from __future__ import annotations

import argparse
from pathlib import Path

import fitz


def parse_rect(value: str) -> fitz.Rect:
    parts = value.split(",")
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            f"invalid rect {value!r}, expected x0,y0,x1,y1"
        )

    try:
        x0, y0, x1, y1 = (float(part) for part in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"invalid rect {value!r}, expected numeric x0,y0,x1,y1"
        ) from exc

    rect = fitz.Rect(x0, y0, x1, y1)
    if rect.is_empty or rect.is_infinite:
        raise argparse.ArgumentTypeError(f"invalid rect {value!r}")
    return rect


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Place a signature image onto one PDF page at explicit rectangles."
    )
    parser.add_argument("--input", required=True, type=Path, help="source PDF")
    parser.add_argument("--output", required=True, type=Path, help="output PDF")
    parser.add_argument("--signature", required=True, type=Path, help="signature image")
    parser.add_argument(
        "--page",
        required=True,
        type=int,
        help="1-based page number where the signature should be placed",
    )
    parser.add_argument(
        "--rect",
        required=True,
        action="append",
        type=parse_rect,
        help="target rectangle x0,y0,x1,y1 in PDF points; repeat for multiple signatures",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.page < 1:
        raise ValueError("--page must be >= 1")
    if not args.input.is_file():
        raise FileNotFoundError(args.input)
    if not args.signature.is_file():
        raise FileNotFoundError(args.signature)

    doc = fitz.open(args.input)
    page_index = args.page - 1
    if page_index >= len(doc):
        raise ValueError(
            f"--page {args.page} is out of range for {args.input} ({len(doc)} pages)"
        )

    page = doc[page_index]

    # PyMuPDF page coordinates start at the top-left. Using explicit rectangles keeps
    # placement deterministic and makes later nudges a single-number edit.
    for rect in args.rect:
        page.insert_image(rect, filename=str(args.signature), keep_proportion=True, overlay=True)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    doc.save(args.output)

    print(f"saved {args.output}")
    print(f"page {args.page} size: {page.rect.width:.1f} x {page.rect.height:.1f}")
    for index, rect in enumerate(args.rect, start=1):
        print(
            f"rect {index}: "
            f"{rect.x0:.1f},{rect.y0:.1f},{rect.x1:.1f},{rect.y1:.1f}"
        )


if __name__ == "__main__":
    main()
