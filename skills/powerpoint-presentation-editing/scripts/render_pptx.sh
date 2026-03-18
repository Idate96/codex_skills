#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: render_pptx.sh INPUT.pptx OUT_DIR [DPI]" >&2
  exit 2
fi

pptx="$1"
out_dir="$2"
dpi="${3:-170}"

if ! command -v soffice >/dev/null 2>&1; then
  echo "ERROR: 'soffice' not found (install LibreOffice)." >&2
  exit 1
fi

if ! command -v pdftoppm >/dev/null 2>&1; then
  echo "ERROR: 'pdftoppm' not found (install poppler-utils)." >&2
  exit 1
fi

mkdir -p "$out_dir"

base="$(basename "$pptx")"
stem="${base%.*}"

# Avoid stale previews.
rm -f "$out_dir/$stem.pdf" "$out_dir/$stem.PDF" "$out_dir"/slide-*.png

soffice --headless --nologo --nolockcheck --convert-to pdf --outdir "$out_dir" "$pptx"

pdf="$out_dir/$stem.pdf"
if [[ ! -f "$pdf" ]]; then
  pdf="$out_dir/$stem.PDF"
fi
if [[ ! -f "$pdf" ]]; then
  echo "ERROR: expected PDF not found: '$out_dir/$stem.pdf'." >&2
  exit 1
fi

pdftoppm -png -rx "$dpi" -ry "$dpi" "$pdf" "$out_dir/slide"

shopt -s nullglob
imgs=( "$out_dir"/slide-*.png )
shopt -u nullglob

if [[ ${#imgs[@]} -eq 0 ]]; then
  echo "ERROR: no slide images created under '$out_dir'." >&2
  exit 1
fi

# Rename slide-N.png -> slide-0N.png (padding based on total slide count).
count="${#imgs[@]}"
digits="${#count}"
for f in "${imgs[@]}"; do
  bn="$(basename "$f")"        # slide-12.png
  n="${bn#slide-}"             # 12.png
  n="${n%.png}"                # 12
  # pdftoppm may already emit zero-padded numbers (e.g., slide-01.png). Normalize anyway.
  n="$((10#$n))"
  printf -v pad "%0*d" "$digits" "$n"
  dest="$out_dir/slide-$pad.png"
  if [[ "$f" == "$dest" ]]; then
    continue
  fi
  mv -f "$f" "$dest"
done

echo "Rendered PDF: $pdf"
echo "Rendered slides: $count (DPI=$dpi)"
