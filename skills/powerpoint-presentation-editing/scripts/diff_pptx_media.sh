#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: diff_pptx_media.sh OLD.pptx NEW.pptx" >&2
  exit 2
fi

old_pptx="$1"
new_pptx="$2"

if ! command -v unzip >/dev/null 2>&1; then
  echo "ERROR: 'unzip' not found." >&2
  exit 1
fi

tmp_dir="$(mktemp -d)"
trap 'rm -rf "$tmp_dir"' EXIT

extract_parts() {
  local pptx="$1"
  local out="$2"
  mkdir -p "$out"
  # Avoid `set -o pipefail` interactions with `grep -q` (SIGPIPE on unzip).
  if grep -q '^ppt/media/' < <(unzip -Z1 "$pptx"); then
    unzip -qq "$pptx" 'ppt/media/*' -d "$out"
  fi
  if grep -q '^ppt/embeddings/' < <(unzip -Z1 "$pptx"); then
    unzip -qq "$pptx" 'ppt/embeddings/*' -d "$out"
  fi
  if grep -q '^ppt/oleObjects/' < <(unzip -Z1 "$pptx"); then
    unzip -qq "$pptx" 'ppt/oleObjects/*' -d "$out"
  fi
}

hash_parts() {
  local root="$1"
  # sha256 of each extracted file, keyed by relative path.
  (cd "$root" && find ppt -type f -print0 | LC_ALL=C sort -z | xargs -0 -r sha256sum)
}

extract_parts "$old_pptx" "$tmp_dir/old"
extract_parts "$new_pptx" "$tmp_dir/new"

hash_parts "$tmp_dir/old" > "$tmp_dir/old.txt"
hash_parts "$tmp_dir/new" > "$tmp_dir/new.txt"

diff -u "$tmp_dir/old.txt" "$tmp_dir/new.txt"
