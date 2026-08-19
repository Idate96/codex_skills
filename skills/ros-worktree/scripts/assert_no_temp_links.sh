#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 INSTALL_ROOT TEMP_ROOT" >&2
  exit 2
fi

install_root="$(realpath -m -- "$1")"
temp_root="$(realpath -m -- "$2")"
[[ -d "$install_root" ]] || {
  echo "Install root is not a directory: $install_root" >&2
  exit 2
}

found=0
while IFS= read -r -d '' link; do
  raw_target="$(readlink -- "$link")"
  if [[ "$raw_target" = /* ]]; then
    resolved_target="$(realpath -m -- "$raw_target")"
  else
    resolved_target="$(realpath -m -- "$(dirname -- "$link")/$raw_target")"
  fi

  if [[ "$raw_target" == "$temp_root"/* || "$resolved_target" == "$temp_root"/* ]]; then
    printf 'Temporary worktree link: %s -> %s\n' "$link" "$raw_target" >&2
    found=1
  fi
done < <(find "$install_root" -type l -print0)

if (( found )); then
  exit 1
fi

printf 'No install symlinks point into %s\n' "$temp_root"
