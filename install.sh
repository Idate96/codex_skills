#!/usr/bin/env bash
# Link every skill in this repo into both Codex and Claude Code.
#
# Both agents use the same skill format: a directory containing SKILL.md with
# `name` and `description` frontmatter. So one shared tree serves both, and a
# skill can never drift between them.
#
# Usage:
#   ./install.sh            link into ~/.codex/skills and ~/.claude/skills
#   ./install.sh --dry-run  show what would change, touch nothing
#   ./install.sh --codex    only Codex
#   ./install.sh --claude   only Claude
#
# Safe to re-run. Removes stale links pointing into this repo whose skill is
# gone, and never touches real directories it did not create.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$REPO/skills"

DRY_RUN=0
DO_CODEX=1
DO_CLAUDE=1

for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=1 ;;
    --codex)   DO_CLAUDE=0 ;;
    --claude)  DO_CODEX=0 ;;
    -h|--help) sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
  esac
done

[ -d "$SRC" ] || { echo "no skills/ directory at $SRC" >&2; exit 1; }

run() { if [ "$DRY_RUN" -eq 1 ]; then echo "  would: $*"; else "$@"; fi; }

link_into() {
  local agent="$1" dest="$2"
  echo "==> $agent  ($dest)"
  run mkdir -p "$dest"

  local linked=0 updated=0 skipped=0 pruned=0

  # Prune links that point into this repo but whose target no longer exists.
  # Leaves alone real dirs and links pointing somewhere else entirely.
  shopt -s nullglob
  for existing in "$dest"/*; do
    [ -L "$existing" ] || continue
    local target
    target="$(readlink -f "$existing" 2>/dev/null || true)"
    if [ ! -e "$existing" ]; then
      case "$(readlink "$existing")" in
        "$SRC"/*|*/codex_skills/skills/*)
          echo "  prune  $(basename "$existing")  (broken -> $(readlink "$existing"))"
          run rm "$existing"; pruned=$((pruned+1)) ;;
      esac
    fi
  done

  for path in "$SRC"/*/; do
    local name dest_path
    name="$(basename "$path")"
    dest_path="$dest/$name"

    # Only real skills: must carry a SKILL.md
    [ -f "$path/SKILL.md" ] || { echo "  skip   $name  (no SKILL.md)"; skipped=$((skipped+1)); continue; }

    if [ -L "$dest_path" ]; then
      if [ "$(readlink -f "$dest_path")" = "$(readlink -f "$path")" ]; then
        linked=$((linked+1)); continue          # already correct
      fi
      echo "  relink $name  ($(readlink "$dest_path") -> $path)"
      run rm "$dest_path"
      run ln -s "${path%/}" "$dest_path"
      updated=$((updated+1))
    elif [ -e "$dest_path" ]; then
      echo "  KEEP   $name  (real directory here, not replacing — move or delete it manually)"
      skipped=$((skipped+1))
    else
      echo "  link   $name"
      run ln -s "${path%/}" "$dest_path"
      updated=$((updated+1))
    fi
  done

  echo "  -- $agent: $updated changed, $linked already correct, $pruned pruned, $skipped skipped"
}

[ "$DO_CODEX"  -eq 1 ] && link_into "codex"  "$HOME/.codex/skills"
[ "$DO_CLAUDE" -eq 1 ] && link_into "claude" "$HOME/.claude/skills"

# Legacy path compatibility.
# ~23 skills hard-code absolute /home/lorenzo/codex_skills/skills/... paths in
# their SKILL.md and scripts. Keep that path resolving no matter where the repo
# is actually checked out.
LEGACY="$HOME/codex_skills"
echo "==> legacy path  ($LEGACY)"
if [ "$(readlink -f "$LEGACY" 2>/dev/null)" = "$REPO" ]; then
  echo "  ok     already resolves to this repo"
elif [ -L "$LEGACY" ]; then
  echo "  relink $LEGACY  ($(readlink "$LEGACY") -> $REPO)"
  run rm "$LEGACY"
  run ln -s "$REPO" "$LEGACY"
elif [ -e "$LEGACY" ]; then
  echo "  KEEP   $LEGACY is a real directory — not touching it."
  echo "         If it is an old clone, move it aside and re-run so the"
  echo "         hard-coded paths resolve here instead."
else
  echo "  link   $LEGACY -> $REPO"
  run ln -s "$REPO" "$LEGACY"
fi

if [ "$DRY_RUN" -eq 1 ]; then
  echo
  echo "dry run — nothing changed."
fi
