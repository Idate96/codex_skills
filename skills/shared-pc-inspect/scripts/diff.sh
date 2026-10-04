#!/usr/bin/env bash
# diff.sh — compare two snapshots taken by snapshot.sh.
# Usage: diff.sh [old_snap_dir [new_snap_dir]]
#   No args: diff the two most recent snapshots.
#   One arg: diff that snapshot against a fresh one taken now.
set -u
STATE_DIR="${SHARED_PC_STATE_DIR:-$HOME/.local/state/shared-pc-inspect}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

pick() { ls -d "$STATE_DIR"/snap_* 2>/dev/null | sort | tail -"$1" | head -1; }

if [ $# -ge 2 ]; then OLD="$1"; NEW="$2";
elif [ $# -eq 1 ]; then OLD="$1"; NEW="$("$SCRIPT_DIR/snapshot.sh" handover)";
else OLD="$(pick 2)"; NEW="$(pick 1)"; fi

[ -d "${OLD:-}" ] && [ -d "${NEW:-}" ] || { echo "need two snapshots; found OLD='$OLD' NEW='$NEW'" >&2; exit 1; }
[ "$OLD" = "$NEW" ] && { echo "only one snapshot exists ($OLD) — take a baseline at session start next time" >&2; exit 1; }

echo "### DIFF  $(basename "$OLD")  ->  $(basename "$NEW")"
CHANGED=0
for f in network docker systemd etc can host_toolchain; do
  if ! diff -u "$OLD/$f.txt" "$NEW/$f.txt" > /tmp/.spc_diff.$$ 2>&1; then
    CHANGED=1
    echo; echo "--- CHANGED: $f ---"
    grep -E '^[+-][^+-]' /tmp/.spc_diff.$$ | head -40
  fi
done
rm -f /tmp/.spc_diff.$$
[ $CHANGED -eq 0 ] && echo "No differences — machine state matches the baseline."
exit 0
