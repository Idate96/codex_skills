#!/usr/bin/env bash
# Run inside gravis_ugep. Start missing CAT323 windows; preserve existing work.
set -euo pipefail
workspace=/workspaces/gravis_ws
bash "$workspace/codex_skills/skills/gravis-cat323/scripts/install-cat323-ssh.bash"
bash "$workspace/codex_skills/skills/gravis-cat323/scripts/install-cat323-skills.bash"
session=moleworks_ros
config="$workspace/tmux-cat323.conf"
if ! tmux has-session -t "$session" 2>/dev/null; then
  tmux -f "$config" new-session -d -s "$session" -n commands -c "$workspace" \
    "bash --init-file $workspace/operator-shell.bash"
fi
ensure_window() {
  local name=$1 command=$2
  if ! tmux list-windows -t "$session" -F '#{window_name}' | grep -Fxq "$name"; then
    tmux new-window -d -t "$session" -n "$name" -c "$workspace" "$command"
  fi
}
ensure_window bringup "bash $workspace/run-cat323-bringup.bash"
ensure_window commands "bash --init-file $workspace/operator-shell.bash"
ensure_window machine-status "bash $workspace/evidence/watch-machine-status.bash"
ensure_window checks "bash --init-file $workspace/operator-shell.bash"
ensure_window foxglove "bash $workspace/evidence/foxglove-bridge.sh"
index=0
for name in bringup commands machine-status checks foxglove; do
  window_id=$(tmux list-windows -t "$session" -F '#{window_name} #{window_id}' | awk -v n="$name" '$1==n {print $2}')
  current=$(tmux display-message -p -t "$window_id" '#{window_index}')
  if [[ "$current" != "$index" ]]; then
    if tmux list-windows -t "$session" -F '#{window_index}' | grep -Fxq "$index"; then
      tmux swap-window -s "$window_id" -t "$session:$index"
    else
      tmux move-window -s "$window_id" -t "$session:$index"
    fi
  fi
  index=$((index+1))
done
tmux select-window -t "$session:commands"
tmux list-windows -t "$session"
