#!/usr/bin/env bash
# Run on integration x86; the tmux server and all windows live inside Docker.
set -euo pipefail
container=gravis_ugep
if tmux has-session -t gravis_ugep 2>/dev/null; then
  echo 'Legacy host tmux session gravis_ugep still exists; stop its managed stack before starting container tmux.' >&2
  exit 1
fi
if [[ "$(docker inspect --format '{{.State.Running}}' "$container")" != true ]]; then
  docker start "$container"
fi
docker exec "$container" bash /workspaces/gravis_ws/start-cat323-tmux-inside.bash
