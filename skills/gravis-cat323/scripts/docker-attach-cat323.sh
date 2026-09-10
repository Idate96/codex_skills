#!/usr/bin/env bash
set -euo pipefail
case "${1:-}" in
  --help|-h)
    echo 'docker_attach          Enter CAT323 tmux inside gravis_ugep'
    echo 'docker_attach --shell  Open an independent sourced ROS shell'
    exit 0 ;;
  --shell)
    [[ $# == 1 ]] || { echo 'Use docker_attach --help' >&2; exit 2; }
    exec docker exec -it gravis_ugep bash -c \
      'bash /workspaces/gravis_ws/codex_skills/skills/gravis-cat323/scripts/install-cat323-ssh.bash && exec bash --init-file /workspaces/gravis_ws/operator-shell.bash' ;;
  '')
    [[ $# == 0 ]] || { echo 'Use docker_attach --help' >&2; exit 2; }
    bash /home/integration/gravis_ws/start-cat323-tmux.sh
    exec docker exec -it gravis_ugep tmux attach-session -t moleworks_ros ;;
  *) echo 'Use docker_attach --help' >&2; exit 2 ;;
esac
