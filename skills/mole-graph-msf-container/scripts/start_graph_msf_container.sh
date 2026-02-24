#!/usr/bin/env bash
set -euo pipefail

SESSION="${1:-graph_msf}"
WINDOW="${2:-container}"
IMAGE_TAG="moleworks_ros:graph_msf"

if docker ps --format '{{.Image}}' | grep -q "^${IMAGE_TAG}$"; then
  echo "Container already running for ${IMAGE_TAG}."
  echo "Attach with: tmux attach -t ${SESSION} (or check 'docker ps' for the name)."
  exit 0
fi

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  if ! tmux list-windows -t "${SESSION}" -F '#W' | grep -qx "${WINDOW}"; then
    tmux new-window -t "${SESSION}" -n "${WINDOW}"
  fi
else
  tmux new-session -d -s "${SESSION}" -n "${WINDOW}"
fi

TARGET="${SESSION}:${WINDOW}"

tmux send-keys -t "${TARGET}" "source ~/.bashrc" C-m
tmux send-keys -t "${TARGET}" "moleworks_ros_graph_msf" C-m

echo "Started ${IMAGE_TAG} container in tmux session '${SESSION}', window '${WINDOW}'."
echo "Attach with: tmux attach -t ${SESSION}"
