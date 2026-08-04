#!/usr/bin/env bash
set -euo pipefail

SESSION="${1:-graph_msf}"
WINDOW="${2:-container}"

if docker ps --format '{{.Names}}' | grep -qx 'moleworks_ros'; then
  CONTAINER_COMMAND="docker exec -it moleworks_ros bash"
else
  if ! bash -ic 'type moleworks_ros' >/dev/null 2>&1; then
    echo "Missing shell function 'moleworks_ros'; cannot start the current Moleworks ROS container." >&2
    exit 1
  fi
  CONTAINER_COMMAND="bash -ic 'moleworks_ros'"
fi

created_window=false
if tmux has-session -t "${SESSION}" 2>/dev/null; then
  if ! tmux list-windows -t "${SESSION}" -F '#W' | grep -Fxq "${WINDOW}"; then
    tmux new-window -t "${SESSION}" -n "${WINDOW}"
    created_window=true
  fi
else
  tmux new-session -d -s "${SESSION}" -n "${WINDOW}"
  created_window=true
fi

TARGET="${SESSION}:${WINDOW}"

if [[ "${created_window}" == true ]]; then
  tmux send-keys -t "${TARGET}" "${CONTAINER_COMMAND}" C-m
  echo "Opened the Moleworks ROS container in tmux session '${SESSION}', window '${WINDOW}'."
else
  echo "Reusing existing tmux session '${SESSION}', window '${WINDOW}' without sending a command."
fi

echo "Attach with: tmux attach -t ${SESSION}"
