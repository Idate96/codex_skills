#!/usr/bin/env bash
set -euo pipefail

if ! command -v tailscale >/dev/null 2>&1; then
  echo "tailscale command not found" >&2
  exit 1
fi

if ! command -v ssh >/dev/null 2>&1; then
  echo "ssh command not found" >&2
  exit 1
fi

if ! tailscale status | grep -E -q "[[:space:]]rslpc[[:space:]]"; then
  profile_id="$(sudo tailscale switch --list | awk '$2=="fangnan99.github" {print $1; exit}')"
  if [[ -z "${profile_id}" ]]; then
    echo "Could not find tailscale profile for fangnan99.github" >&2
    exit 1
  fi

  sudo tailscale switch "${profile_id}" >/dev/null
fi

ssh_output="$(ssh -o BatchMode=yes -o ConnectTimeout=8 rslpc 'hostname; whoami')"
printf '%s\n' "${ssh_output}"

ssh_host="$(printf '%s\n' "${ssh_output}" | sed -n '1p')"
ssh_user="$(printf '%s\n' "${ssh_output}" | sed -n '2p')"

if [[ "${ssh_host}" != "rslpc" ]]; then
  echo "Unexpected SSH host: ${ssh_host}" >&2
  exit 1
fi

if [[ "${ssh_user}" != "lorenzo" ]]; then
  echo "Unexpected SSH user: ${ssh_user}" >&2
  exit 1
fi

echo "SSH connectivity to lorenzo@rslpc verified."
