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

if ! command -v sudo >/dev/null 2>&1; then
  echo "sudo command not found" >&2
  exit 1
fi

if ! command -v timeout >/dev/null 2>&1; then
  echo "timeout command not found" >&2
  exit 1
fi

ssh_config="$(ssh -G starship-1)"
ssh_user="$(awk '$1=="user" {print $2; exit}' <<<"${ssh_config}")"
if [[ "${ssh_user}" != "lorenzo" ]]; then
  echo "Unexpected SSH user for starship-1: ${ssh_user}" >&2
  exit 1
fi

tailscale_status="$(tailscale status)"
if ! grep -E -q "[[:space:]]starship-1[[:space:]]" <<<"${tailscale_status}"; then
  profile_id="$(sudo tailscale switch --list | awk '$2=="lorenzoterenzi96@gmail.com" {print $1; exit}')"
  if [[ -z "${profile_id}" ]]; then
    echo "Could not find tailscale profile for lorenzoterenzi96@gmail.com" >&2
    exit 1
  fi

  sudo tailscale switch "${profile_id}" >/dev/null
fi

set +e
ssh_output="$(timeout 12 ssh -o BatchMode=yes -o ConnectTimeout=8 starship-1 'hostname; whoami' 2>&1)"
ssh_status=$?
set -e

printf '%s\n' "${ssh_output}"

if [[ "${ssh_status}" -ne 0 ]]; then
  approval_url="$(printf '%s\n' "${ssh_output}" | grep -Eo 'https://login\.tailscale\.com/a/[[:alnum:]]+' | head -n 1 || true)"
  if [[ -n "${approval_url}" ]]; then
    echo "Tailscale SSH approval required: ${approval_url}" >&2
  fi
  if [[ "${ssh_status}" -eq 124 ]]; then
    echo "SSH check timed out while waiting for starship-1." >&2
  fi
  echo "SSH check failed. If Tailscale printed an approval URL, let the user approve it and rerun." >&2
  exit "${ssh_status}"
fi

ssh_host="$(printf '%s\n' "${ssh_output}" | sed -n '1p')"
ssh_user_out="$(printf '%s\n' "${ssh_output}" | sed -n '2p')"

if [[ "${ssh_host}" != "starship-1" ]]; then
  echo "Unexpected SSH host: ${ssh_host}" >&2
  exit 1
fi

if [[ "${ssh_user_out}" != "lorenzo" ]]; then
  echo "Unexpected SSH user: ${ssh_user_out}" >&2
  exit 1
fi

echo "SSH connectivity to lorenzo@starship-1 verified."
