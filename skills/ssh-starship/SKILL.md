---
name: ssh-starship
description: "Connect to `lorenzo@starship` over its Tailscale profile and verify host/user identity. Use for SSH requests naming `starship`, `starship one`, or legacy `starship-1`."
---

# SSH Starship

## Overview

Use this skill to make SSH access to `starship` deterministic. Verify that the peer is visible in Tailscale, switch to the `lorenzoterenzi96@gmail.com` profile if needed, then run a strict SSH check that verifies `hostname=starship` and `whoami=lorenzo`. During the hostname migration, `tailscale status` may briefly still show `starship-1`; the check script tolerates that stale display name but still requires the actual remote hostname to be `starship`. If Tailscale SSH permissions block the check, surface the approval URL directly from the failing SSH output.

## Workflow

1. Check whether the `starship` peer is visible in Tailscale:
```bash
tailscale status | grep -E "[[:space:]](starship|starship-1)[[:space:]]"
```
2. If the peer is missing, record the current Tailscale profile, disclose that switching is host-global state, then switch to the `lorenzoterenzi96@gmail.com` profile:
```bash
profile_id="$(sudo tailscale switch --list | awk '$2=="lorenzoterenzi96@gmail.com" {print $1; exit}')"
sudo tailscale switch "$profile_id"
```
3. Run the strict connectivity check:
```bash
/home/lorenzo/codex_skills/skills/ssh-starship/scripts/check_starship_ssh.sh
```
4. Start the interactive shell only when the user explicitly asked for one:
```bash
ssh starship
```

## Requirements

- `~/.ssh/config` must resolve `Host starship` to `User lorenzo`.
- `tailscale`, `ssh`, `sudo`, and `timeout` must be available.
- `sudo tailscale switch --list` and `sudo tailscale switch <id>` must be permitted.

## Failure Policy

- Fail fast on missing prerequisites, missing Tailscale profiles, or SSH mismatches.
- Do not silently retry with alternate users, hosts, or credentials.
- Print the exact SSH output and extract the approval URL when Tailscale requires browser authorization.

## Script

- `/home/lorenzo/codex_skills/skills/ssh-starship/scripts/check_starship_ssh.sh`: Verify peer visibility, switch to `lorenzoterenzi96@gmail.com` if needed, then verify SSH access end-to-end.
