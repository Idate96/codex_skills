---
name: check_starship
description: Connect to `lorenzo@starship-1` over Tailscale and verify that this machine is on the `lorenzoterenzi96@gmail.com` tailnet. Use when the user asks to SSH into `starship-1`, `starship one`, or typo `starhip one`, confirm access from this workstation, or open an interactive shell after Tailscale browser approval.
---

# Check Starship

## Overview

Use this skill to make SSH access to the `starship-1` SSH alias deterministic. Verify that the `starship-1` peer is visible in Tailscale, switch to the `lorenzoterenzi96@gmail.com` profile if needed, then run a strict SSH check that verifies `hostname=starship` and `whoami=lorenzo`. If Tailscale SSH permissions block the check, surface the approval URL directly from the failing SSH output.

## Workflow

1. Check whether the `starship-1` peer is visible in Tailscale:
```bash
tailscale status | grep -E "[[:space:]]starship-1[[:space:]]"
```
2. If `starship-1` is missing, switch to the `lorenzoterenzi96@gmail.com` tailnet profile:
```bash
profile_id="$(sudo tailscale switch --list | awk '$2=="lorenzoterenzi96@gmail.com" {print $1; exit}')"
sudo tailscale switch "$profile_id"
```
3. Run the strict connectivity check:
```bash
bash scripts/check_starship.sh
```
4. Start the interactive shell if needed:
```bash
ssh starship-1
```

## Requirements

- `~/.ssh/config` must resolve `Host starship-1` to `User lorenzo`.
- `tailscale`, `ssh`, `sudo`, and `timeout` must be available.
- `sudo tailscale switch --list` and `sudo tailscale switch <id>` must be permitted.

## Failure Policy

- Fail fast on missing prerequisites, missing Tailscale profiles, or SSH mismatches.
- Do not silently retry with alternate users, hosts, or credentials.
- Print the exact SSH output and extract the approval URL when Tailscale requires browser authorization.

## Script

- `scripts/check_starship.sh`: Verify peer visibility, switch to `lorenzoterenzi96@gmail.com` if needed, then verify SSH access end-to-end.
