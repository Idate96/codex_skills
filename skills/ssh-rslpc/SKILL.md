---
name: ssh-rslpc
description: "Connect to `lorenzo@rslpc` over Tailscale and verify host/user identity. Use for SSH requests naming `rslpc` or the common typo `rlspc`."
---

# SSH RSLPC

## Overview

Use this skill to make SSH access to `rslpc` deterministic. Verify Tailscale peer visibility, switch to the correct tailnet if needed, and validate that SSH returns `hostname=rslpc` and `whoami=lorenzo`.

## Workflow

1. Check whether the `rslpc` peer is visible in Tailscale:
```bash
tailscale status | grep -E "[[:space:]]rslpc[[:space:]]"
```
2. If `rslpc` is missing, record the current Tailscale profile, disclose that switching is host-global state, then switch to the `fangnan99.github` profile:
```bash
profile_id="$(sudo tailscale switch --list | awk '$2=="fangnan99.github" {print $1; exit}')"
sudo tailscale switch "$profile_id"
```
3. Run the strict connectivity check:
```bash
/home/lorenzo/codex_skills/skills/ssh-rslpc/scripts/check_rslpc_ssh.sh
```
4. Start the interactive shell only when the user explicitly asked for one:
```bash
ssh rslpc
```

## Requirements

- `~/.ssh/config` must include `Host rslpc` and `User lorenzo`.
- `tailscale` and `ssh` commands must be available.
- `sudo tailscale switch --list` must be permitted.

## Failure policy

- Fail fast on missing prerequisites or command failures.
- Do not silently retry with alternate credentials.
- Always print the exact failing command output.

## Script

- `/home/lorenzo/codex_skills/skills/ssh-rslpc/scripts/check_rslpc_ssh.sh`: Verify peer visibility and SSH access end-to-end.
