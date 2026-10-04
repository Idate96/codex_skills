---
name: codex-auth-recovery
description: Diagnose and recover Codex token-refresh errors after an account switch, especially in remote Docker sessions, while preserving the current login. Use when new terminals still report that the user logged out or signed into another account.
---

# Codex account-switch recovery

Preserve the account the user already authenticated. Do not run logout, delete or replace auth.json, copy credentials from another machine, or force another login as the first repair. Never print tokens or raw credential files.

## Locate the failing client

- Confirm the host, container, OS user, effective home/Codex directory, and CLI version. A successful host test does not establish container health. Use the existing ssh-rslpc skill when applicable.
- Discover the current container name with `docker ps`; names change. Run Codex as the user's UID/GID, not Docker's default root. On rslpc, Lorenzo was UID/GID 1005:1005; verify before reuse. Docker may reject a username even when container `getent passwd` resolves it; numeric IDs worked.
- Compare host/container auth-file device/inode and timestamps if shared credentials are suspected. Shared home mounts mean a login on one side affects the other. Do not compare by printing file contents.
- Read the latest session's structured `task_complete.error` (or error event). Skip session metadata and instruction text; do not dump entire JSONL files. Record the exact error, not unrelated old log entries.

## Recognize stale daemon state

Typical error: “Your access token could not be refreshed because you have since logged out or signed in to another account.”

Check `codex login status`, the auth-file modification/last-refresh time (metadata only), and the start time of `codex app-server --managed-daemon` under the same user. A daemon predating a successful account switch, followed by this error in fresh interactive sessions, is evidence of stale in-memory account state, not proof that the saved credential is invalid.

`codex login status` only reports stored login state. A successful `codex exec` request is also insufficient to clear an interactive failure: in the observed incident, fresh exec requests succeeded while the interactive client reused the stale daemon and failed.

## Recover once, then verify

1. Check whether the affected daemon owns running work. A restart may interrupt its tasks or disconnect clients. For an idle/failed session under an authorized repair request, announce and proceed; coordinate if work is active. Do not restart ROS or the container.
2. Verify this CLI supports `codex app-server daemon restart` with its help output. Restart only the affected user's daemon in the failing environment:

   ```bash
   codex app-server daemon restart
   codex login status
   ```

   From rslpc's host, after resolving the actual container and UID/GID:

   ```bash
   docker exec -u 1005:1005 -e HOME=/home/lorenzo <container> codex app-server daemon restart
   ```

3. Verify the daemon PID changed. Test through the **interactive CLI using that daemon**, in an already trusted working directory, with “Reply exactly OK. Do not use tools.” Use a PTY if testing remotely. Exit only your test client afterward. Do not grant new folder trust just to run the test.
4. If the user's terminal disconnected, tell them to reopen Codex or resume their existing task. No new login is needed when the interactive test succeeds.

Stop after one restart if the same error persists. Inspect current daemon/client diagnostics and report the failure; do not enter logout/restart loops. Distinguish network failures, actual rejected credentials, and unsupported-model errors. Test any model alternative as a per-command override first; do not silently change the user's persistent model setting as a token repair.

## Prevent recurrence

After intentionally switching accounts, once affected Codex tasks are idle, run `codex app-server daemon restart` in each environment whose daemon uses that credential store, then reopen the interactive client. Closing a terminal alone may leave the daemon running. Avoid simultaneous account switching across a shared host/container credential directory. Separate account-specific credential directories are an optional redesign, not part of routine recovery.

Do not install unconditional shell-startup restarts or credential-file watchers: normal token refresh also updates credentials, and restarts could interrupt healthy work. Do not claim an upgrade fixes this unless a relevant fix is documented and verified.

Observed on rslpc, 2026-09-25: container CLI/daemon 0.157.0; host/container auth.json shared an inode; daemon started 08:57 UTC, successful login 08:58 UTC; interactive sessions at 09:00 and 09:07 failed, while exec succeeded. One daemon restart restored an interactive OK response without another login. This strongly supports stale daemon state; the exact implementation defect was not source-confirmed.

Official reference: [Codex authentication](https://learn.chatgpt.com/docs/auth). Codex caches credentials and normally refreshes tokens automatically; this recovery addresses an account-switch failure in the persistent client process.
