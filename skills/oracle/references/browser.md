# Oracle Browser And Session Reference

## Persistent Profile

On the host, use
`/home/lorenzo/.oracle/browser-profile-real-google-profile1-current-experts`
and DevTools port `9222`. In the Moleworks ROS container, use the isolated
`/home/lorenzo/.oracle/browser-profile-moleworks-ros-container-gpt56` profile
and port `9223`. The container profile persists through the mounted home
directory; never open the host profile from a container process. Do not default
to temporary `/tmp/oracle-browser-*`, `/tmp/oracle-reattach-*`, or an
unverified copied profile; those can lose ChatGPT authentication.

Use the launcher helper to start the appropriate Chrome process after a host or
container restart:

```bash
node /home/lorenzo/codex_skills/skills/oracle/scripts/browser_ensure.js
```

Then run `/home/lorenzo/codex_skills/skills/oracle/scripts/browser_preflight.js`. If it fails, ask Lorenzo to sign in in that exact window. A page title or URL alone is not proof of authentication.

Before send, verify the live composer shows the expected `Pro` picker label, the prompt/sentinel, and all attachment chips without upload errors.

## Concurrency

- Query active runs first with `oracle status` through the local CLI.
- Prefer reattach, waiting, or a unique slug on the same verified profile.
- Do not copy profiles as the default concurrency fix.
- If isolated browser state is explicitly required, copy only after validating destination and exclusions, then verify that copied profile through its own DevTools port before send.
- Do not broadly kill Chrome. Validate exact PIDs, profile directory, port, and session ownership.

## Deep Research

API Deep Research requires explicit paid-API consent. Browser Deep Research uses the signed-in ChatGPT UI and may show progress only visually; a quiet DOM or output log is not proof of failure. Inspect the conversation screenshot/accessibility state and Oracle session status before rerunning or stopping it.

## Attachments

For repo-scale browser reviews, use `--browser-attachments always --browser-bundle-files`. Prefer host-readable paths such as a repository root or `~/Downloads`; avoid `/tmp` when Chrome reports zero-size uploads.

If ChatGPT says a bundle was already uploaded, create a fresh archive name with a tiny unique marker. Treat “cannot access attachment” as a failed run.

## Session Recovery

Sessions live under `~/.oracle/sessions` unless `ORACLE_HOME_DIR` overrides it. Reattach after CLI timeout instead of rerunning. Use unique short slugs and a prompt sentinel when other Oracle requests are active.

If a session is `chrome-disconnected`, inspect the exact Oracle and Chrome processes before restarting. Preserve other browser work.

### Preamble-only or prematurely completed capture

A browser response may first contain a short plan such as “I’ll inspect the
files” and then continue with the actual review. Oracle's saved transcript can
capture only that first text and report the session complete while the same
ChatGPT conversation is still generating.

When the rendered transcript is promise-only or lacks the required sentinel:

1. Treat it as an incomplete capture. Do not retry, follow up, change models, or
   create another conversation.
2. Keep the exact conversation URL from the original run.
3. Use the `chrome-cdp` skill read-only on that exact tab. Do not inspect
   unrelated tabs, type in the composer, click controls, or navigate.
4. Check the latest assistant message for the original prompt's substantive
   structure and sentinel, and check whether generation is still active.
5. If active, wait and inspect the same tab again. Do not use stored CLI status
   alone as proof of browser completion.
6. Recover the answer only after generation stops and validation passes. If the
   page has stopped without a valid answer, report the failure and obtain
   explicit user approval before submitting anything new.
