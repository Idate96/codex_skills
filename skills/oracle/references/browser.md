# Oracle Browser And Session Reference

## Persistent Profile

Use `/home/lorenzo/.oracle/browser-profile-real-google-profile1-current-experts` and DevTools port `9222`. Do not default to temporary `/tmp/oracle-browser-*`, `/tmp/oracle-reattach-*`, or an unverified copied profile; those can lose ChatGPT authentication.

If DevTools has no signed-in ChatGPT page, launch Chrome explicitly:

```bash
/opt/google/chrome/chrome \
  --remote-debugging-port=9222 \
  --user-data-dir=/home/lorenzo/.oracle/browser-profile-real-google-profile1-current-experts \
  https://chatgpt.com
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
