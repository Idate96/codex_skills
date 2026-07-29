---
name: oracle
description: "Use the Oracle CLI for an explicit second-model review with repository context. Use for independent debugging, refactor, design, research, or cross-validation requests."
---

# Oracle

Use Oracle only when the user asks for a second-model review or independent cross-validation. Treat its output as advisory and verify conclusions against the repository, tests, and measured evidence.

## Safety And Cost

- Never attach `.env` files, credentials, keys, tokens, certificates, browser profiles, or unrelated private data.
- Preview resolved files before sending. Exclude generated output, large binaries, checkpoints, full datasets, rosbags, and videos unless directly required.
- Browser mode is the default on this machine and uploads the selected bundle to the signed-in ChatGPT
  account. API mode incurs paid usage and requires explicit user consent before launch.
- Do not kill or overwrite another Oracle/Chrome session. Reattach or wait when a matching run already exists.

## Preflight

Oracle requires Node 22+. Prefer the built local fork:

```bash
CLI=/home/lorenzo/oracle/dist/bin/oracle-cli.js
node --version
test -f "$CLI"
```

Rebuild `/home/lorenzo/oracle` if the CLI is stale or missing. For a high-value
browser run, start or attach to the verified persistent Chrome profile first,
then require both preflights. The helper selects the dedicated container
profile and port `9223` inside Docker, or the host profile and port `9222`
outside Docker:

```bash
node "$CLI" status --hours 6 --limit 20
node /home/lorenzo/codex_skills/skills/oracle/scripts/browser_ensure.js
node /home/lorenzo/codex_skills/skills/oracle/scripts/browser_preflight.js
```

Do not send unless the second command prints `ORACLE_BROWSER_PREFLIGHT_OK`. Read [references/browser.md](references/browser.md) for profile recovery, concurrency, Deep Research, and session validation.

## Golden Path

1. Resolve the current git root and inspect applicable repository instructions.
2. Write a self-contained question with the goal, exact evidence/error, constraints, attempts, and desired output.
3. Attach the smallest coherent context. A repository root is appropriate only after the dry-run report
   shows that its tracked and untracked contents are in scope; for diff-focused review, include status,
   a binary patch, applicable `AGENTS.md`, and relevant surrounding files.
4. Run a dry preview and inspect file count, size, secret exposure, and generated artifacts.
5. Run browser mode with the verified `Pro` picker label.
6. If the CLI detaches or times out, reattach to the stored session instead of launching a duplicate.
7. Verify the saved answer is a substantive final review for the correct prompt before using it. If
   the saved transcript contains only a preamble or lacks the requested sentinel, treat it as an
   incomplete capture and follow the live-browser recovery below. Never infer permission to retry.

Preview:

```bash
ROOT="$(git rev-parse --show-toplevel)"
node "$CLI" --dry-run summary --files-report \
  -p "<self-contained task>" --file "$ROOT"
```

Browser run:

```bash
if test -f /.dockerenv; then
  ORACLE_PROFILE="${ORACLE_BROWSER_PROFILE:-/home/lorenzo/.oracle/browser-profile-moleworks-ros-container-gpt56}"
  ORACLE_PORT="${ORACLE_BROWSER_PORT:-9223}"
else
  ORACLE_PROFILE="/home/lorenzo/.oracle/browser-profile-real-google-profile1-current-experts"
  ORACLE_PORT=9222
fi
node "$CLI" \
  --model gpt-5.6-pro \
  --engine browser \
  --browser-manual-login \
  --browser-manual-login-profile-dir "$ORACLE_PROFILE" \
  --browser-port "$ORACLE_PORT" \
  --browser-chrome-path /opt/google/chrome/chrome \
  --browser-model-strategy select \
  --browser-model-label "Pro" \
  --browser-attachments always \
  --browser-bundle-files \
  --slug "<short-unique-slug>" \
  -p "<self-contained task>" \
  --file "$ROOT"
```

For cross-repo work, add separate `--file /absolute/repo-or-evidence` arguments. Read [references/context-and-prompts.md](references/context-and-prompts.md) for attachment and research-context guidance.

## Reattach And Validate

```bash
node "$CLI" status --hours 72
node "$CLI" session <id-or-slug> --render
```

Reject an answer that cannot access its attachments, belongs to another prompt, or only promises future inspection. For concurrent browser runs, include a unique sentinel requirement in the prompt and verify it in the final answer.

### Live-browser completion recovery

ChatGPT can emit a short planning preamble and then continue the same assistant
response. The CLI can capture that preamble and mark its stored session complete
before the live conversation has finished.

- A promise-only transcript, missing sentinel, or disagreement between CLI
  status and the live page means **incomplete capture**, not failed review.
- Do not launch another run, send a follow-up, change the model, navigate away,
  or open a replacement conversation.
- Preserve the exact conversation URL and use the `chrome-cdp` skill to inspect
  only that tab, read-only. Check the latest assistant message, requested
  sentinel, and whether the page still shows active generation.
- If generation is active, wait and re-inspect the same conversation. When the
  CLI transcript conflicts with that page, the exact live conversation is the
  completion source of truth.
- Accept the answer only after it is substantive, belongs to the original
  prompt, includes the sentinel when one was required, and generation has
  stopped.
- If the exact conversation has visibly stopped without a valid answer, report
  that failure and ask the user before any resubmission. Never auto-resubmit.
