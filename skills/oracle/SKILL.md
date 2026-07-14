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

Rebuild `/home/lorenzo/oracle` if the CLI is stale or missing. For a high-value browser run, start or attach to the verified persistent Chrome profile first, then require both preflights:

```bash
node "$CLI" status --hours 6 --limit 20
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
7. Verify the saved answer is a substantive final review for the correct prompt before using it.

Preview:

```bash
ROOT="$(git rev-parse --show-toplevel)"
node "$CLI" --dry-run summary --files-report \
  -p "<self-contained task>" --file "$ROOT"
```

Browser run:

```bash
node "$CLI" \
  --engine browser \
  --browser-manual-login \
  --browser-manual-login-profile-dir /home/lorenzo/.oracle/browser-profile-real-google-profile1-current-experts \
  --browser-port 9222 \
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
