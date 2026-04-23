---
name: oracle
description: Use the @steipete/oracle CLI to bundle a prompt plus the right files and get a second-model review (API or browser) for debugging, refactors, design checks, or cross-validation.
---

# Oracle (CLI) — best use

Oracle bundles your prompt + selected files into one “one-shot” request so another model can answer with real repo context (API or browser automation). Treat outputs as advisory: verify against the codebase + tests.

## Prerequisite: Node runtime

`@steipete/oracle@0.9.0` requires Node `>=22`.

- If your shell already has Node 22+, the standard commands below work as written.
- If your default `node` is older (for example Node 18 on this machine), the CLI crashes before startup with a `SyntaxError` from `string-width`.
- If `/home/lorenzo/oracle` changes, rebuild before using the absolute `dist` CLI: `corepack pnpm -C /home/lorenzo/oracle run build`.
- Quick fallback without changing the system install:
  - `npx -y -p node@22 -p @steipete/oracle -c 'node $(readlink -f $(which oracle)) --help'`
  - Replace `--help` with any normal Oracle command, for example a dry run:
  - `npx -y -p node@22 -p @steipete/oracle -c 'node $(readlink -f $(which oracle)) --dry-run summary --files-report -p "<task>" --file .'`

## Main use case (browser, GPT‑5.5 Pro with exact picker selection)

Default workflow here: use Lorenzo's local Oracle fork at `/home/lorenzo/oracle` in browser mode. It reuses Oracle's persistent browser profile, selects the exact ChatGPT picker label `GPT-5.5 Pro`, and sets thinking time to `Extended`.

Important tested behavior as of April 23, 2026:

- OpenAI officially released `GPT-5.5` and `GPT-5.5 Pro` on April 23, 2026.
- Upstream `@steipete/oracle@0.9.0` does not currently map `GPT-5.5 Pro` correctly in browser mode.
- Upstream local tests on this machine:
  - `--model "5.5 Pro"` resolved to `gpt-5.4-pro`
  - `--model gpt-5.5-pro` resolved to `gpt-5-pro`
- The local fork adds `--browser-model-label <label>` and honors it for GPT browser runs.
- Verified live: `--browser-model-label "GPT-5.5 Pro" --browser-thinking-time extended` selected the Pro picker option, set `Thinking time: Extended`, submitted successfully, and the model self-reported `GPT-5.5 Pro`.
- Do **not** rely on `--model` alone for GPT‑5.5 Pro in browser mode. Use the exact browser label flag.

Recommended defaults:

- Engine: browser (`--engine browser`)
- CLI: `node /home/lorenzo/oracle/dist/bin/oracle-cli.js`
- Model target: `--browser-model-label "GPT-5.5 Pro"`
- Browser flags: `--browser-manual-login --browser-port 9222 --browser-chrome-path /usr/bin/google-chrome --browser-model-strategy select --browser-thinking-time extended`
- Attachments: always include the current git root repo by default; add sibling repos only when needed; avoid secrets.

## Golden path (fast + reliable)

1. Resolve the current git root (`git rev-parse --show-toplevel`) and include that repo as the default Oracle context.
2. Preview what you’re about to send (`--dry-run` + `--files-report` when needed).
3. Do not spawn subagents only to inspect or filter the repo context; attaching the root repo is faster and preserves structure.
4. Add sibling repos or evidence files with extra `--file` arguments only when the bug crosses repo boundaries.
5. Run the local fork in browser mode with exact model-label selection; use API only when you explicitly want it.
6. If the run detaches/timeouts: reattach to the stored session (don’t re-run).

## Commands (preferred)

- Show help (once/session):
  - `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --help`

- Preview (no tokens):
  - From the repo root: `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --dry-run summary --files-report -p "<task>" --file .`
  - From anywhere inside a repo: `ROOT=$(git rev-parse --show-toplevel) && node /home/lorenzo/oracle/dist/bin/oracle-cli.js --dry-run summary --files-report -p "<task>" --file "$ROOT"`

- Token/cost sanity:
  - `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --dry-run summary --files-report -p "<task>" --file .`

- Browser run (main path; long-running is normal):
  - `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --engine browser --browser-manual-login --browser-port 9222 --browser-chrome-path /usr/bin/google-chrome --browser-model-strategy select --browser-model-label "GPT-5.5 Pro" --browser-thinking-time extended -p "<task>" --file .`
  - Cross-repo example: `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --engine browser --browser-manual-login --browser-port 9222 --browser-chrome-path /usr/bin/google-chrome --browser-model-strategy select --browser-model-label "GPT-5.5 Pro" --browser-thinking-time extended -p "<task>" --file /path/to/root-repo --file /path/to/sibling-repo --file /path/to/evidence.log`
  - The UI may still summarize the top bar as `ChatGPT`; trust the Oracle log lines `Model picker: Pro...` and `Thinking time: Extended`, or ask the model to self-report for a smoke test.

- Manual paste fallback (assemble bundle, copy to clipboard):
  - `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --render --copy -p "<task>" --file .`
  - Note: `--copy` is a hidden alias for `--copy-markdown`.
  - Use this only when you explicitly want a flattened text bundle for manual paste. It is not the default recommendation for mixed code + artifact review.

## Attaching files (`--file`)

`--file` accepts files, directories, and globs. You can pass it multiple times; entries can be comma-separated.

- Include:
  - `--file .` (current root repo, preferred default)
  - `--file "$(git rev-parse --show-toplevel)"` (root repo from a subdirectory)
  - `--file src/index.ts` (literal file)
  - `--file docs --file README.md` (extra literal directory + file)
  - `--file ../sibling_repo --file /tmp/evidence.log` (additional repo/evidence when needed)

- Exclude (prefix with `!`):
  - `--file . --file "!**/*.snap" --file "!**/generated/**"`

- Defaults (important behavior from the implementation):
  - Default-ignored dirs: `node_modules`, `dist`, `coverage`, `.git`, `.turbo`, `.next`, `build`, `tmp` (skipped unless you explicitly pass them as literal dirs/files).
  - Honors `.gitignore` when expanding globs.
  - Does not follow symlinks (glob expansion uses `followSymbolicLinks: false`).
  - Dotfiles are filtered unless you explicitly opt in with a pattern that includes a dot-segment (e.g. `--file ".github/**"`).
  - Default cap: files > 1 MB are rejected unless you raise `ORACLE_MAX_FILE_SIZE_BYTES` or `maxFileSizeBytes` in `~/.oracle/config.json`.

## Prefer real files over flattened text

- Prefer attaching the full root repo with `--file .` instead of manually filtering a small file set or flattening everything into one rendered text blob.
- This matters especially when the context includes both code and non-code artifacts such as CSVs, logs, JSON, notebooks, screenshots, or docs.
- Real files preserve filenames, boundaries, and directory structure better than one giant pasted markdown bundle.
- When the input is a software worktree, preserve the whole root layout and relative paths so Oracle can reason about modules, tests, docs, and configs in context.
- If the review spans many related files, consider sending them as one archive or zip-style bundle that keeps the directory structure intact rather than flattening everything into plain text.
- Add a short explicit tree note in the prompt for orientation, especially for larger software uploads. Example: list the main roots and what lives there (`src/`, `test/`, `docs/`, `configs/`) or paste a short `tree` / `find` summary.
- In the prompt, explicitly list what you attached and why. Do not assume Oracle will infer the role of each file or archive from filenames alone.
- If you attach an archive plus sidecar notes, spell out both: what the archive contains, what structure it preserves, and what each extra note contributes.
- Use `--render` / `--copy` mainly for manual paste workflows or when you intentionally want a single text artifact.
- Do not spend time spawning agents solely to curate a minimal file list. Use `--files-report` to catch obvious generated/secret/huge files, then attach the root repo directly.

## Research-grade context for ML and robotics

- For ML, robotics, controls, and systems-debugging tasks, assume Oracle needs **more** context than a normal code review to reason well about the problem.
- Write the prompt as if briefing an expert machine learning researcher and engineer who has strong software instincts, but **zero** prior knowledge of your repos, experiments, or naming conventions.
- Always include the root repo for the task, not just the suspected module.
- When the issue spans a parent repo such as `moleworks_ros` plus sibling repos, add those repos as separate `--file` arguments while preserving repo boundaries and relative paths.
- If a single repo is not enough to explain the bug, it is acceptable to send sibling repos plus evidence files.
- The goal is complete structural context for the relevant root repo, with extra repos/evidence added only when they carry real truth.

Recommended contents for a research-style bundle:

- Source code, configs, launch files, scripts, tests, docs, and any notebooks that are genuinely needed.
- Distilled evidence such as benchmark CSVs, result JSON files, plots, screenshots, small log excerpts, failure traces, and experiment manifests.
- Small representative data samples or short bag excerpts when they are necessary to explain a failure mode.
- A short manifest or tree note that says what each repo or artifact contributes.

Things to exclude unless they are directly required:

- `.git`, `node_modules`, `build`, `dist`, `coverage`, `.venv`, `__pycache__`, and other generated caches.
- Large checkpoints, full raw datasets, full rosbags, videos, binaries, and other heavyweight artifacts.
- Secret-bearing files such as `.env`, service credentials, API keys, or private certificates.

Practical pattern for cross-repo investigations:

- Prefer direct repo attachments first: `--file /path/to/root-repo --file /path/to/sibling-repo --file /path/to/evidence`.
- If direct attachments are awkward, stage a clean temporary directory that keeps original repo names, for example `bundle/moleworks_ros`, `bundle/moleworks_ext`, `bundle/newton`.
- Add a small `README.md` or `TREE.txt` at the bundle root describing the included repos and evidence.
- Zip the staged directory and attach the zip directly with `--file /abs/path/to/bundle.zip`.
- In the prompt, explicitly say that Oracle should treat the bundle as a multi-repo robotics/ML workspace with attached experimental evidence.

## Budget + observability

- Target: keep total input under ~196k tokens.
- Use `--files-report` (and/or `--dry-run json`) to spot the token hogs before spending.
- If you need hidden/advanced knobs: `npx -y @steipete/oracle --help --verbose`.

## Engines (API vs browser)

- Auto-pick: uses `api` when `OPENAI_API_KEY` is set, otherwise `browser`.
- Browser engine supports GPT + Gemini only; use `--engine api` for Claude/Grok/Codex or multi-model runs.
- **API runs require explicit user consent** before starting because they incur usage costs.
- Browser model-selection warning for this machine:
  - With Lorenzo's local fork, use `--browser-model-strategy select --browser-model-label "GPT-5.5 Pro" --browser-thinking-time extended`.
  - With upstream `npx -y @steipete/oracle@0.9.0`, treat `--model "5.5 Pro"` and `--model gpt-5.5-pro` as unsafe in browser mode.
  - Upstream fallback only: use `--browser-model-strategy current` and verify `GPT-5.5 Pro` directly in the ChatGPT UI.
- Browser auth fallback for ChatGPT:
  - Symptom: `No ChatGPT cookies were applied from your Chrome profile`.
  - First retry with `--browser-cookie-wait 5s`.
  - If cookie sync still fails, switch to manual login and reuse Oracle's own browser profile:
    - `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --engine browser --browser-manual-login --browser-port 9222 --browser-chrome-path /usr/bin/google-chrome --browser-model-strategy select --browser-model-label "GPT-5.5 Pro" --browser-thinking-time extended -p "<task>" --file .`
  - Sign into ChatGPT in the Oracle-launched Chrome window, which uses `~/.oracle/browser-profile`.
  - Keep that Chrome window open until the run finishes; closing it early leaves the session in a `chrome-disconnected` state.
  - After the first successful manual login, future browser runs can reuse that Oracle profile with `--browser-manual-login`.
  - Important on this machine: do not expect auth to persist from temporary Oracle browser profiles such as `/tmp/oracle-browser-*` or `/tmp/oracle-reattach-*`. Those are throwaway by design. If you need reusable ChatGPT auth across runs, always use `--browser-manual-login` and verify the live Chrome process is using `--user-data-dir=/home/lorenzo/.oracle/browser-profile`.
  - If Oracle browser auth looks inconsistent, first check the actual Chrome process rather than guessing from the window title:
    - `ps -ef | rg '/opt/google/chrome/chrome .*user-data-dir='`
    - good persistent profile: `--user-data-dir=/home/lorenzo/.oracle/browser-profile`
    - bad throwaway profiles: `--user-data-dir=/tmp/oracle-browser-*` or `/tmp/oracle-reattach-*`
  - If stale throwaway Oracle windows are still around, kill them before asking the user to log in again. Otherwise they will log into the wrong window and nothing will persist.
  - When bringing up a fresh persistent session, prefer a fixed DevTools port so reattach/debugging is deterministic:
    - `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --engine browser --browser-manual-login --browser-port 9222 --browser-chrome-path /usr/bin/google-chrome --browser-model-strategy select --browser-model-label "GPT-5.5 Pro" --browser-thinking-time extended ...`
- Browser attachments:
  - `--browser-attachments auto|never|always` (auto pastes inline up to ~60k chars then uploads).
  - Important: in this Oracle version, `--browser-bundle-files` does **not** upload a real zip. It creates an `attachments-bundle.txt` text bundle and uploads that as one file.
  - If you want a real archive that ChatGPT can inspect as a zip, build the zip yourself and attach the `.zip` file directly.
  - For explicit zip upload, prefer:
    - `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --engine browser --browser-manual-login --browser-port 9222 --browser-chrome-path /usr/bin/google-chrome --browser-model-strategy select --browser-model-label "GPT-5.5 Pro" --browser-thinking-time extended --browser-attachments always -p "<task>" --file /abs/path/to/bundle.zip`
  - Use `--browser-bundle-files` only when a single flattened text bundle is acceptable.
  - For mixed code + artifact review where directory structure matters, a user-built zip is often more ergonomic than Oracle's text bundle.
- Remote browser host (signed-in machine runs automation):
  - Host: `oracle serve --host 0.0.0.0 --port 9473 --token <secret>`
  - Client: `oracle --engine browser --remote-host <host:port> --remote-token <secret> -p "<task>" --file .`

## Sessions + slugs (don’t lose work)

- Stored under `~/.oracle/sessions` (override with `ORACLE_HOME_DIR`).
- Runs may detach or take a long time (browser + GPT‑5.5 Pro work often does). If the CLI times out: don’t re-run; reattach.
  - List: `oracle status --hours 72`
  - Attach: `oracle session <id> --render`
- If a browser run gets stuck in `chrome-disconnected`, kill the stale Oracle/Chrome processes and start a fresh run instead of piling on more reattach commands.
- Use `--slug "<3-5 words>"` to keep session IDs readable.
- Duplicate prompt guard exists; use `--force` only when you truly want a fresh run.

## Prompt template (high signal)

Oracle starts with **zero** project knowledge. Assume the model cannot infer your stack, build tooling, conventions, or “obvious” paths. Include:

- Role framing when relevant ("Act like an expert ML researcher and engineer reviewing a robotics/software workspace with experimental evidence, but assume no prior repo knowledge.").
- Project briefing (stack + build/test commands + platform constraints).
- “Where things live” (key directories, entrypoints, config files, dependency boundaries).
- Attachment inventory ("I attached X, Y, Z; X contains..., Y is a tree note..., Z is a failure excerpt...").
- Exact question + what you tried + the error text (verbatim).
- Constraints (“don’t change X”, “must keep public API”, “perf budget”, etc).
- Desired output (“return patch plan + tests”, “list risky assumptions”, “give 3 options with tradeoffs”).

### “Exhaustive prompt” pattern (for later restoration)

When you know this will be a long investigation, write a prompt that can stand alone later:

- Top: 6–30 sentence project briefing + current goal.
- Middle: concrete repro steps + exact errors + what you already tried.
- Bottom: attach the root repo plus any sibling repos/evidence needed so a fresh model can fully understand the workspace.

For Moleworks-style work, this often means:

- Attach the parent/root repo, for example `moleworks_ros`, when that is the real unit of context.
- Include relevant sibling repos when the behavior crosses repo boundaries.
- Keep any staged bundle repo-heavy and evidence-heavy, not artifact-heavy.
- Prefer small, representative experiment outputs over giant raw dumps.

If you need to reproduce the same context later, re-run with the same prompt + `--file …` set (Oracle runs are one-shot; the model doesn’t remember prior runs).

## Safety

- Don’t attach secrets by default (`.env`, key files, auth tokens). Redact aggressively; share only what’s required.
- Prefer root-repo context plus explicit excludes for secrets/generated artifacts over hand-curated minimal file lists.
