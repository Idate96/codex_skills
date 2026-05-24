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

## Main use case (browser, GPT-5.5 Pro with exact picker selection)

Default workflow here: use Lorenzo's local Oracle fork at `/home/lorenzo/oracle` in browser mode. It reuses Oracle's persistent browser profile, selects the exact ChatGPT picker label `Pro`, sets `--browser-thinking-time extended`, and uploads resolved source files as a real ZIP archive for larger reviews. `Extended` is the thinking-time option, not a model-picker label.

Important tested behavior as of April 23, 2026:

- OpenAI officially released `GPT-5.5` and `GPT-5.5 Pro` on April 23, 2026.
- Upstream `@steipete/oracle@0.9.0` does not currently map `GPT-5.5 Pro` correctly in browser mode.
- Upstream local tests on this machine:
  - `--model "5.5 Pro"` resolved to `gpt-5.4-pro`
  - `--model gpt-5.5-pro` resolved to `gpt-5-pro`
- The local fork adds `--browser-model-label <label>` and `--browser-thinking-time <level>` for GPT browser runs.
- Do **not** rely on old Pro aliases for browser mode. Use the current GPT-5.5 Pro browser target explicitly.

Recommended defaults:

- Engine: browser (`--engine browser`)
- CLI: `node /home/lorenzo/oracle/dist/bin/oracle-cli.js`
- Model target: `--browser-model-label "Pro" --browser-thinking-time extended`
- Browser flags: `--browser-manual-login --browser-port 9222 --browser-chrome-path /usr/bin/google-chrome --browser-model-strategy select`
- Attachments: always include the current git root repo by default; add sibling repos only when needed; avoid secrets.

## Golden path (fast + reliable)

1. Resolve the current git root (`git rev-parse --show-toplevel`) and include that repo as the default Oracle context.
2. Preview what you’re about to send (`--dry-run` + `--files-report` when needed), mainly to check resolved files, upload size, secrets, and generated artifacts.
3. Do not trim context only because the dry-run token estimate looks large in browser bundle mode; ChatGPT can inspect uploaded files, so the real browser constraint is a lightweight, valid ZIP with the right files.
4. Do not spawn subagents only to inspect or filter the repo context; attaching the root repo is faster and preserves structure.
5. Add sibling repos or evidence files with extra `--file` arguments only when the bug crosses repo boundaries.
6. Run the local fork in browser mode with exact model-label selection; use API only when you explicitly want it.
7. If the run detaches/timeouts: reattach to the stored session (don’t re-run).

## Commands (preferred)

- Show help (once/session):
  - `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --help`

- Preview (no API tokens; browser upload estimates are advisory):
  - From the repo root: `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --dry-run summary --files-report -p "<task>" --file .`
  - From anywhere inside a repo: `ROOT=$(git rev-parse --show-toplevel) && node /home/lorenzo/oracle/dist/bin/oracle-cli.js --dry-run summary --files-report -p "<task>" --file "$ROOT"`

- Upload/context sanity:
  - `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --dry-run summary --files-report -p "<task>" --file .`

- Browser run (main path; long-running is normal):
  - `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --engine browser --browser-manual-login --browser-port 9222 --browser-chrome-path /usr/bin/google-chrome --browser-model-strategy select --browser-model-label "Pro" --browser-thinking-time extended --browser-attachments always --browser-bundle-files -p "<task>" --file .`
  - Cross-repo example: `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --engine browser --browser-manual-login --browser-port 9222 --browser-chrome-path /usr/bin/google-chrome --browser-model-strategy select --browser-model-label "Pro" --browser-thinking-time extended --browser-attachments always --browser-bundle-files -p "<task>" --file /path/to/root-repo --file /path/to/sibling-repo --file /path/to/evidence.log`
  - If the saved run reports only generic `ui=ChatGPT` after a Pro selection, treat the run as unverified and rerun after checking model selection.

- Deep Research API run (only after explicit user consent because it uses paid API tokens):
  - `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --engine api --model o3-deep-research -p "<research question>"`
  - Faster/cheaper option: `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --engine api --model o4-mini-deep-research -p "<research question>"`
  - API Deep Research uses OpenAI model ids `o3-deep-research` / `o4-mini-deep-research`, uses web search by default, and will reject `--search off` because Oracle does not yet expose file-search/vector-store or remote-MCP data sources.

- ChatGPT browser Deep research run (uses the Plus/tools menu, not API tokens):
  - `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --engine browser --browser-manual-login --browser-port 9222 --browser-chrome-path /usr/bin/google-chrome --browser-deep-research -p "<research question>"`
  - Shorthand: `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --engine browser --model deepresearch --browser-manual-login --browser-port 9222 --browser-chrome-path /usr/bin/google-chrome -p "<research question>"`
  - Important status rule: ChatGPT Deep Research often renders its progress card only visually inside the ChatGPT page/sandbox frame. `document.body.innerText` may still show only the user prompt, and the `connector_openai_deep_research` target can have an empty DOM while the report is actively running. Before calling a browser Deep Research run stuck, failed, or safe to rerun/kill, take a viewport screenshot or accessibility/visual check of the ChatGPT conversation and look for the Deep Research checklist/progress card. If the Oracle session status is `running` and Chrome is alive, do not start a duplicate run.

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

## Prefer real files and ZIP archives

- Prefer attaching the full root repo with `--file .` and let Oracle package the resolved source files as uploads.
- This matters especially when the context includes both code and non-code artifacts such as CSVs, logs, JSON, notebooks, screenshots, or docs.
- Real files and ZIP archives preserve filenames, boundaries, and directory structure.
- In browser mode with `--browser-attachments always --browser-bundle-files`, do not treat the dry-run token estimate as a hard context limit. The uploaded ZIP is a file corpus the model can navigate; use the estimate only as a smell for unexpectedly broad context or accidental huge text files.
- Prefer broad, structurally intact repo context when the ZIP is lightweight enough to upload. Snippet or stage a reduced bundle only when the upload is too large, contains secrets/binaries/generated artifacts, or the task truly needs a small exact excerpt.
- When the input is a software worktree, preserve the whole root layout and relative paths so Oracle can reason about modules, tests, docs, and configs in context.
- If the review spans many related files, use `--browser-attachments always --browser-bundle-files` so Oracle uploads a real `.zip` archive.
- Add a short explicit tree note in the prompt for orientation, especially for larger software uploads. Example: list the main roots and what lives there (`src/`, `test/`, `docs/`, `configs/`) or paste a short `tree` / `find` summary.
- In the prompt, explicitly list what you attached and why. Do not assume Oracle will infer the role of each file or archive from filenames alone.
- If you attach an archive plus sidecar notes, spell out both: what the archive contains, what structure it preserves, and what each extra note contributes.
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

## Upload Budget + Observability

- Browser bundle mode: prioritize lightweight upload size, sane file count, successful attachment chips, and no secrets/generated artifacts. The token estimate is advisory and should not by itself cause aggressive hand-snipping.
- API mode or inline paste mode: token budget still matters; reduce context when the model would otherwise exceed its actual input window or cost target.
- Use `--files-report` (and/or `--dry-run json`) to spot accidental token hogs, oversized text files, binaries, generated output, and files that should be excluded before spending time on a browser run.
- If you need hidden/advanced knobs: `npx -y @steipete/oracle --help --verbose`.

## Engines (API vs browser)

- Auto-pick: uses `api` when `OPENAI_API_KEY` is set, otherwise `browser`.
- Browser engine supports GPT + Gemini only; use `--engine api` for Claude/Grok/Codex, API Deep Research, or multi-model runs. Use `--browser-deep-research` for ChatGPT's browser Deep research composer mode.
- **API runs require explicit user consent** before starting because they incur usage costs.
- Browser model-selection warning for this machine:
  - With Lorenzo's local fork, use `--browser-model-strategy select --browser-model-label "Pro" --browser-thinking-time extended`.
  - With upstream `npx -y @steipete/oracle@0.9.0`, treat `--model "5.5 Pro"` and `--model gpt-5.5-pro` as unsafe in browser mode.
  - Upstream fallback only: use `--browser-model-strategy current` and verify `Pro` with extended thinking directly in the ChatGPT UI.
- Browser auth fallback for ChatGPT:
  - Symptom: `No ChatGPT cookies were applied from your Chrome profile`.
  - First retry with `--browser-cookie-wait 5s`.
  - If cookie sync still fails, switch to manual login and reuse Oracle's own browser profile:
    - `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --engine browser --browser-manual-login --browser-port 9222 --browser-chrome-path /usr/bin/google-chrome --browser-model-strategy select --browser-model-label "Pro" --browser-thinking-time extended --browser-attachments always --browser-bundle-files -p "<task>" --file .`
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
    - `node /home/lorenzo/oracle/dist/bin/oracle-cli.js --engine browser --browser-manual-login --browser-port 9222 --browser-chrome-path /usr/bin/google-chrome --browser-model-strategy select --browser-model-label "Pro" --browser-thinking-time extended ...`
- Browser attachments:
  - `--browser-attachments auto|never|always` (auto pastes inline up to ~60k chars then uploads).
  - `--browser-bundle-files` packages resolved text attachments into a real `attachments-bundle.zip` upload.
  - For repo-scale reviews, prefer `--browser-attachments always --browser-bundle-files`.
  - Validate with `--dry-run summary --files-report --browser-attachments always --browser-bundle-files --file .` and inspect the printed `.zip` path if needed.
  - When the active work is inside a container but browser upload is needed, run the browser-mode Oracle command from the host side where `/usr/bin/google-chrome` and the persistent `~/.oracle/browser-profile` are available, then attach the archive path from the shared filesystem.
  - For high-value reviews, do not click send unless the live composer shows all three required facts: the exact model picker label (`Pro` here), the prompt/sentinel text, and the attachment chip or file-count UI with no upload/dedup/error message.
  - If ChatGPT says the file was already uploaded, use a fresh archive name and add a tiny unique marker file to the archive before retrying.
  - If driving Chrome through CDP, prefer archive paths under `~/Downloads` or another host-readable directory. `/tmp` can resolve to a zero-size upload in some Chrome sessions.
  - If using `DOM.setFileInputFiles`, do not manually dispatch `input` or `change` events afterward unless you are deliberately debugging. In this ChatGPT UI that can double-trigger the upload and produce a duplicate-file error.
  - Treat any answer that says it cannot access the attachment as a failed Oracle run, not as valid review feedback.
- Remote browser host (signed-in machine runs automation):
  - Host: `oracle serve --host 0.0.0.0 --port 9473 --token <secret>`
  - Client: `oracle --engine browser --remote-host <host:port> --remote-token <secret> -p "<task>" --file .`

## Sessions + slugs (don’t lose work)

- Stored under `~/.oracle/sessions` (override with `ORACLE_HOME_DIR`).
- Runs may detach or take a long time (browser + GPT‑5.5 Pro work often does). If the CLI times out: don’t re-run; reattach.
  - List: `oracle status --hours 72`
  - Attach: `oracle session <id> --render`
- For browser Deep Research specifically, a quiet `output.log`, blank sandbox iframe DOM, or prompt-only `innerText` is not evidence of failure. Verify with a screenshot of the ChatGPT conversation first; the visible page can show an active checklist and search count even when the DOM text APIs are uninformative.
- If a browser session is marked completed but the captured answer is only a stub such as “I’ll inspect…” or “I’ll review…”, treat it as a failed review, not as Oracle feedback. Reopen the conversation tab or rerun with an explicit instruction to return the final review directly and not a plan to inspect files.
- When the user has other browser/Oracle queries running, treat isolation as part of the run:
  - Use a unique `--slug` for the new request.
  - Check active runs first: `node /home/lorenzo/oracle/dist/bin/oracle-cli.js status --hours 6 --limit 20`.
  - After launch, verify Chrome has a separate ChatGPT page/tab for the new slug or title:
    - `curl -s http://127.0.0.1:9222/json/list | rg -n '"title"|"url"'`
  - Do not assume the selected/visible Chrome tab is the one Oracle is driving; trust the DevTools page list and the Oracle session slug.
  - Browser-mode reattach/capture can still pick up an existing ChatGPT tab if several Oracle runs are active. For high-value reviews, add a short sentinel requirement to the prompt, such as `Start the final answer with ORACLE_REVIEW_SENTINEL_<slug>`. After completion, reject the saved answer if the sentinel is missing or the content obviously belongs to another request, inspect `~/.oracle/sessions/<slug>/output.log`, and rerun only after the competing browser runs have completed or been detached.
  - If the user explicitly needs concurrent browser runs, isolate the new run with a copied Chrome profile and a different DevTools port instead of sharing `~/.oracle/browser-profile` on port `9222`. Example:
    - `rsync -a --delete --exclude='Singleton*' --exclude='DevToolsActivePort' ~/.oracle/browser-profile/ ~/.oracle/browser-profile-<slug>/`
    - `ORACLE_BROWSER_PROFILE_DIR=$HOME/.oracle/browser-profile-<slug> node /home/lorenzo/oracle/dist/bin/oracle-cli.js --engine browser --browser-port 9333 ...`
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
