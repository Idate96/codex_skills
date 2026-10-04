---
name: moleworks-docs-cleanup
description: Audit, simplify, and de-stale Moleworks documentation in `moleworks_ros`. Use when cleaning package READMEs, `AGENTS.md`/`CLAUDE.md`, `docs/`, mdBook nav, or package-local runbooks; when splitting docs into information/design/implementation; or when you need reviewer-backed verification that docs match the current code.
---

# Moleworks Docs Cleanup

Keep Moleworks docs short, current, and source-backed.

## Use This For

- stale-doc audits in `src/moleworks_ros`
- simplifying package `README.md`, `AGENTS.md`, and `CLAUDE.md`
- reorganizing docs into:
  - information docs: current facts, commands, topics, services, runtime usage
  - design docs: rationale, architecture, contracts, decisions
  - implementation docs: plans, task breakdowns, work-in-progress notes
- cleaning mdBook routing under `docs/src/`
- reviewer-backed doc verification against current launch/config/test surfaces

## Source Of Truth

Prefer live repo surfaces over prose:

- `launch/*.py`
- `config/*.yaml`
- `package.xml`
- tests
- scripts actually used by operators
- message/service definitions

If docs and code disagree, fix the docs first unless the user explicitly asked for a code change.

## Doc Shape Rules

- Top-level docs should route, not duplicate package manuals.
- Package `README.md` files should explain the current runtime surface, not the package history.
- `AGENTS.md` should be short and agent-facing: gotchas, source-of-truth pointers, non-obvious constraints.
- Keep `CLAUDE.md` as a one-line alias if that pattern is already used nearby.
- Prefer one canonical doc over mirrored copies.
- Remove stale templates, abandoned plans, and historical runbooks unless the user explicitly wants archive preservation.

## Workflow

1. Map the docs surface.
   - Search for `docs/`, `README.md`, `README`, `AGENTS.md`, `CLAUDE.md`, and standalone package-adjacent markdown files.
   - Identify which packages are active from `package.xml`, launches, and tests.
2. Classify each touched doc.
   - keep
   - rewrite
   - move
   - delete
3. Rewrite to the smallest current contract.
   - Keep commands runnable from the current repo/workspace.
   - Prefer namespaced topic/service examples when the runtime is namespaced.
   - Replace long duplicated manuals with short routers plus links to the right package docs.
4. Split by type when needed.
   - information: “how it works today”
   - design: “why it is built this way”
   - implementation: “what still needs to be done”
5. Validate.
   - run repo-aware verification from the correct Moleworks Docker/container workflow when the check depends on the ROS environment
   - run `git diff --check`
   - run the bundled markdown-link checker on touched docs
   - grep for known stale terms or deleted paths
   - if the surface is substantial, use reviewer agents on disjoint scopes
6. Stop only after the docs and the checks agree.

## Moleworks-Specific Heuristics

- In `docs/src/`, keep the mdBook surface thin and navigable. Avoid copying full package docs into mdBook when a package README is the better source.
- In `docs/information/`, keep plain markdown operator docs simple. Do not require mdBook to read them.
- In `docs/robot_agent/`, preserve the operator-facing robot runbook surface as its own category. Do not fold it blindly into mdBook or package docs.
- In package docs, document the current launch wrapper that users should actually start from.
- Treat non-namespaced topic examples with suspicion, but verify before “fixing” them. Many Moleworks stacks run under `/<robot_namespace>/...`, while some operator-facing topics and machine interfaces are intentionally root-scoped.
- Do not document historical node candidates or retired startup paths.
- When a launch wrapper injects behavior implicitly, mention that explicitly in docs. Example: estimator-ready gates or auto-added helper nodes.

## Reviewer Loop

When the user wants high-quality docs, split reviews by scope:

- top-level routing and nav
- package runtime contract
- namespacing and topic/service examples
- new docs and links

Ask reviewers for severity-ordered findings with file:line, then patch, rerun checks, and re-review until clean.

## Commands

Prefer fast repo inspection:

```bash
rg --files src/moleworks_ros | rg '(^|/)(README(\.md)?|AGENTS|CLAUDE)\.md$|(^|/)README$|/docs/|\.md$'
rg -n "stale_term_here" src/moleworks_ros
git -C src/moleworks_ros diff --check -- <paths...>
python3 ~/.codex/skills/moleworks-docs-cleanup/scripts/check_markdown_links.py <paths...>
```

## Done Criteria

- touched docs reflect the current code surface
- docs are classified cleanly as information/design/implementation where relevant
- duplicate or stale docs are removed or reduced to routers
- `git diff --check` passes
- local markdown links for touched docs resolve
- reviewer findings are addressed or explicitly documented
