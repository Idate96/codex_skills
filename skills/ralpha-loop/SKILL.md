---
name: ralpha-loop
description: "Run an implement-test-review loop until one persistent reviewer reports no findings. Use only when the user explicitly requests strict iterative coding, repeated review, or a clean-review completion gate; do not use for routine or small changes."
---

# Ralpha Loop

This workflow is opt-in and intentionally heavyweight. For a small or routine change, make the
minimal edit, run one focused check, and continue. Do not spawn a reviewer unless the user asked for
strict iteration or a clean-review gate. Optimize for execution speed when that gate was not asked
for.

Execute a deterministic loop:
1. Implement or patch.
2. Run targeted tests/checks.
3. Send the same reviewer agent the actual updated diff/files and evidence.
4. Address findings.
5. Retest.
6. Repeat until reviewer reports no findings.

## Loop Contract

- Keep one persistent reviewer agent for the whole task.
- Keep one persistent worker agent only if parallel implementation is needed.
- Reuse agent IDs across iterations; do not respawn unless the agent is dead/unrecoverable.
- Require at least one concrete verification step after every code change.
- Treat reviewer findings as the primary queue; do not add unrelated refactors during the loop.

## Setup

Create and track an in-memory registry at loop start:
- `reviewer_agent_id`
- `worker_agent_id` (optional)
- `iteration` (start at 1)
- `open_findings` (list)
- `resolved_findings` (list)

Spawn reviewer once with scope, files, and review rubric (bugs/regressions/tests). Keep this reviewer alive until completion.

## Iteration Procedure

### 1) Implement

Apply the smallest change set that addresses current scope or findings.

### 2) Verify

Run fast, relevant checks first:
- Linters/compilation/static checks for touched files.
- Focused tests for modified behavior.
- Runtime smoke test when applicable.

If a check cannot run, record why and continue to review with that gap explicit.

### 3) Review (same reviewer)

Send reviewer a delta-focused prompt:
- Current objective.
- Files changed this iteration.
- Verification evidence.
- Previous findings status (fixed/pending).

Ask for severity-ordered findings with file:line and explicit “no findings” when clean.
Require the reviewer to inspect the repository artifacts directly; a worker summary alone is not review evidence.

### 4) Triage

For each finding:
- Confirm reproducibility or code-level validity.
- Mark as `accepted` or `contested` with short rationale.
- Patch all accepted findings before next review round.

### 5) Retest

Re-run affected checks after patches.

### 6) Loop Gate

- If reviewer reports findings: increment iteration and repeat.
- If a delta review reports no findings: run one final holistic review of the complete diff and current test evidence. Exit only when that review also reports no findings.

## Agent Coordination

- Use `followup_task` to give the existing reviewer another review turn; use `send_message` only to add context while it is already running.
- Use `interrupt_agent` only to redirect stale work; otherwise queue normally.
- Keep the reviewer available between rounds to preserve context continuity.
- If the reviewer times out, use `wait_agent` before deciding to respawn.
- Respawn reviewer only when agent is closed, unresponsive across retries, or context is corrupted.
- When respawning is unavoidable, send a compact handoff:
  - latest objective
  - current diff summary
  - findings ledger
  - verification results

## Findings Ledger Format

Track each finding as:
- `id`: stable label (e.g., `R1-F2`)
- `severity`
- `location` (`path:line`)
- `summary`
- `status` (`open`, `fixed`, `contested`)
- `evidence` (test/check or reasoning)

Reuse the same IDs when reporting progress to reviewer.

## Completion Criteria

Exit only when all are true:
- Reviewer explicitly reports no findings.
- The final no-findings result covers the complete integrated diff, not only the last iteration.
- Latest checks for touched behavior pass (or remaining gaps are documented).
- Final summary includes:
  - what changed
  - what was tested
  - reviewer outcome
  - residual risks/gaps
