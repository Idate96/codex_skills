---
name: moleworks-subagent-orchestrator
description: "Orchestrate sub-agents for long-horizon Moleworks code, RL, and ROS tasks. Use when the task needs many code/doc searches, Slurm or W&B inspection, benchmark comparisons, or ROS process inspection without polluting the parent context."
---

# Moleworks Sub-Agent Orchestrator

Use this skill when the task is too large or too noisy for one parent context window.

## Goal

Keep the parent thread in the smart zone. The parent agent should orchestrate, decide, and integrate. Sub-agents should do context-heavy exploration and return compact results.

## When To Use

Use sub-agents when the task involves one or more of these:

- repeated `rg` or file-discovery passes across the Newton or ROS repos
- Slurm log inspection, `squeue`, `sacct`, and run-dir archaeology
- W&B run lookup or metric comparison
- benchmark JSON comparison across multiple artifacts
- ROS/tmux/process inspection with lots of noisy output
- tracing a code path through several packages before patching

Do not spawn sub-agents just because the task is hard. Spawn them when there is clear context-isolation value.

## Recommended Delegation Pattern

Parent thread responsibilities:

- decide scope
- choose which repo or branch matters
- assign a narrow question
- integrate the result
- make final code changes unless write ownership is explicitly delegated

Good sub-agent task shapes:

- "Find the exact branch-local entrypoints and docs for shared-turn benchmark + terrain-bank replay."
- "Inspect synced Slurm log `X` and summarize failure mode, first traceback, and likely owning script."
- "Compare benchmark JSON `A` vs `B` and report success/full/close/timeout deltas."
- "Inspect tmux panes and process list for the parity stack; return only stale PIDs, missing topics, and TF failures."

## Role Suggestions In Codex

- Use `explorer` for read-only codebase or doc questions.
- Use `worker` only for bounded code changes with a disjoint write set.
- Reuse the same agent for follow-up questions on the same narrow topic instead of respawning.

## Output Contract For Sub-Agents

Require compact answers:

- answer first
- then evidence as file paths, line refs, commands, or artifact names
- no raw log dumps unless the parent explicitly asks
- no speculative architecture essays

Good result shape:

1. direct answer
2. 3-8 bullets of evidence
3. open risks or unknowns

## Moleworks-Specific Delegation Patterns

- Newton repo locator: docs, scripts, task config, benchmark entrypoints
- Cluster/log digester: `cluster/*.sh`, synced Slurm logs, `docs/experiments/*`
- Benchmark diff analyst: `outputs/...`, `logs/rsl_rl/...`, benchmark JSON, W&B identifiers
- ROS parity inspector: tmux panes, process lists, TF/topic health, duplicate publishers

## Fallback If Sub-Agents Are Unavailable

If the harness cannot spawn sub-agents:

- keep exploration tightly scoped
- prefer `rg` over broad file reads
- inspect one subsystem at a time
- summarize findings before moving to the next subsystem
- avoid pasting large logs or command output into the main thread
