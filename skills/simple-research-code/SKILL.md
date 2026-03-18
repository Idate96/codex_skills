---
name: simple-research-code
description: Write and review short-lived research code with an aggressively simple, skimmable style. Use when the user asks for simpler code, fewer states, narrower APIs, discriminated unions, exhaustive handling, assertions instead of defensive code, or pre-PR cleanup that removes non-essential changes and complexity.
---

# Simple Research Code

Keep the code easy to skim. Prefer the smallest design that clearly solves the current problem.

## Rules

- Cut scope first. Remove any change that is not strictly required.
- Prefer fewer lines, fewer branches, fewer arguments, and fewer moving parts.
- Keep code local and direct. Do not split simple logic into many helpers.
- Use early returns to flatten control flow.
- Minimize state. Reduce argument count, remove fake flexibility, and narrow objects to the smallest useful shape.
- Prefer discriminated unions over loose bags of fields when a value can be one of several cases.
- Exhaustively handle unions or tagged objects. Fail on unknown variants.
- Do not add defensive code for states that the types already rule out.
- Use `assert` when loading data or when a value must exist. Do not hide missing values behind defaults or fallbacks.
- Do not add backwards compatibility, override plumbing, or optional parameters unless the task strictly needs them.
- Be opinionated about parameter values. Hardcode choices when there is only one real mode.
- If a mathematical step is non-obvious, add a short explanation next to the code.

## While Implementing

- Rewrite the shape of the code if that removes state or arguments.
- Inline one-off logic instead of introducing a new abstraction.
- Pass concrete values instead of config objects when only one mode is supported.
- Delete dead branches, compatibility layers, unused options, and speculative extension points.

## While Reviewing

- Flag optional arguments that are actually required.
- Flag state spaces that are wider than the real problem.
- Flag defensive code, defaults, retries, and catch blocks that hide failures.
- Flag helper functions that make a simple path harder to read.
- Flag changes that are unrelated to the task.
- Prefer one obvious rewrite over incremental patchwork.
