---
name: meeting-updates
description: Update recurring team meeting notes and standup documents, especially Google Docs, while matching the surrounding style, nesting, and level of abstraction. Use when Codex must add a new dated entry, rewrite update bullets, summarize recent work into meeting-note form, or clean up meeting-note formatting after edits.
---

# Meeting Updates

Use this skill to add or revise updates in recurring meeting notes without turning the document into a commit dump.

## Primary Document

Use this team meeting doc by default unless the user points to another one:

- `https://docs.google.com/document/d/1NnnKeuBg6DJZtpL5gxA5A6kkEhDlZXGiFEkZpbGcAUw/edit?usp=drive_web&ouid=108547263280155251152`
- Document id: `1NnnKeuBg6DJZtpL5gxA5A6kkEhDlZXGiFEkZpbGcAUw`

## Workflow

1. Read the surrounding section before editing.
2. Identify the local formatting pattern:
   - date lines
   - whether `Updates:` is a bullet
   - initials blocks such as `[LT]`
   - bullet nesting depth and indentation
   - whether nearby content uses bold or plain text
3. Draft the content at meeting-note level, not commit-log level.
4. Insert or rewrite the entry.
5. Read the edited block back and verify the structure matches the surrounding document.

## Writing Rules

- Group updates by system or project first, for example `moleworks_ros` or `newton`.
- Put concrete work under those parent bullets as nested sub-bullets.
- Keep bullets informative but compressed. Summarize themes, integrations, fixes, or outcomes rather than listing individual commits.
- Prefer standup wording such as `controller tuning`, `digging integration`, `infrastructure cleanup`, `dynamics consistency`.
- Avoid strangely mixed abstraction, for example a parent bullet that is vague and child bullets that are raw commit subjects.
- Avoid accidental bold unless the surrounding section uses it for the same element type.
- Preserve the local document convention even if it is not globally ideal.

## Standup Heuristics

- If the source material is git history, collapse repeated commits into one contribution theme.
- If several updates belong to the same repo, keep the repo as the parent bullet and nest the topics underneath.
- If a line only makes sense to someone who saw the PR title, rewrite it.
- If a line is too generic to be useful in a meeting, rewrite it.
- Default to 2-4 parent bullets and a small number of nested details.

## Google Docs Notes

- Use `gws docs documents get` to inspect nearby text and paragraph structure before editing.
- Use `gws docs documents batchUpdate` for deterministic edits.
- After editing, read back the affected paragraph styles and confirm indentation and bold state match the intended hierarchy.
- When fixing formatting, prefer correcting the whole inserted block instead of stacking many tiny follow-up patches.
