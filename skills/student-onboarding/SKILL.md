---
name: student-onboarding
description: Onboard a new RSL student project across the student spreadsheet, shared Google Drive project folders, and follow-up access/admin requests. Use when Codex must add a new student entry, create the project folder from the template, place the grading sheet correctly, or follow the ETH/RSL student-start checklist.
---

# Student Onboarding

Use this skill for new student-project setup in the RSL shared Drive and spreadsheet.

## Workflow

1. Read [references/rsl-student-onboarding.md](references/rsl-student-onboarding.md) before mutating Sheets or Drive.
2. Confirm the student type and final folder name, for example `MT - Student Name` or `MA - Student Name`.
3. Check for existing spreadsheet rows or Drive folders before creating anything.
4. For the spreadsheet:
   - insert from the top under the header
   - do not append at the bottom
5. For Drive:
   - create the outer project folder under the current year
   - copy the grading sheet into the outer folder
   - create the nested `<outer folder name> Student Folder`
   - copy the template tree into that nested folder
6. If an earlier run was interrupted, inspect the partial state and resume idempotently instead of recreating the whole tree.
7. After the Drive and sheet steps, continue with the remaining admin steps from the reference.
8. Distinguish Google Form types before automating:
   - anonymous or non-account-bound forms can use direct `viewform` / `formResponse` submission
   - signed-in forms that record the supervisor email must use the live Chrome session through local CDP tooling
9. Before the GitHub-group step, ask for the student's GitHub handle if it is not already known. Do not contact Fang Nan without the exact handle.
10. For Chat-related steps, send only the specific message the user asked for. Do not create autonomous watchers, auto-repliers, or background chat loops.

## Guardrails

- Prefer explicit row insertion and explicit ranges for Sheets writes.
- Prefer existence checks before Drive folder creation or template copy.
- If a folder-copy run was interrupted, clean only the true duplicates rather than deleting the whole folder.
- Keep the grading sheet at the outer project-folder level.
- Preserve the local naming conventions documented in the reference unless the user explicitly overrides them.
- Do not assume all Google Forms can be submitted through `gws`; `gws` does not create Google Forms responses.
- For signed-in Google Forms, prefer the local Chrome CDP workflow over brittle UI key injection.
- For Fang Nan / GitHub-group requests, prefer the Google Chat API over browser automation when Chat scopes are available.
- Use the Google Chat API as an explicit assistant tool, not as a persistent bot.
- Do not run persistent chat watchers or auto-reply loops in shared spaces.
- When sending a message as the assistant, use first-person wording and keep the tone short and informal unless the user asks otherwise.
- If needed, send the same Fang Nan request by email as a fallback or visibility duplicate.
- For lab Chat messages, prefer short, informal wording. Avoid stiff language unless the situation actually requires it.

## Reference

Load [references/rsl-student-onboarding.md](references/rsl-student-onboarding.md) for:

- shared Drive and spreadsheet IDs
- exact spreadsheet row placement and columns
- template source IDs
- folder naming conventions
- remaining admin follow-up steps
