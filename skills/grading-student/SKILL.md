---
name: grading-student
description: Finalize RSL student grading and offboarding. Use for grading sheets, grade handoffs, tracker evidence, eDoz requests, access revocation, or status replies.
---

# Grading Student

Use this skill for the end-of-project grading/offboarding path for RSL student projects.

## References

- Load `../student-onboarding/references/rsl-student-onboarding.md` before editing the RSL project tracker.
- Use `../chat-replies/SKILL.md` for Google Chat API details when a chat reply is requested.
- Use Google Drive, Sheets, and Gmail connector tools when available. Use `gws` for Google Chat only when `gws auth status` shows active credentials with Chat scopes.

## Google Chat Auth

Before sending a Chat confirmation, run:

```bash
gws auth status
```

For grading/offboarding Chat replies, the stored `gws` credentials must include at least:

- `https://www.googleapis.com/auth/chat.spaces`
- `https://www.googleapis.com/auth/chat.messages.readonly`
- `https://www.googleapis.com/auth/chat.messages.create`

If resolving people or DMs as part of the workflow, also include:

- `https://www.googleapis.com/auth/chat.memberships.readonly`
- `https://www.googleapis.com/auth/directory.readonly`

If credentials exist but Chat scopes are missing, re-auth with an explicit union scope set rather than `--full`:

```bash
gws auth login --scopes https://www.googleapis.com/auth/drive,https://www.googleapis.com/auth/spreadsheets,https://www.googleapis.com/auth/gmail.modify,https://www.googleapis.com/auth/calendar,https://www.googleapis.com/auth/documents,https://www.googleapis.com/auth/presentations,https://www.googleapis.com/auth/tasks,https://www.googleapis.com/auth/pubsub,https://www.googleapis.com/auth/cloud-platform,https://www.googleapis.com/auth/chat.spaces,https://www.googleapis.com/auth/chat.messages.create,https://www.googleapis.com/auth/chat.messages.readonly,https://www.googleapis.com/auth/chat.memberships.readonly,https://www.googleapis.com/auth/directory.readonly,openid,https://www.googleapis.com/auth/userinfo.email,https://www.googleapis.com/auth/userinfo.profile
```

If `gws auth login` says no OAuth client is configured, first create or provide `/home/lorenzo/.config/gws/client_secret.json`; do not try unrelated credential files from other environments.

## Workflow

1. Find the student project folder and grading sheet in Drive.
   - Prefer exact student-name searches, then project-title or supervisor searches.
   - Confirm the grading sheet belongs to the same student and project before using it.
   - Download or export the grading sheet locally if it must be attached to email.
2. Read the final grade from the grading sheet.
   - Prefer explicit cells/labels such as final proposed grade or supervisor mean.
   - If multiple grade values exist, report the final proposed grade and mention the supporting supervisor values.
3. Inspect the project folder for offboarding artifacts.
   - Grading available: true only when the grading sheet exists.
   - Report available: true only when a report PDF/DOCX or final submission is present.
   - Source files available: true only when source/data/code artifacts are present in the expected folders or clearly linked.
   - Access revoked: true only after revocation is confirmed, not just because the project is complete.
4. Update `List of Student Projects` carefully.
   - Spreadsheet id: `1w_IP91LTrw-JgUT1bXRjrK6xoZ87yLGG4lZd0Q4qfS0`
   - Main tab: `Ongoing Projects`
   - Search for the student row first; do not insert a new row for offboarding.
   - Use explicit bounded ranges or row/column indexes for writes.
   - Correct stale folder names in column `L` when the matching Drive folder is clear.
   - Set `M Completed` only when the project is actually finished.
   - Set `O Grading available` when the grading sheet exists.
   - Leave `T Grade eDoz (Mariat)` blank unless eDoz entry is already confirmed.
5. Email the admin/contact.
   - Use Gmail connector tools when available.
   - For Lorena Luzi, use `lluzi@ethz.ch` if confirmed by Gmail context.
   - Keep the request short: student name, final proposed grade, grading sheet attachment.
6. Reply in Chat only after the requested actions are actually done.
   - Preferred command shape:
     ```bash
     gws chat spaces messages create \
       --params '{"parent":"spaces/SPACE_ID"}' \
       --json '{"text":"message body"}'
     ```
   - If `gws auth status` has no credentials, say Chat is blocked by missing local `gws` credentials. Do not switch to browser automation unless Lorenzo explicitly asks for it.

## Tracker Columns

- `L`: GDrive Folder
- `M`: Completed
- `N`: Report available
- `O`: Grading available
- `P`: Source files available
- `Q`: Access revoked
- `T`: Grade eDoz (Mariat)
- `V`: Agreed RSL Policy

## Guardrails

- Do not mark report/source/access-revoked/grade-entered as complete without evidence.
- Do not overwrite unrelated tracker cells just to clean up old rows.
- Do not use credentials from unrelated environments or clusters.
- Do not expose OAuth tokens, client secrets, downloaded credentials, or private grading-sheet contents beyond the requested summary.
- Keep chat confirmations informal and factual, for example: `Done - I sent Lorena the grading sheet for Luca and updated the tracker for the completed project/grading-sheet handoff.`
