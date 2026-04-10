---
name: chat-morning-triage
description: Scan Lorenzo's recent Google Chat activity at the start of the day and turn it into a concise todo list. Use when Lorenzo asks for a morning chat scan, chat triage, "what do I need to do from chat", "scan my chat and make a todo list", or similar. Reads recent collaborator DMs from the collaborator note and recent active smaller spaces via the Google Chat API, then extracts action items, reply obligations, and deadlines.
---

# Chat Morning Triage

Use this skill for Lorenzo's beginning-of-day Google Chat sweep.

## Workflow

1. Confirm Chat read access works.
   - Required scopes for normal use:
     - `https://www.googleapis.com/auth/chat.spaces`
     - `https://www.googleapis.com/auth/chat.messages.readonly`
   - If `messages.list` still returns `403 insufficient authentication scopes` right after a re-auth, move `~/.config/gws/token_cache.json` aside and retry once. This forces a fresh token refresh without losing saved credentials.
2. Run:
   ```bash
   python3 /home/lorenzo/codex_skills/skills/chat-morning-triage/scripts/scan_chat.py --since-hours 18 --page-size 15
   ```
3. Read the JSON output and produce a compact todo list with these buckets:
   - `Urgent today`
   - `Needs reply`
   - `Waiting / follow-up`
   - `FYI`
   - Treat a blank Chat message with an attachment as content. A recent PDF or image can be the actual task.
4. For each actionable item, include:
   - who asked
   - the concrete action
   - due date or time if stated
   - source DM or space
5. Default scan scope:
   - known collaborator DMs from `/home/lorenzo/codex_skills/skills/chat-replies/references/collaborators.md`
   - recent named spaces active within the scan window when they are small enough to be useful for triage
6. If an important DM is missing from the collaborator mapping, update the collaborator note after confirming the identity.
7. Do not send replies unless Lorenzo explicitly asks.

## Output Conventions

- Lead with the top 3 concrete actions first.
- Use exact dates and times when they appear in chat.
- Mention uncertainty instead of over-claiming.
- Ignore credential-like content unless it is itself the task.
- Keep the final list short and actionable.
- If a message is attachment-only, say that explicitly instead of calling it empty.

## Troubleshooting

- `403 insufficient authentication scopes` on `messages.list` after a successful re-auth usually means the cached access token is stale even though the new scope is present in `gws auth status`.
- Fix:
  ```bash
  ts=$(date +%Y%m%d_%H%M%S)
  mv ~/.config/gws/token_cache.json ~/.config/gws/token_cache.json.bak_$ts
  ```
- Then rerun the read command or the scan script.

## Reference

- Collaborator and contact mapping:
  - `/home/lorenzo/codex_skills/skills/chat-replies/references/collaborators.md`
