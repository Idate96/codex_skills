---
name: chat-morning-triage
description: "Triage Lorenzo's recent Google Chat into urgent actions, replies, follow-ups, and FYIs. Use for morning chat scans and chat-derived todo lists."
---

# Chat Morning Triage

Use this skill for Lorenzo's beginning-of-day Google Chat sweep.

## Workflow

1. Confirm Chat read access works.
   - Required scopes for normal use:
     - `https://www.googleapis.com/auth/chat.spaces`
     - `https://www.googleapis.com/auth/chat.messages.readonly`
   - If `messages.list` still returns `403 insufficient authentication scopes` right after a re-auth, move `~/.config/gws/token_cache.json` aside and retry once. This forces a fresh token refresh without losing saved credentials.
2. Choose a window that reaches the last working period. Use 18 hours on an ordinary weekday, about
   72 hours on Monday, and widen further after a holiday or time away. Then run, for example:
   ```bash
   python3 /home/lorenzo/codex_skills/skills/chat-morning-triage/scripts/scan_chat.py --since-hours "$SINCE_HOURS" --page-size 15
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
6. If an important DM is missing from the collaborator mapping, report it. Update the local mapping
   only when the user requested reference maintenance and the identity is confirmed.
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
