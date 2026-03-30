---
name: chat-replies
description: Read recent Google Chat context, draft or send a reply in the correct DM or space, and handle simple meeting coordination by creating or updating a Google Calendar invite and posting the Meet link back in Chat. Use when Lorenzo asks to read a collaborator's recent messages, understand chat context before replying, send a Google Chat reply through the Chat API, or create a meeting from a chat exchange.
---

# Chat Replies

Use this skill when Lorenzo asks to read or reply in Google Chat.

## Workflow

1. Resolve the correct DM or space first.
2. Fetch a small recent message window before reasoning about context.
   - Use `pageSize: 10` by default for routine DM history reads.
   - Do not jump to `pageSize: 200` or similar large fetches unless there is a specific reason.
   - Expand the window only if the recent context is unclear, looks truncated, or Lorenzo explicitly asks for deeper history.
3. Sort messages by `createTime`.
4. Read the latest visible message and the recent sequence before it.
5. Treat consecutive short messages from the same person as potentially one combined thought.
6. If the user asked to reply, draft against that recent sequence, not just the final line.
7. If the user asked to create a meeting from the chat context:
   - Resolve the attendee email first. Prefer `references/collaborators.md` or other local mappings before searching elsewhere.
   - Convert relative time like `today` or `3:15` into an absolute date and timezone before acting.
   - Require an explicit duration. If it is missing or ambiguous, ask Lorenzo instead of assuming.
   - Check nearby calendar events around the requested slot before creating anything.
   - If the matching event already exists and Lorenzo owns it, patch that event instead of creating a duplicate.
   - If a nearby matching event exists but is organized by someone else, do not silently create a duplicate invite. Surface the conflict or ask Lorenzo if a new invite should still be created.
   - After creating or updating the event, verify the returned attendee list and `hangoutLink`, then post the exact meeting link back in Chat.
8. Send only the specific reply the user asked for. Do not create autonomous watchers or background reply loops.

## Guardrails

- Do not trust the raw order returned by Chat list calls.
- For a simple "check chat" request, the default is the last 10 messages after sorting by `createTime`.
- If a short recent fetch looks incomplete, increase the window before deciding what is "latest".
- If you need older context before Lorenzo's most recent reply, widen the fetch and anchor on the recent sequence after sorting by `createTime`.
- For meeting creation, always use absolute date and time in the Calendar invite and in the Chat confirmation.
- For meeting creation, do not assume the duration if Lorenzo did not specify it.
- For meeting creation, inspect nearby calendar events first so you do not create duplicate overlapping invites for the same person.
- Prefer patching an existing owned event over creating a new one.
- If attendee email is missing, fail early and ask Lorenzo or update `references/collaborators.md`. Do not guess the email.
- Verify the Calendar API response after mutation. The event is not done until the attendee and Meet link are present in the returned object.
- Prefer short, informal first-person wording unless Lorenzo asks for a different tone.
- Prefer the Google Chat API over browser automation for read/send operations.

## Reference

Read [references/collaborators.md](references/collaborators.md) when you need known collaborator DM mappings.
