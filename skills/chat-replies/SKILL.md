---
name: chat-replies
description: Read recent Google Chat context, draft or send a reply in the correct DM or space, download collaborator attachments such as timesheets or PDFs, and handle simple meeting coordination by creating or updating a Google Calendar invite and posting the Meet link back in Chat. Use when Lorenzo asks to read a collaborator's recent messages, understand chat context before replying, send a Google Chat reply through the Chat API, pull a PDF or timesheet out of Chat, or create a meeting from a chat exchange.
---

# Chat Replies

Use this skill when Lorenzo asks to read or reply in Google Chat, or to pull files out of Chat for follow-up.

## Workflow

1. Resolve the correct DM or space first.
2. Fetch a small recent message window before reasoning about context.
   - This skill requires Chat read access, not just send access.
   - Required user-auth scopes for normal use are:
     - `https://www.googleapis.com/auth/chat.spaces`
     - `https://www.googleapis.com/auth/chat.messages.readonly`
     - `https://www.googleapis.com/auth/chat.messages.create` when sending
   - Before the first Chat API call, run `which gws` and `gws auth status`.
     - Use the installed `gws` binary for API calls. An installed/source checkout is not credential storage; active credentials normally live under `~/.config/gws`.
     - If `gws auth status` reports `auth_method: none` or no credential source, say the local `gws` credentials are missing instead of repeatedly retrying Chat calls or switching to browser automation by default.
   - Use `pageSize: 10` by default for routine DM history reads.
   - Do not jump to `pageSize: 200` or similar large fetches unless there is a specific reason.
   - Expand the window only if the recent context is unclear, looks truncated, or Lorenzo explicitly asks for deeper history.
   - If the latest message looks blank, inspect `attachment[]` before assuming the sender sent an empty message.
   - With the local `gws` CLI, Chat list/create commands pass required API parameters through `--params`, not first-class flags:
     ```bash
     gws chat spaces messages list \
       --params '{"parent":"spaces/SPACE_ID","pageSize":10}'
     gws chat spaces messages create \
       --params '{"parent":"spaces/SPACE_ID"}' \
       --json '{"text":"message body"}'
     ```
   - To reply in a thread, include the thread in the body:
     ```bash
     gws chat spaces messages create \
       --params '{"parent":"spaces/SPACE_ID"}' \
       --json '{"text":"message body","thread":{"name":"spaces/SPACE_ID/threads/THREAD_ID"}}'
     ```
   - If a thread reply returns `404` or Lorenzo says he cannot see it, repost a concise top-level message in the space and include any PR/review link directly.
   - For uploaded files, prefer `attachment[].attachmentDataRef.resourceName` over `attachment[].name` when downloading.
   - `downloadUri` can bounce to an interactive Google sign-in page from CLI usage, so do not rely on it for automation.
3. Sort messages by `createTime`.
4. Read the latest visible message and the recent sequence before it.
5. Treat consecutive short messages from the same person as potentially one combined thought.
6. If the user asked to reply, draft against that recent sequence, not just the final line.
7. If the user asked to download a timesheet or other PDF from Chat:
   - Prefer the newest PDF whose `contentName` matches the request, for example `timesheet`.
   - Download it with:
     ```bash
     python3 /home/lorenzo/codex_skills/skills/chat-replies/scripts/download_chat_attachment.py \
       --space spaces/SPACE_ID \
       --match timesheet
     ```
   - Inspect the saved PDF locally with `pdftotext -layout` or `pdftoppm`.
   - For admin handoff, resolve all recipient emails from local context first. Maria is `mariat@ethz.ch`. Diego is `digarcia@ethz.ch`.
   - If the task requires a signature, use `/home/lorenzo/codex_skills/skills/pdf-signing/SKILL.md`.
8. If the user asked to create a meeting from the chat context:
   - Resolve the attendee email first. Prefer `references/collaborators.md` or other local mappings before searching elsewhere.
   - Convert relative time like `today` or `3:15` into an absolute date and timezone before acting.
   - Require an explicit duration. If it is missing or ambiguous, ask Lorenzo instead of assuming.
   - Check nearby calendar events around the requested slot before creating anything.
   - If the matching event already exists and Lorenzo owns it, patch that event instead of creating a duplicate.
   - If a nearby matching event exists but is organized by someone else, do not silently create a duplicate invite. Surface the conflict or ask Lorenzo if a new invite should still be created.
   - After creating or updating the event, verify the returned attendee list and `hangoutLink`, then post the exact meeting link back in Chat.
9. Send only the specific reply the user asked for. Do not create autonomous watchers or background reply loops.

## Guardrails

- Do not trust the raw order returned by Chat list calls.
- For a simple "check chat" request, the default is the last 10 messages after sorting by `createTime`.
- If a short recent fetch looks incomplete, increase the window before deciding what is "latest".
- If you need older context before Lorenzo's most recent reply, widen the fetch and anchor on the recent sequence after sorting by `createTime`.
- For file-handling tasks, treat a no-text message with `attachment[]` as content, not noise.
- Use `attachment[].attachmentDataRef.resourceName` with the Chat media endpoint.
- Do not use the human-readable `attachment[].name` with `/v1/media/...`; it can return `400`.
- Do not rely on `downloadUri` for automation; it can return an HTML sign-in flow instead of the file bytes.
- Do not guess collaborator email addresses for follow-up or CCs. Use a local mapping or ask Lorenzo.
- For meeting creation, always use absolute date and time in the Calendar invite and in the Chat confirmation.
- For meeting creation, do not assume the duration if Lorenzo did not specify it.
- For meeting creation, inspect nearby calendar events first so you do not create duplicate overlapping invites for the same person.
- Prefer patching an existing owned event over creating a new one.
- If attendee email is missing, fail early and ask Lorenzo or update `references/collaborators.md`. Do not guess the email.
- Verify the Calendar API response after mutation. The event is not done until the attendee and Meet link are present in the returned object.
- Prefer short, informal first-person wording unless Lorenzo asks for a different tone.
- Prefer the Google Chat API over browser automation for read/send operations.
- If `messages.list` still returns `403 insufficient authentication scopes` right after a re-auth, move `~/.config/gws/token_cache.json` aside and retry once.

## Reference

Read [references/collaborators.md](references/collaborators.md) when you need known collaborator DM mappings.

Use `/home/lorenzo/codex_skills/skills/pdf-signing/SKILL.md` when a Chat-delivered PDF needs a visual signature.

Known recurring workspace shortcut:

- RSL GitHub access workflow test space: `https://chat.google.com/u/1/app/chat/AAQAHKnH5sY` -> `spaces/AAQAHKnH5sY`.
