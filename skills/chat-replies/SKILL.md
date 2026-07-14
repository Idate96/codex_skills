---
name: chat-replies
description: "Read or reply in Google Chat, download Chat attachments, or turn chat context into a Calendar invite. Use for DMs, spaces, PDFs, timesheets, meeting links, and Chat-based scheduling."
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
     SPACE='spaces/SPACE_ID'
     PARAMS=$(jq -cn --arg parent "$SPACE" '{parent: $parent, pageSize: 10}')
     gws chat spaces messages list --params "$PARAMS"

     MESSAGE='message body'
     BODY=$(jq -cn --arg text "$MESSAGE" '{text: $text}')
     gws chat spaces messages create \
       --params "$(jq -cn --arg parent "$SPACE" '{parent: $parent}')" \
       --json "$BODY"
     ```
     Always serialize user-derived message text with `jq --arg` (or an equivalent JSON encoder); do
     not interpolate it into hand-written JSON.
   - To reply in a thread, include the thread in the body:
     ```bash
     gws chat spaces messages create \
       --params '{"parent":"spaces/SPACE_ID"}' \
       --json '{"text":"message body","thread":{"name":"spaces/SPACE_ID/threads/THREAD_ID"}}'
     ```
   - If a thread reply returns `404` or Lorenzo says he cannot see it, repost a concise top-level
     message in the space and include any PR/review link directly. Then re-list a small recent window
     and verify the top-level message's text and `createTime`.
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

- Sort by `createTime`; widen the default 10-message window only when context is incomplete.
- Treat attachment-only messages as content. Download through `attachmentDataRef.resourceName`, never the human-readable attachment name or interactive `downloadUri`.
- Do not guess collaborator email addresses for follow-up or CCs. Use a local mapping or ask Lorenzo.
- For meetings, require an attendee, absolute time, and explicit duration; inspect nearby events and patch a matching owned event instead of duplicating it.
- Verify Calendar mutations include the intended attendee and Meet link.
- Verify sent Chat messages by reading back the returned message resource or a small recent window.
- Prefer short, informal first-person wording unless Lorenzo asks for a different tone.
- Prefer the Google Chat API over browser automation for read/send operations.
- If `messages.list` still returns `403 insufficient authentication scopes` right after a re-auth, move `~/.config/gws/token_cache.json` aside and retry once.

## Reference

Read [references/collaborators.md](references/collaborators.md) when you need known collaborator DM mappings.

Use `/home/lorenzo/codex_skills/skills/pdf-signing/SKILL.md` when a Chat-delivered PDF needs a visual signature.

Known recurring workspace shortcut:

- RSL GitHub access workflow test space: `https://chat.google.com/u/1/app/chat/AAQAHKnH5sY` -> `spaces/AAQAHKnH5sY`.
