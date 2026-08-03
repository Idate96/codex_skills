---
name: google-chat-cdp
description: "Browser-only fallback for Google Chat through an approved live Chrome session. Use only when the user explicitly requests browser/CDP control, or after the standard chat-replies gws CLI workflow is unavailable and the user explicitly approves the fallback. A Chat URL alone must not trigger this skill."
---

# Google Chat CDP

Normal Google Chat reads and replies must use
[`chat-replies`](../chat-replies/SKILL.md) and the authenticated `gws` CLI.
Do not select this skill merely because a request contains a `chat.google.com`
URL. Use this skill together with [chrome-cdp](../chrome-cdp/SKILL.md) only when:

- Lorenzo explicitly asks for browser or CDP interaction; or
- the `gws` workflow is unavailable, that failure has been reported, and
  Lorenzo explicitly approves browser fallback.

Do not silently switch from `gws` to browser automation.

## Preconditions

- `chrome-cdp` is available at `/home/lorenzo/codex_skills/skills/chrome-cdp/`.
- Chrome remote debugging is already enabled in `chrome://inspect/#remote-debugging`.
- The user has explicitly asked for a Google Chat action and approved live-browser interaction.

## Approval Model

Chrome 146+ may still show approval prompts even with the daemon flow:

- once for a fresh browser debugging session
- once for the first attach to a new tab target
- again after login or account switching, because Google auth often replaces the target ID

Do not fight this with repeated retries. If Chat just went through auth or account switching, expect to re-run `scripts/cdp list` and attach to the new Chat target.

## Workflow

### 1. Reuse an existing Chat tab

Prefer an existing Chat target from:

```bash
/home/lorenzo/codex_skills/skills/chrome-cdp/scripts/cdp list
```

Pick `https://chat.google.com/...` if present. Reuse the same target for the whole task instead of opening fresh tabs.

If no Chat tab exists, navigate one approved tab with:

```bash
/home/lorenzo/codex_skills/skills/chrome-cdp/scripts/cdp nav <target> https://chat.google.com
```

### 2. Verify the active account

Check the page accessibility tree first:

```bash
/home/lorenzo/codex_skills/skills/chrome-cdp/scripts/cdp snap <target>
```

Look for:

- `Google Account: ... (email@domain)`

If the wrong account is active, let the user complete login or account switching manually. After that, re-run `list`; the Chat target may have changed.

### 3. Handle login interruptions

If Chat shows `User session expired`, click `Log in` only to resume the already intended manual auth flow. Do not try to guess passwords, OTPs, or SSO steps.

After the user finishes login:

- run `scripts/cdp list` again
- find the new Chat target
- re-verify the account label before sending anything

### 4. Open the conversation carefully

Prefer existing DM rows already visible in the left sidebar. For Google Chat, target the exact row element, not a broad container:

- inspect candidates with `eval` over `[role=link]`
- click the exact row whose text matches the full contact name
- verify the conversation opened by checking the page title or `snap` root title

For example, after clicking a DM, the root title should become the contact name, not just `Chat`.

### 5. Send the message

In an open conversation:

- focus the composer, usually the textbox with aria like `History is on`
- use `/home/lorenzo/codex_skills/skills/chrome-cdp/scripts/cdp type <target> '<message>'`
- click the button whose aria-label is `Send message`

Pass message text as one wrapper argument. Do not concatenate user text into JavaScript, selectors,
or shell source; if quoting is ambiguous, write the draft visibly and stop before Send.

Verify completion:

- the draft textbox should clear back to a blank newline
- the thread should show the new message
- it should not remain in `Sending...`

## Practical Patterns

- Prefer `snap` when structure matters and `eval` when you need precise DOM targeting.
- Avoid index-based selectors across separate calls if the Chat list can change.
- If login/account switching just happened, assume the old target ID is stale until `list` proves otherwise.
- Keep the user informed before any action that sends a real message.
- Scope inspection to the approved Chat tab and conversation; do not inspect unrelated tabs, cookies,
  storage, or account data.
