---
name: chrome-cdp
description: "Control an existing local Chrome session through CDP for page inspection, screenshots, navigation, and interaction. Use only when the user has explicitly approved browser control."
---

# Chrome CDP

Use the bundled Chrome DevTools Protocol CLI against the user's approved live browser session.

Stay within the requested page and task. Do not inspect unrelated tabs, cookies, local storage, credentials, or account data. Limit JavaScript evaluation to the selected page and avoid broad extraction when a targeted accessibility snapshot or selector is enough.

## Prerequisites

- Chrome (or Chromium, Brave, Edge, Vivaldi) with remote debugging enabled: open `chrome://inspect/#remote-debugging` and toggle the switch
- Node.js 22+ available through `nvm` on this machine. Use the absolute wrapper below; it enters Node 22 before running the CLI.
- If your browser's `DevToolsActivePort` is in a non-standard location, set `CDP_PORT_FILE` to its full path

## Commands

Set `CDP=/home/lorenzo/codex_skills/skills/chrome-cdp/scripts/cdp`. The `<target>` is a unique targetId prefix from `list`; copy the full prefix shown in the output. The CLI rejects ambiguous prefixes.

### List open pages

```bash
"$CDP" list
```

### Take a screenshot

```bash
"$CDP" shot <target> [file]    # default: screenshot-<target>.png in runtime dir
```

Captures the **viewport only**. Scroll first with `eval` if you need content below the fold. Output includes the page's DPR and coordinate conversion hint (see **Coordinates** below).

### Accessibility tree snapshot

```bash
"$CDP" snap <target>
```

### Evaluate JavaScript

```bash
"$CDP" eval <target> <expr>
```

> **Watch out:** avoid index-based selection (`querySelectorAll(...)[i]`) across multiple `eval` calls when the DOM can change between them (e.g. after clicking Ignore, card indices shift). Collect all data in one `eval` or use stable selectors.

### Other commands

```bash
"$CDP" html    <target> [selector]   # full page or element HTML
"$CDP" nav     <target> <url>        # navigate and wait for load
"$CDP" net     <target>              # resource timing entries
"$CDP" click   <target> <selector>   # click element by CSS selector
"$CDP" clickxy <target> <x> <y>      # click at CSS pixel coords
"$CDP" type    <target> <text>       # Input.insertText at current focus
"$CDP" loadall <target> <selector> [ms]
"$CDP" evalraw <target> <method> [json]
"$CDP" open    [url]
"$CDP" stop    [target]
```

## Coordinates

`shot` saves an image at native resolution: image pixels = CSS pixels × DPR. CDP Input events (`clickxy` etc.) take **CSS pixels**.

```
CSS px = screenshot image px / DPR
```

`shot` prints the DPR for the current page. Typical Retina (DPR=2): divide screenshot coords by 2.

## Tips

- Prefer `snap` over `html` for a compact page-structure view.
- Use `type` (not eval) to enter text in cross-origin iframes — `click`/`clickxy` to focus first, then `type`.
- Pass typed text as one correctly shell-quoted argument. Do not interpolate arbitrary text into an `eval` expression.
- Chrome shows an "Allow debugging" modal once per tab on first access. A background daemon keeps the session alive so subsequent commands need no further approval. Daemons auto-exit after 20 minutes of inactivity.
