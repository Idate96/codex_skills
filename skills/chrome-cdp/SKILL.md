---
name: chrome-cdp
description: Interact with local Chrome browser session (only on explicit user approval after being asked to inspect, debug, or interact with a page open in Chrome)
---

# Chrome CDP

Lightweight Chrome DevTools Protocol CLI. Connects directly via WebSocket — no Puppeteer, works with 100+ tabs, instant connection.

## Prerequisites

- Chrome (or Chromium, Brave, Edge, Vivaldi) with remote debugging enabled: open `chrome://inspect/#remote-debugging` and toggle the switch
- Node.js 22+ available through `nvm` on this machine. Use the wrapper `scripts/cdp`, which enters Node 22 explicitly before running the CLI.
- If your browser's `DevToolsActivePort` is in a non-standard location, set `CDP_PORT_FILE` to its full path

## Commands

All commands use `scripts/cdp`. The `<target>` is a **unique** targetId prefix from `list`; copy the full prefix shown in the `list` output (for example `6BE827FA`). The CLI rejects ambiguous prefixes.

### List open pages

```bash
scripts/cdp list
```

### Take a screenshot

```bash
scripts/cdp shot <target> [file]    # default: screenshot-<target>.png in runtime dir
```

Captures the **viewport only**. Scroll first with `eval` if you need content below the fold. Output includes the page's DPR and coordinate conversion hint (see **Coordinates** below).

### Accessibility tree snapshot

```bash
scripts/cdp snap <target>
```

### Evaluate JavaScript

```bash
scripts/cdp eval <target> <expr>
```

> **Watch out:** avoid index-based selection (`querySelectorAll(...)[i]`) across multiple `eval` calls when the DOM can change between them (e.g. after clicking Ignore, card indices shift). Collect all data in one `eval` or use stable selectors.

### Other commands

```bash
scripts/cdp html    <target> [selector]   # full page or element HTML
scripts/cdp nav     <target> <url>         # navigate and wait for load
scripts/cdp net     <target>               # resource timing entries
scripts/cdp click   <target> <selector>    # click element by CSS selector
scripts/cdp clickxy <target> <x> <y>       # click at CSS pixel coords
scripts/cdp type    <target> <text>         # Input.insertText at current focus; works in cross-origin iframes unlike eval
scripts/cdp loadall <target> <selector> [ms]  # click "load more" until gone (default 1500ms between clicks)
scripts/cdp evalraw <target> <method> [json]  # raw CDP command passthrough
scripts/cdp open    [url]                  # open new tab (each triggers Allow prompt)
scripts/cdp stop    [target]               # stop daemon(s)
```

## Coordinates

`shot` saves an image at native resolution: image pixels = CSS pixels × DPR. CDP Input events (`clickxy` etc.) take **CSS pixels**.

```
CSS px = screenshot image px / DPR
```

`shot` prints the DPR for the current page. Typical Retina (DPR=2): divide screenshot coords by 2.

## Tips

- Prefer `snap --compact` over `html` for page structure.
- Use `type` (not eval) to enter text in cross-origin iframes — `click`/`clickxy` to focus first, then `type`.
- Chrome shows an "Allow debugging" modal once per tab on first access. A background daemon keeps the session alive so subsequent commands need no further approval. Daemons auto-exit after 20 minutes of inactivity.
