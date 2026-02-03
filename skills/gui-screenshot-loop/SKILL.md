---
name: gui-screenshot-loop
description: General GUI screenshot loop via MCP. Use for any screen/window/region capture while debugging UI or visual tools.
---

# GUI Screenshot Loop

## When To Use
- You need to **see the screen** or a **specific window** while debugging.
- You want visual verification after running commands (RViz, browsers, dashboards, GUIs).

## Prereqs
- X11 session.
- MCP server `screenshot` is configured in `~/.codex/config.toml` and points to `~/.codex/mcp-servers/screenshot/server.py`.

## Quick Discovery
- List monitors: `xrandr --listmonitors`
- List windows: `wmctrl -l`
- Search window titles: `xdotool search --name '.*PATTERN.*'`

## Tool Calls (MCP)
Use the MCP tool `screenshot.screenshot`:

- Full screen:
  - `{"mode":"full"}`
- Monitor by index:
  - `{"mode":"monitor","monitor":1}`
- Monitor by name:
  - `{"mode":"monitor","monitor_name":"DP-2"}`
- Window by title (if multiple matches, provide index):
  - `{"mode":"window","window_title":"RViz","window_index":0}`
- Window by id (most reliable):
  - `{"mode":"window","window_id":"0x064001f6"}`
- Region:
  - `{"mode":"region","x":2600,"y":50,"width":1800,"height":1000}`

Optional:
- `max_width=0` to keep full resolution.

## Failure Handling
- If window title matches multiple IDs, use `wmctrl -l` and capture by `window_id`.
