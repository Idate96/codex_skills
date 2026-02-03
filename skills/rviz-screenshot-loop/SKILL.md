---
name: rviz-screenshot-loop
description: Capture RViz/GUI screenshots via MCP to close the loop while debugging ROS. Use when you need visual verification in RViz or other windows.
---

# RViz Screenshot Loop

If the task is not RViz-specific, use `gui-screenshot-loop` instead.

## When To Use
- You need to **see RViz** or other GUI state while debugging ROS.
- You want to verify **fixed frame**, **display status**, or **visual artifacts** after running ROS commands.

## Prereqs
- X11 session.
- MCP server `screenshot` is configured in `~/.codex/config.toml` and points to `~/.codex/mcp-servers/screenshot/server.py`.

## Quick Discovery
- List monitors: `xrandr --listmonitors`
- List windows: `wmctrl -l`
- Find RViz windows: `xdotool search --name '.*RViz.*'`

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

## Workflow
1. Run your ROS or sim command.
2. Capture RViz with `mode="window"` (prefer `window_id`).
3. Read the RViz text/status (Global Status, TF, fixed frame, PointCloud status).
4. If the user asked for sim bringup, **use `sim-startup` skill first**, then capture RViz.
5. If debugging TF or topics, **use `ros2-debugging` skill first**, then capture RViz to confirm.

## Failure Handling
- If window title matches multiple IDs, pick the correct one from `wmctrl -l` and use `window_id`.
