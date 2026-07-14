---
name: rviz-screenshot-loop
description: "Capture and inspect RViz or ROS GUI screenshots. Use for iterative visual validation of frames, display status, overlays, maps, point clouds, and rendering artifacts."
---

# RViz Screenshot Loop

If the task is not RViz-specific, use `chrome-cdp` for approved Chrome capture
or an available GUI screenshot tool directly.

## Prereqs
- X11 session.
- An enabled screen-capture capability. Discover the available GUI or screenshot tool at runtime; tool names vary by environment.

## Quick Discovery
- List monitors: `xrandr --listmonitors`
- List windows: `wmctrl -l`
- Find RViz windows: `xdotool search --name '.*RViz.*'`

## Capture Parameters

Use the available screenshot tool's equivalent of:

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

Request original/full resolution when text or small status icons must be read.

## Workflow
1. Run your ROS or sim command.
2. Capture RViz with `mode="window"` (prefer `window_id`).
3. Read the RViz text/status (Global Status, TF, fixed frame, PointCloud status).
4. If the user asked for sim bringup, **use `sim-startup` skill first**, then capture RViz.
5. If debugging TF or topics, **use `ros2-debugging` skill first**, then capture RViz to confirm.
6. Inspect the resulting image rather than treating a successful capture call as visual verification.

## Failure Handling
- If window title matches multiple IDs, pick the correct one from `wmctrl -l` and use `window_id`.
- If no capture capability is available, report the limitation and preserve useful text diagnostics from RViz/ROS logs.
