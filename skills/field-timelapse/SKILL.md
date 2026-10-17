---
name: field-timelapse
description: "Turn camcorder or interval-recording footage of a robot field test into one edited, privacy-safe time-lapse: remove idle (near-duplicate), people, cab-boarding, dark and glitch frames; blur other companies' machines, vehicles and objects while the robot and the soil stay sharp; give all clips the same pace; stitch with transitions and date/time captions. Use for post-processing, stitching or anonymising site time-lapse or camcorder videos of the Moleworks robot."
---

# Field Time-Lapse

The tools are `scripts/field_timelapse/` in moleworks_ros (on `main` since 7d008c69c). Its `README.md` has
the commands and what to check after each step; `example_courtyard.yaml` (the first shoot) is the template
config. Read both before starting.

## Where Things Are

- starship-1 (RTX 4090, run everything here):
  - tools: `/home/lorenzo/moleworks/.worktrees/moleworks_ros_field_timelapse/scripts/field_timelapse`
    (detached at `origin/main`; update with `git fetch origin main && git checkout --detach origin/main` there)
  - Python with ultralytics, SAM2, matplotlib, pyyaml:
    `/mnt/ssd0/projects/terra/edit/timelapse_20261015-16/.venv/bin/python`
  - new runs: `/mnt/ssd0/projects/terra/edit/<run_name>/` holding `src/` (the clips) and `timelapse.yaml`
  - worked example (15-16 Oct 2026 courtyard): run directory `/mnt/ssd0/projects/terra/edit/timelapse_20261015-16/`
    (GNSS interval fit in `work/interval/`), result and `NOTES.md` in
    `/mnt/ssd0/projects/terra/videos/2026-10-15-16_panasonic_timelapse/`
- laptop: moleworks_ros at `/home/lorenzo/moleworks/ros2_ws/src/moleworks_ros` (pull `main` first).

## Ask The User Up Front

- What must not be shown: which machines, vehicles and objects. On the first shoot: other excavators and
  concrete blocks; vehicles, a parked lorry and stray people were blurred too. Whether the operator sitting
  in the robot cab may stay visible (kept on the first shoot).
- Which clips go into the edit, in which order; target length if any.

## Establish The Facts Before Tuning

- Copy clips off the card with a per-file checksum; never edit the originals.
- Clip type: real audio = real-time recording (make a time-lapse first, for example one frame per
  keyframe); digital-silence audio = in-camera interval recording. Measure it: both have an audio track.
- Real capture times: camcorder clocks are often wrong. If the robot recorded bags at the same time, fit
  `start_utc` and `step_s` by matching the robot's image position in the video to its GNSS track (good
  work for a background agent). Cross-check other days with sunset when you reuse a clock offset.
- Run `explore.py` and read `overview.png` for every clip: camera bumps (steps in dx/dy), dusk/night, fog,
  exposure glitches.
- Pace: give every clip the same `resample_s` (the capture interval of the interval recordings, 5 s on the
  first shoot). Without it a real-time clip plays visibly slower; the user asked for this fix.

## Masks: What Went Wrong Last Time

- Colour (orange/yellow) fails in fog: add a `parked` window while a machine stands still in the fog.
- Sunlit autumn foliage triggers colour detection: keep `colour_machines: false` for clips whose machines
  are parked and covered by static masks.
- When the robot is in front of another machine, its SAM mask can swallow that machine and leave it sharp.
  The code keeps orange/yellow out of the robot mask and treats big colour blobs as machines; re-audit
  after any change to privacy.py.
- SAM leaves dark crawler tracks out; the lower-half-of-machine rule covers them.
- SAM picks what dominates a box: check every `static_seg<k>.jpg`; use `mode: box` where another machine
  parks on the object.
- Objects that come and go (a parked lorry): `conditional_boxes` with their colours.
- People half-hidden by the robot (fog) can be missed by YOLO: look at the first frames of every clip and
  exclude by hand (`robot_blur_check.py --manual`).

## Always Verify Before Calling It Done

- Preview renders (`render_clip.py --limit 200`) and look at `review/` frames from all parts of all clips.
- After the full render: `robot_blur_check.py`, then render again (masks are cached, so it is fast).
- Both audits in `audit.py` and every crop sheet. Acceptable flags: the operator in the cab, the robot itself
  (fog, crossfades), small distant vehicles if the user accepted them. Report anything else with frame numbers.
- Show the user the final video (copy it to their laptop and open it) before publishing; publish with a
  `NOTES.md` that states what was removed, what was blurred and what was not.

## Operations

- Run the clips' analysis and renders in parallel; check the GPU is not starved by another job.
  Delete `work/<clip>/masks/` after changing privacy logic or config boxes.
- Avoid `pkill -f <pattern>` when the pattern also appears in your own command line.
- Only the edited video is anonymised. Say so wherever unblurred copies (originals, 1080p versions) sit
  next to it.
