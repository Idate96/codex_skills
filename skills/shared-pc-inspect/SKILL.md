---
name: shared-pc-inspect
description: "Inspect and hand over the shared robot PC (rslpc) cleanly: baseline snapshot at session start, diff at session end, machine health (thermals, disk, gateway, CAN, time sync), and docker hygiene. Use at the start and end of any robot session, or when asked whether the machine is healthy / was left in a good state."
---

# Shared PC Inspect (rslpc)

Goal: leave the machine exactly as the next user expects it. The mechanism is
snapshot -> work -> diff, plus a health pass. All checks are read-only; any
restore action is proposed to the user, never applied silently.

Scripts live in `scripts/` next to this file. State goes to
`~/.local/state/shared-pc-inspect/` (per-user, never committed).

## Session start — take the baseline

```bash
scripts/snapshot.sh baseline     # prints the snapshot dir
scripts/health.sh                # PASS/WARN/FAIL; exit 1 on FAIL
```

If health FAILs before you touched anything, report it to the group chat —
that establishes the problem predates your session.

## Session end — handover check

```bash
scripts/diff.sh <baseline_dir>   # snapshots current state, diffs vs baseline
scripts/health.sh
```

Walk every CHANGED section with the user. For each change decide:
revert (you changed it for the session), keep + announce (intentional
improvement — tell the other users), or investigate (you didn't do it).

## What the snapshot covers and why

| Surface | Shared-PC failure mode |
|---|---|
| Default route + netplan | Only the `via:` line ever legitimately changes: `192.168.19.2` = Gravis **4G** uplink (field), `192.168.19.110` = **Trailer Router** uplink (Wi-Fi/Rocket available). The robot (`gravis_pc` = `.1`) is on-link — gateway choice can only break internet, never the robot. `addresses:` must stay exactly `192.168.19.30/24` (an Aug-2026 incident pasted the gateway into the address field → IP conflict). Backup convention: `99-gravis_connection.yaml.bak_<ts>_before_switch_to_<target>` — always create one before editing. |
| Docker containers + restart policies | A container left with `restart=always/unless-stopped` resurrects on boot, grabs the GPU/ports/CAN and runs stale code autonomously. Policy on this machine: every container `restart=no`. |
| Docker images | Someone retags `moleworks_ros:latest` and the next user silently runs different code. Snapshot records image IDs. |
| Enabled systemd units | Robot infra (`can.service`, `ptp4l_client`, `m4-maintenance-proxy`, `enp6s0-*`, `force-performance-profile`) must stay enabled+active; the dormant PTP units (`ptp4l`, `ptp-master`, `livox-ptp-master`, `phc2sys`) must stay **disabled** — enabling one (e.g. while chasing Livox sync) fights `ptp4l_client`. The two `enp6s0-*` units + the EEE-off pre-step in `ptp4l_client` are a deliberate triple-redundant NIC workaround — don't "clean up". |
| /etc checksums (hosts, environment, udev 99-slabs.rules) | Silent edits to shared config. |
| Group membership (docker/sudo/dialout/video) | Privilege drift. |
| CAN interfaces (can0..4 bitrate/state) | Robot bus misconfigured for next user. |
| Host CUDA links + apt history | "I changed CUDA inside my docker container" is fine; `apt install cuda-*` or `/usr/local/cuda` symlink changes on the **host** are not. Snapshot catches host-side drift. |

## Health pass (`health.sh`)

Checks: gateway reachability, root-disk fill, CPU Tctl / GPU temp, available
RAM, failed systemd units, container restart policies, running containers,
kernel error volume (24 h), `ptp4l_client` active, reboot-required flag.

Known machine context (audited 2026-08):
- **Time**: `ptp4l_client` (software timestamping, slaved to gravis_pc `.1`)
  is the **only** time source — chrony/timesyncd are inactive. If someone
  stops it, the clock free-runs fast (~-24000 ppb) with nothing catching it.
- **Thermals**: Tctl ≈95 °C at near-idle is explained: the (intentional)
  `force-performance-profile.service` pins min freq to ~3 GHz on all 24
  threads → ~90 W idle package power at Tjmax. Consequence: zero headroom —
  heavy jobs throttle immediately; cooling hardware also suspect (fan RPM is
  unreadable, no nct6775 module — verification needs BIOS/physical check).
  A GNOME "Balanced" click silently undoes the profile until reboot.
- **Disk**: chronically ~90% full. Docker images ~417 GB reclaimable;
  `~lorenzo/mcap/dig` ≈895 G; bag data across users ≈1.7 T. Prune only your
  own data; `docker image prune` only with confirmation, announced.
- **CAN**: can2/can3 (PEAK, 250k) + can4 (slcan) come from
  `/usr/sbin/can-start`, which hardcodes the Lawicel adapter serial — a
  swapped adapter makes can4 silently vanish.
- **udev** `99-slabs.rules`: the i2c `0666` mode is load-bearing (no `i2c`
  group exists) — "tightening" it locks all users out of ZED-GMSL cameras.
- **sudo/groups**: all 11 users are in `sudo`+`docker` (root-equivalent), and
  a hand-added NOPASSWD line sits directly in `/etc/sudoers` — validate any
  sudoers change with `visudo -c`; a syntax error bricks sudo for everyone.
- **Clean autostart baseline**: no rc.local, no user crontabs, no linger, no
  `@reboot` entries anywhere — anything like that appearing in a diff is new.
- `NetworkManager-wait-online.service` failing is a known benign WARN.
- NVMe wear is unmeasurable until `nvme-cli`/`smartmontools` is installed.

## Docker hygiene rules for this machine

Launch convention: `~/ros2_ws/src/moleworks_ros/docker/docker_launch.sh`
(hand-rolled `docker run`, no compose, never passes `--restart`). Facts that
matter for handover (audited 2026-08):

- Containers run `--privileged --cap-add=ALL --net=host --ipc=host` as root
  and bind-mount the **real** `/etc/passwd`, `/etc/shadow`, `/etc/group`
  read-write (via symlinks in `docker/.etc/`), plus `/var/run/docker.sock`,
  `$HOME`, `/lib/modules`, `/dev/input`. "Inside my container" is **not**
  isolated — anything root does in there can be a host mutation.
- `--net=host` + default `ROS_DOMAIN_ID=0`: your stack binds host ports
  (foxglove 8765, FastDDS 7000–7020, 7410+). Another user starting ROS 2 at
  domain 0 joins your live graph. Check `ss -tlnp | grep 8765` at handover.
- **Tag drift trap:** the bare no-arg `docker_launch.sh` path does
  `docker pull rslheap/moleworks_ros:latest`, which can differ from the local
  `moleworks_ros:latest` build and orphans ~40 GB images. The snapshot records
  image IDs so a retagged `latest` shows up in the diff.
- No log rotation is configured in `/etc/docker/daemon.json` — json logs grow
  unbounded (289 MB observed). Long-running containers: check log size.
- Before handover: `docker stop` your containers, never someone else's.
  Exited containers of other users are not yours to prune. Dangling images
  routinely accumulate to ~400 GB reclaimable; `docker image prune` only with
  explicit user confirmation, announced to the group.
- CUDA/toolchain experiments: keep them in your image/container. If you must
  change the host, announce it and record it in the handover diff.

## Boundaries

- Never modify netplan, systemd units, group membership, or another user's
  containers/files without explicit user confirmation in that session.
- This skill inspects the PC only. Robot bringup, controllers, and motion are
  owned by `robot-ros` and its narrow skills — do not restart robot processes
  from here.
- If the diff shows a change you cannot attribute, surface it verbatim rather
  than "fixing" it; another user's session may depend on it.
