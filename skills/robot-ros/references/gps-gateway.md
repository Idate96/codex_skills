# Menzi GPS gateway and RTK recovery

Use this for the Menzi M4 Septentrio receiver, not for CAT323 without first
checking its topology. This is a network operation; keep the running ROS stack
and receiver configuration otherwise intact.

## Addresses and scope

| Device | Known address / role |
| --- | --- |
| Septentrio GPS receiver | `192.168.19.4` |
| rslpc | `192.168.19.30`, wired interface `enp6s0` |
| Router | `192.168.19.2` |
| Lorenzo's laptop | `192.168.19.88`, wired interface `enp0s31f6` |
| Laptop Internet uplink | `wlp0s20f3`; discover its current gateway and SSID |

These are known allocations, not proof of current connectivity. Inspect live
addresses and routes. GPS uses its own gateway: a working rslpc connection
through `.88` does not imply that GPS also uses `.88`.

For a requested temporary recovery, change the current receiver configuration
only. Do not save it to boot or edit Netplan as part of the trial. Existing user
authorization to recover GPS Internet/RTK covers this gateway change; do not
ask again merely because it is a receiver setting.

## 1. Establish the working path and baseline

From the laptop, verify its uplink with interface-bound HTTPS, and inspect
`ip -4 rule show`, `ip -4 route show table 19130`, IPv4 forwarding, and the
FORWARD/POSTROUTING firewall chains. For rslpc, use `ssh-rslpc` to verify
hostname and user, then inspect its route and test DNS and HTTPS separately.
Gateway ping alone is insufficient; an interface binding also does not select
between multiple gateways on that same interface.

Read receiver configuration and live RTK status:

```bash
GPS_TOOL="$HOME/.codex/skills/robot-ros/scripts/gps_gateway.py"
python3 "$GPS_TOOL" status
```

The helper talks directly to the receiver's HTTP interface, bypassing proxies.
It decodes the receiver's own SBF status blocks and prints no NTRIP credentials.

## 2. Make the laptop forward GPS traffic

When laptop sharing already works for rslpc, reuse its current uplink and
policy table. In the verified setup, table `19130` routes through the Pixel
hotspot and source rules select it for rslpc and GPS. Inspect before adding
rules; retain existing robot/Docker/Tailscale rules and avoid duplicates.

The GPS-specific runtime rules in that setup were:

```bash
# Add only missing rules after confirming the interface names and table.
sudo sysctl -w net.ipv4.ip_forward=1
sudo ip -4 rule add priority 5101 from 192.168.19.4/32 lookup 19130
sudo iptables -I FORWARD 1 -i enp0s31f6 -o wlp0s20f3 \
  -s 192.168.19.4/32 -m comment --comment gps-laptop-internet -j ACCEPT
sudo iptables -I FORWARD 1 -i wlp0s20f3 -o enp0s31f6 \
  -d 192.168.19.4/32 -m conntrack --ctstate RELATED,ESTABLISHED \
  -m comment --comment gps-laptop-internet -j ACCEPT
sudo iptables -t nat -I POSTROUTING 1 -s 192.168.19.4/32 \
  -o wlp0s20f3 -m comment --comment gps-laptop-internet -j MASQUERADE
```

If table `19130` is absent, create a source-policy table using the laptop's
**current** working uplink gateway and connected subnet. Do not assume the
hotspot's old address or overwrite a table belonging to another setup. Confirm
the actual forwarded route:

```bash
ip -4 route get 1.1.1.1 from 192.168.19.4 iif enp0s31f6
```

It must resolve through the tested uplink. Source-scoped rules ahead of the
FORWARD drop/Docker chains were necessary in the verified setup.

## 3. Change only the receiver gateway

```bash
python3 "$GPS_TOOL" set 192.168.19.88
```

The helper fetches the Ethernet form, backs it up under
`~/.local/state/gps-laptop-internet/`, posts the same fields with only the
gateway changed, then verifies every network field. An already-selected
gateway is a read-only no-op. If application or readback fails, inspect current
settings before retrying; an HTTP timeout can happen after a setting applied.

Receiver API verified on the installed firmware:

- Read: `/scr?cmd=1.60.5.0.0_1.60.11.0.0&fra0=ethernet.html`
- Write: form-encoded POST to `/upcmd`
- Gateway field: `1.60.11.4.0`; preserve the rest of the fetched form.
- Live status: `/sbf?returnImmediate=true` returns base64-encoded SBF.

The equivalent receiver web page is **Communication → Ethernet → Gateway →
OK**. **Save current configuration to boot configuration** is a separate
action; leave it unsaved for a temporary trial. Stop if authentication is
required; use authorized access rather than guessing credentials.

## 4. Verify correction recovery and ROS receipt

Sample `python3 "$GPS_TOOL" status` over 30–60 seconds. Require NTRIP
`Running` with no error, increasing accepted RTCMv3 bytes/messages, fresh
correction age, and PVT mode `4` for **RTK Fixed**. Mode `5` is RTK Float and
mode `6` is SBAS; report those accurately. A configured gateway or a receiver
Internet indicator alone does not establish a usable correction stream.

On rslpc, discover the current Moleworks ROS container with `docker ps`; do
not hard-code a transient container name. Source its actual workspace and use
the live DDS profile. This installation used `~/ros2_ws` and profiles under
`/tmp/rsl/fastdds_configs/`; an observer super-client profile is available for
diagnostics. Verify ROS CLI options against the installed version.
Run local observers as the running node's numeric UID/GID (inspect its
`/proc/<pid>/status`; `docker exec -u <uid>:<gid>` avoids container username
lookup issues). In the verified recovery, a root/runtime-client probe missed
the corrected topic, while a same-user/observer-profile probe received it.

```bash
# Inside the running container, with its ROS workspace and DDS profile sourced:
timeout 20 ros2 topic echo /hal/septentrio_gnss_driver/pvtgeodetic \
  septentrio_gnss_driver/msg/PVTGeodetic --once --qos-reliability best_effort
```

Use `mode & 15`, `error`, `nr_sv`, `mean_corr_age / 100` seconds, and
`h_accuracy / 100`, `v_accuracy / 100` metres. Check timestamp freshness.
The observed HAL driver can publish raw `NavSatFix.status = 0` even while PVT
reports RTK Fixed. If the estimator uses a GNSS status shim, inspect
`/mole/gnss/navsatfix_rtk_corrected` separately; receiver recovery does not by
itself prove estimator readiness. Do not restart the estimator just to change
the receiver's gateway.

Verified recovery on 2026-09-15: `.2 → .88` reconnected NTRIP and changed SBAS
to RTK Fixed, with roughly one-second-old corrections and receiver-reported
centimetre accuracy. rslpc received fresh PVT mode `4` samples. Treat these as
historical evidence, not current status. The corrected estimator-input topic
was also verified with `NavSatFix.status = 2` (`STATUS_GBAS_FIX`).

## Rollback and lifetime

Read `network-before.json` in the helper's printed backup directory, then set
its original gateway with the same helper (in the verified trial, `.2`). This
preserves any later changes to other fields instead of replaying an old form.
Remove only forwarding/routing rules introduced for the trial if requested;
keep rslpc's sharing intact. Successful laptop sharing requires the laptop,
wired link, and hotspot to remain connected. Runtime host rules can disappear
on restart, and the receiver gateway was not saved to boot.
