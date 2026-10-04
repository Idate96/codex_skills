#!/usr/bin/env bash
# health.sh — read-only machine health report for rslpc.
# Prints PASS/WARN/FAIL lines; exit code 1 if any FAIL.
set -u
RC=0
warn() { echo "WARN  $*"; }
fail() { echo "FAIL  $*"; RC=1; }
pass() { echo "PASS  $*"; }

# --- gateway / network ---
GW="$(ip route | awk '/^default/ {print $3; exit}')"
if [ -n "$GW" ]; then
  if ping -c1 -W1 "$GW" >/dev/null 2>&1; then pass "gateway $GW reachable"; else fail "gateway $GW NOT reachable"; fi
  echo "INFO  default route via $GW (.2 = Gravis 4G uplink, .110 = Trailer Router uplink; robot gravis_pc is .1, on-link, unaffected by gateway choice)"
else
  fail "no default route"
fi
# The Aug-2026 netplan incident was a corrupted addresses: field, not the gateway.
ADDR="$(ip -brief addr show enp6s0 | awk '{print $3}')"
[ "$ADDR" = "192.168.19.30/24" ] && pass "enp6s0 address is 192.168.19.30/24" || fail "enp6s0 address is '$ADDR' (must be exactly 192.168.19.30/24 -- check /etc/netplan/99-gravis_connection.yaml addresses: field)"

# --- disk ---
USEP="$(df --output=pcent / | tail -1 | tr -dc '0-9')"
AVAIL="$(df -h --output=avail / | tail -1 | tr -d ' ')"
if [ "$USEP" -ge 95 ]; then fail "root disk ${USEP}% full (${AVAIL} free)"
elif [ "$USEP" -ge 85 ]; then warn "root disk ${USEP}% full (${AVAIL} free) — consider clearing bags/mcap/docker images"
else pass "root disk ${USEP}% used (${AVAIL} free)"; fi

# --- thermals ---
TCTL="$(sensors 2>/dev/null | awk '/^Tctl/ {gsub(/[+°C]/,"",$2); print int($2)}')"
MINF="$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq 2>/dev/null || echo 0)"
if [ -n "${TCTL:-}" ]; then
  # Zen4 Tjmax is 95C by design -- temp alone is not diagnostic; pair with cause.
  if [ "$TCTL" -ge 95 ]; then
    if [ "$MINF" -ge 2000000 ]; then
      warn "CPU Tctl ${TCTL}C (at Tjmax). Known cause on rslpc: force-performance-profile.service pins min freq to $((MINF/1000))MHz on all threads (~90W at idle). No thermal headroom for heavy workloads; cooling may also be degraded (fan RPM unreadable -- needs BIOS/physical check)."
    else
      fail "CPU Tctl ${TCTL}C with min freq NOT pinned -- unexpected; check top and cooling"
    fi
  elif [ "$TCTL" -ge 85 ]; then warn "CPU Tctl ${TCTL}C"
  else pass "CPU Tctl ${TCTL}C"; fi
fi
NVMET="$(sensors 2>/dev/null | awk '/^Composite/ {gsub(/[+°C]/,"",$2); print int($2); exit}')"
[ -n "${NVMET:-}" ] && { [ "$NVMET" -ge 80 ] && warn "NVMe ${NVMET}C" || pass "NVMe ${NVMET}C"; }
command -v smartctl >/dev/null 2>&1 || command -v nvme >/dev/null 2>&1 || echo "INFO  NVMe wear unmeasurable: neither smartmontools nor nvme-cli installed (worth installing on a 4TB drive at 90%)"
NVT="$(nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader 2>/dev/null | head -1)"
[ -n "${NVT:-}" ] && { [ "$NVT" -ge 85 ] && warn "GPU ${NVT}C" || pass "GPU ${NVT}C"; }

# --- memory ---
AVAILMEM="$(free -g | awk '/^Mem:/ {print $7}')"
[ "$AVAILMEM" -lt 4 ] && warn "only ${AVAILMEM}G RAM available" || pass "${AVAILMEM}G RAM available"

# --- systemd ---
NFAILED="$(systemctl --failed --no-legend | wc -l)"
[ "$NFAILED" -gt 0 ] && { warn "$NFAILED failed systemd unit(s):"; systemctl --failed --no-legend --no-pager | sed 's/^/      /'; } || pass "no failed systemd units"

# --- docker restart policies (the classic shared-PC trap) ---
BAD="$(docker ps -aq 2>/dev/null | xargs -r docker inspect --format '{{.Name}} {{.HostConfig.RestartPolicy.Name}}' | awk '$2!="no" && $2!="" {print $1}' )"
[ -n "$BAD" ] && fail "containers with auto-restart policy (will resurrect on boot): $BAD" || pass "no containers with auto-restart policy"
RUNNING="$(docker ps --format '{{.Names}}' 2>/dev/null | tr '\n' ' ')"
[ -n "$RUNNING" ] && echo "INFO  running containers: $RUNNING(stop yours before handover)"


# --- docker disk + logs ---
RECL="$(docker system df --format '{{.Type}} {{.Reclaimable}}' 2>/dev/null | awk '/^Images/ {print $2}')"
[ -n "$RECL" ] && echo "INFO  docker images reclaimable: $RECL (18+ dangling images have accumulated before; docker image prune only with user confirmation)"
BIGLOG="$(sudo -n du -sh /var/lib/docker/containers/*/*-json.log 2>/dev/null | sort -rh | head -1)"
[ -n "$BIGLOG" ] && case "$BIGLOG" in
  *G[[:space:]]*) warn "container json log >1G: $BIGLOG (no log rotation configured in daemon.json)";;
  *) echo "INFO  largest container log: $BIGLOG";;
esac

# --- ROS domain 0 collision surface ---
if [ -n "$RUNNING" ]; then
  ss -tlnp 2>/dev/null | grep -q ':8765 ' && echo "INFO  port 8765 (foxglove_bridge) bound on host — a net=host container is on ROS_DOMAIN_ID 0; other users starting ROS at domain 0 will join this graph"
fi

# --- kernel errors last 24h ---
ERRS="$(journalctl -k -p err --since "24 hours ago" --no-pager 2>/dev/null | grep -vciE 'acpi|usb .*device descriptor' || true)"
[ "${ERRS:-0}" -gt 20 ] && warn "$ERRS kernel errors in last 24h — journalctl -k -p err --since '24 hours ago'" || pass "kernel error volume normal (${ERRS:-0} in 24h)"

# --- GPU Xid errors (anchor on "NVRM: Xid" -- plain "xid" matches Realtek NIC chip rev) ---
XID="$(journalctl -k --since "7 days ago" --no-pager 2>/dev/null | grep -c "NVRM: Xid" || true)"
[ "${XID:-0}" -gt 0 ] && fail "$XID NVIDIA Xid error(s) in 7 days -- journalctl -k | grep 'NVRM: Xid'" || pass "no NVIDIA Xid errors (7d)"

# --- time sync (ptp for robot) ---
# ptp4l_client (slaved to gravis_pc .1) is the ONLY time source: no chrony/timesyncd
# fallback exists. Stopping it lets the clock free-run at ~-24000 ppb.
ST="$(systemctl is-active ptp4l_client.service 2>&1)"
[ "$ST" = "active" ] && pass "ptp4l_client active (sole time source -- never leave it stopped)" || fail "ptp4l_client: $ST -- clock is free-running, no NTP fallback configured"
for u in ptp4l.service ptp-master.service livox-ptp-master.service phc2sys.service; do
  EN="$(systemctl is-enabled "$u" 2>&1)"
  [ "$EN" = "disabled" ] || warn "dormant PTP unit $u is '$EN' (must stay disabled or it fights ptp4l_client)"
done

# --- power profile (GNOME 'Balanced' click silently undoes force-performance-profile until reboot) ---
PP="$(powerprofilesctl get 2>/dev/null)"
[ -z "$PP" ] || [ "$PP" = "performance" ] && pass "power profile: ${PP:-n/a}" || warn "power profile is '$PP' -- force-performance-profile.service sets performance at boot and never re-asserts"

# --- CAN buses ---
for i in can2 can3 can4; do
  OPER="$(cat /sys/class/net/$i/operstate 2>/dev/null)"
  [ "$OPER" = "up" ] || [ "$OPER" = "unknown" ] && pass "$i up" || warn "$i state: ${OPER:-missing} (can4 missing often means the Lawicel adapter serial doesn't match the one hardcoded in /usr/sbin/can-start)"
done

# --- reboot pending ---
[ -f /var/run/reboot-required ] && warn "reboot required flag set (unattended upgrades)" || true

exit $RC
