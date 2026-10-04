#!/usr/bin/env bash
# snapshot.sh — capture shared-machine state that a session could alter.
# Read-only w.r.t. the system; writes only into $STATE_DIR.
# Usage: snapshot.sh [label]     (label defaults to "session")
set -u
STATE_DIR="${SHARED_PC_STATE_DIR:-$HOME/.local/state/shared-pc-inspect}"
LABEL="${1:-session}"
TS="$(date +%Y%m%d_%H%M%S)"
OUT="$STATE_DIR/snap_${TS}_${LABEL}"
mkdir -p "$OUT"

section() { echo "== $1 =="; }

# --- network / gateway ---
{ section "default-route"; ip route show default
  section "addresses";     ip -brief addr
  section "netplan-active"; sudo -n cat /etc/netplan/99-gravis_connection.yaml 2>/dev/null || cat /etc/netplan/99-gravis_connection.yaml 2>/dev/null
  section "netplan-checksums"; md5sum /etc/netplan/*.yaml 2>/dev/null
} > "$OUT/network.txt" 2>&1

# --- docker ---
{ section "containers"; docker ps -a --format '{{.Names}}\t{{.Image}}\t{{.Status}}'
  section "restart-policies"; docker ps -aq | xargs -r docker inspect --format '{{.Name}} restart={{.HostConfig.RestartPolicy.Name}} running={{.State.Running}}'
  section "images"; docker images --format '{{.Repository}}:{{.Tag}}\t{{.ID}}\t{{.Size}}'
  section "volumes"; docker volume ls --format '{{.Name}}'
} > "$OUT/docker.txt" 2>&1

# --- systemd / autostart ---
{ section "enabled-units"; systemctl list-unit-files --state=enabled --no-legend --no-pager
  section "failed-units";  systemctl --failed --no-legend --no-pager
  section "robot-units"; for u in can.service ptp4l_client.service m4-maintenance-proxy.service m4-maintenance-proxy.socket enp6s0-boot-recover.service enp6s0-eee-off.service force-performance-profile.service openvpn.service docker.service ptp4l.service ptp-master.service livox-ptp-master.service phc2sys.service; do printf '%s: %s / %s\n' "$u" "$(systemctl is-enabled "$u" 2>&1)" "$(systemctl is-active "$u" 2>&1)"; done
  section "power-profile"; powerprofilesctl get 2>/dev/null
} > "$OUT/systemd.txt" 2>&1

# --- shared /etc surfaces ---
{ section "etc-checksums"; md5sum /etc/hosts /etc/environment /etc/udev/rules.d/99-slabs.rules 2>/dev/null
  section "groups"; getent group docker sudo dialout video
  section "cron.d"; ls -la /etc/cron.d/ 2>/dev/null
  section "profile.d"; ls -la /etc/profile.d/ 2>/dev/null
  section "sudoers"; sudo -n md5sum /etc/sudoers 2>/dev/null; sudo -n visudo -c 2>&1 | tail -2; sudo -n ls /etc/sudoers.d/ 2>/dev/null
} > "$OUT/etc.txt" 2>&1

# --- CAN ---
{ for i in can0 can1 can2 can3 can4; do ip -details link show "$i" 2>/dev/null | head -4; done
} > "$OUT/can.txt" 2>&1

# --- host toolchain (CUDA etc.) ---
{ section "cuda-links"; ls -l /usr/local/ | grep -i cuda
  section "nvidia"; nvidia-smi --query-gpu=driver_version,name --format=csv,noheader 2>/dev/null
  section "apt-recent"; grep -hE '^(Start-Date|Commandline)' /var/log/apt/history.log 2>/dev/null | tail -20
} > "$OUT/host_toolchain.txt" 2>&1

echo "$OUT"
