#!/usr/bin/env python3
"""Read Menzi Septentrio RTK status or change its current IPv4 gateway."""

import argparse
import base64
import binascii
from datetime import datetime
from html.parser import HTMLParser
from ipaddress import IPv4Address
import json
from pathlib import Path
import struct
import urllib.parse
import urllib.request


ETHERNET = "/scr?cmd=1.60.5.0.0_1.60.11.0.0&fra0=ethernet.html"
GATEWAY = "1.60.11.4.0"
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def request(base, path, data=None):
    body = None if data is None else urllib.parse.urlencode(data).encode()
    with OPENER.open(base + path, data=body, timeout=12) as response:
        return response.read()


class EthernetForm(HTMLParser):
    def __init__(self):
        super().__init__()
        self.inside = False
        self.values = {}

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == "form" and attrs.get("action") == "/upcmd":
            self.inside = True
        if not self.inside or tag != "input" or "name" not in attrs:
            return
        kind = attrs.get("type", "text")
        if kind in {"button", "submit"} or "disabled" in attrs:
            return
        if kind in {"radio", "checkbox"} and "checked" not in attrs:
            return
        self.values[attrs["name"]] = attrs.get("value", "")

    def handle_endtag(self, tag):
        if tag == "form":
            self.inside = False


def network(base):
    html = request(base, ETHERNET).decode()
    form = EthernetForm()
    form.feed(html)
    expected = {"uri", "1.60.5.1.0"} | {f"1.60.11.{i}.0" for i in range(1, 9)}
    if set(form.values) != expected:
        raise RuntimeError("Ethernet form differs from verified firmware; inspect it before writing")
    return form.values, html


def set_gateway(base, gateway):
    gateway = str(IPv4Address(gateway))
    old, html = network(base)
    if old[GATEWAY] == gateway:
        return {"gateway": gateway, "changed": False}
    if old["1.60.11.1.0"] != "2":
        raise RuntimeError("Expected static addressing; inspect DHCP configuration before changing it")
    backup = (Path.home() / ".local/state/gps-laptop-internet" /
              datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
    backup.mkdir(parents=True, mode=0o700)
    (backup / "network-before.json").write_text(json.dumps(old, indent=2) + "\n")
    (backup / "ethernet-before.html").write_text(html)
    print(json.dumps({"backup": str(backup), "previous_gateway": old[GATEWAY]}), flush=True)
    new = dict(old)
    new[GATEWAY] = gateway
    response = request(base, "/upcmd", new)
    (backup / "apply-response.html").write_bytes(response)
    actual, html = network(base)
    (backup / "ethernet-after.html").write_text(html)
    if actual != new:
        raise RuntimeError(f"Network readback mismatch; inspect current settings and backup {backup}")
    return {"gateway": gateway, "previous_gateway": old[GATEWAY], "changed": True,
            "other_fields_unchanged": True, "saved_to_boot": False, "backup": str(backup)}


def receiver_status(base):
    encoded = b"".join(request(base, "/sbf?returnImmediate=true").split())
    data = base64.b64decode(encoded, validate=True)
    result = {}
    offset = 0
    while offset < len(data):
        if data[offset:offset + 2] != b"$@" or len(data) - offset < 8:
            raise RuntimeError("Malformed SBF status stream")
        crc, block_id, size = struct.unpack_from("<3H", data, offset + 2)
        block = data[offset:offset + size]
        if size < 8 or len(block) != size or binascii.crc_hqx(block[4:], 0) != crc:
            raise RuntimeError("Invalid SBF block length or checksum")
        block_id &= 8191
        if block_id == 4007:
            mode = block[14] & 15
            result["pvt"] = {
                "mode": mode, "mode_name": {4: "RTK Fixed", 5: "RTK Float", 6: "SBAS"}.get(mode, "Other"),
                "error": block[15], "satellites": block[74],
                "correction_age_s": struct.unpack_from("<H", block, 78)[0] / 100,
                "h_accuracy_m": struct.unpack_from("<H", block, 90)[0] / 100,
                "v_accuracy_m": struct.unpack_from("<H", block, 92)[0] / 100,
            }
        elif block_id == 4053:
            result["ntrip"] = []
            for index in range(block[14]):
                connection, status, error, info = struct.unpack_from("4B", block, 16 + index * block[15])
                result["ntrip"].append({"connection": connection, "status": status,
                    "status_name": {0: "Disabled", 1: "Initializing", 2: "Running", 3: "Error", 4: "Retrying", 5: "Duplicate"}.get(status, "Unknown"),
                    "error": error})
        elif block_id == 4090:
            result["correction_inputs"] = []
            for index in range(block[14]):
                values = struct.unpack_from("<BBH4I", block, 16 + index * block[15])
                if values[1] == 97:
                    result["correction_inputs"].append(dict(zip(
                        ["port", "type", "age_raw", "bytes_received", "bytes_accepted",
                         "messages_received", "messages_accepted"], values)))
        offset += size
    if "pvt" not in result or "ntrip" not in result:
        raise RuntimeError("Receiver response lacks PVT or NTRIP status; retry a fresh sample")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receiver", default="192.168.19.4")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("status", help="Read network configuration and live RTK status")
    change = commands.add_parser("set", help="Change current gateway, without saving to boot")
    change.add_argument("gateway", type=IPv4Address)
    args = parser.parse_args()
    base = "http://" + args.receiver
    if args.command == "set":
        result = set_gateway(base, str(args.gateway))
    else:
        settings, _ = network(base)
        result = {"checked_at": datetime.now().astimezone().isoformat(),
                  "receiver": settings["1.60.11.2.0"], "gateway": settings[GATEWAY],
                  "dns": [settings["1.60.11.6.0"], settings["1.60.11.7.0"]],
                  **receiver_status(base)}
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
