#!/usr/bin/env python3
"""Scoped inspection/restore request, streamed to Python inside Orin's AMG container."""

import hashlib
import json
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET


EXE = b"/amg/install/lib/autonomy_visualization/gridmap_selected_layers_republisher_node"
NODE = b"__node:=grid_map_selected_layers_interface_republisher_node"
ENV_KEYS = ("FASTRTPS_DEFAULT_PROFILES_FILE", "RMW_FASTRTPS_USE_QOS_FROM_XML", "CAT323_SELECTED_MAP_DDS_PROFILE")


def argv_at(proc):
    return proc.joinpath("cmdline").read_bytes().rstrip(b"\0").split(b"\0")


def status(proc_root=Path("/proc")):
    publishers = []
    probes = []
    for proc in proc_root.iterdir():
        if not proc.name.isdigit():
            continue
        try:
            argv = argv_at(proc)
        except OSError:
            continue
        if b"__node:=cat323_transport_probe" in argv or any(a.endswith(b"/run_probe_orin.py") for a in argv):
            probes.append(int(proc.name))
        if not argv or argv[0] != EXE or NODE not in argv:
            continue
        env = {}
        for entry in proc.joinpath("environ").read_bytes().split(b"\0"):
            key, separator, value = entry.partition(b"=")
            if separator and os.fsdecode(key) in ENV_KEYS:
                env[os.fsdecode(key)] = os.fsdecode(value)
        profile = Path(env.get("FASTRTPS_DEFAULT_PROFILES_FILE", ""))
        entry = {"pid": int(proc.name), "environment": env, "temporary_supervisor": None}
        if profile.is_file():
            data = profile.read_bytes()
            entry["profile_sha256"] = hashlib.sha256(data).hexdigest()
            xml = ET.fromstring(data)
            ns = {"f": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
            properties = {n.findtext("f:name", namespaces=ns): n.findtext("f:value", namespaces=ns) for n in xml.findall(".//f:property", ns)}
            entry["outgoing_message_size"] = properties.get("fastdds.max_message_size")
            entry["flow_bytes_per_period"] = xml.findtext(".//f:flow_controller_descriptor/f:max_bytes_per_period", namespaces=ns)
            entry["flow_period_ms"] = xml.findtext(".//f:flow_controller_descriptor/f:period_ms", namespaces=ns)
            entry["publish_mode"] = xml.findtext(".//f:data_writer/f:qos/f:publishMode/f:kind", namespaces=ns)
            if profile.parent.parent == Path("/tmp") and profile.parent.name.startswith("cat323-map-transport."):
                state_file = profile.parent / "replacement-state.json"
                if state_file.is_file():
                    state = json.loads(state_file.read_text())
                    supervisor = proc_root / str(state["supervisor_pid"])
                    try:
                        supervisor_args = argv_at(supervisor)
                    except OSError:
                        supervisor_args = []
                    script = os.fsencode(profile.parent / "replace_publisher.py")
                    entry["temporary_supervisor"] = {
                        "root": str(profile.parent),
                        "pid": state["supervisor_pid"],
                        "phase": state["phase"],
                        "replacement_pid": state.get("replacement_pid"),
                        "identity_matches": script in supervisor_args,
                    }
        publishers.append(entry)
    return {"selected_publisher_count": len(publishers), "publishers": publishers, "diagnostic_probe_pids": probes}


def restore(report):
    if report["selected_publisher_count"] != 1:
        raise RuntimeError("Require exactly one selected-map publisher before requesting restore")
    publisher = report["publishers"][0]
    supervisor = publisher["temporary_supervisor"]
    if not supervisor or not supervisor["identity_matches"]:
        raise RuntimeError("No verified temporary supervisor owns this publisher; no changes made")
    if supervisor["phase"] != "temporary_replacement_running" or supervisor["replacement_pid"] != publisher["pid"]:
        raise RuntimeError("Temporary state does not match the live publisher; no changes made")
    root = Path(supervisor["root"])
    root.joinpath("restore").touch()
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        state = json.loads(root.joinpath("replacement-state.json").read_text())
        if state["phase"] == "restored_original":
            current = status()
            if current["selected_publisher_count"] != 1 or current["publishers"][0]["temporary_supervisor"]:
                raise RuntimeError("Supervisor reported restore but original publisher is not uniquely running; inspect state")
            restored = current["publishers"][0]
            if restored["pid"] != state.get("restored_pid") or restored["environment"].get("FASTRTPS_DEFAULT_PROFILES_FILE") != state["original_profile"]:
                raise RuntimeError("Restored publisher identity/profile does not match the supervisor's original state")
            return {"restore_phase": state["phase"], **current}
        time.sleep(0.2)
    raise RuntimeError(f"Restore requested but not confirmed within 20 s; inspect {root}/replacement-state.json")


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("status", "restore"):
        raise SystemExit("Expected status or restore")
    report = status()
    if sys.argv[1] == "restore":
        report = restore(report)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
