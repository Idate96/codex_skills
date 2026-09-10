#!/usr/bin/env python3
"""
Report effective joint velocity/acceleration limits used during tuning.

This combines:
1) Diagnostic limit columns from the latest benchmark CSV row.
2) Controller runtime parameters (`command.max_velocity`, `command.max_accel`,
   `command.accel_scale`) from the active controller node.

Fail fast on missing files/columns/params to avoid tuning with ambiguous state.
"""

from __future__ import annotations

import argparse
import csv
import json
import pathlib
import re
import subprocess
from dataclasses import dataclass
from typing import Dict, List


JOINTS = [
    ("j_turn", "J_TURN"),
    ("j_boom", "J_BOOM"),
    ("j_stick", "J_STICK"),
    ("j_tele", "J_TELE"),
    ("j_ee_pitch", "J_EE_PITCH"),
]


@dataclass
class JointSnapshot:
    vel_limit_csv: float
    accel_limit_csv: float
    vel_ratio: float
    accel_ratio: float


def _read_latest_row(csv_path: pathlib.Path) -> Dict[str, str]:
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")
    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        last = None
        for row in reader:
            last = row
    if last is None:
        raise RuntimeError(f"CSV has no data rows: {csv_path}")
    return last


def _f(row: Dict[str, str], key: str) -> float:
    if key not in row:
        raise KeyError(f"Missing CSV column: {key}")
    raw = row[key].strip()
    if raw == "":
        raise RuntimeError(f"CSV column is empty: {key}")
    return float(raw)


def _ros2_param_get(workspace: pathlib.Path, node: str, param: str) -> str:
    cmd = (
        f"source '{workspace}/install/setup.bash' && "
        f"ros2 param get '{node}' '{param}'"
    )
    proc = subprocess.run(
        ["bash", "-lc", cmd],
        capture_output=True,
        text=True,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f"ros2 param get failed for {node}:{param}\nstdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )
    out = f"{proc.stdout}\n{proc.stderr}"
    value_lines = [
        ln
        for ln in out.splitlines()
        if re.search(r"\bvalue(s)?\s+(is|are):", ln, flags=re.IGNORECASE)
    ]
    if not value_lines:
        raise RuntimeError(f"Could not parse value for {node}:{param}\noutput:\n{out}")
    line = value_lines[-1]
    m = re.search(r"\bvalue(s)?\s+(is|are):\s*(.*)$", line, flags=re.IGNORECASE)
    if m is None:
        raise RuntimeError(f"Could not extract value payload for {node}:{param}\nline:\n{line}")
    return m.group(3).strip()


def _parse_float_list(value_text: str) -> List[float]:
    nums = re.findall(r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?", value_text)
    if not nums:
        raise RuntimeError(f"Could not parse numeric list from: {value_text}")
    return [float(x) for x in nums]


def _parse_scalar_float(value_text: str) -> float:
    nums = _parse_float_list(value_text)
    return float(nums[0])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="", help="Benchmark CSV path. If empty, use --active-csv-file.")
    ap.add_argument("--active-csv-file", default="~/mpc_tuning/current/.active_csv")
    ap.add_argument("--workspace", default="~/ros2_ws")
    ap.add_argument("--controller-node", default="/mole/mole_arm_mpc_controller")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    workspace = pathlib.Path(args.workspace).expanduser().resolve()
    if not (workspace / "install" / "setup.bash").exists():
        raise FileNotFoundError(f"Workspace setup not found: {workspace}/install/setup.bash")

    if args.csv.strip() != "":
        csv_path = pathlib.Path(args.csv).expanduser().resolve()
    else:
        active_csv_path = pathlib.Path(args.active_csv_file).expanduser().resolve()
        if not active_csv_path.exists():
            raise FileNotFoundError(f"Active CSV pointer not found: {active_csv_path}")
        csv_path = pathlib.Path(active_csv_path.read_text(encoding="utf-8").strip()).expanduser().resolve()

    latest = _read_latest_row(csv_path)

    by_joint: Dict[str, JointSnapshot] = {}
    for slug, jname in JOINTS:
        by_joint[jname] = JointSnapshot(
            vel_limit_csv=_f(latest, f"diag_cmd_vel_limit_{slug}_radps"),
            accel_limit_csv=_f(latest, f"diag_cmd_accel_limit_{slug}_radps2"),
            vel_ratio=_f(latest, f"diag_cmd_vel_ratio_{slug}"),
            accel_ratio=_f(latest, f"diag_cmd_accel_ratio_{slug}"),
        )

    max_velocity = _parse_float_list(
        _ros2_param_get(workspace, args.controller_node, "command.max_velocity")
    )
    max_accel = _parse_float_list(
        _ros2_param_get(workspace, args.controller_node, "command.max_accel")
    )
    accel_scale = _parse_scalar_float(
        _ros2_param_get(workspace, args.controller_node, "command.accel_scale")
    )

    if len(max_velocity) != len(JOINTS):
        raise RuntimeError(f"command.max_velocity size mismatch: got {len(max_velocity)}, expected {len(JOINTS)}")
    if len(max_accel) != len(JOINTS):
        raise RuntimeError(f"command.max_accel size mismatch: got {len(max_accel)}, expected {len(JOINTS)}")

    comparison = {}
    tol = 1e-6
    for idx, (_, jname) in enumerate(JOINTS):
        snap = by_joint[jname]
        vel_diff = abs(snap.vel_limit_csv - max_velocity[idx])
        accel_diff = abs(snap.accel_limit_csv - max_accel[idx])
        comparison[jname] = {
            "vel_limit_param_radps": max_velocity[idx],
            "vel_limit_csv_radps": snap.vel_limit_csv,
            "vel_abs_diff": vel_diff,
            "vel_match": vel_diff <= tol,
            "accel_limit_param_radps2": max_accel[idx],
            "accel_limit_csv_radps2": snap.accel_limit_csv,
            "accel_abs_diff": accel_diff,
            "accel_match": accel_diff <= tol,
            "vel_ratio_latest": snap.vel_ratio,
            "accel_ratio_latest": snap.accel_ratio,
        }

    worst_vel = max(((v.vel_ratio, j) for j, v in by_joint.items()), key=lambda x: x[0])
    worst_accel = max(((v.accel_ratio, j) for j, v in by_joint.items()), key=lambda x: x[0])

    out = {
        "csv_path": str(csv_path),
        "latest_row": {
            "t_sec": _f(latest, "t_sec"),
            "seg_id": int(_f(latest, "seg_id")),
        },
        "controller_node": args.controller_node,
        "controller_params": {
            "command.max_velocity": max_velocity,
            "command.max_accel": max_accel,
            "command.accel_scale": accel_scale,
        },
        "joint_comparison": comparison,
        "worst_ratio_latest": {
            "cmd_vel_ratio": {"joint": worst_vel[1], "value": worst_vel[0]},
            "cmd_accel_ratio": {"joint": worst_accel[1], "value": worst_accel[0]},
        },
    }

    if args.json:
        print(json.dumps(out, indent=2, sort_keys=True))
        return

    print(f"csv: {out['csv_path']}")
    print(f"latest: seg={out['latest_row']['seg_id']} t_sec={out['latest_row']['t_sec']:.3f}")
    print(
        "controller params: "
        f"max_velocity={max_velocity} max_accel={max_accel} accel_scale={accel_scale:.3f}"
    )
    print(
        "worst latest ratios: "
        f"vel={worst_vel[0]:.3f} ({worst_vel[1]}), "
        f"accel={worst_accel[0]:.3f} ({worst_accel[1]})"
    )
    for _, jname in JOINTS:
        c = comparison[jname]
        print(
            f"{jname}: vel_limit csv/param={c['vel_limit_csv_radps']:.3f}/{c['vel_limit_param_radps']:.3f} "
            f"match={c['vel_match']} | "
            f"accel_limit csv/param={c['accel_limit_csv_radps2']:.3f}/{c['accel_limit_param_radps2']:.3f} "
            f"match={c['accel_match']} | "
            f"ratios vel/accel={c['vel_ratio_latest']:.3f}/{c['accel_ratio_latest']:.3f}"
        )


if __name__ == "__main__":
    main()
