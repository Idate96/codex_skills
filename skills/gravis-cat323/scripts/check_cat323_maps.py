#!/usr/bin/env python3
"""Measure advancing source stamps on the local canonical map, without native-map subscriptions."""

import argparse
import json
import time


def summarize(rows, seconds):
    advancing = []
    regressions = 0
    for row in rows:
        if not advancing or row["stamp_ns"] > advancing[-1]["stamp_ns"]:
            advancing.append(row)
        elif row["stamp_ns"] < advancing[-1]["stamp_ns"]:
            regressions += 1
    ages = [r["age_s"] for r in rows]
    gaps = [b["receipt_s"] - a["receipt_s"] for a, b in zip(advancing, advancing[1:])]
    invalid = sum(r["stamp_ns"] <= 0 or not 0 <= r["age_s"] < 3 for r in rows)
    passed = len(advancing) >= int(seconds * 0.8) and invalid == 0 and regressions == 0 and bool(gaps) and max(gaps) <= 2.5
    return {"passed": passed, "duration_s": seconds, "messages": len(rows), "advancing_source_stamps": len(advancing), "invalid_or_stale_publications": invalid, "regressing_source_stamps": regressions, "max_published_source_age_s": max(ages, default=None), "max_advance_gap_s": max(gaps, default=None), "native_map_subscriptions_added": 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=int, default=30)
    args = parser.parse_args()
    if not 10 <= args.seconds <= 60:
        parser.error("--seconds must be between 10 and 60")
    import rclpy
    from grid_map_msgs.msg import GridMap
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node("cat323_map_transport_check")
    rows = []

    def receive(msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        rows.append({"stamp_ns": stamp, "receipt_s": time.monotonic(), "age_s": (time.time_ns() - stamp) / 1e9})

    node.create_subscription(GridMap, "/excavation_mapping/grid_map", receive, qos_profile_sensor_data)
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    report = summarize(rows, args.seconds)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["passed"] else 2)


if __name__ == "__main__":
    main()
