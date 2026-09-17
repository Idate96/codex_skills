#!/usr/bin/env python3
"""Check recorded H265 parameter sets and an IDR before a CAT323 action; no subscriptions."""
import argparse
import json
from pathlib import Path
import re
import struct


def nal_types(data):
    return {(part[0] >> 1) & 63 for part in re.split(b"\x00\x00(?:\x00)?\x01", data)[1:] if part}


def inspect_bag(directory, before_ns=None):
    from foxglove_msgs.msg import CompressedVideo
    from mcap.exceptions import EndOfFile
    from mcap.records import Channel, Message
    from mcap.stream_reader import StreamReader
    from rclpy.serialization import deserialize_message

    parameter_sets = set()
    count = 0
    first_keyframe = None
    latest_source = None
    complete = True
    for path in sorted(directory.glob("*.mcap")):
        channels = {}
        try:
            with path.open("rb") as stream:
                for record in StreamReader(stream).records:
                    if isinstance(record, Channel):
                        channels[record.id] = record.topic
                    elif isinstance(record, Message) and channels.get(record.channel_id) == "/hal/perception/main/compressed_video":
                        msg = deserialize_message(record.data, CompressedVideo)
                        if msg.format.lower() not in {"h265", "hevc"}:
                            raise ValueError(f"Expected H265, got {msg.format!r}")
                        count += 1
                        latest_source = msg.timestamp.sec * 1_000_000_000 + msg.timestamp.nanosec
                        kinds = nal_types(bytes(msg.data))
                        parameter_sets.update(kinds & {32, 33, 34})
                        if first_keyframe is None and {32, 33, 34} <= parameter_sets and kinds & {19, 20}:
                            first_keyframe = {"source_ns": latest_source, "recorded_ns": record.log_time}
        except (EndOfFile, struct.error):
            # An active MCAP can end in an unfinished chunk. Require its flushed data.
            complete = False
    passed = first_keyframe is not None and (before_ns is None or first_keyframe["recorded_ns"] <= before_ns)
    return {"passed": passed, "video_messages": count, "first_complete_idr": first_keyframe,
            "latest_source_ns": latest_source, "before_ns": before_ns, "all_files_complete": complete,
            "scope": "Parameter sets plus IDR admission only; does not certify lossless video or ongoing freshness."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-dir", type=Path, required=True)
    parser.add_argument("--before-unix-s", type=float, help="Require keyframe recorded before this action start")
    args = parser.parse_args()
    result = inspect_bag(args.bag_dir, None if args.before_unix_s is None else int(args.before_unix_s * 1e9))
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
