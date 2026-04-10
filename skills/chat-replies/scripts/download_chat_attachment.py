#!/usr/bin/env python3
"""Download the newest matching Google Chat attachment from a DM or space."""

from __future__ import annotations

import argparse
import json
import subprocess
import urllib.parse
import urllib.request
from datetime import datetime
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Download the newest matching Google Chat attachment."
    )
    parser.add_argument("--space", required=True, help="Chat space name, e.g. spaces/zGXYm8AAAAE")
    parser.add_argument(
        "--match",
        default="",
        help="Case-insensitive substring to match in attachment contentName",
    )
    parser.add_argument(
        "--page-size",
        type=int,
        default=10,
        help="How many recent messages to inspect",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path.home() / "Downloads",
        help="Directory where the attachment should be saved",
    )
    parser.add_argument(
        "--content-type-prefix",
        default="application/pdf",
        help="Require attachment contentType to start with this prefix; empty disables the filter",
    )
    return parser.parse_args()


def extract_json(stdout: str) -> dict:
    start = stdout.find("{")
    if start == -1:
        raise ValueError("gws output did not contain JSON")
    return json.loads(stdout[start:])


def run_messages_list(space: str, page_size: int) -> dict:
    params = json.dumps(
        {"parent": space, "pageSize": page_size, "orderBy": "create_time desc"},
        separators=(",", ":"),
    )
    proc = subprocess.run(
        [
            "gws",
            "chat",
            "spaces",
            "messages",
            "list",
            "--params",
            params,
            "--format",
            "json",
        ],
        check=True,
        text=True,
        capture_output=True,
    )
    return extract_json(proc.stdout)


def parse_time(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


def pick_attachment(payload: dict, match: str, content_type_prefix: str) -> dict:
    wanted = match.lower()
    candidates: list[dict] = []
    for message in payload.get("messages", []):
        create_time = message.get("createTime", "")
        for attachment in message.get("attachment", []):
            content_name = attachment.get("contentName", "")
            content_type = attachment.get("contentType", "")
            if wanted and wanted not in content_name.lower():
                continue
            if content_type_prefix and not content_type.startswith(content_type_prefix):
                continue
            resource_name = attachment.get("attachmentDataRef", {}).get("resourceName")
            if not resource_name:
                continue
            candidates.append(
                {
                    "create_time": create_time,
                    "message_name": message.get("name", ""),
                    "thread_name": message.get("thread", {}).get("name", ""),
                    "content_name": content_name,
                    "content_type": content_type,
                    "resource_name": resource_name,
                    "attachment_name": attachment.get("name", ""),
                }
            )

    if not candidates:
        raise FileNotFoundError("No matching attachment found in the fetched message window")
    candidates.sort(key=lambda item: parse_time(item["create_time"]), reverse=True)
    return candidates[0]


def get_access_token() -> str:
    creds = json.loads(
        subprocess.check_output(["gws", "auth", "export", "--unmasked"], text=True)
    )
    params = urllib.parse.urlencode(
        {
            "client_id": creds["client_id"],
            "client_secret": creds["client_secret"],
            "refresh_token": creds["refresh_token"],
            "grant_type": "refresh_token",
        }
    ).encode()
    request = urllib.request.Request(
        "https://oauth2.googleapis.com/token",
        data=params,
        headers={"Content-Type": "application/x-www-form-urlencoded"},
    )
    with urllib.request.urlopen(request) as response:
        return json.load(response)["access_token"]


def safe_filename(name: str) -> str:
    cleaned = name.strip().replace("/", "_")
    return cleaned or "chat_attachment.bin"


def download_attachment(resource_name: str, token: str, output_path: Path) -> None:
    url = (
        "https://chat.googleapis.com/v1/media/"
        + urllib.parse.quote(resource_name, safe="")
        + "?alt=media"
    )
    request = urllib.request.Request(url, headers={"Authorization": f"Bearer {token}"})
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(request) as response, output_path.open("wb") as fh:
        fh.write(response.read())


def main() -> int:
    args = parse_args()
    payload = run_messages_list(args.space, args.page_size)
    attachment = pick_attachment(payload, args.match, args.content_type_prefix)
    token = get_access_token()
    output_path = args.output_dir / safe_filename(attachment["content_name"])
    download_attachment(attachment["resource_name"], token, output_path)

    print(str(output_path))
    print(json.dumps(attachment, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
