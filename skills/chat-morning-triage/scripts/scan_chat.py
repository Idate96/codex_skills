#!/usr/bin/env python3

import argparse
import datetime as dt
import json
import pathlib
import re
import subprocess
import sys
from typing import Any


COLLABORATORS_MD = pathlib.Path(
    "/home/lorenzo/codex_skills/skills/chat-replies/references/collaborators.md"
)


def run_gws(args: list[str]) -> dict[str, Any]:
    proc = subprocess.run(
        ["gws", *args],
        check=True,
        capture_output=True,
        text=True,
    )
    stdout = proc.stdout
    start = stdout.find("{")
    if start == -1:
        raise RuntimeError(f"gws did not return JSON for args={args!r}: {stdout}")
    return json.loads(stdout[start:])


def parse_dt(value: str | None) -> dt.datetime | None:
    if not value:
        return None
    return dt.datetime.fromisoformat(value.replace("Z", "+00:00"))


def parse_collaborators(path: pathlib.Path) -> dict[str, dict[str, str]]:
    mapping: dict[str, dict[str, str]] = {}
    current_name: str | None = None
    current_entry: dict[str, str] | None = None

    for raw_line in path.read_text().splitlines():
        line = raw_line.rstrip()
        top_match = re.match(r"^- ([^-].+)$", line)
        nested_match = re.match(r"^  - ([^:]+): `(.*)`$", line)
        nested_plain = re.match(r"^  - ([^:]+): (.+)$", line)

        if top_match and not raw_line.startswith("  "):
            current_name = top_match.group(1).strip()
            current_entry = {"name": current_name}
            continue

        if current_entry is None:
            continue

        if nested_match:
            key = nested_match.group(1).strip().lower()
            value = nested_match.group(2).strip()
            current_entry[key] = value
        elif nested_plain:
            key = nested_plain.group(1).strip().lower()
            value = nested_plain.group(2).strip()
            current_entry[key] = value

        dm = current_entry.get("google chat dm")
        if dm:
            mapping[dm] = dict(current_entry)

    return mapping


def normalize_message(message: dict[str, Any]) -> dict[str, Any]:
    text = message.get("argumentText") or message.get("text") or ""
    attachments = []
    for attachment in message.get("attachment", []):
        attachments.append(
            {
                "contentName": attachment.get("contentName"),
                "contentType": attachment.get("contentType"),
            }
        )
    if not text.strip() and attachments:
        names = [a["contentName"] for a in attachments if a.get("contentName")]
        if names:
            text = "[attachment] " + ", ".join(names)
        else:
            text = "[attachment]"
    return {
        "createTime": message.get("createTime"),
        "sender": (message.get("sender") or {}).get("name"),
        "thread": (message.get("thread") or {}).get("name"),
        "text": text.strip(),
        "attachments": attachments,
    }


def should_include_space(space: dict[str, Any], cutoff: dt.datetime, known_dms: set[str]) -> bool:
    name = space.get("name")
    last_active = parse_dt(space.get("lastActiveTime"))
    if last_active is None or last_active < cutoff:
        return False

    if name in known_dms:
        return True

    if space.get("spaceType") != "SPACE":
        return False

    count = ((space.get("membershipCount") or {}).get("joinedDirectHumanUserCount")) or 0
    return count <= 20


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--since-hours", type=int, default=18)
    parser.add_argument("--page-size", type=int, default=15)
    args = parser.parse_args()

    now = dt.datetime.now(dt.timezone.utc)
    cutoff = now - dt.timedelta(hours=args.since_hours)
    cutoff_rfc3339 = cutoff.replace(microsecond=0).isoformat().replace("+00:00", "Z")

    collaborators = parse_collaborators(COLLABORATORS_MD)
    spaces_resp = run_gws(["chat", "spaces", "list", "--params", '{"pageSize":200}'])
    spaces = spaces_resp.get("spaces", [])

    selected = [
        space
        for space in spaces
        if should_include_space(space, cutoff, set(collaborators.keys()))
    ]
    selected.sort(key=lambda s: s.get("lastActiveTime") or "", reverse=True)

    conversations: list[dict[str, Any]] = []
    for space in selected:
        parent = space["name"]
        params = json.dumps(
            {
                "parent": parent,
                "pageSize": args.page_size,
                "orderBy": "createTime desc",
                "filter": f'create_time > "{cutoff_rfc3339}"',
            }
        )
        try:
            messages_resp = run_gws(["chat", "spaces", "messages", "list", "--params", params])
        except subprocess.CalledProcessError as exc:
            conversations.append(
                {
                    "space": parent,
                    "label": collaborators.get(parent, {}).get("name") or space.get("displayName"),
                    "spaceType": space.get("spaceType"),
                    "lastActiveTime": space.get("lastActiveTime"),
                    "error": exc.stderr.strip() or exc.stdout.strip(),
                }
            )
            continue

        raw = messages_resp.get("messages", [])
        messages = [normalize_message(m) for m in raw]
        # Fetched newest-first; present oldest-first so a thread reads in order.
        messages.sort(key=lambda m: m.get("createTime") or "")
        if not messages:
            continue

        conversations.append(
            {
                "space": parent,
                "truncated": len(raw) >= args.page_size,
                "label": collaborators.get(parent, {}).get("name") or space.get("displayName"),
                "spaceType": space.get("spaceType"),
                "displayName": space.get("displayName"),
                "lastActiveTime": space.get("lastActiveTime"),
                "source": "collaborator-note" if parent in collaborators else "recent-active-space",
                "messages": messages,
            }
        )

    output = {
        "generatedAt": now.replace(microsecond=0).isoformat().replace("+00:00", "Z"),
        "sinceHours": args.since_hours,
        "cutoff": cutoff_rfc3339,
        "conversationCount": len(conversations),
        "conversations": conversations,
    }
    json.dump(output, sys.stdout, indent=2)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
