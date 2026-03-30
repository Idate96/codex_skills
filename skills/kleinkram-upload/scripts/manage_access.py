#!/usr/bin/env python3
"""Inspect and update Kleinkram project access.

This helper keeps the live workflow simple:

1. Read the selected endpoint and prod credentials from ~/.kleinkram.json.
2. Refresh the short-lived authtoken from the stored refreshtoken.
3. Resolve users and projects through the public API.
4. Mutate project access with the most direct route available.

Some deployments expose /access/addUserToProject. Others only expose
/projects/:uuid/access. The grant-user command tries the legacy route first.
If the deployment returns 404, the script falls back to a full project-access
rewrite and requires the target user's primary access-group UUID.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any


CLIENT_VERSION = "0.59.0"
UUID_RE = re.compile(
    r"^[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$",
    re.IGNORECASE,
)
RIGHTS = {
    "read": 0,
    "create": 10,
    "write": 20,
    "delete": 30,
}


@dataclass
class Session:
    api: str
    refresh_token: str
    auth_token: str

    @property
    def cookie_header(self) -> str:
        return (
            f"authtoken={self.auth_token}; "
            f"refreshtoken={self.refresh_token}"
        )


def is_uuid(value: str) -> bool:
    return UUID_RE.fullmatch(value) is not None


def load_endpoint_credentials(endpoint: str) -> tuple[str, str]:
    config_path = Path.home() / ".kleinkram.json"
    config = json.loads(config_path.read_text())
    api = config["endpoints"][endpoint]["api"]
    refresh_token = config["endpoint_credentials"][endpoint]["refresh_token"]
    if not refresh_token:
        raise SystemExit(
            f"{config_path} has no refresh token for endpoint {endpoint!r}"
        )
    return api.rstrip("/"), refresh_token


def parse_set_cookie(header_blob: str, cookie_name: str) -> str | None:
    match = re.search(
        rf"^set-cookie:\s*{re.escape(cookie_name)}=([^;]+);",
        header_blob,
        re.IGNORECASE | re.MULTILINE,
    )
    return None if match is None else match.group(1)


def refresh_session(endpoint: str) -> Session:
    api, refresh_token = load_endpoint_credentials(endpoint)
    request = urllib.request.Request(
        f"{api}/auth/refresh-token",
        method="POST",
        headers={
            "Kleinkram-Client-Version": CLIENT_VERSION,
            "Cookie": f"refreshtoken={refresh_token}",
        },
    )
    try:
        with urllib.request.urlopen(request) as response:
            header_blob = str(response.headers)
    except urllib.error.HTTPError as error:
        body = error.read().decode()
        raise SystemExit(
            f"failed to refresh Kleinkram auth on {endpoint}: "
            f"{error.code} {body}"
        ) from error
    auth_token = parse_set_cookie(header_blob, "authtoken")
    if auth_token is None:
        raise SystemExit("refresh-token call succeeded but returned no authtoken")
    return Session(api=api, refresh_token=refresh_token, auth_token=auth_token)


def api_request(
    session: Session,
    method: str,
    path: str,
    payload: Any | None = None,
) -> tuple[int, Any]:
    url = path if path.startswith("http") else f"{session.api}{path}"
    headers = {
        "Kleinkram-Client-Version": CLIENT_VERSION,
        "Cookie": session.cookie_header,
    }
    data = None
    if payload is not None:
        headers["Content-Type"] = "application/json"
        data = json.dumps(payload).encode()
    request = urllib.request.Request(
        url,
        data=data,
        method=method,
        headers=headers,
    )
    try:
        with urllib.request.urlopen(request) as response:
            body = response.read().decode()
            parsed = json.loads(body) if body else None
            return response.status, parsed
    except urllib.error.HTTPError as error:
        body = error.read().decode()
        parsed = None
        if body:
            try:
                parsed = json.loads(body)
            except json.JSONDecodeError:
                parsed = body
        return error.code, parsed


def pretty_dump(data: Any) -> None:
    print(json.dumps(data, indent=2, sort_keys=False))


def resolve_single_user(session: Session, query: str) -> dict[str, Any]:
    status, payload = api_request(
        session,
        "GET",
        "/user/search?"
        + urllib.parse.urlencode(
            {"search": query, "skip": 0, "take": 20},
            doseq=True,
        ),
    )
    if status != 200:
        raise SystemExit(f"user search failed: {status} {payload}")
    users = payload["users"]
    if len(users) != 1:
        raise SystemExit(
            "user query must resolve to exactly one user:\n"
            + json.dumps(users, indent=2)
        )
    return users[0]


def list_projects(session: Session, pattern: str | None, take: int) -> list[dict[str, Any]]:
    params = {
        "take": take,
        "skip": 0,
        "sortBy": "name",
        "sortOrder": "ASC",
    }
    if pattern:
        params["projectPatterns"] = pattern
    status, payload = api_request(
        session,
        "GET",
        "/projects?" + urllib.parse.urlencode(params, doseq=True),
    )
    if status != 200:
        raise SystemExit(f"project listing failed: {status} {payload}")
    return payload["data"]


def resolve_projects(
    session: Session,
    project_specs: list[str],
    project_patterns: list[str],
) -> list[dict[str, Any]]:
    resolved: dict[str, dict[str, Any]] = {}
    for spec in project_specs:
        if is_uuid(spec):
            status, payload = api_request(session, "GET", f"/projects/{spec}")
            if status != 200:
                raise SystemExit(f"failed to resolve project {spec}: {status} {payload}")
            resolved[payload["uuid"]] = payload
            continue
        matches = list_projects(session, spec, 200)
        if len(matches) != 1:
            raise SystemExit(
                f"project spec {spec!r} must resolve to exactly one project:\n"
                + json.dumps(matches, indent=2)
            )
        resolved[matches[0]["uuid"]] = matches[0]
    for pattern in project_patterns:
        for project in list_projects(session, pattern, 200):
            resolved[project["uuid"]] = project
    if not resolved:
        raise SystemExit("no projects resolved")
    return [resolved[key] for key in sorted(resolved)]


def get_project_access(session: Session, project_uuid: str) -> list[dict[str, Any]]:
    status, payload = api_request(session, "GET", f"/projects/{project_uuid}/access")
    if status != 200:
        raise SystemExit(
            f"failed to read project access for {project_uuid}: {status} {payload}"
        )
    return payload["data"]


def try_legacy_grant(
    session: Session,
    project_uuid: str,
    user_uuid: str,
    rights: int,
) -> bool:
    status, payload = api_request(
        session,
        "POST",
        "/access/addUserToProject",
        {
            "uuid": project_uuid,
            "userUUID": user_uuid,
            "rights": rights,
        },
    )
    if status == 404:
        return False
    if status not in (200, 201):
        raise SystemExit(
            f"legacy addUserToProject failed for {project_uuid}: {status} {payload}"
        )
    return True


def update_project_access(
    session: Session,
    project_uuid: str,
    access_rows: list[dict[str, Any]],
) -> None:
    status, payload = api_request(
        session,
        "POST",
        f"/projects/{project_uuid}/access",
        access_rows,
    )
    if status not in (200, 201):
        raise SystemExit(
            f"project access update failed for {project_uuid}: {status} {payload}"
        )


def cmd_search_users(args: argparse.Namespace) -> None:
    session = refresh_session(args.endpoint)
    status, payload = api_request(
        session,
        "GET",
        "/user/search?"
        + urllib.parse.urlencode({"search": args.query, "skip": 0, "take": args.take}),
    )
    if status != 200:
        raise SystemExit(f"user search failed: {status} {payload}")
    pretty_dump(payload)


def cmd_list_projects(args: argparse.Namespace) -> None:
    session = refresh_session(args.endpoint)
    pretty_dump(
        {
            "data": list_projects(session, args.pattern, args.take),
        }
    )


def cmd_project_access(args: argparse.Namespace) -> None:
    session = refresh_session(args.endpoint)
    projects = resolve_projects(session, args.project, args.project_pattern)
    for project in projects:
        print(f"# {project['name']} ({project['uuid']})")
        pretty_dump(get_project_access(session, project["uuid"]))


def cmd_grant_user(args: argparse.Namespace) -> None:
    session = refresh_session(args.endpoint)
    rights = RIGHTS[args.rights]
    user = (
        resolve_single_user(session, args.user_query)
        if args.user_query
        else {"uuid": args.user_uuid, "name": "resolved-user"}
    )
    projects = resolve_projects(session, args.project, args.project_pattern)

    used_fallback = False
    for project in projects:
        if try_legacy_grant(session, project["uuid"], user["uuid"], rights):
            print(
                f"granted {args.rights} to {user['uuid']} on "
                f"{project['name']} via /access/addUserToProject"
            )
            continue

        used_fallback = True
        if args.primary_group_uuid is None:
            raise SystemExit(
                "deployment does not expose /access/addUserToProject; "
                "rerun with --primary-group-uuid"
            )

        current_rows = get_project_access(session, project["uuid"])
        new_rows = []
        seen = False
        for row in current_rows:
            if row["uuid"] == args.primary_group_uuid:
                updated = dict(row)
                updated["rights"] = rights
                new_rows.append(updated)
                seen = True
            else:
                new_rows.append(row)
        if not seen:
            new_rows.append(
                {
                    "uuid": args.primary_group_uuid,
                    "name": args.primary_group_name
                    or f"Personal: {user.get('name', user['uuid'])}",
                    "type": "PRIMARY",
                    "memberCount": args.primary_group_member_count,
                    "rights": rights,
                }
            )

        if args.dry_run:
            print(f"# dry-run {project['name']} ({project['uuid']})")
            pretty_dump(new_rows)
            continue

        update_project_access(session, project["uuid"], new_rows)
        print(
            f"granted {args.rights} to {user['uuid']} on "
            f"{project['name']} via /projects/:uuid/access"
        )

    if used_fallback:
        print("fallback route used: /projects/:uuid/access")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inspect and update Kleinkram project access."
    )
    parser.add_argument("--endpoint", default="prod")
    subparsers = parser.add_subparsers(dest="command", required=True)

    search_users = subparsers.add_parser("search-users")
    search_users.add_argument("--query", required=True)
    search_users.add_argument("--take", type=int, default=20)
    search_users.set_defaults(func=cmd_search_users)

    list_projects_cmd = subparsers.add_parser("list-projects")
    list_projects_cmd.add_argument("--pattern")
    list_projects_cmd.add_argument("--take", type=int, default=200)
    list_projects_cmd.set_defaults(func=cmd_list_projects)

    project_access = subparsers.add_parser("project-access")
    project_access.add_argument("--project", action="append", default=[])
    project_access.add_argument("--project-pattern", action="append", default=[])
    project_access.set_defaults(func=cmd_project_access)

    grant_user = subparsers.add_parser("grant-user")
    grant_user.add_argument("--project", action="append", default=[])
    grant_user.add_argument("--project-pattern", action="append", default=[])
    grant_user.add_argument("--user-query")
    grant_user.add_argument("--user-uuid")
    grant_user.add_argument(
        "--rights",
        choices=sorted(RIGHTS),
        required=True,
    )
    grant_user.add_argument("--primary-group-uuid")
    grant_user.add_argument("--primary-group-name")
    grant_user.add_argument("--primary-group-member-count", type=int, default=1)
    grant_user.add_argument("--dry-run", action="store_true")
    grant_user.set_defaults(func=cmd_grant_user)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.command == "grant-user":
        if not args.project and not args.project_pattern:
            parser.error("grant-user requires --project and/or --project-pattern")
        if not args.user_query and not args.user_uuid:
            parser.error("grant-user requires --user-query or --user-uuid")
        if args.user_uuid and not is_uuid(args.user_uuid):
            parser.error("--user-uuid must be a UUID")
        if args.primary_group_uuid and not is_uuid(args.primary_group_uuid):
            parser.error("--primary-group-uuid must be a UUID")
    if args.command == "project-access" and not args.project and not args.project_pattern:
        parser.error("project-access requires --project and/or --project-pattern")
    args.func(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
