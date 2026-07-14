---
name: kleinkram-upload
description: "Upload and verify Kleinkram missions or manage project access. Use for datasets, rosbags, sysid artifacts, mission reconstruction, users, groups, and Moleworks permissions."
---

# Kleinkram Upload And Access

Use project UUIDs for stable targeting. Verify every upload and every access mutation through a readback.

Set the skill path once:

```bash
SKILL=/home/lorenzo/codex_skills/skills/kleinkram-upload
```

## Upload Workflow

1. Build a safe, flattened payload from one or more sources.
2. Inspect `<payload>/upload_name_map.yaml` for provenance and filename mapping.
3. Upload to the exact project and mission.
4. Verify remote size/hash and inspect the resulting mission.

```bash
"$SKILL/scripts/prepare_payload.py" \
  --input /path/to/source_a \
  --input /path/to/source_b \
  --output /path/to/payload

"$SKILL/scripts/upload_verify.sh" \
  --project moleworks \
  --mission track__scenario__YYYYMMDD \
  --payload /path/to/payload
```

Read [references/upload-and-reconstruction.md](references/upload-and-reconstruction.md) for extension constraints, mission naming, downloads, and split-bag reconstruction.

## Access Workflow

Inspection/status requests are read-only. Before a grant:

1. Resolve the exact user by email and confirm user UUID.
2. Resolve the exact project UUID and current ACL.
3. For `--project-pattern`, enumerate every matched project and show the scope.
4. Run `grant-user --dry-run` first.
5. Apply only after the user/project/rights scope matches the request.
6. Re-read each project ACL and verify the new entry.

```bash
"$SKILL/scripts/manage_access.py" search-users --query colleague@ethz.ch
"$SKILL/scripts/manage_access.py" list-projects --pattern moleworks
"$SKILL/scripts/manage_access.py" project-access --project <project-uuid>

"$SKILL/scripts/manage_access.py" grant-user \
  --user-uuid <user-uuid> \
  --project <project-uuid> \
  --rights create \
  --dry-run

"$SKILL/scripts/manage_access.py" grant-user \
  --user-uuid <user-uuid> \
  --project <project-uuid> \
  --rights create
```

Rights are `read`, `create`, `write`, and `delete`. Do not infer a higher right than requested. Read [references/access-management.md](references/access-management.md) for direct-grant fallback behavior, primary groups, custom groups, and the verified Diego shortcut.

## Network And Secret Boundaries

- Authentication comes from `~/.kleinkram.json`; never print or attach its tokens.
- If the endpoint is unavailable from this host, use the on-network runbook at `/home/lorenzo/moleworks/ros2_ws/src/moleworks_ros/mole_utils/docs/KLEINKRAM_ON_NETWORK_AGENT_RUNBOOK.md` when present.
- Do not broaden a one-project request into all Moleworks projects without explicit scope.
