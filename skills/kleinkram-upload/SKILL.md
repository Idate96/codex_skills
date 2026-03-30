---
name: kleinkram-upload
description: Stage and upload files to Kleinkram with mission creation and verification, or manage project/user access for Kleinkram datasets. Use when asked to upload datasets, run bags, or sysid artifacts to Kleinkram, or when asked to add users, inspect project access, or update Moleworks project permissions.
---

# Kleinkram Upload And Access

## Overview

This skill has two workflows:

1. Prepare a Kleinkram-safe payload from one or more files/folders, then upload and verify it against a project/mission.
2. Inspect or modify project access when you need to add users, verify current permissions, or update Moleworks project membership.

Use project IDs when possible for stable targeting. For access changes, prefer project-level updates and verify after every mutation.

## Moleworks Standard (2026-03)

- Canonical project for new Mole data: `moleworks`
- Canonical mission format: `track__scenario__YYYYMMDD[__run_id]`
- Suggested tracks: `est`, `dig`, `sysid`, `hwtest`
- Historical Mole projects may remain as legacy/archive if server-side move is unavailable
- Current Moleworks projects matching the canonical name pattern may include `moleworks` and focused subprojects such as `moleworks_digging`

## Quick Start: Upload

```bash
# 1) Build payload from one or more sources (recurses into directories).
scripts/prepare_payload.py \
  --input /path/to/source_a \
  --input /path/to/source_b \
  --output /path/to/payload_dir

# 2) Upload + verify to Kleinkram mission.
scripts/upload_verify.sh \
  --project moleworks \
  --mission sysid__arm_ground_truth__20260224 \
  --payload /path/to/payload_dir
```

## Quick Start: Access Management

```bash
# 1) Find the target user.
scripts/manage_access.py search-users --query "Christian Helms"

# 2) List Moleworks projects.
scripts/manage_access.py list-projects --pattern moleworks

# 3) Inspect current access on a project.
scripts/manage_access.py project-access --project moleworks

# 4) Grant project access.
# The script first tries the legacy /access/addUserToProject route.
# If the deployment does not expose it, pass the target user's primary
# access-group UUID and it will fall back to POST /projects/:uuid/access.
scripts/manage_access.py grant-user \
  --user-query "Christian Helms" \
  --project-pattern moleworks \
  --rights create \
  --primary-group-uuid <target_primary_group_uuid>
```

## Upload Workflow

1. Build payload with `prepare_payload.py`.
2. Inspect mapping file at `<payload>/upload_name_map.yaml`.
3. Upload and verify with `upload_verify.sh`.
4. Confirm mission contents with `klein mission info` and `klein list files`.

## Access Workflow

1. Use `manage_access.py search-users` to resolve the target user UUID.
2. Use `manage_access.py list-projects` or `manage_access.py project-access` to confirm the exact project set and current rights.
3. Prefer `manage_access.py grant-user` for adding a user to one or more projects.
4. Re-run `manage_access.py project-access` after each mutation and confirm the new entry is present.

## Access Notes

- Authentication for access management uses `~/.kleinkram.json` and refreshes the prod `authtoken` from the stored `refreshtoken`.
- Rights map is:
  - `read` = `0`
  - `create` = `10`
  - `write` = `20`
  - `delete` = `30`
- On some deployments, the older `/access/*` management routes are absent even though `/projects/:uuid/access` works.
- When `/access/addUserToProject` is missing, project updates still work, but you must provide the target user's primary access-group UUID via `--primary-group-uuid`.
- Use `create` when you want the same collaborator level that the `Leggedrobotics` affiliation group has on the canonical Moleworks projects.

## On-Network Fallback

If this machine cannot reach Kleinkram (network/endpoint issue), hand off to an agent in the same network as the server with this runbook:

- `/home/lorenzo/ros2_ws/src/moleworks_ros/mole_utils/docs/KLEINKRAM_ON_NETWORK_AGENT_RUNBOOK.md`

## Constraints Enforced

- File extensions: `.bag`, `.mcap`, `.db3`, `.svo2`, `.tum`, `.yaml`, `.yml`.
- Filename stem normalization: only `A-Za-z0-9_-`, max 50 chars, deterministic de-duplication (`__01`, `__02`, ...).
- Upload includes a mapping YAML in the mission for provenance.

## Common Patterns

YAML-only artifact upload:

```bash
scripts/prepare_payload.py \
  --input /path/to/stage \
  --include-ext .yaml \
  --include-ext .yml \
  --output /tmp/klein_payload_yaml
```

Ground-truth bag/MCAP upload:

```bash
scripts/prepare_payload.py \
  --input /path/to/run_1 \
  --input /path/to/run_2 \
  --include-ext .mcap \
  --include-ext .bag \
  --output /tmp/klein_payload_gt
```

Grant the same access level across all Moleworks projects matched by name:

```bash
scripts/manage_access.py grant-user \
  --user-query "Christian Helms" \
  --project-pattern moleworks \
  --rights create \
  --primary-group-uuid <target_primary_group_uuid>
```

## Resources

- `scripts/prepare_payload.py`: Build filtered, renamed payload + traceability map.
- `scripts/upload_verify.sh`: Create mission (safe rerun), upload, verify hash+size, print mission summary.
- `scripts/manage_access.py`: Refresh auth from `~/.kleinkram.json`, search users/projects, inspect project access, and grant project access with a legacy-route fallback.
