---
name: kleinkram-upload
description: Upload and verify Kleinkram missions or manage project access. Use for datasets, rosbags, sysid artifacts, users, groups, and Moleworks permissions.
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
scripts/manage_access.py search-users --query "colleague@ethz.ch"

# 2) List Moleworks projects.
scripts/manage_access.py list-projects --pattern moleworks

# 3) Inspect current access on a project.
scripts/manage_access.py project-access --project moleworks

# 4) Grant project access directly when the primary group UUID is known.
# The script first tries the legacy /access/addUserToProject route.
# If the deployment does not expose it, pass the target user's primary
# access-group UUID and it will fall back to POST /projects/:uuid/access.
scripts/manage_access.py grant-user \
  --user-query "colleague@ethz.ch" \
  --project-pattern moleworks \
  --rights create \
  --primary-group-uuid <target_primary_group_uuid>

# 5) If the primary group UUID is not readily available, the script can
# auto-create or reuse a dedicated custom fallback group.
scripts/manage_access.py grant-user \
  --user-query "colleague@ethz.ch" \
  --project 12345678-1234-1234-1234-123456789abc \
  --rights create

# 6) Optional: override the generated fallback group name.
scripts/manage_access.py grant-user \
  --user-query "colleague@ethz.ch" \
  --project 12345678-1234-1234-1234-123456789abc \
  --rights create \
  --fallback-group-name project_slug__colleague
```

## Upload Workflow

1. Build payload with `prepare_payload.py`.
2. Inspect mapping file at `<payload>/upload_name_map.yaml`.
3. Upload and verify with `upload_verify.sh`.
4. Confirm mission contents with `klein mission info` and `klein list files`.

## Download + Reconstruct Split Bags

Kleinkram missions built with `prepare_payload.py` are intentionally flattened for upload safety. For split DIG runs, download the mission first, then reconstruct one replayable run root per original source directory from the mission's `upload_name_map.yaml`.

Example:

```bash
# 1) Download one mission into a local folder.
klein download \
  --project moleworks_rl_digging \
  --mission dig__soil_sysid_manual__20260331 \
  --dest /home/lorenzo/Downloads/kleinkram/moleworks_rl_digging/dig__soil_sysid_manual__20260331 \
  --nested \
  --create-dirs \
  --yes

# 2) Reconstruct split rosbag roots under reconstructed_runs/.
python3 scripts/reconstruct_split_bags.py \
  /home/lorenzo/Downloads/kleinkram/moleworks_rl_digging/dig__soil_sysid_manual__20260331/moleworks_rl_digging/dig__soil_sysid_manual__20260331
```

The helper defaults to hardlinks, so reconstructing large missions does not duplicate the bag data on disk.
If Kleinkram skipped a known-corrupt remote file during download, rerun reconstruction with `--skip-missing` so the healthy runs still get staged.

## Access Workflow

1. Use `manage_access.py search-users` with the collaborator's email to resolve the user UUID.
2. Use `manage_access.py list-projects` or `manage_access.py project-access` to confirm the exact project UUID and current rights.
3. Prefer `manage_access.py grant-user` for adding a user to one or more projects.
4. Re-run `manage_access.py project-access` after each mutation and confirm the new entry is present.
5. If the deployment rejects direct user grant because `/access/addUserToProject` is missing and you do not have the target user's primary-group UUID, the script should fall back automatically to a dedicated custom access group:
   - first reuse an existing custom project group that already contains that user, if one exists
   - otherwise create or reuse a small project-specific group
   - add the user to that group
   - grant that group access to the project

## Access Notes

- Authentication for access management uses `~/.kleinkram.json` and refreshes the prod `authtoken` from the stored `refreshtoken`.
- Rights map is:
  - `read` = `0`
  - `create` = `10`
  - `write` = `20`
  - `delete` = `30`
- On some deployments, the older `/access/*` management routes are absent even though `/projects/:uuid/access` works.
- When `/access/addUserToProject` is missing, the script first tries the automatic custom-group fallback; use `--primary-group-uuid` only when you explicitly want the direct `/projects/:uuid/access` rewrite path.
- Primary access groups are hidden from normal access-group search on this deployment, so the primary-group UUID is not always easy to resolve quickly.
- Use `--fallback-group-name` only when you want to force a specific custom group name.
- Use `create` when you want the same collaborator level that the `Leggedrobotics` affiliation group has on the canonical Moleworks projects.

## UUID Resolution

- User UUID:
  - `python3 scripts/manage_access.py search-users --query "user@domain.com"`
  - Prefer the exact email address over a partial name search.
- Project UUID:
  - `python3 scripts/manage_access.py list-projects --pattern moleworks`
  - or inspect a known project directly with `project-access`
  - or reuse the project UUID from the upload confirmation if the project was just created
- Primary-group UUID:
  - not reliably searchable on this deployment because primary groups are hidden
  - use it only if you already have it from a trusted source

## Fast Path: Diego + `moleworks_arm_control`

- Diego's Kleinkram / ETH email: `digarcia@ethz.ch`
- Diego user UUID: `5e4e0af7-0fca-4934-9470-070131c207e5`
- `moleworks_arm_control` project UUID: `6b62504e-f6fd-4aa6-bacd-fc2a231ed761`
- Verified working fallback group for this project: `moleworks_arm_control__diego`
  - group UUID: `9596a4f5-02eb-4250-b2c4-4f20e4f4c891`
  - rights on project: `create` (`10`)

If Diego reports missing access again on this exact project, check whether the custom group is still present on the project ACL before doing anything else:

```bash
python3 scripts/manage_access.py project-access \
  --project 6b62504e-f6fd-4aa6-bacd-fc2a231ed761
```

If needed, recreate the same pattern with the API:

1. Create or reuse the custom group `moleworks_arm_control__diego`
2. Add user UUID `5e4e0af7-0fca-4934-9470-070131c207e5` to that group
3. Grant the group `create` rights on project UUID `6b62504e-f6fd-4aa6-bacd-fc2a231ed761`

## On-Network Fallback

If this machine cannot reach Kleinkram (network/endpoint issue), hand off to an agent in the same network as the server with this runbook:

- `/home/lorenzo/moleworks/ros2_ws/src/moleworks_ros/mole_utils/docs/KLEINKRAM_ON_NETWORK_AGENT_RUNBOOK.md`

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

Grant access to one project with the generic automatic fallback:

```bash
scripts/manage_access.py grant-user \
  --user-query "colleague@ethz.ch" \
  --project 12345678-1234-1234-1234-123456789abc \
  --rights create
```

## Resources

- `scripts/prepare_payload.py`: Build filtered, renamed payload + traceability map.
- `scripts/upload_verify.sh`: Create mission (safe rerun), upload, verify hash+size, print mission summary.
- `scripts/manage_access.py`: Refresh auth from `~/.kleinkram.json`, search users/projects, inspect project access, and grant project access with a legacy-route fallback.
- `scripts/reconstruct_split_bags.py`: Rebuild split rosbag roots from a flattened downloaded mission payload using `upload_name_map.yaml`.
