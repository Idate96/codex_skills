---
name: kleinkram-upload
description: Upload ROS/ROS2 bag datasets (.mcap/.bag/.db3 plus metadata.yaml) to Kleinkram using the `klein` CLI. Use when asked to create/find a Kleinkram project/mission and upload one or more bag folders, especially when avoiding filename collisions (e.g., repeated metadata.yaml).
---

# Kleinkram Upload (klein CLI)

## Quick Start

```bash
# 1) Pick endpoint and confirm auth
klein endpoint prod   # or: dev / local
klein list projects   # if "Not Authenticated": klein login

# 2) Prefer project UUIDs for upload (work around name lookup issues)
klein project info -p mole_estimator
PROJECT_ID="<uuid>"

# 3) Upload a ROS2 bag folder (metadata.yaml + *.mcap / *.db3)
klein upload -p "$PROJECT_ID" -m <mission_name> --create --experimental-datatypes \
  /path/to/bag/metadata.yaml \
  /path/to/bag/*.mcap
```

## Mission Naming (Avoid Filename Collisions)

Kleinkram stores files in a mission by filename. Many ROS2 bags include `metadata.yaml`,
so uploading multiple bag folders into a single mission will collide unless you rename files first.

Recommended patterns:
- One mission per bag folder.
- If one "run" contains sub-bags (example: `camera/`, `lidar/`, `state/`, `sensors/`), use missions `<run>_camera`, `<run>_lidar`, `<run>_state`, `<run>_sensors`.

## Bulk Upload Helper Script

`scripts/klein_upload_rosbags.py` uploads one or more bag directories and automatically:
- resolves `--project` name to a project UUID
- creates missions (`--create`)
- splits subfolders into separate missions (to avoid `metadata.yaml` collisions)

Examples (run from this skill directory):

```bash
python3 scripts/klein_upload_rosbags.py \
  --project mole_estimator \
  /home/lorenzo/bags/estimator_eval/cabin_rotate_20260208_172438
```

```bash
python3 scripts/klein_upload_rosbags.py \
  --project mole_estimator \
  --mission-prefix estimator_eval \
  /home/lorenzo/bags/estimator_eval/all_sensors_cabin_rotate_20260208_171946 \
  /home/lorenzo/bags/lidar_raw/legs_updown_20260208_152409
```

Dry run:

```bash
python3 scripts/klein_upload_rosbags.py --dry-run --project mole_estimator <paths...>
```

## Verify Uploads

```bash
klein list files -p <project_id_or_name> -m <mission_name>
```

Optional verify (compares local files to remote by name, size, and hash):

```bash
klein verify -p <project_id_or_name> -m <mission_name> /path/to/file1 /path/to/file2
```
