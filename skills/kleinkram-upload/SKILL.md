---
name: kleinkram-upload
description: Stage and upload files to Kleinkram with mission creation and verification. Use when asked to upload datasets, run bags, or sysid artifacts to Kleinkram, especially from mixed folders that need extension filtering, filename normalization, and source-to-upload mapping for traceability.
---

# Kleinkram Upload

## Overview

Prepare a Kleinkram-safe payload from one or more files/folders, then upload and verify it against a project/mission. Use project IDs when possible for stable targeting.

## Quick Start

```bash
# 1) Build payload from one or more sources (recurses into directories).
/home/lorenzo/.codex/skills/kleinkram-upload/scripts/prepare_payload.py \
  --input /path/to/source_a \
  --input /path/to/source_b \
  --output /path/to/payload_dir

# 2) Upload + verify to Kleinkram mission.
/home/lorenzo/.codex/skills/kleinkram-upload/scripts/upload_verify.sh \
  --project 9ad977dd-806f-4f84-800e-a5b31574ece5 \
  --mission arm_sys_id \
  --payload /path/to/payload_dir
```

## Workflow

1. Build payload with `prepare_payload.py`.
2. Inspect mapping file at `<payload>/upload_name_map.yaml`.
3. Upload and verify with `upload_verify.sh`.
4. Confirm mission contents with `klein mission info` and `klein list files`.

## Constraints Enforced

- File extensions: `.bag`, `.mcap`, `.db3`, `.svo2`, `.tum`, `.yaml`, `.yml`.
- Filename stem normalization: only `A-Za-z0-9_-`, max 50 chars, deterministic de-duplication (`__01`, `__02`, ...).
- Upload includes a mapping YAML in the mission for provenance.

## Common Patterns

YAML-only artifact upload:

```bash
/home/lorenzo/.codex/skills/kleinkram-upload/scripts/prepare_payload.py \
  --input /path/to/stage \
  --include-ext .yaml \
  --include-ext .yml \
  --output /tmp/klein_payload_yaml
```

Ground-truth bag/MCAP upload:

```bash
/home/lorenzo/.codex/skills/kleinkram-upload/scripts/prepare_payload.py \
  --input /path/to/run_1 \
  --input /path/to/run_2 \
  --include-ext .mcap \
  --include-ext .bag \
  --output /tmp/klein_payload_gt
```

## Resources

- `scripts/prepare_payload.py`: Build filtered, renamed payload + traceability map.
- `scripts/upload_verify.sh`: Create mission (safe rerun), upload, verify hash+size, print mission summary.
