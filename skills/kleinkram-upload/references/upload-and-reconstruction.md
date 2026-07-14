# Kleinkram Upload And Reconstruction

## Moleworks Convention

- Canonical project for new Mole data: `moleworks`.
- Mission format: `track__scenario__YYYYMMDD[__run_id]`.
- Common tracks: `est`, `dig`, `sysid`, `hwtest`.
- Historical or focused projects can remain separate when a server-side move is unavailable.

`prepare_payload.py` accepts `.bag`, `.mcap`, `.db3`, `.svo2`, `.tum`, `.yaml`, and `.yml`; normalizes filename stems to `A-Za-z0-9_-`; caps stems at 50 characters; deterministically de-duplicates names; and writes `upload_name_map.yaml`.

Use `--include-ext` to narrow a payload. Always inspect the mapping YAML before upload.

## Split-Bag Download And Reconstruction

Download one mission into a dedicated directory, then reconstruct original run roots from `upload_name_map.yaml`:

```bash
klein download \
  --project <project> \
  --mission <mission> \
  --dest /home/lorenzo/Downloads/kleinkram/<project>/<mission> \
  --nested --create-dirs --yes

python3 /home/lorenzo/codex_skills/skills/kleinkram-upload/scripts/reconstruct_split_bags.py \
  /home/lorenzo/Downloads/kleinkram/<project>/<mission>/<project>/<mission>
```

The helper uses hardlinks by default to avoid duplicating large bag data. Use `--skip-missing` only when Kleinkram skipped a known-corrupt remote file and the healthy runs should still be staged.

After reconstruction, run `ros2 bag info` or equivalent metadata checks on each reconstructed root before replay.
