---
name: powerpoint-presentation-editing
description: Edit or restructure PPTX decks with render-and-inspect verification. Use to preserve media, change text or layout, reorder slides, or export previews.
---

# PowerPoint Presentation Editing

## Workflow (Edit -> Render -> Inspect)

1. Create a backup and work in versioned copies (never overwrite the only deck).
2. Baseline render to PNGs for fast visual review:
   - Run `bash scripts/render_pptx.sh INPUT.pptx OUT_DIR [DPI]`.
   - Inspect the relevant `OUT_DIR/slide-XX.png` files with `functions.view_image`.
3. Inspect deck contents programmatically (optional):
   - `python3 scripts/extract_slide_text.py INPUT.pptx` for a per-slide text dump.
   - `python3 scripts/inspect_slide.py INPUT.pptx --slide N` for shape bounds + text.
4. Make one small change at a time (one slide, one layout fix, etc.).
5. After every change:
   - Verify media preservation: `bash scripts/diff_pptx_media.sh BEFORE.pptx AFTER.pptx`.
     - If the diff is non-empty, STOP: you likely dropped/modified embedded assets.
   - Re-render and visually re-check only the changed slide(s).

## Scripts

### Render PPTX -> PDF -> per-slide PNGs

```bash
bash scripts/render_pptx.sh INPUT.pptx OUT_DIR
bash scripts/render_pptx.sh INPUT.pptx OUT_DIR 220  # higher DPI for small text
```

Outputs:
- `OUT_DIR/<deck-name>.pdf`
- `OUT_DIR/slide-01.png`, `OUT_DIR/slide-02.png`, ...

### Diff embedded assets between two PPTX files

```bash
bash scripts/diff_pptx_media.sh OLD.pptx NEW.pptx
```

Compares zipped package entries under:
- `ppt/media/`
- `ppt/embeddings/`
- `ppt/oleObjects/`

### Quick inspection utilities

```bash
python3 scripts/extract_slide_text.py INPUT.pptx
python3 scripts/inspect_slide.py INPUT.pptx --slide 7
```

## Notes

- PDF/PNG renders cannot play videos/animations; they show a poster frame/placeholder.
- python-pptx cannot reorder slides via public API. For a pragmatic internal workaround, see `references/reorder_slides.md`.
