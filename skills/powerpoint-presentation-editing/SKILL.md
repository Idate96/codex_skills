---
name: powerpoint-presentation-editing
description: "Edit or restructure PPTX decks with render-and-inspect verification. Use to preserve media, change text or layout, reorder slides, inspect assets, and export previews."
---

# PowerPoint Presentation Editing

## Workflow (Edit -> Render -> Inspect)

1. Create a backup and work in versioned copies (never overwrite the only deck).
2. Baseline render to PNGs for fast visual review:
   - Run `bash /home/lorenzo/codex_skills/skills/powerpoint-presentation-editing/scripts/render_pptx.sh INPUT.pptx OUT_DIR [DPI]`.
   - Inspect the relevant `OUT_DIR/slide-XX.png` files with `view_image`.
3. Inspect deck contents programmatically (optional):
   - `python3 /home/lorenzo/codex_skills/skills/powerpoint-presentation-editing/scripts/extract_slide_text.py INPUT.pptx` for a per-slide text dump.
   - `python3 /home/lorenzo/codex_skills/skills/powerpoint-presentation-editing/scripts/inspect_slide.py INPUT.pptx --slide N` for shape bounds + text.
4. Make one small change at a time (one slide, one layout fix, etc.).
5. After every change:
   - Check media-package differences: `bash /home/lorenzo/codex_skills/skills/powerpoint-presentation-editing/scripts/diff_pptx_media.sh BEFORE.pptx AFTER.pptx`.
     - If the diff contains unexpected changes, stop and inspect. Intentional media edits may produce an expected diff.
   - Re-render and visually re-check only the changed slide(s).

## Scripts

### Render PPTX -> PDF -> per-slide PNGs

```bash
bash /home/lorenzo/codex_skills/skills/powerpoint-presentation-editing/scripts/render_pptx.sh INPUT.pptx OUT_DIR
bash /home/lorenzo/codex_skills/skills/powerpoint-presentation-editing/scripts/render_pptx.sh INPUT.pptx OUT_DIR 220
```

Outputs:
- `OUT_DIR/<deck-name>.pdf`
- `OUT_DIR/slide-01.png`, `OUT_DIR/slide-02.png`, ...

### Diff embedded assets between two PPTX files

```bash
bash /home/lorenzo/codex_skills/skills/powerpoint-presentation-editing/scripts/diff_pptx_media.sh OLD.pptx NEW.pptx
```

Compares zipped package entries under:
- `ppt/media/`
- `ppt/embeddings/`
- `ppt/oleObjects/`

### Quick inspection utilities

```bash
python3 /home/lorenzo/codex_skills/skills/powerpoint-presentation-editing/scripts/extract_slide_text.py INPUT.pptx
python3 /home/lorenzo/codex_skills/skills/powerpoint-presentation-editing/scripts/inspect_slide.py INPUT.pptx --slide 7
```

## Notes

- PDF/PNG renders cannot play videos/animations; they show a poster frame/placeholder.
- The media diff covers embedded package entries only; it does not prove preservation of relationships, animations, masters, notes, or hyperlinks. Verify the parts affected by the edit.
- python-pptx cannot reorder slides via public API. For a pragmatic internal workaround, see [references/reorder_slides.md](references/reorder_slides.md).
