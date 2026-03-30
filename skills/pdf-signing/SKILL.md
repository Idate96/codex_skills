---
name: pdf-signing
description: Sign a local PDF contract or form by placing an existing handwritten signature image onto one or more pages, then visually verify the result. Use when Lorenzo asks to sign a PDF in Downloads, adjust signature placement, or reuse an existing local signature image for a fast non-cryptographic signature workflow.
---

# PDF Signing

Use this when the task is to place an existing signature image onto a PDF. This is for visual signing only, not for cryptographic digital signatures.

## Workflow

1. Find the target PDF, usually in `~/Downloads`.
2. Find an existing signature image. Prefer a transparent PNG if one already exists.
3. Inspect the signature page before editing:
   - Render the relevant page with `pdftoppm -png`.
   - Use `view_image` on the rendered page.
4. Place the signature with `scripts/place_signature.py`.
5. Render the edited page and verify the placement visually.
6. Save to a new output PDF unless Lorenzo explicitly asks to overwrite the source.

## Commands

Apply one or more signatures:

```bash
python3 /home/lorenzo/git/codex_skills/skills/pdf-signing/scripts/place_signature.py \
  --input "/path/to/input.pdf" \
  --output "/path/to/output.pdf" \
  --signature "/path/to/signature.png" \
  --page 3 \
  --rect 76,320,220,352 \
  --rect 76,478,220,510
```

Render a page for visual checking:

```bash
mkdir -p /tmp/pdf_signing_preview
pdftoppm -f 3 -l 3 -png "/path/to/output.pdf" /tmp/pdf_signing_preview/page
```

Then open `/tmp/pdf_signing_preview/page-3.png` with `view_image`.

## Notes

- `--page` is 1-based.
- Each `--rect` is `x0,y0,x1,y1` in PDF points.
- PyMuPDF uses a top-left origin with `y` increasing downward.
- Prefer matching the size and baseline of any existing handwritten signature already present in the document.
- If the user asks for a true digital signature, stop and say this skill only handles visual signature placement.
