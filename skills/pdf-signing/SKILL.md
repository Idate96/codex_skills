---
name: pdf-signing
description: Place Lorenzo's existing handwritten signature image in a local PDF and verify it visually. Use for non-cryptographic PDF signing or placement fixes.
---

# PDF Signing

Use this when the task is to place an existing signature image onto a PDF. This is for visual signing only, not for cryptographic digital signatures.

## Workflow

1. Find the target PDF, usually in `~/Downloads`.
2. Use the default signature image at `/home/lorenzo/Documents/General/signature_tight_crop_transparent.png`.
3. If that file is missing, find another existing signature image. Prefer a transparent PNG.
4. Inspect the signature page before editing:
   - Render the relevant page with `pdftoppm -png`.
   - Use `view_image` on the rendered page.
5. Place the signature with `scripts/place_signature.py`.
6. Render the edited page and verify the placement visually.
7. Save to a new output PDF unless Lorenzo explicitly asks to overwrite the source.
8. If `python3` does not have `fitz`/`PyMuPDF`, run the script through `uv` instead of trying to mutate the system Python.

## Commands

Apply one or more signatures:

```bash
python3 /home/lorenzo/codex_skills/skills/pdf-signing/scripts/place_signature.py \
  --input "/path/to/input.pdf" \
  --output "/path/to/output.pdf" \
  --signature "/home/lorenzo/Documents/General/signature_tight_crop_transparent.png" \
  --page 3 \
  --rect 76,320,220,352 \
  --rect 76,478,220,510
```

If `fitz` is missing from `python3`, use:

```bash
uv run --with PyMuPDF python3 /home/lorenzo/codex_skills/skills/pdf-signing/scripts/place_signature.py \
  --input "/path/to/input.pdf" \
  --output "/path/to/output.pdf" \
  --signature "/home/lorenzo/Documents/General/signature_tight_crop_transparent.png" \
  --page 1 \
  --rect 686,146,818,175
```

Render a page for visual checking:

```bash
mkdir -p /tmp/pdf_signing_preview
pdftoppm -f 3 -l 3 -png "/path/to/output.pdf" /tmp/pdf_signing_preview/page
```

Then open `/tmp/pdf_signing_preview/page-3.png` with `view_image`.

## Notes

- Default signature asset: `/home/lorenzo/Documents/General/signature_tight_crop_transparent.png`.
- `--page` is 1-based.
- Each `--rect` is `x0,y0,x1,y1` in PDF points.
- PyMuPDF uses a top-left origin with `y` increasing downward.
- Verified ETH monthly timesheet supervisor signature rectangle on the current SAP landscape A4 form:
  - page `1`
  - rect `686,146,818,175`
- Prefer matching the size and baseline of any existing handwritten signature already present in the document.
- If the user asks for a true digital signature, stop and say this skill only handles visual signature placement.
