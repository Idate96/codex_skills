# Reorder Slides (python-pptx internal API)

python-pptx does not expose slide reordering as a public API. For research code, you can
reorder the internal `<p:sldId>` list (`prs.slides._sldIdLst`) to match a desired order.

This is an internal hack, but it works in practice with the current python-pptx versions.

## Reorder by slide indices

```python
from pptx import Presentation

prs = Presentation("in.pptx")

# Desired slide order as 0-based indices (example).
order = [0, 2, 1, 3]

sldIdLst = prs.slides._sldIdLst  # internal lxml element
sldIds = list(sldIdLst)         # current <p:sldId> elements (aligned with prs.slides)

# Clear and re-append in the new order.
for el in list(sldIdLst):
    sldIdLst.remove(el)
for old_idx in order:
    sldIdLst.append(sldIds[old_idx])

prs.save("out.pptx")
```

## Notes / pitfalls

- Build `order` from your own logic (e.g., after listing titles/text with `extract_slide_text.py`).
- Do not rely on this if you need strict forwards/backwards compatibility across python-pptx versions.
