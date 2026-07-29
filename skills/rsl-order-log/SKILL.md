---
name: rsl-order-log
description: Log Lorenzo's purchases and order updates in the canonical RSL Google Sheets order list. Use when Lorenzo asks to add, record, or update an order, invoice, purchase, delivery, or payment status in the RSL or shared-Drive order list.
---

# RSL Order Log

## Overview

Use the Google Drive and Google Sheets workflows to update the current canonical RSL order spreadsheet without rediscovering its location.

## Canonical Sheet

- 2026 order list: https://docs.google.com/spreadsheets/d/1s_TiI_6tf1WdDnIcXiXbkUp6Xf0MLO3qkpHiPFuvAT4/edit
- Spreadsheet ID: `1s_TiI_6tf1WdDnIcXiXbkUp6Xf0MLO3qkpHiPFuvAT4`

For orders outside 2026, search Drive for `RSL Order list <year>` and verify the result before writing; do not reuse the 2026 sheet automatically.

## Workflow

1. Read and follow the Google Drive and Google Sheets skills.
2. Open spreadsheet metadata and select the quarter tab matching the order date.
3. Search the bounded table for the order number to prevent duplicates.
4. Inspect the header, latest populated row, first blank row, and live validation rules before writing.
5. Preserve existing formatting and validation. Use Lorenzo's provided order data and do not invent missing invoice, payment, delivery, or project details.
6. Read back the written row and report its sheet and row number.
