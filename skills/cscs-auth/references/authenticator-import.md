# Importing the CSCS authenticator account

Use this only when the CSCS entry is missing or the user requests an import.
Routine login uses the existing entry in Chrome.
If the entry is missing but the task is renewal, use the existing export directly
through [Chrome and export recovery](chrome-export-recovery.md). Re-importing is
not a prerequisite to authenticating.

## Select and import

1. Check the [Authenticator Extension store page](https://chromewebstore.google.com/detail/authenticator/bhghoamapcdpbohphigoooaddinpkbai).
   “Remove from Chrome” means it is already installed in that profile.
2. In Google Authenticator, use **Transfer accounts → Export accounts** and select
   CSCS. If the user supplies an email or local export, retrieve only that artifact.
   For a self-sent Gmail export, a bounded `from:me to:me newer_than:1d` search was
   sufficient. Use the Gmail connector's supported attachment download.
3. Inspect the export locally without printing its QR payload or secrets. An
   `otpauth-migration://offline` QR may contain multiple accounts. Select only the
   requested issuer/account; reject an absent or ambiguous match. Do not import
   the whole export just because the user called it “the code.”
4. Open the extension UI → **Settings → Backup → Import Backup**. The import page
   is `chrome-extension://bhghoamapcdpbohphigoooaddinpkbai/view/import.html`.
   Use **Import QR Image Backup** for a CSCS-only image. For a multi-account export,
   locally extract only the CSCS entry and submit its standard `otpauth://totp/…`
   URI through **Import Text Backup**, with **Encrypted** unchecked. Send the URI
   through the local CDP socket, not a command argument or printed tool result.
5. Reopen the popup and verify one `CSCS` / requested-username entry. Test its code
   in the intended CSCS login. Remove temporary export files created by the task;
   leave the user's original email/file unless they ask to remove it.

The tested export was TOTP, SHA-1, six digits, 30 seconds. Preserve the selected
entry's parameters when converting; verify other formats before importing them.
Local QR decoding used `uv run --with zxing-cpp --with pillow python …`.
Google migration protobuf field 1 contains repeated accounts; within each account,
fields 2/3 are name/issuer, 1 is the secret, 4/5/6 are algorithm/digits/type, and
7 is the HOTP counter. Keep secret-bearing values in memory.

## Stalled import page

Authenticator Extension 8.0.1 saves the account **before** its success `alert()`,
then closes the import page after dismissal. URI imports assign fresh account IDs,
so blindly repeating a stalled submission creates duplicates.

If CDP reports `Timeout: Runtime.enable` after import:

- Check and dismiss the success dialog if present. A “No dialog is showing” result
  does not establish that the import failed.
- If import/popup tabs remain unresponsive, close only those selected extension
  tabs with browser-level `Target.closeTarget`, then reopen `view/popup.html`.
  This recovered the saved CSCS entry in the tested session.
- Verify account presence before retrying. Do not remove/reinstall the extension
  or clear its storage to recover a tab.

Sources: [official import instructions](https://authenticator.cc/docs/en/export-and-import.html),
[Google export instructions](https://support.google.com/accounts/answer/1066447?hl=en),
[8.0.1 text-import handler](https://github.com/Authenticator-Extension/Authenticator/blob/v8.0.1/src/components/Import/TextImport.vue).
