# Chrome and existing-export recovery

Use this when the CDP helper cannot connect, the visible authenticator is empty,
or Lorenzo says the OTP was already exported. This is the route verified on
Starship on 2026-09-15. Refresh windows, profiles, sessions, and coordinates live.

## Find the correct browser context

The verified Starship profiles have different roles:

| Purpose | Existing profile | Verified account |
| --- | --- | --- |
| ETH/CSCS login with saved ETH autofill | `~/.config/google-chrome`, `Profile 1` (School) | `lterenzi@leggedrobotics.com` |
| Existing authenticator export in personal Gmail | `~/.gmail_web_profile`, `Default` | `lorenzoterenzi96@gmail.com` |

Inspect only profile metadata in `Local State` (`profile.info_cache`: directory,
display name, username) when discovery is necessary. Do not inspect password,
cookie, or extension-storage databases. The School-profile extension and the
local GNOME Authenticator both displayed no accounts in the verified run;
neither established that the export was missing.

Check the current endpoint/`DevToolsActivePort` before using CDP. If Chrome shows
an organization-policy block for `chrome://inspect/#remote-debugging`, leave it
alone and use the normal desktop UI. Do not remove policies, clone profiles, or
restart Chrome to add debugging flags.

From a ROS container, reach the Starship desktop through the configured SSH route.
On the host, discover `DISPLAY`, `XAUTHORITY`, visible Chrome windows, and current
geometry. `xdotool`, `scrot`, and Python Pillow were available. Use window IDs
and coordinates from the current run, never IDs or coordinates from a transcript.
With `scrot`, use `-o` to overwrite a chosen scratch screenshot; without it, a
new suffixed filename can leave you inspecting an old screenshot.

## Retrieve the existing export through Chrome

1. Use the personal Gmail profile above. If it is not running, open that existing
   profile with `/opt/google/chrome/chrome --user-data-dir=/home/lorenzo/.gmail_web_profile URL`.
   Do not change the default browser or enable telemetry if a first-run dialog
   appears. A Gmail connector is usable only after its account is confirmed to
   match; in the verified run the connector belonged to a different person.
2. The known export is the self-sent message **Screenshot (13 Sept 2026 22:22:45)**,
   Gmail message ID `1a09c70195c2d05e`, attachment
   **Screenshot_20260913-222245.png**. A non-credential message locator is
   `https://mail.google.com/mail/u/0/#all/1a09c70195c2d05e` in the personal profile.
   If needed, search that account for
   `from:me to:me after:2026/09/12 before:2026/09/14 has:attachment`.
   These are message metadata, not an OTP seed or a device-authorization link.
3. Verify sender, subject, and attachment name. Use the attachment viewer's
   Download control. Track the actual new download path; preserve any preexisting
   local file with the same name. Do not render the QR export into tool output.
   For UI inspection, crop screenshots to message headers/toolbars and delete
   any temporary full captures that include the QR.
4. Validate the export without printing secrets or codes:

   ```bash
   uv run --with zxing-cpp --with pillow python \
     /home/lorenzo/.codex/skills/cscs-auth/scripts/cscs_export_otp.py \
     --export-image /path/to/download.png --inspect
   ```

   The helper selects exactly one `CSCS` / `lterenzi` entry from a multi-account
   Google export. It supports the verified TOTP, SHA-1, six-digit, 30-second format
   and rejects unsupported parameters, wrong accounts, and ambiguous matches.
   Seeds and current codes remain in process memory. No re-import is needed for
   renewal. Import into an authenticator only when setup/import is requested.

## Complete login through the desktop UI

1. Keep one `cscs-key --headless sign -f EXISTING_KEY` process pending. Use its
   fresh device URL/code in the School Chrome profile. Select **ETH Zurich**.
   If necessary, select Chrome's saved **lterenzi** autofill entry on
   `https://aai-logon.ethz.ch` and click Login. Never reveal or extract its password.
2. Inspect the current **OTP authentication** page on `https://auth.cscs.ch`.
   Determine the OTP field and Log In button coordinates relative to the selected
   window using its current screenshot/geometry. Then run:

   ```bash
   uv run --with zxing-cpp --with pillow python \
     /home/lorenzo/.codex/skills/cscs-auth/scripts/cscs_export_otp.py \
     --export-image /path/to/download.png --window WINDOW_ID \
     --field FIELD_X FIELD_Y --submit BUTTON_X BUTTON_Y
   ```

   This verifies the foreground window and address-bar origin, generates a fresh
   code away from rollover, sends it over stdin to `xdotool`, and submits once.
   It temporarily uses and clears the clipboard for the page URL; the OTP never
   enters the clipboard, arguments, output, or a file. It still depends on your
   live inspection of the correct OTP form and coordinates. Avoid concurrent
   desktop interaction during this short step. Do not screenshot the filled code.
3. On the grant page, inspect the client and permissions. The verified pending
   `authx-cli` grant requests `openid`, user profile, and email. Accept it within
   the renewal request; do not ask Lorenzo to approve the same intended login.
   Do not accept an unrelated client or additional unexplained permissions.
4. Require the signing process to exit successfully and report the downloaded
   certificate. Inspect its interval, load it with `ssh-add -t 1d EXISTING_KEY`,
   and run the skill's Ela/Daint and Slurm checks. Browser success alone is not
   certificate or SSH success. Do not launch a test job.
5. Delete task-created export downloads and QR/code-bearing screenshots, including
   suffixed screenshot copies. Preserve the original Gmail message and any
   preexisting user file. Clear the attachment preview when finished.

If this route fails, report the specific state: signed-in personal mailbox missing,
export absent, unsupported/ambiguous export, rejected credentials, or SSH failure.
An unavailable CDP endpoint alone is not grounds to ask the user for the OTP.
