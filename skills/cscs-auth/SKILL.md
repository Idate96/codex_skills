---
name: cscs-auth
description: "Authenticate to CSCS through Chrome, renew existing SSH certificates, and verify cluster and Slurm access. Use for CSCS login, cscs-key reauthentication, job-access readiness, or importing the CSCS authenticator account."
---

# CSCS Authentication

Carry a renewal request through browser authentication, certificate download,
SSH-agent loading, and verification of the intended cluster. Lorenzo has requested
autonomous use of Chrome, saved ETH login, and his existing CSCS authenticator
export for this workflow. Do not ask him to enable debugging, locate a profile,
re-export an account, or supply an OTP before trying the documented recovery path.

Use the installed [chrome-cdp skill](../chrome-cdp/SKILL.md) when its endpoint is
available. When CDP is unavailable, use normal Chrome desktop interaction as
described in [Chrome and export recovery](references/chrome-export-recovery.md).
An empty extension in one profile does not establish that the exported account is
unavailable. Do not change organization policies or restart the user's browser.

## Execution context and first checks

- Check `hostname`, `/proc/1/comm`, `/.dockerenv`, and the requested SSH identity.
  The ROS container and its Starship host can both report `starship`; hostname
  alone does not identify the execution context. From the container, use the
  `ssh-starship` skill to reach `lorenzo@starship` for desktop work. No container
  restart or Tailscale profile switch is needed when that SSH route already works.
- Inspect the certificate before starting a flow. Keep the signing command and
  resulting certificate on the host/filesystem whose SSH identity is being renewed.
- Discover Chrome's current endpoint and profile. Port 9222 refusing a connection
  means the CDP helper is unavailable there; it does not mean Chrome is unavailable.
- Read [Chrome and export recovery](references/chrome-export-recovery.md) early if
  a saved login, browser profile, or local/exported OTP source is needed. It names
  the two verified profiles and the existing export message.

## Routine login

1. Inspect the requested login task before starting a new flow. For SSH access,
   `cscs-key --headless list` can show that an existing command-line session still
   works. Do not discard working sessions or generate/sign/revoke keys merely to
   test browser authentication. When the task is restoring SSH or job access,
   renew an expired certificate and complete the checks below before reporting ready.
2. Use the verification URL and initial device code from the user's actual
   pending CLI request. For a portal login, use `https://user-account.cscs.ch`.
3. With CDP available, run the bundled helper. It selects ETH Zurich and reads the
   CSCS OTP from Chrome. For an ETH password prompt, first use the saved `lterenzi`
   login through Chrome's normal autofill UI; do not extract saved passwords.
   The helper can prompt locally if no saved login or session is usable:

   ```bash
   python3 /home/lorenzo/.codex/skills/cscs-auth/scripts/cscs_browser_login.py \
     --url https://auth.cscs.ch/auth/realms/cscs/device --device-code DEVICE-CODE
   ```

   For an existing tab, use `--target TAB-ID` instead of `--url`; discover current
   IDs with chrome-cdp. `--inspect` reports page state without submitting.
   `--username` defaults to `lterenzi`. `--otp-export IMAGE` uses the existing
   Google Authenticator export when the extension is empty; invoke with
   `uv run --with zxing-cpp --with pillow python` for that option. `--manual-otp`
   is a last resort after the existing sources have been checked.
4. Verify the result: the portal's authenticated dashboard or the pending CLI's
   successful completion. The ETH password being accepted is only an intermediate
   step. A device authorization screen must correspond to the intended CLI task.
   The helper does not click unfamiliar confirmation/consent screens: inspect
   them through Chrome, confirm the client and requested action match the pending
   login, and complete that step within the user's authorization. Then verify the
   original CLI completes; a portal dashboard alone does not prove device approval.
   The observed `authx-cli` grant page requests `openid`, user profile, and email;
   its accept control is `input[name="accept"]`. Accept it when it belongs to the
   pending authorized CLI request. Do not treat unrelated client consent as covered.

The helper requires Python 3, Node.js 22+, chrome-cdp, and the approved Chrome CDP
endpoint (default local port 9222; override with `--port`). It discovers the
browser WebSocket endpoint and makes its own temporary port file. If the generic
chrome-cdp shell wrapper fails because nvm is absent but Node 22 is installed,
invoke its `scripts/cdp.mjs` with `node` as done by this helper.

## SSH and job-access readiness

Browser/CLI authentication, a valid SSH certificate, and cluster access are separate
checks. An authenticated `cscs-key --headless list` with an empty table means there
are no valid certificates listed; it does not mean SSH is ready.

For restoring authorized SSH/job access, inspect the configured identity with
`ssh -G CLUSTER` and inspect its certificate with `ssh-keygen -L -f CERTIFICATE`.
Renew the existing key's certificate when expired or absent:

```bash
cscs-key --headless sign
ssh-keygen -L -f /home/lorenzo/.ssh/cscs-key-cert.pub
```

The default identity is `~/.ssh/cscs-key`. If the target uses another identity,
pass that existing key's path with `sign --file`. Do not replace the private key.
Confirm the printed validity interval instead of assuming a fixed expiration.

After successful signing, load the same private key and its new certificate into
the current agent with `ssh-add -t 1d /home/lorenzo/.ssh/cscs-key` (substitute the
configured identity when different). This fixed the observed
`sign_and_send_pubkey: ... agent refused operation` error without another login.
Do not remove other identities or restart the agent. If a passphrase is required
and unavailable, report that specific blocker rather than renewing again.

Then check the intended cluster through its configured Ela jump host. Example
for Daint (replace the host and username for another requested target):

```bash
ssh -o BatchMode=yes -o ConnectTimeout=15 daint '
  hostname
  id -un
  command -v sbatch
  sinfo -h -o "%P %a %l"
  sacctmgr -nP show assoc where user=lterenzi format=Cluster,Account,Partition,QOS,DefaultQOS
'
```

Confirm the target login, `sbatch`, partition availability, and project association
from their outputs. Use a concrete project account rather than the `root`
association. These establish access readiness, not acceptance of a particular job
or immediate scheduling. Check resources, account, partition, and limits against
the actual job before submitting; do not launch a test job just to validate auth.

If a cluster denies login, test Ela separately to distinguish jump-host access
from cluster access:

```bash
ssh -o BatchMode=yes -o ConnectTimeout=10 ela 'hostname; id -un'
```

Ela success
does not imply access to every Alps cluster. For host-key errors, verify the host
identity; do not disable host-key checking. Respect an explicitly requested cluster
and report any alternate cluster as an alternative, not an equivalent success.

Report the specific cluster and account verified, certificate expiry if useful,
any unverified/failed target, and whether a job was actually submitted.

## OTP source and credential handling

- Extension ID: `bhghoamapcdpbohphigoooaddinpkbai` (authenticator.cc).
- Open its UI at
  `chrome-extension://bhghoamapcdpbohphigoooaddinpkbai/view/popup.html`.
  Chrome-cdp's `open` supports this; its HTTP-only `nav` command does not.
- Match exactly one displayed account with issuer `CSCS` and the requested
  username. Read only that entry's current six-digit code; do not dump extension
  storage or unrelated accounts. The helper waits away from the 30-second rollover.
- Use the CSCS code only on `https://auth.cscs.ch`. An OTP challenge on
  `https://aai-logon.ethz.ch` is a separate ETH factor and requires its own code.
- Passwords remain local prompts; existing session-provided credentials may be
  used within that authorization, including Chrome's saved-login UI. Do not copy passwords, OTP seeds, migration
  payloads, QR exports, or generated codes into skill files, logs, or agent prompts.
  The helper passes password/OTP values through a local Unix socket and assigns
  inputs in an origin-checked evaluation, avoiding focused-field typing races.

## Recovery and setup

- On credential rejection, stop. A user-supplied correction can authorize a new
  attempt; do not invent variants or loop on a rejected value.
- On an expired or stale ETH request, start again from the intended CSCS service
  or a fresh CLI verification URL, not the old ETH form URL.
- If the extension is locked, missing, empty, or ambiguous, do not submit a code
  from it. Follow [Chrome and export recovery](references/chrome-export-recovery.md)
  and select the exact CSCS account from the existing export instead. Do not
  re-import an existing account as a generic repair.
- Ask for help only if the documented sources are inaccessible/locked, the
  selected credentials are rejected, or an unfamiliar factor requires the user.
  Give the exact blocker and completed steps. Do not keep issuing new device
  requests while an authorized request is still usable.
- For an authorized first import or a stalled import dialog, read
  [references/authenticator-import.md](references/authenticator-import.md).

## Verified behavior

On 2026-09-13, importing only the CSCS entry from a multi-account Google export,
reading its code from Chrome, and submitting it after ETH login opened the CSCS
dashboard. The helper's Chrome-code lookup and wrong-account rejection were
checked live.

On 2026-09-14, the full pending `cscs-key --headless list` device flow completed:
device code, ETH login, Chrome CSCS OTP, and `authx-cli` access grant. The CLI
reported authentication success but listed no valid certificates. Signing the
existing key renewed the certificate; Ela and Daint SSH succeeded. Daint exposed
`sbatch`, an available `normal` partition, and project association `d130` with
`normal` QOS. Clariden denied login; Santis hit host-key verification failure.
No jobs were submitted. These are dated observations: recheck access, allocations,
partitions, and certificate validity for each new readiness request.

On 2026-09-15, the Starship desktop route completed renewal despite unavailable
CDP and an empty School-profile authenticator. Chrome's saved ETH login and the
existing personal-Gmail export supplied authentication; `ssh-add -t 1d` repaired
the agent refusal. Daint via Ela and Slurm account `d130` were verified. No jobs
were submitted. See the recovery reference for the reproducible route.
