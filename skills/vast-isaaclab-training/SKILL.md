---
name: vast-isaaclab-training
description: "Operate billable Vast.ai IsaacLab/UGEP runs. Use for instance selection, runtime hydration, W&B training, monitoring, sync, billing cleanup, and experiment ledgers."
---

# Vast IsaacLab Training

## Core Rule

Treat live Vast.ai actions as billable infrastructure changes. Before creating,
stopping, destroying, or modifying an instance, identify the target instance and
separate it from unrelated running work.

Status, inventory, recommendation, and command-preparation requests are read-only. Creating or
modifying an instance, launching paid training, stopping, and destroying are distinct actions; perform
only the stages the user requested. Cleanup of a newly created failed task instance is allowed within
an authorized launch workflow to prevent accidental billing, with the result reported.

For command details and known Moleworks paths, read
`references/vast-isaaclab-runbook.md` before taking live Vast actions.

## Workflow

1. Load the repo context.
   - Start from `/home/lorenzo/moleworks/moleworks_ext` unless the user names a different checkout.
   - Read repo `AGENTS.md` and relevant docs before changing code or launching jobs.
   - Verify the selected branch contains `docs/VAST_INSTANCES.md`, `docs/EXPERIMENTS_ONGOING.md`, and the `docker/cluster/*_vast.sh` helpers. If the default checkout does not, locate a compatible active experiment worktree with `git worktree list`; do not combine ledgers and scripts from different branches or create replacement ledgers on the wrong branch.
   - Check those ledgers for live state before acting.

2. Audit Vast state.
   - Load the Vast API key from `~/vast_key` without printing it.
   - Run `vastai show instances` and preserve unrelated instances.
   - Prefer verified direct-SSH offers, sufficient disk, and a conservative price over suspicious cheapest offers.

3. Create or select the instance.
   - Use the GPU count requested by the user. If the request does not determine an existing instance,
     GPU type/count, or acceptable cost envelope, ask before creating billable infrastructure.
   - Use at least 120 GB disk for Moleworks Ext / UGEP unless the user says otherwise.
   - Record the instance immediately in `docs/VAST_INSTANCES.md`.

4. Hydrate and verify the runtime.
   - Update `docker/cluster/.env.vast` for the target host, port, key, image, and W&B defaults.
   - Run `docker/cluster/sync_code_vast.sh`.
   - Run `docker/cluster/setup_vast_env.sh`.
   - Sync UGEP assets explicitly if fast sync excluded large asset directories.
   - Verify GPU visibility, CUDA/Torch, IsaacLab wrapper, and `moleworks_ext` import.

5. Smoke before real training.
   - Run a bounded W&B-disabled smoke first, typically 16 envs and 3 iterations.
   - Monitor the process log until it reaches RSL-RL startup or learning iterations.
   - Fix scoped compatibility issues if they are clear; otherwise stop before launching a longer job.

6. Start the W&B run.
   - Use W&B entity/user `idate96` and project `ugep` unless the user overrides it.
   - Load `WANDB_API_KEY` from `~/.netrc` without printing it.
   - Use a bounded run first on new infrastructure, then scale once stability is proven.
   - Capture remote PID, remote log path, W&B id, W&B URL, run dir, and current iteration evidence.

7. Keep ledgers live.
   - Update `docs/VAST_INSTANCES.md` after instance create/stop/destroy and after each run launch.
   - Update `docs/EXPERIMENTS_ONGOING.md` when a training job is submitted and while it is live.
   - Sync logs with `docker/cluster/sync_logs_vast.sh` before relying on local evidence.

8. Billing cleanup.
   - Do not stop unrelated instances.
   - Do not destroy any instance unless the user explicitly asks or has already approved that cleanup.
   - Provide exact reconnect, tail, log-sync, and stop commands in the final response.

## Failure Handling

- If an image pull fails, record the failed instance and stop it promptly.
- If a smoke fails before training, inspect the log, fix only narrowly scoped setup or compatibility issues, and rerun one bounded smoke.
- If the real run starts but later fails, sync logs, update ledgers, and report the failure with the first actionable traceback.
- Never paste API keys, W&B tokens, Vast instance API keys, or private SSH material into docs or chat.
