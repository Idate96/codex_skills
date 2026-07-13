# Vast IsaacLab Runbook

Use this reference after `vast-isaaclab-training` triggers and before live Vast
actions.

## Local Paths

- Default repo: `/home/lorenzo/moleworks/moleworks_ext`
- Experiment checkout: use one active worktree whose branch contains both
  ledgers and all listed `docker/cluster/*_vast.sh` helpers. The default
  checkout may predate the ledgers, while a newer worktree may omit the
  helpers; verify with `git worktree list` and file existence before any live
  action. Do not combine files from different worktrees.
- Vast API key file: `/home/lorenzo/vast_key`
- SSH private key usually registered with Vast: `/home/lorenzo/.ssh/cscs-key`
- Cluster env file: `docker/cluster/.env.vast`
- Instance ledger: `docs/VAST_INSTANCES.md`
- Live experiment ledger: `docs/EXPERIMENTS_ONGOING.md`
- Vast helpers:
  - `docker/cluster/sync_code_vast.sh`
  - `docker/cluster/setup_vast_env.sh`
  - `docker/cluster/submit_job_vast.sh`
  - `docker/cluster/status_job_vast.sh`
  - `docker/cluster/sync_logs_vast.sh`

## Authentication

Do not print secrets.

```bash
vastai set api-key "$(tr -d '\n\r ' < /home/lorenzo/vast_key)"
vastai show user
vastai show instances
```

Load W&B only for the submit command:

```bash
WANDB_API_KEY="$(awk '$1=="machine" && $2=="api.wandb.ai"{f=1} f&&$1=="password"{print $2; exit}' ~/.netrc)" \
WANDB_USERNAME=idate96 \
WANDB_ENTITY=idate96 \
WANDB_MODE=online \
docker/cluster/submit_job_vast.sh ...
```

## Offer Selection

For UGEP / Moleworks Ext trials:

- GPU: user requested count, usually 1x RTX 3090 for trial or 2x RTX 3090 for
  larger runs.
- Disk: at least 120 GB.
- Prefer verified direct-SSH offers.
- Avoid offers marked unreliable, deverified, no direct ports, or suspiciously
  constrained on RAM/disk/network.
- Record price, location, disk, image, instance id, SSH endpoint, and workspace
  persistence.

## Image Choices

Known working fresh path from 2026-06-22:

- `nvcr.io/nvidia/isaac-lab:2.3.2`
- IsaacSim 5.1 / IsaacLab 2.3.2 layout
- Requires synced `moleworks_ext`, sibling `rsl_rl`, and UGEP assets.

Known failed path from 2026-06-22:

- `rslheap/isaac-lab-ros2-isaacsim5:latest`
- Vast could not pull it: `pull access denied`.
- Stop failed instances promptly and record them.

## `.env.vast` Essentials

Update `docker/cluster/.env.vast` for the selected instance:

```bash
CLUSTER_TYPE=vast
VAST_SSH_HOST=<host>
VAST_SSH_PORT=<port>
VAST_SSH_USER=root
VAST_SSH_KEY=/home/lorenzo/.ssh/cscs-key
DOCKERHUB_IMAGE=nvcr.io/nvidia/isaac-lab
DOCKERHUB_TAG=2.3.2
VAST_ISAACLAB_DIR=/workspace/isaaclab
VAST_ISAACLAB_SH=/workspace/isaaclab/isaaclab.sh
VAST_SYNC_RSL_RL=1
VAST_WANDB_BY_DEFAULT=1
VAST_WANDB_PROJECT=ugep
WANDB_USERNAME=idate96
WANDB_ENTITY=idate96
WANDB_MODE=online
```

## Hydration Checks

From repo root:

```bash
docker/cluster/sync_code_vast.sh
docker/cluster/setup_vast_env.sh
```

If UGEP assets were excluded by fast sync, explicitly sync the needed asset
trees, for example:

```bash
rsync -az --partial -e "ssh -p <port> -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o IdentitiesOnly=yes -o CertificateFile=none -i /home/lorenzo/.ssh/cscs-key" \
  source/moleworks_ext/moleworks_ext/rsc/ugep/m445_teles \
  root@<host>:/workspace/moleworks/moleworks_ext/source/moleworks_ext/moleworks_ext/rsc/ugep/
rsync -az --partial -e "ssh -p <port> -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o IdentitiesOnly=yes -o CertificateFile=none -i /home/lorenzo/.ssh/cscs-key" \
  source/moleworks_ext/moleworks_ext/rsc/ugep/real_machines \
  root@<host>:/workspace/moleworks/moleworks_ext/source/moleworks_ext/moleworks_ext/rsc/ugep/
```

## Smoke Command Pattern

Use W&B disabled / TensorBoard first:

```bash
RUN_ID="ugep_vast_1x3090_official_smoke_$(date -u +%Y%m%dT%H%M%SZ)"
docker/cluster/submit_job_vast.sh \
  --task Moleworks-Isaac-ugep-v0 \
  --num-envs 16 \
  --gpus 1 \
  --time 20m \
  --run-id "$RUN_ID" \
  -- \
  --max_iterations 3 \
  --runtime-mode train \
  --seed 214 \
  --run_name "$RUN_ID" \
  --logger tensorboard \
  --ppo_preset newton_fee \
  --ugep_asset_set real_machines_m445 \
  --ugep_finish_profile g21_simple_plus_rw8_e10_x25_soilfill140_full120_pcomp_neg250_nopartial_tassist150_trk040_tau080_tbar300_ramp10_termtau300 \
  --ugep_actuator_model newton_nominal \
  --ugep_observation_history newton_g21_task_space_5 \
  --ugep_no_auto_morphology_obs \
  --ugep_morphology_obs none
```

Success evidence:

- Process exits cleanly or reaches learning iterations.
- Run dir under `/workspace/logs/moleworks_ext/moleworks_ext_logs/rsl_rl/...`.
- TensorBoard event file and checkpoint such as `model_0.pt` exist.

## W&B Training Command Pattern

Conservative first run on one RTX 3090:

```bash
RUN_ID="g21_realmachines_m445_vast1x3090_official_seed214_env1024_it200_$(date -u +%Y%m%dT%H%M%SZ)"
WANDB_API_KEY="$(awk '$1=="machine" && $2=="api.wandb.ai"{f=1} f&&$1=="password"{print $2; exit}' ~/.netrc)" \
WANDB_USERNAME=idate96 \
WANDB_ENTITY=idate96 \
WANDB_MODE=online \
docker/cluster/submit_job_vast.sh \
  --task Moleworks-Isaac-ugep-v0 \
  --num-envs 1024 \
  --gpus 1 \
  --time 4h \
  --run-id "$RUN_ID" \
  -- \
  --max_iterations 200 \
  --runtime-mode train \
  --seed 214 \
  --run_name "$RUN_ID" \
  --logger wandb \
  --log_project_name ugep \
  --ppo_preset newton_fee \
  --ugep_asset_set real_machines_m445 \
  --ugep_finish_profile g21_simple_plus_rw8_e10_x25_soilfill140_full120_pcomp_neg250_nopartial_tassist150_trk040_tau080_tbar300_ramp10_termtau300 \
  --ugep_actuator_model newton_nominal \
  --ugep_observation_history newton_g21_task_space_5 \
  --ugep_no_auto_morphology_obs \
  --ugep_morphology_obs none
```

Monitor:

```bash
ssh -p <port> -i /home/lorenzo/.ssh/cscs-key -o IdentitiesOnly=yes -o CertificateFile=none root@<host> \
  'tail -f /workspace/logs/moleworks_ext/process_logs/vast-<run-id>.log'
```

Extract W&B:

```bash
grep -Ei 'wandb|View run|View project' /workspace/logs/moleworks_ext/process_logs/vast-<run-id>.log
```

## Log Sync

```bash
docker/cluster/sync_logs_vast.sh
```

Local destination:

```text
logs/vast/<host>_<port>/
```

## Billing Controls

Stop compute while preserving instance state:

```bash
vastai stop instance <instance_id>
```

Destroy only after explicit user approval and log/checkpoint sync:

```bash
vastai destroy instance <instance_id>
```

Inside-instance fallback if the local CLI is unavailable:

```bash
ssh -p <port> root@<host> \
  'vastai stop instance "$CONTAINER_ID" --api-key "$CONTAINER_API_KEY"'
```

## Known IsaacLab 2.3.2 Compatibility Issues

Fix only when the log confirms the issue.

- `isaaclab.utils.io.dump_pickle` may be absent. Use a local `pickle.dump`
  helper in `scripts/rsl_rl/train.py`.
- `Articulation.has_external_wrench` is replaced by wrench composers. For the
  Moleworks global-force subclass, mirror upstream `write_data_to_sim` but pass
  `is_global=True`.
- Ray caster helpers may have moved from `isaaclab.sensors.utils` to
  `isaaclab.sensors.ray_caster.ray_cast_utils`; verify against the installed
  image before patching.
