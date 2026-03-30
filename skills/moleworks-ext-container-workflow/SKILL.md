---
name: moleworks-ext-container-workflow
description: Launch and debug Moleworks IsaacLab commands inside the moleworks_ext Docker container with persistent tmux windows and captured output. Use when starting containerized training/play/eval scripts and when validating command startup.
---

# Moleworks Ext Container Workflow

## When To Use
Use this skill when you need to run or debug commands inside `isaac-lab-moleworks_ext-dev`, especially for:
- `scripts/rsl_rl/train.py`, `scripts/rsl_rl/play.py`, or similar IsaacLab entrypoints
- local smoke tests before longer runs
- any workflow where persistent logs in tmux are required

## Non-Negotiables
- Run IsaacLab scripts via `/workspace/isaaclab/isaaclab.sh -p`.
- Launch from a new tmux window so output remains inspectable.
- Use reliable tmux command dispatch:
  - either one `send-keys ... C-m` per command
  - or a single `bash -lc '...; ...'` string
- Capture pane output immediately after launch (`tmux capture-pane -p ...`).
- Reuse long-running command sessions/panes instead of spawning many short-lived process handles.
  This avoids hitting the unified exec process limit during long debug loops.
- Before killing/restarting Isaac, verify the GPU owner first:
  - `nvidia-smi`
  - `ps -fp <pid>` for each suspicious CUDA PID
  - Only kill the actual Isaac kit process for this workflow, not unrelated CUDA jobs.
- After cleanup, re-run `nvidia-smi` and confirm VRAM dropped.
  Killing an outer `docker exec` or `timeout` can leave the inner Isaac kit child alive.

## Preflight
1. Verify the container is up:
```bash
docker ps --format '{{.Names}}\t{{.Status}}' | rg isaac-lab-moleworks_ext-dev
```
2. If it is not running, start it:
```bash
cd ~/moleworks/moleworks_ext/docker
docker compose --env-file .env.moleworks_ext-dev \
  -f docker-compose.yaml -f docker-compose.override.yaml \
  up -d isaac-lab-ext-dev
```

## Standard Launch Pattern
1. Create a tmux window in your current session:
```bash
tmux new-window -n mwext-run
```
2. Run the command inside the container:
```bash
tmux send-keys -t mwext-run "docker exec -it isaac-lab-moleworks_ext-dev bash -lc 'cd /workspace/moleworks_ext && <YOUR_COMMAND>'" C-m
```
3. Capture output right away:
```bash
tmux capture-pane -pt mwext-run -S -200
```

## Smoke-Test Templates
Disable external logging for quick local checks:
```bash
export WANDB_MODE=disabled
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --task Moleworks-Isaac-m445-digging-3D-w-cabin \
  --num_envs 4 \
  --max_iterations 3 \
  --headless
unset WANDB_MODE
```

Full tmux + container invocation:
```bash
tmux new-window -n mwext-smoke
tmux send-keys -t mwext-smoke "docker exec -it isaac-lab-moleworks_ext-dev bash -lc 'cd /workspace/moleworks_ext && export WANDB_MODE=disabled && /workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py --task Moleworks-Isaac-m445-digging-3D-w-cabin --num_envs 4 --max_iterations 3 --headless'" C-m
tmux capture-pane -pt mwext-smoke -S -200
```

## Quick Triage
- If startup fails, recapture deeper logs:
```bash
tmux capture-pane -pt <window> -S -400
```
- If path-dependent files are missing, include:
```bash
export MOLEWORKS_ROOT=/home/lorenzo/moleworks
```
inside the `bash -lc` command.
- If imports fail, confirm command uses `isaaclab.sh -p` (not plain `python`).

## False-Hang Triage (Important)
Sometimes runs look "hung" but are actually script exceptions masked by shutdown.

1. Add explicit progress prints in the script around:
   - `gym.make`
   - `env.reset`
   - fixed-state setters
   - observation compute
   - `torch.save`
2. Temporarily skip app shutdown to expose real exceptions:
```bash
export MW_SKIP_APP_CLOSE=1
```
3. Relaunch in tmux and capture output right after:
```bash
tmux capture-pane -pt <window> -S -300
```
4. If you see `torch.save:done` and `saved=...`, the run is healthy even if teardown logs warnings.
5. If no new logs appear for >60s, stop with `C-c`, then relaunch `--headless` in the same tmux window.
