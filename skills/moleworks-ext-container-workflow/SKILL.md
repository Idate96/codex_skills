---
name: moleworks-ext-container-workflow
description: "Run and debug IsaacLab train, play, or eval commands in the `moleworks_ext` container with persistent tmux logs. Use for bounded local container runs and false-hang diagnosis."
---

# Moleworks Ext Container Workflow

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
compose_files=(-f docker-compose.yaml)
[[ -f docker-compose.override.yaml ]] && compose_files+=(-f docker-compose.override.yaml)
docker compose --env-file .env.moleworks_ext-dev "${compose_files[@]}" up -d isaac-lab-ext-dev
```

## Standard Launch Pattern
1. Create or reuse a stable tmux session, then add a window:
```bash
SESSION=mwext
tmux has-session -t "$SESSION" 2>/dev/null || tmux new-session -d -s "$SESSION" -n shell
tmux new-window -t "$SESSION" -n mwext-run
```
2. Run the command inside the container:
```bash
tmux send-keys -t "$SESSION:mwext-run" "docker exec -it isaac-lab-moleworks_ext-dev bash -lc 'cd /workspace/moleworks_ext && <YOUR_COMMAND>'" C-m
```
3. Capture output right away:
```bash
tmux capture-pane -pt "$SESSION:mwext-run" -S -200
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
5. If logs are quiet for more than 60 seconds, inspect the pane process, CPU, and GPU activity before deciding it is hung. Isaac startup and compilation can be quiet. Stop only the scoped Isaac process after confirming it is inactive or failed, then relaunch `--headless` in the same tmux window.
