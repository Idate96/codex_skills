---
name: rl-isaaclab
description: "End-to-end IsaacLab RL workflow in moleworks_ext: run with isaaclab.sh -p, local smoke tests, cluster submit/monitor/debug on Euler, sync results, and play policies."
---

# RL + IsaacLab Workflow (moleworks_ext)

Single-skill workflow for IsaacLab-based RL: local smoke test → cluster submit → monitor/debug → sync → playback.

## 1) Always Run IsaacLab Scripts via Wrapper

**IMPORTANT**: Any script that touches Isaac Sim/Lab must use:

```bash
/workspace/isaaclab/isaaclab.sh -p <script.py> [args]
```

Use this for:
- Isaac Sim/Lab imports
- USD/PhysX
- GPU/RTX
- training/testing scripts

### pxr Import Errors

If you hit `ImportError: No module named 'pxr'`, load the app launcher **inside the file**:

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom
```

## 2) Local Smoke Test (ALWAYS first)

Disable W&B for local smoke tests to avoid junk runs.

```bash
export WANDB_MODE=disabled

/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --task <TASK> --num_envs 4 --max_iterations 3 --headless

unset WANDB_MODE
```

Example (excavation3d_w_cabin):

```bash
export WANDB_MODE=disabled
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --task Moleworks-Isaac-m445-digging-3D-w-cabin --num_envs 4 --max_iterations 3 --headless
unset WANDB_MODE
```

## 3) Submit to Euler Cluster

```bash
JOB_TIME=30m NUM_GPUS=2 GPU_TYPE=rtx_3090 ./docker/cluster/cluster_interface.sh job \
  --task <TASK> --num_envs 64000 --max_iterations 10000
```

Example (excavation3d_w_cabin, 24h, 1 GPU):

```bash
JOB_TIME=24h NUM_GPUS=1 GPU_TYPE=rtx_3090 ./docker/cluster/cluster_interface.sh job \
  --task Moleworks-Isaac-m445-digging-3D-w-cabin --num_envs 64000 --max_iterations 10000
```

Custom entrypoint (optional):

```bash
CLUSTER_EXECUTABLE=scripts/mole_environments/grading/train_grading.py \
  JOB_TIME=2h NUM_GPUS=2 GPU_TYPE=rtx_3090 ./docker/cluster/cluster_interface.sh job \
  --task Moleworks-Isaac-m445-grading --num_envs 64000 --max_iterations 10000
```

## 4) Monitor Jobs (Euler)

```bash
ssh euler 'squeue -u $USER'
```

Job states:
- RUNNING: executing
- PENDING: waiting on resources
- FAILED/NODE_FAIL: crashed

## 5) Debug Failed Jobs

```bash
ssh euler 'find /cluster/scratch/$USER -name "slurm-*.out" -mmin -60'
ssh euler 'tail -100 /cluster/scratch/<user>/moleworks_ext_<timestamp>/slurm-<jobid>.out'
ssh euler 'cat /cluster/scratch/<user>/moleworks_ext_<timestamp>/slurm-<jobid>.err'
ssh euler 'tail -50 /cluster/scratch/<user>/moleworks_ext_<timestamp>/slurm-<jobid>.err'
ssh euler 'grep -n "bind mount detected" /cluster/scratch/<user>/moleworks_ext_<timestamp>/slurm-<jobid>.out | head'
```

**Note**: `.err` is where Python tracebacks are.

## 6) Sync Results

```bash
./docker/cluster/sync_experiments.sh
# or to save space
./docker/cluster/sync_experiments.sh --remove
```

## 7) Play Policy Locally

```bash
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task <TASK> \
  --checkpoint logs/rsl_rl/<exp>/<run_dir>/model_150.pt \
  --num_envs 1
```

## RL Debugging Tips

- Init order matters: buffers must exist before ObservationManager uses them.
- CurriculumManager uses `compute()` (EventManager uses `apply()`).
- Keep rewards bounded: `exp(-penalty)` keeps in (0,1].
- Reference `excavation3d_w_cabin` for established API patterns.
- If behavior is odd: check reward farming, velocity distribution, and speed penalties.

## tmux Requirement

When running IsaacLab scripts or ROS commands, open a **new tmux window** so output persists:

```bash
tmux new-window -n rl-isaaclab
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py --task <TASK> --num_envs 4

tmux capture-pane -p
```
