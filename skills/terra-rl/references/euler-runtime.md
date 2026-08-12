# Euler Runtime Reference

## Required Slurm Runtime Shape

Use only verified NVIDIA GeForce RTX 3090 or RTX 4090 allocations for new Terra RL training unless the user explicitly approves another GPU family. Generic `--gpus=N` requests are insufficient by themselves because they can land on a different GPU type; constrain the node family and verify the actual allocation with both Slurm and `nvidia-smi`.

Full default 3090 run:

```bash
#SBATCH -n 1
#SBATCH --cpus-per-task=4
#SBATCH --gpus=rtx_3090:4
#SBATCH --partition=gpu.24h
#SBATCH --time=24:00:00
#SBATCH --mem-per-cpu=8G
```

One-GPU 3090 comparison:

```bash
#SBATCH -n 1
#SBATCH --cpus-per-task=4
#SBATCH --gpus=rtx_3090:1
#SBATCH --partition=gpu.24h
#SBATCH --time=24:00:00
#SBATCH --mem-per-cpu=8G
```

On Euler, `#SBATCH --gpus=rtx_3090:N` resolves to
`gres/gpu:nvidia_geforce_rtx_3090=N` and may appear under `gpuhe.24h` in
`scontrol show job`. Verify with `scontrol show job JOBID | grep ReqTRES`.

## Runtime Exports

```bash
module load stack/2024-06 cuda/12.1.1

TERRA_EULER_USER=${TERRA_EULER_USER:-alesweber}
HOME_ROOT=/cluster/home/$TERRA_EULER_USER
SCRATCH_ROOT=/cluster/scratch/$TERRA_EULER_USER
PROJECT_ROOT=/cluster/project/rsl/$TERRA_EULER_USER
WORK=$SCRATCH_ROOT/codex_terra_edge_validation   # reproducible code snapshots
RUNS=$SCRATCH_ROOT/codex_terra_edge_runs         # W&B, checkpoints, run logs
VENV=${TERRA_REMOTE_VENV:-/cluster/project/rsl/lterenzi/terra_curriculum_20260730_c14bd7d_3ce0e84_py312_jax0426}
test "$(id -un)" = "$TERRA_EULER_USER"
test "$HOME" = "$HOME_ROOT"
test -w "$SCRATCH_ROOT" -a -x "$VENV/bin/python"
SITE_PACKAGES=$("$VENV/bin/python" - <<'PY'
import site
print(site.getsitepackages()[0])
PY
)

export PYTHONPATH="$WORK/terra:$WORK/terra-baselines:${PYTHONPATH:-}"
export XLA_PYTHON_CLIENT_PREALLOCATE=false
export LD_LIBRARY_PATH="$SITE_PACKAGES/nvidia/cudnn/lib:$SITE_PACKAGES/nvidia/cuda_cupti/lib:$SITE_PACKAGES/nvidia/cublas/lib:$SITE_PACKAGES/nvidia/cuda_nvrtc/lib:$SITE_PACKAGES/nvidia/nccl/lib:${LD_LIBRARY_PATH:-}"
export WANDB_ENTITY=aless-weber-eth
export WANDB_PROJECT=mixed-agents
export WANDB_DIR="$RUNS/wandb"      # scratch, NOT home — see Storage Targets below
mkdir -p "$WANDB_DIR"
export DATASET_PATH=/cluster/project/rsl/alesweber/TerraProject/terra/data/terra/train
export DATASET_SIZE=600
```

Point checkpoint output at `$RUNS` too (e.g. `--checkpoint_dir "$RUNS/checkpoints/$SLURM_JOB_ID"`). Symlink from the home workspace if a tool expects a local path: `ln -sfn "$RUNS" "$WORK/runs"`.

For jobs that hit cuDNN autotune failures, add:

```bash
export XLA_FLAGS="${XLA_FLAGS:+$XLA_FLAGS }--xla_gpu_autotune_level=0"
```

Expect a small steady-state throughput cost. Compare policy curves by update/W&B step unless both
variants use the same XLA flags.

## Storage Targets

Resolve home/scratch/project paths from the selected account and refresh
`lquota` before use. Current observations are from 2026-08-12 unless noted:

| Location | Quota | Purge | Use for Terra |
|---|---|---|---|
| `/cluster/home/$TERRA_EULER_USER` | 45 GB soft / **50 GB hard** | never | Credentials and small config only. `alesweber` was 37.36 GB after cache cleanup. |
| `/cluster/scratch/$TERRA_EULER_USER` | 2.5 TB soft / 2.7 TB hard | **files not accessed for ~15 days are deleted** | Code snapshots, inputs, W&B, checkpoints, run logs, transient caches. |
| `/cluster/project/rsl/$TERRA_EULER_USER` | account/group dependent | never | Long-term venvs and archives after live writability/quota checks. |
| `/cluster/work/rsl/$TERRA_EULER_USER` | account dependent | never | Optional large-file archive only when the path exists and is writable; absent for `alesweber` on 2026-08-12. |
| `$TMPDIR` (local node scratch) | up to 800 GB | at end of job | Stage dataset/container for a single run's I/O. |

Rules:

- Run artifacts (checkpoints, `WANDB_DIR`, logs) go under
  `/cluster/scratch/$TERRA_EULER_USER/codex_terra_edge_runs/`, NEVER home. On
  2026-07-20 a Terra job died with `Disk quota exceeded` because artifacts were
  written to home while it was at 44/50 GB.
- Reproducible code snapshots go under
  `/cluster/scratch/$TERRA_EULER_USER/codex_terra_edge_validation/`.
- Scratch is purged after ~15 days of no access. Anything needed long-term must
  be copied to verified writable project/work storage or be cheaply
  rebuildable. Do not keep the only copy of a final checkpoint on scratch.
- The Terra dataset already lives read-only at `/cluster/project/rsl/alesweber/TerraProject/...`; never copy it into home.

## Mandatory GPU Preflight

Run inside the allocation, after the exports above:

```bash
RUNTIME_CHECK="$WORK/terra-baselines/scripts/euler/check_jax_runtime.py"
test -f "$RUNTIME_CHECK"
"$VENV/bin/python" "$RUNTIME_CHECK" --min-devices 4
```

This catches the two important classes of failure that `jax.devices()` alone misses:

- missing NCCL, which only appears at pmap collective execution;
- broken cuDNN conv backward/autotune, which only appears during compiled model training.

## Monitoring Commands

```bash
squeue -j JOBID -o "%.18i %.9T %.12M %.24j %.20N %.24R"
sacct -j JOBID --format=JobID,JobName%28,State,ExitCode,Elapsed,Start,End,NodeList%18,AllocTRES%50 -P
tail -200 "$RUNS"/CAMPAIGN/runs/REVISION/PHASE/SEED/ARM/slurm_JOBID.out
```

Do not rely on `squeue` alone. A job can remain `RUNNING` after a runtime error if the Python
process is still alive. If the log contains a fatal runtime error, the process is sleeping, and
GPU utilization is 0%, cancel and relaunch after a targeted smoke.

## Checkpointed Wall-Time Continuation

Use 24-hour and 120-hour allocations as resumable compute segments. Set the
trainer's absolute update target above the work expected to fit in one segment,
and checkpoint every 100--500 updates. Do not depend on a `FINAL` checkpoint:
a hard Slurm timeout skips all shell commands after the trainer.

After a timeout:

1. locate the latest complete checkpoint and verify finite parameters,
   optimizer state, `train_state_step`, `next_update`, and adaptive sampler
   state when enabled;
2. run fixed evaluation separately if the timed-out job could not do so;
3. relaunch the unchanged treatment with `--resume_from CHECKPOINT` and an
   absolute final `total_timesteps` target greater than `next_update`; and
4. resume the same W&B run with `WANDB_RESUME=must` only if its last logged
   update does not exceed the checkpoint; otherwise start a linked continuation
   run with the same absolute `train/update` axis.

Classify a wall-time exit with a valid checkpoint as `CONTINUABLE`. Classify it
as failed only when no valid checkpoint exists or the runtime/finite/integrity
contract failed. A soft stop before wall time is cleaner, but is not required
for research continuity when rolling checkpoints are frequent.

For allocated-node GPU state:

```bash
srun --jobid=JOBID --overlap -N1 -n1 bash -lc \
  'hostname; echo CUDA_VISIBLE_DEVICES=$CUDA_VISIBLE_DEVICES; nvidia-smi --query-gpu=index,name,memory.used,utilization.gpu --format=csv,noheader'
```

## W&B Interpretation

Check both W&B state and history:

```python
import wandb
api = wandb.Api()
run = api.run("aless-weber-eth/mixed-agents/RUN_ID")
print(run.state, run.summary.keys())
print(len(list(run.scan_history(page_size=1000))))
```

If state is `crashed` and summary/history are empty, the run produced no training evidence.

## Known Failure Signatures

- `Unable to load NCCL library. Multi-GPU collectives will not work.`:
  install `nvidia-nccl-cu12==2.19.3` and include `$SITE_PACKAGES/nvidia/nccl/lib` in
  `LD_LIBRARY_PATH`.
- `CUDNN_STATUS_INTERNAL_ERROR` in `convBackwardInput`:
  validate cuDNN with the runtime preflight, then try `XLA_FLAGS=--xla_gpu_autotune_level=0` for
  a smoke before full training.
- Mask/no-mask graph differences can affect JAX/XLA graph shape enough for cuDNN autotune to pick
  different convolution algorithms. If only one side of an A/B fails, treat it as a runtime/compiler
  issue until a first-update smoke proves otherwise.
- `Training JAX devices: [CpuDevice(id=0)]`:
  CUDA libraries or module setup are wrong; do not train.
- Job stuck before update 1:
  record as compile/runtime investigation, not a training result.
- Import fails on an empty `jax` (or similar) namespace package when the venv lives on
  `/cluster/scratch`: the scratch purge deleted files not accessed for ~15 days and left a hollow
  package tree. Rebuild the venv, and keep long-lived venvs on verified project storage.
- `Disk quota exceeded` / `OSError: [Errno 122]` while writing checkpoints or W&B files: home
  (50 GB hard) is full because run artifacts were pointed at `/cluster/home`. Repoint `WANDB_DIR`
  and checkpoint dirs to `/cluster/scratch/$TERRA_EULER_USER/codex_terra_edge_runs/`; check `lquota`.
