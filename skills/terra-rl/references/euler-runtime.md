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

WORK=/cluster/home/lterenzi/codex_terra_edge_validation
VENV=/cluster/scratch/lterenzi/codex_terra_edge_venv
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
export WANDB_DIR="$WORK/wandb"
export DATASET_PATH=/cluster/project/rsl/alesweber/TerraProject/terra/data/terra/train
export DATASET_SIZE=600
```

For jobs that hit cuDNN autotune failures, add:

```bash
export XLA_FLAGS="${XLA_FLAGS:+$XLA_FLAGS }--xla_gpu_autotune_level=0"
```

Expect a small steady-state throughput cost. Compare policy curves by update/W&B step unless both
variants use the same XLA flags.

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
tail -200 /cluster/home/lterenzi/codex_terra_edge_validation/logs/JOBID_*.out
```

Do not rely on `squeue` alone. A job can remain `RUNNING` after a runtime error if the Python
process is still alive. If the log contains a fatal runtime error, the process is sleeping, and
GPU utilization is 0%, cancel and relaunch after a targeted smoke.

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
