---
name: supercluster-compute
description: Safely operate the shared `rsl@supercluster` workstation for GPU evaluation, simulation, compilation, checkpoint analysis, and data transfer. Use when a request names Supercluster, asks to keep Lorenzo's primary GPU free, needs a local RTX GPU (GPU 0 only), requires a tmux-managed long diagnostic, or needs account-scoped transfer between Supercluster and Euler.
---

# Supercluster Compute

Use Supercluster as a shared workstation, not disposable CI. Inspect live state
before allocating resources, isolate the task, batch useful work per GPU, and
remove only processes and files created by the current task.

## Connect and establish identity

Connect with the canonical route:

```bash
ssh -o BatchMode=yes rsl@supercluster
hostname
id -un
```

Require `hostname=supercluster` and `user=rsl`. Do not silently substitute
`rslpc`, another account, or another Tailscale host.

If `ssh` fails with `Permission denied (publickey,password)` while it worked
before, the default agent socket has usually died (`ssh-add -l` says
`Connection refused`). Use the GNOME keyring agent:
`export SSH_AUTH_SOCK=/run/user/1000/gcr/ssh`.

Inspect live state before any write or launch:

```bash
uptime
nproc
free -h
nvidia-smi --query-gpu=index,name,memory.total,memory.used,utilization.gpu,temperature.gpu --format=csv,noheader
nvidia-smi pmon -c 1
tmux ls 2>/dev/null || true
ps -eo user,pid,etimes,pcpu,pmem,args --sort=-pcpu | head -30
df -h /home /tmp
```

Expected hardware as of 2026-08-13, which must still be verified live:

- two NVIDIA RTX 3060 GPUs with 12 GiB each;
- 56 logical CPUs from two Xeon E5-2680 v4 sockets;
- 125 GiB RAM;
- about 1.8 TiB local NVMe root storage.

GPU rule (Lorenzo, 20 October 2026): use GPU 0 only, and only when it has no
other compute process (`nvidia-smi -i 0 --query-compute-apps=pid
--format=csv,noheader` is empty); never use GPU 1. Put this rule into every
subagent brief that could reach Supercluster. Never stop Xorg, GNOME, desktop
services, or an unknown GPU process.

## Isolate the task

Use task-specific locations rather than modifying a canonical checkout:

```text
/home/rsl/moleworks/.worktrees/<repo>-<task>-<date>
/home/rsl/moleworks/.venv-<task>
/home/rsl/moleworks/<project>_runs/<task>_<date>
```

Pin and record repository revisions before execution. Use a named tmux session
and unbuffered logs:

```bash
tmux new-session -d -s <task-name> \
  "bash -lc 'PYTHONUNBUFFERED=1 <command> > <artifact-root>/<task>.log 2>&1'"
```

Do not edit or clean an existing dirty checkout, reuse an unrelated virtual
environment blindly, prune Docker, reboot the host, or stop unknown work.

Source trees on Supercluster:

- There is no GitHub access and no `git-lfs`. Bring sources as a synced tree
  (rsync from the workstation) or as git objects: `git bundle` for full
  history, or `git push --no-verify rsl@supercluster:<repo> <branch>:refs/heads/<name>`
  (`--no-verify` skips the LFS pre-push hook). A shallow local repository
  pushes only after `git config receive.shallowUpdate true` in the target.
- A `git worktree` or checkout made on Supercluster holds LFS pointer files
  for meshes and other LFS assets, and env construction then fails with a
  parse error (for example `collada ... XML Parsing Error`). Copy the real
  files from a synced tree for every file whose first line is
  `version https://git-lfs.github.com/spec/v1`.
- Evaluators that record provenance (for example
  `benchmark_fee_excavation.py` and `evaluate_carved_chain_state_bank.py`)
  require `moleworks_newton`, `newton` and `rsl_rl` to be git checkouts. Make a
  synced tree a repository at its exact commit: `git init`, fetch the commit
  from a bundle or push, then `git reset <commit>` (index only), and confirm
  `git status --porcelain` is empty.

## Allocate GPUs efficiently

Choose the smallest allocation that answers the question:

- One workload: `CUDA_VISIBLE_DEVICES=0` once GPU 0 is free.
- Several workloads: queue them in one script that waits for a free GPU 0
  before each run, rather than sharing the GPU or using GPU 1.
- A multi-GPU job: only if Lorenzo explicitly lifts the GPU 1 rule for it.

Batch vectorized environments or examples inside each process. Do not launch
one process per map when one batched policy call can evaluate the set. Record
the environment batch and inference chunk separately: chunking may be needed
for memory/backend limits, and bfloat16 greedy decisions can change with batch
or compilation shape.

For matched comparisons:

- keep checkpoint, map order, seed, model mode, and inference chunk identical;
- run the arms one after another on GPU 0;
- verify action/state prefixes when one run is a horizon counterfactual;
- avoid interpreting cross-device bit differences as treatment effects.

JAX first-step compilation is often CPU-bound. During compilation, a healthy
process may show about one busy CPU core, stable multi-GiB GPU allocation, and
near-zero instantaneous GPU utilization for several minutes. Check process
state, RSS, log progress, and elapsed time before declaring a hang. Do not
discard a healthy compile merely to change batching. After compilation,
confirm both GPU utilization and application-level progress.

## Keep runs observable and recoverable

Monitor without attaching destructively:

```bash
tmux capture-pane -p -t <task-name> -S -100
tail -100 <artifact-root>/<task>.log
nvidia-smi --query-gpu=index,memory.used,utilization.gpu,power.draw --format=csv,noheader
ps -p <owned-pid> -o pid,etime,pcpu,rss,stat,args
```

Write results to a task artifact root with source revisions, input hashes,
batch/chunk shape, GPU identity, and a fail-loud receipt. Preserve failed and
partial logs as provenance. When superseding a diagnostic, terminate only its
exact owned tmux session or PID:

```bash
tmux kill-session -t <owned-task-name>
```

Never use broad commands such as `pkill python`, kill every CUDA process, or
delete another task's artifact directory.

## Transfer to and from Euler

Use the active Euler account `lterenzi` and verify its identity:

```bash
ssh -o BatchMode=yes euler-lterenzi 'id -un; hostname'
```

The former Weber account is closed. Use `euler-lterenzi` for new transfers.
Keep account-owned keys, multiplexing sockets, and storage paths consistent.
Create or install a scoped key only when authorized; never reuse or expose a
private key in logs, prompts, or subagent messages. Before copying, verify the
source hash, destination account, destination path, free space, and resulting
hash. Prefer direct Euler-to-Supercluster transfer when it avoids occupying
Lorenzo's primary workstation.

Treat scheduler state, run directories, and checkpoints as volatile. Refresh
them live rather than relying on a prior note.

## Handoff

Report:

- exact host/user and repository revisions;
- GPU allocation, batch shape, and inference chunk;
- tmux session and artifact/log paths;
- which processes were stopped, if any;
- receipt and input hashes;
- whether work is compiling, running, complete, failed, or merely queued.

Do not call a quiet compile complete, a finite smoke scientifically valid, or
a queued cluster job running without the corresponding terminal evidence.
