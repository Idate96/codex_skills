# Terra W&B Success Calibration

Read this reference when comparing Terra policies against historical W&B runs or replaying legacy checkpoints.

## W&B Success Calibration

Use `aless-weber-eth/mixed-agents` as the historical comparison project. Prefer
`solo_excavator` runs as the apples-to-apples baseline; `trench_excavator` and
`trench_masked_excavator` are useful upper-bound references but use different task configs.

Closest successful `solo_excavator` references:

- `o9aewzsx`: 4 GPUs, 1024 envs/device, `eval/positive_terminations=9.56`,
  `eval/rewards=0.174`, `eval/max_reward=6.87`, `eval/DO=0.343`, `DO_NOTHING ~= 0`.
- `hcwvorkm`: 4 GPUs, 1024 envs/device, `eval/positive_terminations=3.45`,
  `eval/rewards=0.157`, `eval/max_reward=6.87`, `eval/DO=0.311`, `DO_NOTHING=0.018`.
- `o8bpdoex`: 4 GPUs, 128 envs/device, `eval/positive_terminations=4.01`,
  `eval/rewards=0.180`, `eval/max_reward=6.87`, `eval/DO=0.316`, `DO_NOTHING=0.011`.
  This run started with nonzero success, so treat it as a performance reference, not a clean
  from-scratch learning curve.

Strong non-solo references: `xjncfmr6`, `jnwj5tnj`, and `rov40bmt` reached
`eval/positive_terminations ~= 26-31`, `eval/rewards ~= 0.37-0.43`,
`eval/max_reward ~= 6.87`, and `eval/DO ~= 0.33-0.35`.

Historical replay compatibility:

- Do not judge old healthy policies only in the current `multi-agent` worktrees. `hcwvorkm`
  fails under current Terra replay (`0` positive terminations in a 64-env local check) even with
  action masking disabled, but replays healthily with training-era code.
- For `hcwvorkm`, use Terra commit `de698b7f` and terra-baselines commit `8091d3e` when trying to
  reproduce W&B behavior locally. A local training-era eval on 2026-05-11 with 64 envs, 550 steps,
  seed 123 produced `positive_terminations_per_env=7.5625`, `reward=0.1226`,
  `max_reward=6.8679`, `DO=0.2706`, and `DO_NOTHING=0.0762`.
- The single-env training-era GIF
  `/home/lorenzo/moleworks/terra-baselines/logs/local_policy_play/hcwvorkm_training_era_seed123.gif`
  terminated successfully in 61 steps with return about `8.41`.
- Old checkpoints do not store `use_action_mask`. When unpickled with current classes, plain
  `getattr(config, "use_action_mask", True)` silently inherits the new class default. For legacy
  replay, infer whether the field was saved on the instance; if it was missing, default to
  `False` to match the historical no-mask policy/eval path.

Interpret W&B signals as follows:

- First terminal signal: `eval/max_reward >= 6.8` and `eval/positive_terminations > 0`.
- A tiny one-off positive termination can be luck. Call it real only if it persists across
  several evals and has plausible `eval/avg_positive_episode_length`.
- Healthy learning: `eval/rewards > 0.01`, then trending toward `0.05+`; `eval/DO` climbing
  toward `0.25-0.35`; `eval/DO_NOTHING %` falling below about `0.05`.
- Strong solo target: `eval/positive_terminations >= 3`, `eval/rewards ~= 0.15-0.18`,
  `eval/DO ~= 0.31-0.34`, and `eval/DO_NOTHING % <= 0.02`.
- No-success signature: `eval/positive_terminations == 0`, `eval/max_reward < 1`,
  `eval/DO ~= 0.05-0.10`, and `eval/DO_NOTHING % ~= 0.20+`.

`eval/positive_terminations` is normalized by `num_envs_per_device * num_devices`, so it is
comparable across GPU counts; it can exceed 1 because multiple successful episodes may happen per
eval environment over the eval horizon. `progress/episode_completion_rate` is not the same as task
success because it includes ordinary done/timeouts; use `eval/positive_terminations` plus
`eval/max_reward` for clean success calls.
