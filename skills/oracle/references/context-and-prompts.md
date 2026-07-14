# Oracle Context And Prompt Reference

## File Selection

`--file` accepts files, directories, globs, and repeated arguments. Attach the smallest coherent set
that preserves structure, tests, docs, and configs needed for the question. A repository root is useful
for repo-scale work only after `--files-report` confirms that tracked and untracked resolved content is
appropriate to upload. Add sibling repos and distilled evidence only when they carry necessary truth.

Use explicit exclusions for snapshots, generated output, or large artifacts, for example:

```bash
--file "$ROOT" --file '!**/*.snap' --file '!**/generated/**'
```

Oracle honors `.gitignore`, does not follow symlinks during glob expansion, filters dotfiles unless explicitly selected, and rejects files over its configured size cap. Inspect `--files-report` rather than guessing what resolved.

For ML, robotics, controls, or system debugging, include:

- source, configs, launch files, scripts, tests, and relevant docs;
- small benchmark CSV/JSON, plots, screenshots, traces, and experiment manifests;
- a short tree/manifest explaining each repo or artifact.

Exclude secrets, caches, builds, environments, full datasets, checkpoints, rosbags, and videos unless the question truly requires them.

## Prompt Contract

Assume the second model has no project knowledge. State:

1. the expert role and project/stack;
2. the current objective and why it matters;
3. key directories and entrypoints;
4. exact reproduction steps, errors, and measured evidence;
5. what was already tried;
6. constraints and non-goals;
7. the attachment inventory and what each item contributes;
8. the desired response shape, such as findings, options, patch plan, or tests.

For a long investigation, make the prompt independently restorable: include enough briefing and evidence that the same prompt plus file set can be rerun later without conversation history.

## Validation

After Oracle returns, verify claims against current files and tests. Do not treat a confident narrative as evidence, and do not apply proposed changes without inspecting their assumptions and scope.
