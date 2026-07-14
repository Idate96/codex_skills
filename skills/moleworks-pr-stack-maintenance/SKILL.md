---
name: moleworks-pr-stack-maintenance
description: "Restack, verify, and merge `moleworks_ros` PR stacks. Use for rebases, PR splitting, shared CI fixes, review resolution, dependency-order merges, and worktree cleanup."
---

# Moleworks PR Stack Maintenance

Use this skill for stacked `moleworks_ros` PR maintenance where correctness matters more than speed alone: rebasing dependent branches, preserving split history from an older monolithic PR, re-running local CI in Docker on this workstation, and merging only after GitHub is green.

Match actions to the request. Inventory, review, and status requests are read-only. Rebasing or
editing authorizes local branch changes only. Force-pushing, merging, closing PRs, and deleting
worktrees are separate external/destructive actions and require the user to request that stage.

## Quick Workflow

1. Inventory the stack on GitHub.
2. Map merge order and dependency bases before editing anything.
3. Put each active PR branch in a dedicated git worktree.
4. Rebase each PR onto its intended base and resolve conflicts carefully.
5. Apply shared CI or code fixes across the affected branches.
6. Verify locally in Docker on this machine.
7. Run one persistent reviewer loop until it reports `no findings`.
8. When explicitly authorized, force-push rebased branches with lease.
9. Wait for GitHub checks to finish on the pushed heads.
10. When explicitly authorized, merge in dependency order, then close only confirmed superseded PRs.

## Inventory

Start with GitHub state, not local assumptions.

```bash
gh pr list --repo leggedrobotics/moleworks_ros --state open
gh pr view <PR> --repo leggedrobotics/moleworks_ros --json \
  number,title,state,isDraft,baseRefName,headRefName,headRefOid,mergeStateStatus,statusCheckRollup
```

Record:
- which PRs are still open
- which branch each PR targets
- whether a monolithic PR is now superseded by split PRs
- the intended merge order

If only one PR remains open after the stack cleanup, confirm that explicitly before merging.

## Worktree Setup

Keep the main workspace untouched. Use dedicated worktrees for each PR branch.

Suggested path pattern:

```bash
mkdir -p ~/.codex_tmp/mwros_prs
git -C ~/moleworks/ros2_ws/src/moleworks_ros fetch origin <branch>
git -C ~/moleworks/ros2_ws/src/moleworks_ros worktree add \
  ~/.codex_tmp/mwros_prs/pr<PR> \
  -B <local-branch> origin/<remote-branch>
```

If a PR already has a worktree, reuse it instead of recreating it.

## Rebase Rules

- Rebase each PR onto the correct updated base branch, usually `origin/main` or the updated predecessor PR branch.
- Resolve conflicts by preserving the PR's intended scope; do not silently absorb unrelated churn from the old monolithic branch.
- If a fix has already landed on `main`, prefer rebasing so Git drops the duplicate naturally instead of reapplying it.
- Never use `git reset --hard` or revert unrelated user changes.
- After a substantial conflict resolution, run at least a fast syntax/test check before touching the next branch.

Useful commands:

```bash
git -C <worktree> fetch origin
git -C <worktree> rebase origin/main
git -C <worktree> status --short --branch
git -C <worktree> diff --check
```

## Local CI On This Machine

`moleworks_ros` CI should run on this workstation through the Docker image used by the repo.

For isolated testing, create a throwaway ROS workspace that reuses the main `src/` tree but swaps in the PR worktree for `moleworks_ros`.

Suggested pattern:

```bash
DST_WS=~/moleworks/ros2_ws_prXXXX_ci
mkdir -p "$DST_WS/src"
for p in ~/moleworks/ros2_ws/src/*; do ln -s "$p" "$DST_WS/src/"; done
rm "$DST_WS/src/moleworks_ros"
ln -s ~/.codex_tmp/mwros_prs/pr<PR> "$DST_WS/src/moleworks_ros"
```

Then run containerized checks:

```bash
docker run --rm --gpus all \
  -v "$HOME:$HOME" \
  -w "$DST_WS" \
  rslheap/moleworks_ros:latest \
  bash -lc 'set -eo pipefail; source /opt/ros/jazzy/setup.bash; colcon build --packages-select workspace_planner'
```

For planner-stack PRs, the high-value regression command is:

```bash
docker run --rm --gpus all \
  -v "$HOME:$HOME" \
  -w "$DST_WS" \
  rslheap/moleworks_ros:latest \
  bash -lc "set -eo pipefail; source /opt/ros/jazzy/setup.bash; source install/setup.bash; \
    colcon test --packages-select poa_planners terra_planner workspace_planner \
    --return-code-on-test-failure --event-handlers console_direct+; \
    colcon test-result --verbose"
```

Notes:
- Rebuild `workspace_planner` after Python test/helper edits so the installed package matches the source.
- Prefer targeted test subsets while iterating, then rerun the broader package set before pushing.
- If CI uses bind-mounted sources, avoid assumptions that `--symlink-install` is safe.

## Review Loop

Use one persistent reviewer agent for the whole task.

Each iteration should include:
- changed files
- what was fixed
- exact verification commands/results
- a request for findings only, severity-ordered, with `no findings` when clean

Do not merge until the reviewer loop is closed.

## Push And GitHub Checks

After a rebase, and only when the requested scope includes publishing, push with lease:

```bash
git -C <worktree> push --force-with-lease origin HEAD:<remote-branch>
```

Immediately re-query GitHub because the PR head SHA changed:

```bash
gh pr view <PR> --repo leggedrobotics/moleworks_ros --json \
  headRefOid,mergeStateStatus,statusCheckRollup
```

Wait for `statusCheckRollup` to finish. Do not rely on stale local results alone.

## Merge

Merging changes shared repository state. Do it only when the user requested the merge stage. Use the
GitHub API directly, not `gh pr merge`.

```bash
gh api --method PUT repos/leggedrobotics/moleworks_ros/pulls/<PR>/merge \
  -f sha=<head-sha> \
  -f merge_method=merge
```

Only merge when:
- the PR is `OPEN`
- `isDraft` is `false`
- `mergeStateStatus` is clean/mergeable
- required checks are green for the current head SHA
- the user-authorized PR number and head SHA still match the final pre-merge readback

After merging, re-run `gh pr list --state open` to confirm the remaining stack state. Close an older
monolithic PR only when its supersession is confirmed and closing was included in the requested scope.

## Practical Reminders

- Keep PR scope sharp. If a fix belongs to `main`, land it there and restack the PRs.
- Prefer updating tests to reflect deliberate planner behavior changes only after confirming the behavior is intentional and locally green.
- Runtime-based test assertions are fragile; prefer structural or correctness invariants over strict speed comparisons.
- For mock planner helpers, keep service/request field changes synchronized across diagnostics, benchmarks, and sweeps.
