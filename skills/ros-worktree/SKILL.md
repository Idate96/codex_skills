---
name: ros-worktree
description: "Create a minimal isolated ROS 2 workspace from one or more Git worktrees. Use for safe package builds, coordinated cross-repository changes, container tests, and verified publication without disturbing a dirty live workspace."
---

# ROS Worktree

Use a tiny colcon workspace containing only the repositories needed for the task. Do not copy a full `src/` tree.

## 1. Confirm the environment

```bash
if test -e /.dockerenv || test -e /run/.containerenv; then
  echo "Already inside a container"
else
  echo "Running on the host"
fi

BASE_WS="${ROS_WS:-$HOME/ros2_ws}"
test -f "$BASE_WS/install/setup.bash"
```

Do not start or enter another container when the current shell is already the intended container. Keep the dirty live workspace read-only.

## 2. Create direct worktrees

Set explicit repositories, remote bases, and paths. Add a worktree for every repository that will change, including floating dependencies.

```bash
set -euo pipefail
TASK=short-task-name
ROOT="$HOME/moleworks/.worktrees/$TASK"
WS="$HOME/moleworks/ros2_ws_$TASK"

APP_REPO="$BASE_WS/src/application_repo"
DEP_REPO="$BASE_WS/src/dependency_repo"  # Omit when unnecessary.
APP_WT="$ROOT/application_repo"
DEP_WT="$ROOT/dependency_repo"

test ! -e "$ROOT"
test ! -e "$WS"
mkdir -p "$ROOT" "$WS/src"

git -C "$DEP_REPO" fetch origin dependency_branch
git -C "$DEP_REPO" worktree add -b "work/$TASK-dep" "$DEP_WT" origin/dependency_branch
git -C "$APP_REPO" fetch origin main
git -C "$APP_REPO" worktree add -b "work/$TASK" "$APP_WT" origin/main

ln -s "$DEP_WT" "$WS/src/dependency_repo"
ln -s "$APP_WT" "$WS/src/application_repo"
```

Branch from fetched remote refs, never a possibly stale local branch. Never use `git worktree add -B`.

## 3. Build and test only the intended packages

Always enter the exact isolated workspace before invoking colcon. The current directory owns `build/`, `install/`, and `log/`.

```bash
cd "$WS"
test "$(pwd -P)" = "$(realpath "$WS")"
source /opt/ros/jazzy/setup.bash
source "$BASE_WS/install/setup.bash"

BASE_PATHS=("$DEP_WT/graph_package" "$APP_WT")
PACKAGES=(dependency_package application_package)

colcon list --base-paths "${BASE_PATHS[@]}"
colcon build --base-paths "${BASE_PATHS[@]}" --packages-select "${PACKAGES[@]}"
source "$WS/install/setup.bash"
colcon test --packages-select "${PACKAGES[@]}" --return-code-on-test-failure
colcon test-result --verbose
```

Use `--packages-up-to TARGET` instead of `--packages-select` only when workspace dependencies should be included.

## 4. Publish in dependency order

Push a shared branch directly only when the user explicitly requests it. Otherwise push task branches for review.

Immediately before each push: fetch the target branch, require it to be an ancestor of `HEAD`, push normally, then verify the remote hash. Publish floating dependencies before their consumer.

```bash
push_checked() {
  wt=$1
  target=$2
  git -C "$wt" fetch origin "$target"
  git -C "$wt" merge-base --is-ancestor "origin/$target" HEAD
  git -C "$wt" push origin "HEAD:$target"
  test "$(git -C "$wt" rev-parse HEAD)" = \
       "$(git -C "$wt" ls-remote origin "refs/heads/$target" | awk '{print $1}')"
}

push_checked "$DEP_WT" dependency_branch
push_checked "$APP_WT" main
```

Never force-push in this workflow.

## 5. Detach runtime installs before cleanup

Never keep a runtime `--symlink-install` that points into a temporary worktree. Before removing worktrees, either rebuild the deployed packages without `--symlink-install` into their permanent workspace or rebuild them from permanent source.

Then run the mandatory check:

```bash
SKILL_DIR="${CODEX_HOME:-$HOME/.codex}/skills/ros-worktree"
"$SKILL_DIR/scripts/assert_no_temp_links.sh" "$BASE_WS/install" "$ROOT"
```

Do not continue if it reports a link. This check examines both raw and resolved link targets, including broken links.

## 6. Clean up exact paths

After remote-hash verification and the runtime-link check:

```bash
cd /
git -C "$DEP_REPO" worktree remove "$DEP_WT"
git -C "$APP_REPO" worktree remove "$APP_WT"
git -C "$DEP_REPO" worktree prune
git -C "$APP_REPO" worktree prune

test "$(realpath -m "$WS")" = "$(realpath -m "$HOME/moleworks/ros2_ws_$TASK")"
gio trash "$WS"
rmdir "$ROOT"
```

Delete local task branches only after their commits are verified remotely or merged. Report what was removed and whether it is recoverable.
