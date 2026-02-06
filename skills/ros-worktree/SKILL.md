---
name: ros-worktree
description: Create a new isolated ROS 2 workspace ("ROS worktree") for safe builds/testing without touching the main ~/moleworks/ros2_ws. Use when you need to (1) copy an existing ROS2 workspace into ~/moleworks/ros2_ws_<suffix>, (2) clean all git repos to a pristine state, (3) add COLCON_IGNORE for heavy/non-ROS packages, and optionally (4) replace selected repos (e.g. moleworks_ros) with git worktrees from ~/git so changes are PR-able and don't interfere with ongoing work.
---

# ROS Worktree (Isolated Workspace) Workflow

## Goal

Create a new ROS2 workspace directory (example: `~/moleworks/ros2_ws_menzi_base_clean`) that is safe to mutate and build in, while keeping `~/moleworks/ros2_ws` untouched.

## Create A New Workspace By Copying `src/` (Fast Path)

Pick a suffix/name and copy only `src/`.

```bash
SRC_WS=~/moleworks/ros2_ws
DST_WS=~/moleworks/ros2_ws_<suffix>

test -d "$SRC_WS/src"
test ! -e "$DST_WS"  # fail fast if destination already exists

mkdir -p "$DST_WS/src"
rsync -a "$SRC_WS/src/" "$DST_WS/src/"
```

## Clean All Git Repos Inside The New Workspace

This ensures the new workspace is pristine (no local diffs, no build artifacts).

```bash
cd "$DST_WS/src"
for d in *; do
  if test -d "$d/.git"; then
    echo "CLEAN $d"
    git -C "$d" reset --hard
    git -C "$d" clean -fdx
  fi
done
```

## Add `COLCON_IGNORE` For Non-ROS / Heavy Packages (If Needed)

If `colcon` errors while identifying Python packages (common for ML repos) or you simply want faster builds, ignore them.

```bash
# Example: Segment Anything repo isn't a ROS package and can break colcon discovery.
touch "$DST_WS/src/segment-anything-2-real-time-ros-2/COLCON_IGNORE"

# Example: optional stack that may not be needed for most bringup builds.
# touch "$DST_WS/src/isaac_ros_nvblox/COLCON_IGNORE"
```

## Optional: Use Git Worktrees For Repos You Will Modify

Rationale: keep a canonical clone in `~/git/<repo>` and put a *worktree checkout* into the new ROS workspace. This avoids editing a throwaway copy and makes PRs straightforward.

### Example: `moleworks_ros` Worktree Inside The New Workspace

```bash
# Canonical clone lives here.
MOLEWORKS_ROS_GIT=~/git/moleworks_ros
test -d "$MOLEWORKS_ROS_GIT/.git"

# Remove the copied repo in the new workspace (fail fast if you care about its state).
test -d "$DST_WS/src/moleworks_ros"
rm -rf "$DST_WS/src/moleworks_ros"

# Create a dedicated branch and worktree checkout inside the new workspace.
cd "$MOLEWORKS_ROS_GIT"
git fetch origin
git worktree add -B <my-branch> "$DST_WS/src/moleworks_ros" origin/dev/integration_rebase3
```

Notes:
- Use the same pattern for other repos you plan to change (e.g. `holistic_fusion`).
- Prefer creating new branches (no force-push). If a branch already exists and must be updated, create a new branch name.

## Build/Test In Docker Against The New Workspace

Mount the workspace into the container and keep build artifacts in workspace-local folders (so repeated builds are deterministic and don't touch other workspaces).

```bash
docker run --rm -it \
  -v "$DST_WS:/workspaces/ros2_ws" \
  -w /workspaces/ros2_ws \
  <image> \
  bash -lc 'set -ex; source /opt/ros/jazzy/setup.bash; colcon build --packages-up-to <pkg>'
```

If `colcon` tries to crawl everything under `src/`, restrict it:

```bash
colcon build --base-paths src/moleworks_ros src/holistic_fusion/ros2 --packages-up-to <pkg>
```

