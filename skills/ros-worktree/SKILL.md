---
name: ros-worktree
description: "Create an isolated ROS 2 workspace backed by Git worktrees. Use for safe builds, container tests, image validation, and PR-ready changes without disturbing the main workspace."
---

# ROS Worktree (Isolated Workspace) Workflow

## Goal

Create a new ROS2 workspace directory (example: `~/moleworks/ros2_ws_menzi_base`) that is safe to mutate and build in, while keeping `~/moleworks/ros2_ws` untouched.

## Create A New Workspace (Fast Path: Copy `src/`)

Pick a suffix/name and copy only `src/`.

```bash
SRC_WS=~/moleworks/ros2_ws
DST_WS=~/moleworks/ros2_ws_SUFFIX

test -d "$SRC_WS/src"
test ! -e "$DST_WS"  # fail fast if destination already exists

# Keep the destination inside the expected workspace parent.
case "$(realpath -m "$DST_WS")" in
  "$(realpath -m ~/moleworks)"/*) ;;
  *) echo "Unexpected destination: $DST_WS" >&2; exit 2 ;;
esac

mkdir -p "$DST_WS/src"
rsync -a "$SRC_WS/src/" "$DST_WS/src/"
```

## Replace Specific Repos With Git Worktrees (Recommended)

Rationale: keep a canonical clone in `~/git/REPO` and put a *worktree checkout* in `~/git/.worktrees/REPO_SUFFIX`. Then symlink that into the new workspace. This keeps PRs easy and avoids interfering with ongoing work elsewhere.

### Example: `moleworks_ros` worktree + symlink into the new workspace

```bash
MOLEWORKS_ROS_GIT=~/git/moleworks_ros
MOLEWORKS_ROS_WT=~/git/.worktrees/moleworks_ros_SUFFIX

test -d "$MOLEWORKS_ROS_GIT/.git"
test -d "$DST_WS/src"

# Move the copied repo aside after proving it is inside the isolated destination.
test -d "$DST_WS/src/moleworks_ros"
COPIED_REPO="$(realpath -m "$DST_WS/src/moleworks_ros")"
case "$COPIED_REPO" in
  "$(realpath -m "$DST_WS")"/src/*) ;;
  *) echo "Refusing unexpected source path: $COPIED_REPO" >&2; exit 2 ;;
esac
test -z "$(git -C "$COPIED_REPO" status --porcelain)"
mkdir -p "$DST_WS/_replaced_sources"
mv "$COPIED_REPO" "$DST_WS/_replaced_sources/moleworks_ros"

# Create a genuinely new branch and worktree checkout.
cd "$MOLEWORKS_ROS_GIT"
git fetch origin
test ! -e "$MOLEWORKS_ROS_WT"
! git show-ref --verify --quiet refs/heads/MY_BRANCH
! git show-ref --verify --quiet refs/remotes/origin/MY_BRANCH
git worktree add -b MY_BRANCH "$MOLEWORKS_ROS_WT" origin/main

# Symlink the worktree into the workspace.
ln -s "$MOLEWORKS_ROS_WT" "$DST_WS/src/moleworks_ros"
```

Notes:
- Use the same pattern for other repos you plan to change (e.g. `holistic_fusion`, `menzi_docker`, etc.).
- Prefer new branch names; avoid force-push.
- Never use `git worktree add -B` here: it can reset an existing local branch. To reuse an existing
  branch, first inspect its current worktree/remote state, then use `git worktree add PATH BRANCH`.
- Do not delete the moved copy until the isolated workspace builds and its source links are verified.
- If you need a clean build, clean `build/ install/ log` in the workspace, not the git repos (see below).

## Clean Build Artifacts (Recommended Before Testing New Images)

Keep the workspace git state intact; just reset build outputs.

```bash
cd "$DST_WS"
ts="$(date +%Y%m%d_%H%M%S)"
mkdir -p "_old_build_${ts}"
for d in build install log; do
  if test -e "$d"; then
    mv "$d" "_old_build_${ts}/"
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

## Build/Test In Docker Against The New Workspace

### Generic `docker run` pattern

Mount the workspace into the container (so builds are deterministic and don't touch other workspaces).

```bash
docker run --rm -it \
  -v "$DST_WS:/workspaces/ros2_ws" \
  -w /workspaces/ros2_ws \
  IMAGE \
  bash -lc 'set -ex; source /opt/ros/jazzy/setup.bash; colcon build --packages-up-to PKG'
```

If `colcon` tries to crawl everything under `src/`, restrict it:

```bash
colcon build --base-paths src/moleworks_ros src/holistic_fusion/ros2 --packages-up-to PKG
```

### Moleworks-specific: use `moleworks_ros/docker/docker_launch.sh`

If you are testing `moleworks_ros` images, prefer the repo scripts. They mount your full `$HOME`, so the new workspace is visible automatically.

Important: the container bashrc typically auto-sources only `~/ros2_ws` or `/workspaces/ros2_ws`. If your worktree is `~/moleworks/ros2_ws_SUFFIX`, you must `source ~/moleworks/ros2_ws_SUFFIX/install/setup.bash` in each shell before running `ros2 launch`/`ros2 run`.
