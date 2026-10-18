#!/usr/bin/env bash
# Select the maintained Lyrical driver without discarding local work.
set -Eeuo pipefail
if [[ $# -gt 1 ]]; then
    echo "Usage: $0 [driver-checkout]" >&2
    exit 2
fi
driver=${1:-"$HOME/Workspace/colcon_rumblex/src/ydlidar_ros2_driver"}
remote=git@github.com:SteinRobotics/ydlidar_ros2_driver.git
if [[ ! -e $driver ]]; then
    git clone --branch lyrical --single-branch "$remote" "$driver"
    exit 0
fi
if [[ ! -d $driver/.git && ! -f $driver/.git ]]; then
    echo "Not a Git checkout: $driver" >&2
    exit 1
fi
if [[ -n $(git -C "$driver" status --porcelain) ]]; then
    echo "Driver checkout has local changes (possibly the old setup patch): $driver" >&2
    echo "Commit or stash them, then rerun. Local work was not changed." >&2
    exit 1
fi
git -C "$driver" remote set-url origin "$remote"
git -C "$driver" fetch origin '+refs/heads/lyrical:refs/remotes/origin/lyrical'
if git -C "$driver" show-ref --verify --quiet refs/heads/lyrical; then
    # Also reject local commits ahead of origin, so the build uses the published version.
    if ! git -C "$driver" merge-base --is-ancestor lyrical origin/lyrical; then
        echo "Local lyrical has unpublished or divergent commits. Reconcile them before rerunning." >&2
        exit 1
    fi
    git -C "$driver" switch lyrical
    git -C "$driver" merge --ff-only origin/lyrical
else
    git -C "$driver" switch --create lyrical --track origin/lyrical
fi
git -C "$driver" branch --set-upstream-to=origin/lyrical lyrical
