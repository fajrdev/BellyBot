#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
mkdir -p "$ROOT/ros2_ws/src"
TARGET="$ROOT/ros2_ws/src/bellybot_dock_stack_description"
if [[ ! -e "$TARGET" ]]; then ln -s "$ROOT/ros2/bellybot_dock_stack_description" "$TARGET"; fi
if [[ "$(readlink -f "$TARGET")" != "$ROOT/ros2/bellybot_dock_stack_description" ]]; then echo 'Workspace has another revision; stopping without overwriting.' >&2; exit 1; fi
cd "$ROOT/ros2_ws"
colcon build --symlink-install
