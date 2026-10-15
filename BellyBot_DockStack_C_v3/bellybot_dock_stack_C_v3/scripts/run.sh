#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
case "${1:-rviz}" in
 rviz) exec ros2 launch bellybot_dock_stack_description display.launch.py ;;
 demo) exec ros2 launch bellybot_dock_stack_description display.launch.py demo:=true ;;
 bench) exec ros2 launch bellybot_dock_stack_description sim.launch.py world:=bench ;;
 floor) exec ros2 launch bellybot_dock_stack_description sim.launch.py world:=floor_pickup ;;
 batch) exec ros2 launch bellybot_dock_stack_description sim.launch.py world:=batch_transfer ;;
 cargo) exec ros2 launch bellybot_dock_stack_description sim.launch.py world:=cargo_hold ;;
 *) echo 'Usage: scripts/run.sh rviz|demo|bench|floor|batch|cargo' >&2; exit 2 ;;
esac
