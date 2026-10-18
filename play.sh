#!/usr/bin/env bash
# Quick launcher for BellyBot Interactive Teleoperation
# Usage:
#   ./play.sh                 (Default: Boeing 737 hold + 3D GUI)
#   ./play.sh --world open    (Open sandbox playground with max speed)
#   ./play.sh --no-gui        (Headless Gazebo + Pygame HUD window)

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

source /opt/ros/jazzy/setup.bash
if [ -f "$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/ros2_ws/install/setup.bash"
fi

export GZ_IP=127.0.0.1
export PYTHONPATH="$SCRIPT_DIR/scripts:$PYTHONPATH"

exec python3 "$SCRIPT_DIR/scripts/play_bellybot.py" "$@"
