#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="${PROJECT_ROOT}/ros2_ws/install/setup.bash"

echo "==================================================================="
echo "         BellyBot A - Launch RViz (Manual GUI Sliders)"
echo "==================================================================="

if [ ! -f "${ROS_SETUP}" ]; then
    echo "ERROR: ROS 2 Jazzy setup file not found at ${ROS_SETUP}" >&2
    echo "Please install dependencies: sudo bash ${PROJECT_ROOT}/scripts/install_dependencies.sh" >&2
    exit 1
fi

if [ ! -f "${WS_SETUP}" ]; then
    echo "Workspace not built yet. Building now..."
    bash "${PROJECT_ROOT}/scripts/build.sh"
fi

# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WS_SETUP}"

echo "Launching BellyBot A in RViz with joint_state_publisher_gui..."
ros2 launch bellybot_description display.launch.py demo:=false "$@"
