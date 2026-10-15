#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

echo "==================================================================="
echo "              BellyBot DockStack C v3 - Workspace Build Script"
echo "==================================================================="
echo "Project Root: ${PROJECT_ROOT}"

# 1. Check ROS 2 Jazzy installation
if [ ! -f "${ROS_SETUP}" ]; then
    echo "ERROR: ROS 2 Jazzy setup file not found at ${ROS_SETUP}" >&2
    echo "Please install ROS 2 Jazzy first by running:" >&2
    echo "  sudo bash ${PROJECT_ROOT}/scripts/install_dependencies.sh" >&2
    exit 1
fi

echo "--> Sourcing ROS 2 Jazzy: ${ROS_SETUP}"
# shellcheck disable=SC1090
source "${ROS_SETUP}"

# 2. Check colcon
if ! command -v colcon >/dev/null 2>&1; then
    echo "ERROR: 'colcon' build tool not found. Please install python3-colcon-common-extensions." >&2
    exit 1
fi

# 3. Build workspace with colcon
echo "--> Building packages with colcon..."
cd "${PROJECT_ROOT}/ros2_ws"
colcon build --symlink-install

echo ""
echo "==================================================================="
echo "  [SUCCESS] Workspace built successfully!"
echo "  To use the environment in your shell, run:"
echo "    source ${PROJECT_ROOT}/ros2_ws/install/setup.bash"
echo "==================================================================="
