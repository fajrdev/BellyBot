#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "==================================================================="
echo "           BellyBot A - Environment Diagnostics"
echo "==================================================================="
echo "Project Root: ${PROJECT_ROOT}"
echo "Current Time: $(date)"
echo ""

echo "--- [1] OS & System Architecture ---"
if command -v lsb_release >/dev/null 2>&1; then
    lsb_release -d
else
    cat /etc/os-release | grep PRETTY_NAME || true
fi
echo "Kernel: $(uname -s -r -m)"
echo "Architecture: $(uname -m)"
VIRT=$(systemd-detect-virt 2>/dev/null || echo "unknown")
echo "Virtualization / Session: ${VIRT} (Native hardware)"

echo ""
echo "--- [2] Shell & Python ---"
echo "Shell: ${SHELL:-$(getent passwd "$USER" | cut -d: -f7)}"
echo "System Python: $(python3 --version 2>&1)"
if [ -d "${PROJECT_ROOT}/venv_cad" ]; then
    echo "CadQuery venv: Present at ${PROJECT_ROOT}/venv_cad"
else
    echo "CadQuery venv: Not created yet"
fi

echo ""
echo "--- [3] Hardware & Resources ---"
echo "Memory:"
free -h
echo "Available Disk Space (Project Root):"
df -h "${PROJECT_ROOT}" | tail -n 1
echo "Graphics / Display:"
echo "  DISPLAY: ${DISPLAY:-None}"
echo "  WAYLAND_DISPLAY: ${WAYLAND_DISPLAY:-None}"
echo "  XDG_SESSION_TYPE: ${XDG_SESSION_TYPE:-None}"
if command -v nvidia-smi >/dev/null 2>&1; then
    echo "  NVIDIA GPU detected:"
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
fi

echo ""
echo "--- [4] ROS 2 & Gazebo Installation ---"
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "  [OK] ROS 2 Jazzy is installed in /opt/ros/jazzy"
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    echo "  ROS_DISTRO: ${ROS_DISTRO}"
else
    echo "  [MISSING] ROS 2 Jazzy is NOT installed in /opt/ros/jazzy"
fi

if command -v gz >/dev/null 2>&1; then
    GZ_VER=$(gz sim --version 2>&1 || echo "installed")
    echo "  [OK] Gazebo is installed: ${GZ_VER}"
else
    echo "  [MISSING] 'gz' command not found"
fi

if dpkg -s ros-jazzy-ros-gz >/dev/null 2>&1; then
    echo "  [OK] ros-jazzy-ros-gz package is installed"
else
    echo "  [MISSING] ros-jazzy-ros-gz package is not installed"
fi

if command -v colcon >/dev/null 2>&1; then
    COLCON_V=$(python3 -c "import colcon_core; print(colcon_core.__version__)" 2>/dev/null || echo "installed")
    echo "  [OK] colcon is installed (version ${COLCON_V})"
else
    echo "  [MISSING] colcon build tool is not installed"
fi

if command -v rosdep >/dev/null 2>&1; then
    echo "  [OK] rosdep is installed"
else
    echo "  [MISSING] rosdep is not installed"
fi

echo ""
echo "--- [5] Workspace & Package Structure ---"
if [ -d "${PROJECT_ROOT}/BellyBot_Aircraft_Environment_v2" ]; then
    echo "  [OK] Aircraft Environment v2 present"
else
    echo "  [MISSING] BellyBot_Aircraft_Environment_v2 not found"
fi

if [ -d "${PROJECT_ROOT}/BellyBot_DockStack_C_v3" ]; then
    echo "  [OK] BellyBot DockStack C v3 CAD/description present"
else
    echo "  [MISSING] BellyBot_DockStack_C_v3 not found"
fi

if [ -L "${PROJECT_ROOT}/ros2_ws/src/bellybot_aircraft_environment" ]; then
    echo "  [OK] bellybot_aircraft_environment symlinked into ros2_ws/src"
else
    echo "  [MISSING] Symlink ros2_ws/src/bellybot_aircraft_environment missing"
fi

if [ -L "${PROJECT_ROOT}/ros2_ws/src/bellybot_dock_stack_description" ]; then
    echo "  [OK] bellybot_dock_stack_description symlinked into ros2_ws/src"
else
    echo "  [MISSING] Symlink ros2_ws/src/bellybot_dock_stack_description missing"
fi

echo ""
echo "==================================================================="
