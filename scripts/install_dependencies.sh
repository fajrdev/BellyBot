#!/usr/bin/env bash
set -eo pipefail

# Ensure script is run with sudo/root
if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: This script must be run as root." >&2
    echo "Please execute: sudo bash $0" >&2
    exit 1
fi

echo "==================================================================="
echo "    BellyBot A - Dependency Installer (ROS 2 Jazzy + Gazebo)"
echo "==================================================================="

# Force IPv4 for apt if IPv6 is unreachable
echo 'Acquire::ForceIPv4 "true";' > /etc/apt/apt.conf.d/99force-ipv4

echo "--> [1/6] Installing prerequisites..."
apt-get update
apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    wget \
    gnupg \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-venv

echo "--> [2/6] Enabling Ubuntu Universe repository..."
add-apt-repository -y universe

echo "--> [3/6] Configuring ROS 2 Jazzy repository..."
install -m 0755 -d /etc/apt/keyrings /usr/share/keyrings
wget -4 -qO - https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg --yes

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "--> [4/6] Configuring Gazebo Harmonic repository..."
wget -4 -qO - https://packages.osrfoundation.org/gazebo.gpg | \
    gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg --yes

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

echo "--> [5/6] Updating package lists & installing ROS 2, Gazebo Harmonic, and tools..."
apt-get update
apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    gz-harmonic \
    python3-colcon-common-extensions \
    python3-rosdep

echo "--> [6/6] Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init || true
else
    echo "rosdep already initialized in /etc/ros/rosdep."
fi

# Run rosdep update as the original user if invoked via sudo
if [ -n "${SUDO_USER}" ] && [ "${SUDO_USER}" != "root" ]; then
    echo "Running 'rosdep update' as user ${SUDO_USER}..."
    sudo -u "${SUDO_USER}" rosdep update || true
fi

echo "==================================================================="
echo "  [SUCCESS] ROS 2 Jazzy and Gazebo Harmonic dependencies installed!"
echo "==================================================================="
