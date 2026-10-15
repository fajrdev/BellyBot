# BellyBot A — Setup and Installation Guide

## 1. Environment & Target Pairing

- **Operating System:** Ubuntu 24.04.5 LTS (Noble Numbat)
- **Kernel & Architecture:** Linux x86_64 (`7.0.0-38-generic`)
- **System Python:** Python 3.12.3
- **Graphics Support:**
  - NVIDIA GeForce RTX 3070 Mobile / Max-Q (Driver 580.178.04, CUDA 13.0)
  - Intel CometLake-H GT2 UHD Graphics
  - Session Type: Wayland (Hardware display available on `:0`)
- **Disk Availability:** 108 GiB available on `/dev/nvme0n1p5`

### Installed & Verified Software Versions
- **ROS 2:** ROS 2 Jazzy Jalisco (`ros-jazzy-desktop`, Jazzy)
- **Gazebo Sim:** Gazebo Sim 8.15.0 (Harmonic, `gz-harmonic`)
- **ROS-Gazebo Integration:** `ros-jazzy-ros-gz` & `ros-jazzy-ros-gz-bridge` (v1.0.24-1noble)
- **Build System:** `colcon` (v0.21.3) with `colcon-common-extensions`
- **Package Manager:** `rosdep` (v0.25.1)
- **Isolated CAD Environment:** Python 3.12 venv (`venv_cad/`) with CadQuery 2.7.0 (`yourdfpy`, `scipy`, `numpy`)
- **GPU Acceleration:** NVIDIA Driver 580.178.04, OpenGL 4.6, RTX 3070 Mobile (8GB VRAM)

---

## 2. Directory Structure

The project root `/home/fajrda/sutd-hackathon` is organized as follows:

```
sutd-hackathon/
├── models/
│   └── bellybot_A_v02/           # Extracted CAD master, exports, docs & ROS package
│       ├── cad/                  # CadQuery master scripts (build.py, etc.)
│       ├── docs/                 # Engineering notes, simulation notes, validation.json
│       ├── exports/              # STEP and STL CAD exports
│       └── ros2/
│           └── bellybot_description/  # Primary editable ROS 2 description package
├── ros2_ws/
│   └── src/
│       └── bellybot_description  # Symlink -> ../../models/bellybot_A_v02/ros2/bellybot_description
├── scripts/
│   ├── check_environment.sh      # Environment diagnostics script
│   ├── install_dependencies.sh   # System dependency installer (ROS 2 Jazzy + Gazebo)
│   ├── build.sh                  # Colcon build script
│   ├── run_rviz.sh               # Launch RViz with manual GUI sliders
│   ├── run_joint_demo.sh         # Launch RViz with 39s smooth trajectory
│   ├── run_gazebo.sh             # Launch Gazebo Harmonic starter world
│   ├── run_bridge.sh             # Standalone ROS-Gazebo parameter bridge
│   └── test_drive_and_joints.sh  # Automated drive & joint commissioning test
├── docs/
│   ├── setup.md                  # This setup documentation
│   └── known_issues.md           # Engineering limitations & runtime issues
├── results/                      # Test results and log records
├── videos/                       # Simulation recordings and media
└── venv_cad/                     # Dedicated Python 3.12 virtual environment for CadQuery 2.7.0
```

---

## 3. Installation Procedure

### 3.1 System Dependencies & ROS 2 / Gazebo Installation
Because administrative privileges (`sudo`) require password authentication on this system, execute the provided installer in your terminal:

```bash
cd ~/sutd-hackathon
sudo bash scripts/install_dependencies.sh
```

The script automates:
1. Enabling the Ubuntu `universe` repository.
2. Installing GPG keys for ROS 2 (`ros-archive-keyring.gpg`) and Gazebo (`pkgs-osrf-archive-keyring.gpg`).
3. Configuring apt sources for `packages.ros.org` and `packages.osrfoundation.org` (Noble).
4. Forcing IPv4 fallback for reliable package download over networks with IPv6 routing drops.
5. Installing:
   - `build-essential`, `cmake`, `git`, `python3-colcon-common-extensions`, `python3-rosdep`
   - `ros-jazzy-desktop`
   - `ros-jazzy-ros-gz`, `ros-jazzy-ros-gz-bridge`, `ros-jazzy-ros-gz-sim`
   - `ros-jazzy-joint-state-publisher-gui`, `ros-jazzy-xacro`
   - `gz-harmonic`
6. Initializing `rosdep` and updating local indexes.

### 3.2 CadQuery Virtual Environment
CadQuery 2.7.0 requires a distinct Python virtual environment to avoid modifying system Python:

```bash
cd ~/sutd-hackathon
python3 -m venv venv_cad
./venv_cad/bin/pip install -r models/bellybot_A_v02/cad/requirements.txt
```
*(This step has already been completed during initial workspace setup.)*

---

## 4. Building the Workspace

Build the `bellybot_description` package:

```bash
cd ~/sutd-hackathon
./scripts/build.sh
```

Or manually:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/sutd-hackathon/ros2_ws
colcon build --symlink-install --packages-select bellybot_description
source install/setup.bash
```

---

## 5. Running and Verifying BellyBot A

### 5.1 Verification in RViz2
To verify model visuals, materials, transforms, and manual joint sliders:
```bash
./scripts/run_rviz.sh
```
In the GUI slider window, move:
- `lift_mid_joint` (0.0 to 0.120 m) — child stage mimics 1:1, reaching 0.240 m total.
- `extension_mid_joint` (0.0 to 0.275 m) — child stage mimics 1:1, reaching 0.550 m total.
- `yaw_joint` (-1.570796 to +1.570796 rad).

To run the continuous 39-second smooth joint demonstration:
```bash
./scripts/run_joint_demo.sh
```
This publishes 60 Hz quintic trajectories showing wheel rotation, lift, yaw, reach, and return with zero jerk boundaries.

### 5.2 Verification in Gazebo Harmonic
Launch the physics simulation world:
```bash
./scripts/run_gazebo.sh
```
This sets `GZ_SIM_RESOURCE_PATH` so `model://bellybot_description/meshes/...` resolves properly and loads `drive_world.sdf`.

### 5.3 Commissioning Drive and Joint Tests
In a second terminal, execute the commissioning test:
```bash
./scripts/test_drive_and_joints.sh
```
This tests:
1. Low-speed forward driving (0.1 m/s) with synchronous wheel marker rotation.
2. Low-speed reverse driving (-0.1 m/s).
3. In-place pivot turn (0.2 rad/s).
4. Explicit stop.
5. Vertical lift positioning.
6. Yaw turret rotation.
7. Telescopic reach extension.

### 5.4 ROS 2 <-> Gazebo Bridge
To connect ROS 2 topics to Gazebo Sim:
```bash
./scripts/run_bridge.sh
```
Or launch the integrated simulation:
```bash
ros2 launch bellybot_description simulation.launch.py
```
Bridged topics:
- `/clock`
- `/cmd_vel`
- `/left_wheel_speed` & `/right_wheel_speed`
- `/lift_stage_position`
- `/extension_stage_position`
- `/yaw_position`
- `/joint_states`
- `/odom`
