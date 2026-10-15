# Segment 01: Overall Concept & Unobstructed Hold Architecture (0:00–0:08)

## 1. Scene Overview & Judging Objective

| Attribute | Specification |
| :--- | :--- |
| **Video Timestamp** | `0:00–0:08` (8.0 seconds duration) |
| **Target World / Scene** | `transfer_cutaway.sdf` (rebuilt without `corner_transfer` and `longitudinal_buffer`) |
| **Playback Speed** | `1.0x Real-Time` (Simulation time `t = 0.00 s` to `t = 8.00 s`) |
| **Primary Judging Criteria** | Innovation & Creativity (20%), NB Aircraft Cargo-Hold Compatibility & Space Utilisation (15%) |
| **What It Establishes** | Shows the Boeing 737NG-inspired forward cargo hold (`7.70 m × 1.22 m × 1.12 m`), the external apron belt loader terminating cleanly at the doorway sill (`Y = -0.61 m`), and BellyBot (`bellybot_dock_stack`) operating on an unobstructed cargo floor without bulky interior floor conveyors blocking the hold. |

## 2. Architecture Change Reflected in This Scene

The two interior floor-mounted conveyors previously occupying `X = 0.82 m` to `X = 3.656 m` (`model://corner_transfer` at `X = 1.30 m, Y = 0.0 m` and `model://longitudinal_buffer` / `infeed` at `X = 2.70 m, Y = 0.0 m`) are **removed**. Only the exterior inclined loader (`model://belt_loader`, `Y = -6.40 m` to `-2.60 m`) and the shortened cantilevered doorway threshold belt (`model://door_belt`, spanning `Y = -2.588 m` to `Y = -0.610 m` at `X = 1.30 m`, elevated to `Z = 0.498 m` to match BellyBot's stowed receiving tray) remain outside the doorway portal arch. This frees 100% of the internal `1.22 m` wide cargo hold floor for BellyBot traversal and baggage stacking.

## 3. Technical Motion & Kinematic Specification

During `0:00–0:08`, the camera performs a smooth orbital establishing sweep from an isometric cutaway perspective while BellyBot sits at its forward-hold receiving datum (`X = 2.15 m, Y = 0.00 m, Z = 0.003 m, yaw = 0.0 rad`) and executes a brief readiness self-check of its upper yaw and tray rollers while the doorway threshold belt advances the first staged luggage set (`bag_hard_32`, `bag_hard_26`, `bag_duffel_23`) toward the doorway sill handoff point (`X = 1.30 m, Y = -0.61 m, Z = 0.498 m`).

| Time Window (Sim s) | Subsystem / Joint | Motion / Target Value | Velocity / Rate | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| `0.00–2.00 s` | Camera (`inspection`) | Pose `(8.5, -7.2, 4.8, 0, 0.42, 2.25)` → `(5.2, -4.8, 2.6, 0, 0.38, 2.15)` | Smooth cubic interpolation | Establish full 7.70 m hold, doorway arch, external loader, and BellyBot |
| `0.00–8.00 s` | `base_link` | World pose `(2.15, 0.00, 0.003, 0, 0, 0)` | `0.0 m/s` (tracks locked) | Show 1.10 m track width inside 1.22 m floor (28.5 mm liner margin) |
| `1.00–4.00 s` | `lift_stage_1..3_joint` | `0.000 m` → `0.030 m` per stage (`0.090 m` total lift) | `0.05 m/s` per stage | Demonstrate telescopic column clearance above chassis prior to yaw |
| `4.00–7.00 s` | `yaw_joint` | `0.000 rad` → `-1.1345 rad` (`-65.0°`) toward doorway | `0.50 rad/s` | Orient receiving tray toward the doorway sill belt (`door_belt`) |
| `2.00–7.50 s` | `belt_loader` & `door_belt` | Advance 3 staged bags along `+Y` toward `Y = -0.75 m` | `+0.25 m/s` linear belt speed | Show continuous external baggage supply path up to the doorway threshold |

## 4. Gazebo Harmonic (`gz sim`) Configuration & Commands

### 4.1 World & Model Setup
Use the rebuilt `transfer_cutaway.sdf` world where `<include><uri>model://corner_transfer</uri></include>` and `<include><uri>model://longitudinal_buffer</uri></include>` have been removed, leaving only:
- `model://nb_hold_cutaway` (visual cutaway on `-Y` wall/ceiling, full collision geometry intact)
- `model://belt_loader` (anchored to apron, top surface rising from `Z = -1.00 m` to `Z = 0.498 m` at `Y = -2.60 m`)
- `model://door_belt` (cantilevered doorway belt from `Y = -2.588 m` to `Y = -0.610 m`, surface `Z = 0.498 m`, terminating at the doorway portal arch)
- `model://bellybot_dock_stack` spawned at `<pose>2.15 0 0.003 0 0 0</pose>`
- Pre-placed reference cargo stack at the aft bulkhead (`cargo_0` to `cargo_3` at `X = 6.02..6.85 m`) to show target stacking geometry.

### 4.2 Gazebo CLI & Service Calls
```bash
# 1. Launch the rebuilt cutaway world via ROS 2 launch
ros2 launch bellybot_aircraft_environment aircraft.launch.py mode:=transfer cutaway:=true

# 2. Unpause Gazebo Harmonic simulation
gz service -s /world/transfer_cutaway/control   --reqtype gz.msgs.WorldControl   --reptype gz.msgs.Boolean   --timeout 2000   --req 'pause: false'

# 3. Spawn initial queue of 3 free bags on the external loader / door belt
python3 tools/spawn_transfer_bag.py --world transfer_cutaway --bag bag_hard_32 --execute
```

## 5. ROS 2 Jazzy Communications & Topic Sequence

### 5.1 Active Bridged Topics (`config/aircraft_bridge.yaml`)
| ROS 2 Topic | Gazebo Topic | Message Type | Direction | Role in Segment 01 |
| :--- | :--- | :--- | :--- | :--- |
| `/clock` | `/clock` | `rosgraph_msgs/msg/Clock` | `GZ_TO_ROS` | Master simulation clock for HUD overlay (`0:00–0:08`) |
| `/aircraft/robot_pose` | `/model/bellybot_dock_stack/pose` | `tf2_msgs/msg/TFMessage` | `GZ_TO_ROS` | Confirms robot datum `(2.15, 0.00, 0.003)` |
| `/aircraft/robot_joints` | `/model/bellybot_dock_stack/joint_states` | `sensor_msgs/msg/JointState` | `GZ_TO_ROS` | Monitors `lift_stage_1..3_joint` and `yaw_joint` |
| `/aircraft/world_poses` | `/model/tracked_bag/pose` | `tf2_msgs/msg/TFMessage` | `GZ_TO_ROS` | Tracks incoming bag approaching doorway threshold |
| `/aircraft/actuation/loader_speed` | `/environment/loader/speed` | `std_msgs/msg/Float64` | `ROS_TO_GZ` | Drives external inclined loader at `0.25 m/s` |
| `/aircraft/actuation/door_speed` | `/environment/door_belt_contact/speed` | `std_msgs/msg/Float64` | `ROS_TO_GZ` | Drives doorway threshold belt at `0.25 m/s` |
| `/aircraft/pose_target` | *(Controller Input)* | `std_msgs/msg/Float64MultiArray` | `ROS_INTERNAL` | Commands `[0.09, -1.1345, 0.0, 0.0]` (`[lift, yaw, reach, pitch]`) |

### 5.2 ROS 2 Command Execution Sequence
```bash
# Verify fresh feedback and controller readiness
ros2 topic echo --once /aircraft/status

# Advance external belt loader and doorway belt to stage first bag at doorway sill (Y = -0.61 m)
ros2 topic pub -r 10 /aircraft/actuation/loader_speed std_msgs/msg/Float64 '{data: 0.25}' &
ros2 topic pub -r 10 /aircraft/actuation/door_speed std_msgs/msg/Float64 '{data: 0.25}' &
ros2 topic pub -r 10 /aircraft/actuation/door_roller std_msgs/msg/Float64 '{data: 22.73}' &

# Command BellyBot to doorway-receiving readiness pose: 90 mm lift, -65 deg (-1.1345 rad) yaw
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray '{data: [0.09, -1.1345, 0.0, 0.0]}'
```

## 6. On-Screen HUD & Video Overlay Requirements

| Overlay Element | Exact Text / Metric to Display |
| :--- | :--- |
| **Top-Left Title Banner** | `0:00–0:08 | OVERALL CONCEPT: UNOBSTRUCTED HOLD & DOOR-SILL SUPPLY` |
| **Bottom-Left Telemetry** | `Hold Envelope: 7.70m (L) x 1.22m (W) x 1.12m (H) | Stowed Robot Height: 0.538m` |
| **Bottom-Right Clock** | `SIM TIME: 00.00s - 08.00s | PLAYBACK: 1.0x REAL-TIME` |
| **Callout Labels** | `1. External Apron Belt Loader` / `2. Cantilevered Doorway Threshold Belt (Terminates at Y = -0.61m)` / `3. BellyBot DockStack v3 (100% Free Cargo Floor)` |
