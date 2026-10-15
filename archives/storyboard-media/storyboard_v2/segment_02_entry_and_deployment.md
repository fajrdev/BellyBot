# Segment 02: Entry and Equipment Deployment (0:08–0:23)

## 1. Scene Overview & Judging Objective

| Attribute | Specification |
| :--- | :--- |
| **Video Timestamp** | `0:08–0:23` (15.0 seconds duration) |
| **Target World / Scene** | Part A (`0:08–0:17`): `entry_resolved.sdf` → Part B (`0:17–0:23`): `transfer_cutaway.sdf` (Doorway Deployment) |
| **Playback Speed** | Part A: `2.0x Accelerated Playback` (labeled on HUD, covering `18.0 s` of sim time in `9.0 s` video) / Part B: `1.0x Real-Time` (`6.0 s` sim time) |
| **Primary Judging Criteria** | Feasibility & Technical Soundness (15%), NB Aircraft Cargo-Hold Compatibility & Space Utilisation (15%) |
| **What It Establishes** | Demonstrates how BellyBot enters through the `1.22 m × 0.89 m` side cargo door via the temporary raised turn deck (`+180 mm`) and `5.71°` ramp, and how the simplified doorway threshold belt (`door_belt`) reaches its operating position at the doorway sill (`Y = -0.61 m`) without requiring any interior floor conveyors. |

## 2. Resolving the Deployment & Changeover Architecture

By eliminating the two interior floor conveyors (`corner_transfer` and `longitudinal_buffer`), equipment deployment is drastically simplified and physically credible:
1. **Stage 1 — Robot Ingress (`0:08–0:17`, `entry_resolved.sdf`)**: Empty, stowed BellyBot (`0.538 m` height, `1.10 m` width) drives straight through the `1.22 m` wide doorway (`60 mm` nominal jamb clearance per side) onto the `180 mm` raised turn deck (`model://entry_deck`), pivots `90°` in place at `(1.30, 0.00)` (`32.77 mm` minimum triangle-mesh clearance to the doorway liner), and drives down the `1.80 m` ramp (`5.71°` slope) onto the main cargo floor (`X = 4.90 m`).
2. **Stage 2 — Ramp Withdrawal & Doorway Threshold Belt Docking (`0:17–0:23`, `transfer_cutaway.sdf`)**: The lightweight modular entry deck/ramp is withdrawn back out the doorway before loading begins (explicitly labeled as a ground-crew changeover step), and the ground-supported cantilevered `door_belt` + `belt_loader` docks flush at the doorway sill (`X = 1.30 m, Y = -0.61 m, Z = 0.498 m`). BellyBot then drives in reverse (`-X`) along the unobstructed cargo floor from `X = 4.90 m` to its doorway-receiving station (`X = 1.85 m, Y = 0.00 m`) and deploys its upper carriage (`yaw_joint = -1.5708 rad` [`-90°`] at the doorway pocket, `extension = 0.15 m`) to couple directly with the doorway sill belt.

## 3. Technical Motion & Kinematic Specification

| Video Time | Sim Time Window | World | Active Trajectory & Joint States | Checked Clearance / Guard Metric |
| :--- | :--- | :--- | :--- | :--- |
| `0:08–0:11` | `t = 0.0–6.0 s` (2x) | `entry_resolved.sdf` | **INGRESS**: `base_link` drives `+Y` from `(1.30, -3.29, 0.183, yaw=+90°)` to `(1.30, 0.00, 0.183, yaw=+90°)` at `v = 0.08 m/s`. All arm joints stowed `[0, 0, 0, 0]`. | Jamb clearance `60.0 mm` nominal; path error `< 12 mm`. |
| `0:11–0:14` | `t = 6.0–12.0 s` (2x) | `entry_resolved.sdf` | **TURN**: Differential track pivot at `(1.30, 0.00, 0.183)` from `yaw = +1.5708 rad` (`+90°`) to `0.000 rad` (`0°`) at `w = -0.08 rad/s`. Left wheels `+0.44 rad/s`, right wheels `-0.44 rad/s`. | Min triangle-mesh turning clearance `32.77 mm`; turn center drift `< 15 mm`. |
| `0:14–0:17` | `t = 12.0–18.0 s` (2x) | `entry_resolved.sdf` | **EXIT RAMP**: `base_link` drives `+X` from `X = 1.30 m` down the `5.71°` ramp (`X = 2.26..4.06 m`) to `X = 4.90 m, Z = 0.003 m`. | Pitch `< 5.8°`, roof clearance `> 0.40 m`. |
| `0:17–0:20` | `t = 0.0–3.0 s` (1x) | `transfer_cutaway.sdf` | **REPOSITION TO DOORWAY**: With `entry_deck` removed and `door_belt` docked at `Y = -0.61 m`, BellyBot drives `-X` from `X = 2.45 m` to `X = 1.85 m` (`v = -0.20 m/s`), stopping aligned with the doorway edge. | Lateral error `|y| < 15 mm`, yaw error `< 0.015 rad`. |
| `0:20–0:23` | `t = 3.0–6.0 s` (1x) | `transfer_cutaway.sdf` | **FEED COUPLING**: Mast lifts to `0.10 m` (`0.0333 m`/stage), `yaw_joint` rotates to `-1.1345 rad` (`-65°`) or `-1.5708 rad` (`-90°` in doorway pocket `X = 1.30 m`), `extension` reaches `0.20 m` to meet `door_belt` discharge lip at `Y = -0.61 m`. | Handshake gap `< 15 mm` between `door_belt` and `tray_belt`. |

## 4. Gazebo Harmonic (`gz sim`) Configuration & Commands

### 4.1 Part A — Entry Simulation (`entry_resolved.sdf`)
```bash
# 1. Launch entry mode (loads entry_resolved.sdf with entry_deck and robot at (1.30, -3.29, 0.183, yaw=pi/2))
ros2 launch bellybot_aircraft_environment aircraft.launch.py mode:=entry

# 2. Unpause Gazebo physics
gz service -s /world/entry_resolved/control   --reqtype gz.msgs.WorldControl   --reptype gz.msgs.Boolean   --timeout 2000   --req 'pause: false'

# 3. Trigger autonomous guarded entry controller
ros2 topic pub --once /aircraft/start_entry std_msgs/msg/Empty '{}'
```

### 4.2 Part B — Equipment Deployment & Doorway Coupling (`transfer_cutaway.sdf`)
```bash
# 1. Launch transfer world (with corner_transfer and longitudinal_buffer removed)
ros2 launch bellybot_aircraft_environment aircraft.launch.py mode:=transfer cutaway:=true

# 2. Unpause Gazebo physics
gz service -s /world/transfer_cutaway/control   --reqtype gz.msgs.WorldControl   --reptype gz.msgs.Boolean   --timeout 2000   --req 'pause: false'
```

## 5. ROS 2 Jazzy Communications & Controller Updates

### 5.1 Required Update to `aircraft_controller.py` for Longitudinal Repositioning (`REPOSITION` Mode)
In the original `aircraft_controller.py`, work mode locked the base permanently at `X = 4.10 m` (`dx > 0.060` fault) because `longitudinal_buffer` blocked `X < 3.656 m`. With `corner_transfer` and `longitudinal_buffer` removed:
- Update `aircraft_controller.py` to support `/aircraft/base_x_target` (`std_msgs/msg/Float64`, valid range `X ∈ [1.30, 5.20] m`).
- When a new `/aircraft/base_x_target` is received while the upper arm is stowed (`lift <= 0.10 m, abs(yaw) < 0.02 rad, reach < 0.01 m, pitch < 0.01 rad`) and belts are stopped, `aircraft_controller.py` drives `base_link` longitudinally along `Y = 0, yaw = 0` using proportional velocity control (`v = clamp((x_target - x) * 0.8, -0.20, 0.20)`, `w = clamp(-yaw * 1.5 - y * 2.0, -0.05, 0.05)`) until `|x - x_target| < 0.010 m`, and then locks the tracks at the new datum `self.work_x = x_target`.

### 5.2 ROS 2 Command Sequence for Part B (`0:17–0:23`)
```bash
# Drive BellyBot longitudinally along unobstructed hold floor to doorway station X = 1.85 m (or X = 1.30 m)
ros2 topic pub --once /aircraft/base_x_target std_msgs/msg/Float64 '{data: 1.85}'

# Wait for status to report base_locked: true at X = 1.85 m, then deploy receiving arm toward doorway sill
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray '{data: [0.10, -1.1345, 0.20, 0.0]}'
```

## 6. On-Screen HUD & Video Overlay Requirements

| Video Window | Overlay Text / Telemetry Callout |
| :--- | :--- |
| `0:08–0:17` | `0:08–0:17 | ENTRY: 180mm RAISED TURN DECK & 5.71° RAMP | PLAYBACK: 2.0x (SIM TIME: 0.0s - 18.0s)` |
| `0:08–0:17` | `Door Jamb Clearance: 60.0 mm | Min Pivot Mesh Clearance: 32.8 mm | Track Width: 1.10 m` |
| `0:17–0:23` | `0:17–0:23 | DEPLOYMENT: RAMP REMOVED (SETUP CHANGEOVER) & CANTILEVERED DOOR BELT DOCKED | PLAYBACK: 1.0x` |
| `0:17–0:23` | `Zero Interior Floor Conveyors | Full Hold Floor Unobstructed (X = 0.69m to 7.70m)` |
