# Segment 06: Fault Stop and Controlled Recovery (1:33–1:48)

## 1. Scene Overview & Judging Objective

| Attribute | Specification |
| :--- | :--- |
| **Video Timestamp** | `1:33–1:48` (15.0 seconds duration) |
| **Target World / Scene** | `transfer_cutaway.sdf` |
| **Playback Speed** | `1.0x Real-Time` (`15.0 s` simulation time) |
| **Primary Judging Criteria** | Safety and Human Interaction (15%), Autonomy & Intelligent Operation (15%), Feasibility & Technical Soundness (15%) |
| **What It Establishes** | Demonstrates an **implemented, code-backed safety response** to a misaligned/obstructed bag and/or an envelope/emergency stop violation in `aircraft_controller.py` / `aircraft_guard.py`, followed by fault clearing and controlled recovery without silent resets or teleportation. |

## 2. Implemented Fault & Recovery Mechanisms in the Codebase

The existing `aircraft_controller.py`, `transfer_core.py`, and `aircraft_guard.py` already implement four verifiable safety interlocks:
1. **Misaligned / Skewed Bag Interlock (`transfer_core.py` & Doorway Receive Guard)**:
   - Rejects transfer and halts belts immediately (`state = 'FAULT'`) if `tracked_bag` lateral offset exceeds `25 mm` (`abs(bag['x'] - 1.30) > 0.025`) or yaw skew exceeds `3.0°` (`abs(yaw_err) > math.radians(3)`), or if `bag['y']` drifts `> 35 mm` off the conveyor centerline.
2. **Enclosure & Roof-Height Interlock (`aircraft_guard.py`)**:
   - Rejects any pose command that violates the `100 mm` roof clearance margin (`1.12 m` ceiling), `25 mm` sloped side-wall clearance margin, or `600 kg/m²` 4-patch assigned-mass floor pressure limit.
3. **Latched Emergency Stop (`/aircraft/stop`, `std_msgs/msg/Bool`)**:
   - Publishing `data: true` to `/aircraft/stop` immediately latches `self.estop = True`, zeroes all crawler track velocities (`cmd_vel = 0`), halts all conveyor belts (`loader = door = tray = 0.0`), clears the active trajectory queue (`self.queue = []`), and holds current joint positions (`self.held()`).
4. **Controlled Recovery Protocol**:
   - Add `/aircraft/clear_fault_and_recover` (`std_msgs/msg/Empty`) (or controller reset + guarded retract) so that after a skewed bag or obstruction fault is detected and halted, the controller reverses the belt at low speed (`-0.10 m/s` for `2.0 s`) to back the skewed bag against the self-centering side guides (`soft_side_guide_±290`), verifies re-alignment (`yaw_err < 3°`, `lateral_err < 25 mm`), clears the fault state, and resumes normal transfer.

## 3. Technical Motion & Timeline (`1:33–1:48` | `15.0 s` Real-Time)

| Video Time | Sim Time (`t`) | Event Stage | System / Controller Behavior | `/aircraft/status` JSON Output |
| :--- | :--- | :--- | :--- | :--- |
| `1:33–1:37` | `0.0–4.0 s` | **1. Misaligned Bag Injection / Obstruction** | A skewed bag (`tracked_bag` spawned with `+8.5°` yaw skew and `+32 mm` lateral offset on `door_belt`, or an over-height lift request `0.22 m` for a `0.38 m` fragile bag) approaches the transfer interface. | `{"mode": "transfer", "transfer": "RECEIVE", "fault": "", "estop": false}` |
| `1:37–1:40` | `4.0–7.0 s` | **2. Automatic Guard Trip & Latched Stop** | `aircraft_controller.py` detects `abs(bag['x'] - 1.30) = 0.032 > 0.025 m` (or `/aircraft/stop = true` from front depth camera `/model/bellybot_dock_stack/depth` obstacle check). Belts and joints halt within `0.02 s` (1 tick). Status light / HUD flashes `FAULT LATCHED`. | `{"mode": "transfer", "transfer": "FAULT", "fault": "Bag is not aligned with door belt", "estop": true}` |
| `1:40–1:44` | `7.0–11.0 s` | **3. Controlled Recovery Jog** | Operator/supervisor issues `/aircraft/recover`. Controller clears latched fault, runs a `2.0 s` low-speed reverse centering jog (`-0.10 m/s`) between the `soft_side_guide` rails, and retracts arm to safe home `[0.10, -1.1345, 0.0, 0.0]`. | `{"mode": "transfer", "transfer": "RECOVERING", "fault": "", "estop": false}` |
| `1:44–1:48` | `11.0–15.0 s` | **4. Verified Re-Alignment & Resume** | Pose feedback confirms bag is centered (`|dx| = 0.006 m < 0.025 m`, `|dyaw| = 0.8° < 3.0°`). Transfer resumes at `0.25 m/s` and completes cleanly onto `tray_belt`. | `{"mode": "transfer", "transfer": "DONE", "fault": "", "estop": false}` |

## 4. Gazebo Harmonic (`gz sim`) & ROS 2 Jazzy Commands

```bash
# 1. Spawn a deliberately misaligned bag (+32 mm X offset, +8.5 deg yaw skew) on door_belt to trigger the real guard
gz service -s /world/transfer_cutaway/create   --reqtype gz.msgs.EntityFactory   --reptype gz.msgs.Boolean   --timeout 4000   --req 'sdf_filename: "model://bag_fragile_20/model.sdf" name: "tracked_bag" pose: {position: {x: 1.332 y: -1.40 z: 0.690} orientation: {x: 0 y: 0 z: 0.7576 w: 0.6528}}'

# 2. Trigger transfer -> controller immediately trips fault and stops all belts
ros2 topic pub --once /aircraft/start_transfer std_msgs/msg/Empty '{}'
ros2 topic echo --once /aircraft/status

# 3. Also demonstrate explicit /aircraft/stop latching and guarded pose rejection (e.g., requesting 0.25m lift with 0.38m bag)
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray '{data: [0.25, 0.0, 0.50, 0.0]}'
# Controller logs: "Rejected arm target: Aircraft joint operating limit exceeded (cap=0.13m)"

# 4. Trigger controlled recovery (re-centers bag and resumes transfer)
ros2 topic pub --once /aircraft/recover std_msgs/msg/Empty '{}'
```

## 5. On-Screen HUD & Video Overlay Requirements

| Overlay Region | Exact Content |
| :--- | :--- |
| **Top-Left Banner** | `1:33–1:48 | SAFETY INTERLOCK, FAULT STOP & CONTROLLED RECOVERY` |
| **Live `/aircraft/status` Box** | Display real-time JSON terminal output from `ros2 topic echo /aircraft/status` showing `FAULT` transition and recovery |
| **Bottom-Left Safety Metrics** | `Watchdog Timeout: 0.35 s | Roof Margin Guard: >= 100 mm | Wall Margin Guard: >= 25 mm | Alignment Tol: +/-25 mm, +/-3.0°` |
| **Bottom-Right Timer** | `SIM TIME: 15.0s | PLAYBACK: 1.0x REAL-TIME` |
