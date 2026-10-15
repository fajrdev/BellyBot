# Segment 05: Unloading & Reverse Doorway Transfer (1:13–1:33)

## 1. Scene Overview & Judging Objective

| Attribute | Specification |
| :--- | :--- |
| **Video Timestamp** | `1:13–1:33` (20.0 seconds duration) |
| **Target World / Scene** | `transfer_cutaway.sdf` (or `floor_pickup.sdf` / `transfer_cutaway.sdf` with accessible cargo stack) |
| **Playback Speed** | `1.5x Accelerated Playback` (badged on HUD; covers `30.0 s` of simulation time in `20.0 s` video) |
| **Primary Judging Criteria** | Functional Effectiveness & Baggage Handling Strategy (20%), Feasibility & Technical Soundness (15%) |
| **What It Establishes** | Shows an accessible bag being retrieved from the hold and transferred back out onto the doorway belt (`door_belt` → `belt_loader` running in reverse), while clearly explaining the **Last-In, First-Out (LIFO) Top-Down, Door-to-Aft Removal Order**. |

## 2. Unloading Strategy & Removal Order Explanation

A complete cargo-hold robotics demonstration must explain **removal order** so BellyBot never attempts to pull a trapped bottom-layer bag from beneath an upper-layer load:
1. **Bay-by-Bay Order (Forward-to-Aft / LIFO)**: Unloading proceeds from the **doorway bay (`X = 2.10 m`) toward the aft bulkhead (`X = 6.85 m`)**, reversing the loading progression so BellyBot always has an unobstructed longitudinal approach path to the front face of the cargo stack.
2. **Layer-by-Layer Order (Top-Down within Each Bay)**: Within each bay (`X` station), **Layer 2 (upper layer, `Z = 0.32 m`) is retrieved first** at level tray pitch (`tray_pitch_joint = 0.0 rad`, `lift = 0.13..0.19 m`), exposing the **Layer 1 (floor layer, `Z = 0.00 m`)** bags underneath.
3. **Floor-Layer Acquisition**: Once Layer 2 is cleared, BellyBot lowers to `lift = 0.00 m`, extends to `0.84 m`, pitches the tapered replaceable floor nose down to `30.922°` (`0.539691 rad`, `2 mm` floor clearance), inserts the nose `160 mm` (`reach = 0.84 m → 1.00 m`) beneath the leading edge of the floor bag while running `tray_belt` in reverse (`-0.25 m/s`, pulling toward the robot), levels the cassette, retracts, and shuttles/rotates to discharge the bag onto the reversed doorway belt (`door_belt` running at `-0.25 m/s` toward the exterior `belt_loader`).

## 3. Technical Motion & Kinematic Sequence (`30.0 s` Sim Time in `20.0 s` Video @ `1.5x`)

| Video Time | Sim Time Window | Phase | Joint Targets `[lift, yaw, reach, pitch]` & Belt Commands | Physical Verification |
| :--- | :--- | :--- | :--- | :--- |
| `1:13–1:18` | `t = 0.0–7.5 s` | **1. Floor Approach & Nose Insertion** | At `X = 3.85 m`, command `[0.0, 0.0, 0.84, 0.539691]` (extend level first, then pitch nose down `30.922°`). Once settled, advance to `[0.0, 0.0, 1.00, 0.539691]` so the tapered nose (`replaceable_tapered_floor_nose`) slides under the front edge of `tracked_bag` at `X = 5.15 m`. | Nose clearance `Z = 2.0 mm`; uphill gravity component `161.3 N` (32 kg bag) requires belt `mu >= 0.60` (`tray_belt` has `mu = 0.70`). |
| `1:18–1:23` | `t = 7.5–15.0 s` | **2. Reverse Belt Acquisition & Leveling** | Run `tray_belt` inward (`-0.25 m/s`, `tray_roller_speed = -25.0 rad/s`) to draw the bag up the `30.922°` cassette ramp onto the tray center. Stop belt (`0.0 m/s`), retract to `0.84 m`, level `tray_pitch_joint` to `0.0 rad`, and retract reach to `0.0 m`. | Bag CoM supported over `tray_belt` (`Z ≈ 0.65 m` when leveled); 4-patch floor pressure `< 595 kg/m²`. |
| `1:23–1:28` | `t = 15.0–22.5 s` | **3. Shuttle to Doorway & Yaw Alignment** | Raise mast to `lift = 0.10 m`, drive `base_link` in reverse (`-X` at `-0.20 m/s`) along the clear floor from `X = 3.85 m` to doorway station `X = 1.85 m`, and rotate `yaw_joint` to `-1.1345 rad` (`-65°`) or `-1.5708 rad` (`-90°` at `X = 1.30 m`) facing the doorway sill (`Y = -0.61 m`). | Minimum wall clearance `> 25 mm`; roof clearance `> 110 mm`. |
| `1:28–1:33` | `t = 22.5–30.0 s` | **4. Reverse Doorway Discharge** | Extend tray `0.20 m` to meet `door_belt` at `Y = -0.61 m`. Drive `tray_belt` outward (`+0.25 m/s`), `door_belt` outward (`-0.25 m/s` toward `-Y`), and `belt_loader` downward (`-0.25 m/s` toward apron), transferring the retrieved bag out of the aircraft door and down the external loader. | Bag exits doorway aperture (`Y < -1.54 m`) smoothly on `door_belt` → `belt_loader`. |

## 4. Gazebo Harmonic (`gz sim`) & ROS 2 Jazzy Commands

```bash
# 1. Command guarded floor approach (reach 0.84 m, pitch 0.539691 rad)
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray '{data: [0.0, 0.0, 0.84, 0.539691]}'

# 2. Insert tapered nose under accessible floor bag (reach 1.00 m) and pull bag inward (-0.25 m/s)
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray '{data: [0.0, 0.0, 1.00, 0.539691]}'
ros2 topic pub -r 10 /aircraft/actuation/tray_belt_speed std_msgs/msg/Float64 '{data: -0.25}'
ros2 topic pub -r 10 /aircraft/actuation/tray_roller_speed std_msgs/msg/Float64 '{data: -25.0}'

# 3. Stop belt once bag is centered on tray, then level and retract to stowed pose [0.10, 0.0, 0.0, 0.0]
ros2 topic pub --once /aircraft/actuation/tray_belt_speed std_msgs/msg/Float64 '{data: 0.0}'
ros2 topic pub --once /aircraft/actuation/tray_roller_speed std_msgs/msg/Float64 '{data: 0.0}'
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray '{data: [0.10, 0.0, 0.0, 0.0]}'

# 4. Shuttle back to doorway handoff station (X = 1.85 m) and rotate upper carriage toward doorway (-1.1345 rad)
ros2 topic pub --once /aircraft/base_x_target std_msgs/msg/Float64 '{data: 1.85}'
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray '{data: [0.10, -1.1345, 0.20, 0.0]}'

# 5. Discharge bag onto reversed doorway belt and external apron loader (-0.25 m/s)
ros2 topic pub -r 10 /aircraft/actuation/tray_belt_speed std_msgs/msg/Float64 '{data: 0.25}' &
ros2 topic pub -r 10 /aircraft/actuation/door_speed std_msgs/msg/Float64 '{data: -0.25}' &
ros2 topic pub -r 10 /aircraft/actuation/door_roller std_msgs/msg/Float64 '{data: -22.73}' &
ros2 topic pub -r 10 /aircraft/actuation/loader_speed std_msgs/msg/Float64 '{data: -0.25}' &
```

## 5. On-Screen HUD & Video Overlay Requirements

| Overlay Region | Exact Content |
| :--- | :--- |
| **Top-Left Banner** | `1:13–1:33 | UNLOADING OPERATION: LIFO REMOVAL ORDER & REVERSE DOOR TRANSFER` |
| **Top-Right Order Graphic** | `REMOVAL ORDER: 1. Door-to-Aft Bays (LIFO) -> 2. Upper Layer First (Level Tray) -> 3. Floor Layer (30.9° Tapered Nose Pickup)` |
| **Bottom-Left Telemetry** | `Floor Pickup Pitch: 30.92° | Belt Friction: mu = 0.70 (Req >= 0.60) | Reverse Doorway Transfer: -0.25 m/s` |
| **Bottom-Right Timer** | `SIM TIME: 30.0s Window | PLAYBACK: 1.5x ACCELERATED` |
