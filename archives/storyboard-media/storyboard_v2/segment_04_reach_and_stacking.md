# Segment 04: Reach, Multi-Layer Stacking & Repositioning Supply (0:53–1:13)

## 1. Scene Overview & Judging Objective

| Attribute | Specification |
| :--- | :--- |
| **Video Timestamp** | `0:53–1:13` (20.0 seconds duration) |
| **Target World / Scene** | `transfer_cutaway.sdf` (unobstructed floor, `corner_transfer` & `longitudinal_buffer` removed) |
| **Playback Speed** | `1.5x Accelerated Playback` (explicitly badged on HUD; covers `30.0 s` of simulation time in `20.0 s` video) |
| **Primary Judging Criteria** | Functional Effectiveness & Baggage Handling Strategy (20%), NB Aircraft Cargo-Hold Compatibility & Space Utilisation (15%), Feasibility & Technical Soundness (15%) |
| **What It Establishes** | Demonstrates (1) controlled floor-level placement (`30.922°` downward tray pitch with `2 mm` nose clearance), (2) upper-layer lateral stack placement (`+65°` upper yaw + `0.50 m` reach at elevated mast height `0.19 m`–`0.23 m`), and (3) how baggage supply is maintained when BellyBot changes longitudinal position (`X`) along the hold floor. |

## 2. How Baggage Supply Works When BellyBot Changes Position (Without Interior Conveyors)

Removing the two interior floor conveyors (`corner_transfer` and `longitudinal_buffer`) solves the hardest architectural question in the judge's rubric: **how does BellyBot stack bags across the full depth of the hold (`X = 2.0 m` to `X = 7.0 m`) and maintain baggage supply as the stack grows toward the door?**
- **Progressive Aft-to-Doorway Retreat Strategy**:
  1. BellyBot loads the hold from **aft to forward** (starting near the aft bulkhead `X = 5.50 m` and retreating progressively toward the doorway `X = 2.00 m` as each bay of 2-layer stacks is completed).
  2. Because the two interior floor conveyors are removed, BellyBot's crawler tracks (`left_track_link`, `right_track_link`) have a completely clear longitudinal runway between the doorway (`X = 1.30..1.85 m`) and the active stacking front (`X_stack - 1.40 m`).
  3. To receive a bag when operating deeper in the hold (`X_stack = 4.10 m`), BellyBot drives in reverse (`-X` at `0.20 m/s`) to the doorway handoff datum (`X = 1.85 m`), acquires the staged bag directly from `door_belt` (`Y = -0.61 m`), stows its upper carriage (`lift = 0.09 m, yaw = 0.0, reach = 0.0`), drives forward (`+X` at `0.20 m/s`) to the current active stacking line (`X = 4.10 m` for Bay 2, then `X = 3.10 m` for Bay 3, then `X = 2.10 m` for Bay 4), and places the bag onto either the **floor layer (Layer 1)** or the **upper layer (Layer 2)**.
  4. As the cargo hold fills toward the doorway (`X = 2.10 m`), BellyBot's longitudinal shuttle distance shrinks to **zero**, transitioning seamlessly into stationary pivot-and-stack operation right at the doorway!

## 3. Technical Motion & Kinematic Sequence (`30.0 s` Sim Time in `20.0 s` Video @ `1.5x`)

### Sub-Scene 4A: Controlled Floor Placement — Layer 1 (`0:53–1:00` Video | `t_sim = 0.0–10.5 s`)
Demonstrates placing a `32 kg` hard case (`bag_hard_32`) directly onto the cargo hold floor (`Z = 0.000 m`) at `X = 5.30 m`:
1. **Guarded Floor Approach (`t = 0.0–3.5 s`)**: At datum `X = 4.10 m`, `aircraft_guard.py` enforces `lift = 0.000 m, yaw = 0.000 rad` and extends `extension_mid_joint = 0.42 m, extension_tip_joint = 0.42 m` (`0.84 m` total reach) **before** pitching the tray downward to `tray_pitch_joint = 0.539691 rad` (`30.922°`). Nose underside sits at `Z = 0.002 m` (`2.0 mm` nominal floor clearance).
2. **Floor Insertion & Belt Discharge (`t = 3.5–7.0 s`)**: Preserving `pitch = 0.539691 rad`, reach extends from `0.84 m` to `1.00 m` (`0.50 m` per stage, nose at `X = 5.442 m`) while `tray_belt` runs forward at `+0.35 m/s` to slide the bag gently onto the cargo floor without dropping it.
3. **Level-Before-Retract Guard (`t = 7.0–10.5 s`)**: Reach withdraws to `0.84 m`, `tray_pitch_joint` levels to `0.000 rad` first (preventing the tray underside from striking the chassis), and `extension` retracts to `0.000 m`.

### Sub-Scene 4B: Longitudinal Repositioning & Doorway Re-Supply (`1:00–1:06` Video | `t_sim = 10.5–19.5 s`)
Demonstrates how baggage supply is maintained when BellyBot changes position:
1. **Shuttle to Doorway (`t = 10.5–14.5 s`)**: With the tray stowed (`[0.09, 0.0, 0.0, 0.0]`), BellyBot drives `-X` along the unobstructed floor to `X = 1.85 m`, swings `yaw_joint = -1.1345 rad` (`-65°`), and receives the next bag (`bag_hard_20`, `20 kg, height = 0.26 m`) from `door_belt`.
2. **Advance to New Stacking Line (`t = 14.5–19.5 s`)**: BellyBot centers `yaw_joint = 0.0 rad`, drives `+X` to the stepped-back stacking datum `X = 3.85 m`, and locks its tracks.

### Sub-Scene 4C: Upper-Layer Placement with Sideways Yaw — Layer 2 (`1:06–1:13` Video | `t_sim = 19.5–30.0 s`)
Demonstrates placing the `0.26 m` tall bag (`bag_hard_20`) onto the **upper layer (`Z = 0.32 m`)** of the off-center stack using the corrected V3 THK RU178X rotary yaw bearing:
1. **Guarded Raise Before Yaw (`t = 19.5–22.5 s`)**: Mast raises to `lift = 0.21 m` (`0.070 m` per stage; tray surface `Z = 0.708 m`, retaining `152 mm` roof clearance to the `1.12 m` ceiling) with `reach = 0.0 m, pitch = 0.0 rad`.
2. **Upper Yaw Rotation (`t = 22.5–25.5 s`)**: `yaw_joint` rotates smoothly to `+1.134464 rad` (`+65.0°`) toward the port-side stack column (`+Y`), keeping the crawler tracks stationary and lengthwise on the floor.
3. **Upper-Layer Reach & Deposit (`t = 25.5–30.0 s`)**: Telescopic slides extend to `0.50 m` (`0.25 m` per stage, within the guarded `500 mm` yawed-reach envelope so side-liner clearance remains `> 25 mm`), and `tray_belt` discharges the bag at `+0.30 m/s` onto the top of the Layer 1 bag (`Z = 0.32 m`), followed by retraction to home.

## 4. Gazebo Harmonic (`gz sim`) & ROS 2 Jazzy Commands

```bash
# --- SUB-SCENE 4A: FLOOR PLACEMENT (Pose from docs/poses.json: floor_approach -> floor_insert) ---
# 1. Extend level to 0.84 m, then pitch nose down 30.922 deg (0.53969 rad)
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray '{data: [0.0, 0.0, 0.84, 0.53969]}'

# 2. Advance along floor to 1.00 m reach and run tray belt forward (+0.35 m/s) to deposit on floor
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray '{data: [0.0, 0.0, 1.00, 0.53969]}'
ros2 topic pub -r 10 /aircraft/actuation/tray_belt_speed std_msgs/msg/Float64 '{data: 0.35}'
ros2 topic pub -r 10 /aircraft/actuation/tray_roller_speed std_msgs/msg/Float64 '{data: 35.0}'

# 3. Stop belt and return to stowed home (guard automatically levels pitch to 0.0 before retracting)
ros2 topic pub --once /aircraft/actuation/tray_belt_speed std_msgs/msg/Float64 '{data: 0.0}'
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray '{data: [0.0, 0.0, 0.0, 0.0]}'

# --- SUB-SCENE 4B: LONGITUDINAL REPOSITIONING & SUPPLY ---
# 4. Retreat to doorway handoff station X = 1.85 m, receive bag_hard_20 (height=0.26m), then advance to X = 3.85 m
ros2 topic pub --once /aircraft/base_x_target std_msgs/msg/Float64 '{data: 1.85}'
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray '{data: [0.10, -1.1345, 0.20, 0.0]}'
# (Acquire bag from door_belt, then stow and advance to new bay datum X = 3.85 m)
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray '{data: [0.09, 0.0, 0.0, 0.0]}'
ros2 topic pub --once /aircraft/base_x_target std_msgs/msg/Float64 '{data: 3.85}'

# --- SUB-SCENE 4C: UPPER-LAYER PLACEMENT (+65 DEG YAW, 210 MM LIFT, 500 MM REACH) ---
# 5. Set bag_height parameter to 0.28 m (allows up to 0.23 m total lift under 1.12 m ceiling)
ros2 param set /aircraft_controller bag_height 0.28
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray '{data: [0.21, 1.134464, 0.50, 0.0]}'
ros2 topic pub -r 10 /aircraft/actuation/tray_belt_speed std_msgs/msg/Float64 '{data: 0.30}'
```

## 5. On-Screen HUD & Video Overlay Requirements

| Sub-Scene | Overlay Text / Telemetry Callout |
| :--- | :--- |
| `0:53–1:00` | `FLOOR PLACEMENT (LAYER 1): Pitch 30.92° (0.540 rad) | Reach 1.00 m | Nose Floor Clearance: 2.0 mm` |
| `1:00–1:06` | `REPOSITIONING SUPPLY: Aft-to-Doorway Progressive Loading | Clear Floor Shuttle (X = 1.85m <-> 3.85m)` |
| `1:06–1:13` | `UPPER-LAYER STACKING (LAYER 2): Lift 210 mm | Upper Yaw +65.0° | Reach 500 mm | Roof Margin: 130 mm | Wall Margin: >25 mm` |
