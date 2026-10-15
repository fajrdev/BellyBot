# Storyboard Segment 4: 2nd-Layer Stacking Maneuver (1:15 – 1:45)
## Technical ROS 2 Architecture & Simulation Approach

**Segment Duration:** 30 Seconds  
**Focus:** 5-waypoint autonomous transition from floor posture to upper-layer stacking under the $< 1.2\text{ m}$ confined ceiling, ceiling clearance verification, and sidewall yaw articulation.  
**Primary Judging Criterion:** NB Aircraft Cargo-Hold Compatibility & Space Utilisation (15%), Autonomy & Intelligent Operation (15%), Feasibility & Technical Soundness (15%).

---

## 1. Visual Objectives & Camera Setup for Video Production

### 1.1 Shot List & Cinematic Directives
- **Shot 4.1 (1:15 – 1:22) — Tray Leveling & Arm Retraction:**
  - *Angle:* Low 3/4 front perspective inside the cargo hold aisle.
  - *Visual Goal:* Show the tray leveling back from $+30.92^\circ$ to horizontal ($0^\circ$), followed immediately by the telescopic arm retracting fully into the stowed chassis footprint ($reach = 0\text{ m}$).
- **Shot 4.2 (1:22 – 1:35) — Stacking Mast Vertical Lift & Ceiling Clearance Check:**
  - *Angle:* Side-elevation cross-section view showing the full height of the aircraft cargo compartment ($1.15\text{ m}$ ceiling to floor).
  - *Visual Goal:* Show the 3-stage telescopic mast rising to its maximum safe allowable stroke ($+130\text{ mm}$). Clearly annotate the ceiling safety envelope: with a large $380\text{ mm}$ suitcase loaded, the top of the luggage maintains $\mathbf{+314\text{ mm}}$ clearance to the cargo ceiling panels (exceeding the $100\text{ mm}$ mandatory margin).
- **Shot 4.3 (1:35 – 1:45) — Yaw Swing & Upper Tier Placement:**
  - *Angle:* Overhead cutaway view looking down at the cargo hold floor.
  - *Visual Goal:* Show the arm executing a smooth yaw pivot of $+28.6^\circ$ ($+0.50\text{ rad}$) toward the starboard hold sidewall, followed by a $0.400\text{ m}$ telescopic extension to seat the bag squarely onto Layer 1.

### 1.2 Camera Poses in Gazebo SDF
```xml
<!-- Shot 4.1: Retraction Profile in Hold Aisle -->
<camera name='camera_retract_profile'>
  <pose>4.30 -1.50 0.70 0 0.12 1.5708</pose>
</camera>

<!-- Shot 4.2: Side-Elevation Ceiling Clearance Cross-Section -->
<camera name='camera_ceiling_clearance'>
  <pose>3.90 -1.80 0.85 0 0.05 1.5708</pose>
</camera>

<!-- Shot 4.3: Overhead Cutaway View on Stacking Yaw & Extension -->
<camera name='camera_yaw_stack'>
  <pose>4.20 0.00 1.95 0 1.5708 0</pose>
</camera>
```

---

## 2. Technical ROS 2 Architecture

### 2.1 The 5-Waypoint Autonomous Stacking Plan
When transitioning from floor posture to upper-tier stacking, commanding `[0.13, 0.50, 0.40, 0.0]` generates an autonomous 5-waypoint sequence in `aircraft_guard.py`:

```mermaid
flowchart TD
    S0["Floor Posture\n[lift=0, yaw=0, reach=0.747m, pitch=30.92°]"] -->|WP 1| S1["1. Level Pitch\n[0, 0, 0.747m, 0.0rad]\n(dur = 8.43s)"]
    S1 -->|WP 2| S2["2. Retract Reach\n[0, 0, 0.0m, 0.0rad]\n(dur = 7.00s)"]
    S2 -->|WP 3| S3["3. Raise Mast Lift\n[0.13m, 0, 0.0m, 0.0rad]\n(dur = 3.02s)"]
    S3 -->|WP 4| S4["4. Yaw Swing to Wall\n[0.13m, 0.50rad, 0.0m, 0.0rad]\n(dur = 4.69s)"]
    S4 -->|WP 5| S5["5. Upper Reach Extension\n[0.13m, 0.50rad, 0.40m, 0.0rad]\n(dur = 3.75s)"]
```

### 2.2 Active ROS 2 Actuation Mapping
| Segment | Primary Joint Commanded | Commanded ROS Topic | Value |
|---|---|---|---|
| **WP 1** | `tray_pitch_joint` | `/aircraft/actuation/pitch_target` | $0.0000\text{ rad}$ ($0^\circ$) |
| **WP 2** | `extension_mid_joint`, `extension_tip_joint` | `/aircraft/actuation/extension_stage_target` | $0.0000\text{ m}$ (per stage) |
| **WP 3** | `lift_stage_1_joint`, `lift_stage_2_joint`, `lift_stage_3_joint` | `/aircraft/actuation/lift_stage_target` | $0.130 / 3 = 0.0433\text{ m}$ |
| **WP 4** | `yaw_joint` | `/aircraft/actuation/yaw_target` | $+0.5000\text{ rad}$ ($+28.6^\circ$) |
| **WP 5** | `extension_mid_joint`, `extension_tip_joint` | `/aircraft/actuation/extension_stage_target` | $0.400 / 2 = 0.2000\text{ m}$ |

---

## 3. Ceiling Confinement & Lift Cap Formula

### 3.1 Strict Flight Clearance Calculation
To prevent any possibility of hull or liner strikes, the maximum mast lift is dynamically capped based on real-time measured payload height $h$:
$$\text{lift\_cap}(h) = \max\left(0, \min\left(0.60, \left\lfloor \frac{1.120 - 0.100 - 0.003 - 0.495 - \max(0.040, h + 0.005)}{0.01} \right\rfloor \times 0.01 \right)\right)$$
Where:
- $1.120\text{ m}$: Nominal ceiling height datum
- $0.100\text{ m}$: Mandatory safety buffer margin to ceiling liner
- $0.003\text{ m}$: Robot datum offset above aircraft floor
- $0.495\text{ m}$: Stowed tray deck height from floor
- $h = 0.380\text{ m}$: Maximum baggage height under hackathon specification ($0.38\text{ m}$)
- **Result:** $\text{lift\_cap}(0.38) = \mathbf{0.130\text{ m}}$

### 3.2 Dynamic Yaw Interlock
In `aircraft_guard.py`, yaw rotation is structurally interlocked:
```python
if abs(yaw) > 0.01 and (lift < 0.08 or reach > 0.50):
    raise ValueError("Yaw requires >=80 mm lift and <=500 mm reach")
```
*Rationale:* Prevents the tray from colliding with the chassis side batteries or wheel covers during swing motions.

---

## 4. Step-by-Step Recording & Verification Workflow

### 4.1 Standalone Trajectory Trigger
```bash
# Sourcing workspace
source /opt/ros/jazzy/setup.bash && source /home/fajrda/sutd-hackathon/ros2_ws/install/setup.bash

# Triggering 5-waypoint stacking trajectory
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray "{data: [0.13, 0.50, 0.40, 0.0]}"
```

### 4.2 HUD Overlay Telemetry Directives
- **1:15:** Overlay: `PHASE 4: 2ND-LAYER STACKING IN CONFINED ENVELOPE (< 1.2 m)`
- **1:25:** Height Callout:
  - `Telescopic Mast Lift: +130 mm (Dynamic Bag Height Cap)`
  - `Observed Roof Clearance: +314 mm (Safety margin ≥ 100 mm)`
- **1:35:** Yaw & Wall Callout:
  - `Stacking Yaw Swing: +28.6° (+0.50 rad) toward Sidewall`
  - `Sidewall Clearance: +28.5 mm (Safety margin ≥ 25 mm)`
- **1:42:** Final Extension: `Extension Reach: 0.40 m onto Layer 1`
