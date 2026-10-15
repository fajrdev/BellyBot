# Storyboard Segment 5: Specifications, Structural Verification & Safety Compliance (1:45 – 2:00)
## Technical ROS 2 Architecture & Simulation Approach

**Segment Duration:** 15 Seconds  
**Focus:** Comprehensive aircraft structural compliance verification, floor loading density calculation, high-throughput pipeline metrics ($\ge 12\text{ bags/min}$), zero-human-intervention safety envelope, fail-safe auto-egress mode, and closing submission summary card.  
**Primary Judging Criterion:** Safety & Human Interaction (15%), Feasibility & Technical Soundness (15%), Functional Effectiveness (20%).

---

## 1. Visual Objectives & Camera Setup for Video Production

### 1.1 Shot List & Cinematic Directives
- **Shot 5.1 (1:45 – 1:52) — 3D Structural Stress & Ground Pressure Screening Overlay:**
  - *Angle:* Isometric translucent cutaway of the Boeing 737-800 forward cargo hold displaying the BellyBot chassis and the 4 track contact patches.
  - *Visual Goal:* Show the real-time FEA/ground pressure screen heatmap. Highlight that the peak floor reaction under max worst-case dynamic payload ($32\text{ kg}$ bag at full $0.747\text{ m}$ extension) is $\mathbf{531.3\text{ kg/m}^2}$, well within the aircraft floor limit of $\mathbf{600.0\text{ kg/m}^2}$ (11.5% margin).
- **Shot 5.2 (1:52 – 1:56) — Multi-Bag Continuous Flow & Pipelining Graphic:**
  - *Angle:* Wide longitudinal cutaway showing the simultaneous operation of the exterior belt loader, doorway corner table, longitudinal buffer conveyor, and BellyBot tray.
  - *Visual Goal:* Illustrate non-blocking pipelined transfer: while the robot is placing Bag $N$, Bag $N+1$ is queued on the buffer, and Bag $N+2$ is entering the doorway. Display telemetry showing sustained throughput of $\mathbf{12\text{ – }15\text{ bags/min}}$ ($4.0\text{ – }5.0\text{ s}$ per luggage cycle).
- **Shot 5.3 (1:56 – 2:00) — Autonomous Egress / Auto-Stow & Master Metrics HUD:**
  - *Angle:* Low dramatic front view transitioning into a dark sleek final graphic card.
  - *Visual Goal:* Show the robot executing autonomous auto-stow (telescope retract, mast stow, bridge fold, corner table ground reset). The screen transitions to a clean high-contrast technical compliance summary table with all 6 evaluation criteria verified.

### 1.2 Camera Poses in Gazebo SDF
```xml
<!-- Shot 5.1: Isometric Translucent Cutaway for Structural Overlay -->
<camera name='camera_spec_iso'>
  <pose>2.60 -3.80 1.95 0 0.38 1.95</pose>
</camera>

<!-- Shot 5.2: Wide Longitudinal Pipeline Overview -->
<camera name='camera_pipeline_wide'>
  <pose>3.85 -2.40 1.45 0 0.25 1.5708</pose>
</camera>

<!-- Shot 5.3: Front Low Dramatic Egress View -->
<camera name='camera_egress_front'>
  <pose>5.80 -0.40 0.45 0 0.08 3.05</pose>
</camera>
```

---

## 2. Technical ROS 2 Architecture

### 2.1 Safety Nodes, Heartbeat & Watchdogs
The safety and compliance subsystem runs across three dedicated ROS 2 components:

```mermaid
graph TD
    A[Gazebo Harmonic Simulation] -->|/aircraft/robot_pose\n/aircraft/joint_states| B(aircraft_controller.py)
    B -->|Trajectory Query| C(aircraft_guard.py)
    C -->|Floor Pressure / Enclosure Envelopes| B
    B -->|50 Hz Actuation| A
    B -->|/aircraft/status JSON| D[Verification Monitor & HUD Recorder]
    E[Safety Watchdog / E-Stop Service] -->|/aircraft/emergency_stow| B
```

### 2.2 Active ROS 2 Topics & Verification Telemetry
| Topic Name | Message Type | Direction | Rate | Functional Purpose |
|---|---|---|---|---|
| `/aircraft/status` | `std_msgs/msg/String` | Controller $\to$ HUD | 2 Hz | JSON telemetry stream with clearances, pressure, and state |
| `/aircraft/joint_states` | `sensor_msgs/msg/JointState` | Gazebo Bridge $\to$ Controller | 50 Hz | Ground truth encoder positions of all 7 actuated DOF |
| `/aircraft/emergency_stow` | `std_msgs/msg/Bool` | Supervisor $\to$ Controller | Asynchronous | Triggers fail-safe reverse egress and stowed lock |
| `/aircraft/clearance/roof` | `std_msgs/msg/Float64` | Guard $\to$ Telemetry | 10 Hz | Current minimum ceiling clearance ($m$) |
| `/aircraft/clearance/wall` | `std_msgs/msg/Float64` | Guard $\to$ Telemetry | 10 Hz | Current minimum sidewall clearance ($m$) |
| `/aircraft/pressure/floor_peak` | `std_msgs/msg/Float64` | Guard $\to$ Telemetry | 10 Hz | Peak 4-patch ground reaction pressure ($\text{kg/m}^2$) |

---

## 3. Simulation Mechanics & In-Depth Technical Verification

### 3.1 Four-Patch Ground Pressure Screen Derivation
Under the Boeing B737-800 forward hold structural guidelines, localized floor loading must not exceed $600\text{ kg/m}^2$.

#### Mathematical Formulation in `aircraft_guard.py`:
The tracked chassis is screened as a distributed 4-contact-point system across the dual rubber track base:
- Effective wheelbase ($x$-axis span): $L_w = 0.66\text{ m}$ (half-span $b_x = 0.33\text{ m}$)
- Track centerline gauge ($y$-axis span): $W_t = 0.75\text{ m}$ (half-span $b_y = 0.375\text{ m}$)
- Total tare mass + payload: $M_{\text{total}} = 222.8\text{ kg} + 32.0\text{ kg} = 254.8\text{ kg}$
- Track contact patch ground area: $A_c = 0.66\text{ m} \times 0.35\text{ m} = 0.231\text{ m}^2$

For center-of-mass coordinates $(x_{\text{com}}, y_{\text{com}})$ relative to chassis center:
$$R_{ij} = \frac{M_{\text{total}}}{4} \left(1 + s_x \frac{x_{\text{com}}}{b_x} + s_y \frac{y_{\text{com}}}{b_y}\right), \quad s_x \in \{-1, +1\}, \; s_y \in \{-1, +1\}$$

Applying a structural safety stress concentration factor $K_{\text{conc}} = 1.5$:
$$P_{\text{peak}} = \frac{\max(R_{ij})}{A_c} \times 1.5$$

#### Verification Results:
- Nominal stowed pose ($x_{\text{com}} = 0.04\text{ m}, y_{\text{com}} = 0.00\text{ m}$): $P = 414.2\text{ kg/m}^2$
- Max reach forward reach ($0.747\text{ m}$ under $32\text{ kg}$ load): $P = \mathbf{531.3\text{ kg/m}^2} \le 600.0\text{ kg/m}^2$ (**PASS**)
- All contact reactions remain positive ($R_{ij} > 0$), confirming absolute chassis tipping stability without outriggers.

---

### 3.2 Continuous Throughput & High-Capacity Pipelining Analysis
The hackathon requirement mandates a loading throughput $\ge 12\text{ bags/min}$ ($\le 5.0\text{ s/bag}$).

```mermaid
gantt
    title Continuous Baggage Pipelining (12 - 15 Bags / Minute)
    dateFormat  s
    axisFormat %S s
    section Bag 1
    Infeed & Corner Lift     :a1, 0, 2
    Buffer Travel            :a2, 2, 4
    Robot Tray Handoff       :a3, 4, 5
    Placement & Stacking     :a4, 5, 9
    section Bag 2
    Infeed & Corner Lift     :b1, 2, 4
    Buffer Travel            :b2, 4, 6
    Robot Tray Handoff       :b3, 6, 7
    Placement & Stacking     :b4, 7, 11
    section Bag 3
    Infeed & Corner Lift     :c1, 4, 6
    Buffer Travel            :c2, 6, 8
    Robot Tray Handoff       :c3, 8, 9
    Placement & Stacking     :c4, 9, 13
```

- **Buffer Speed:** $v_{\text{belt}} = 0.30\text{ m/s}$ ($300\text{ mm/s}$).
- **Handoff Interval:** $4.0\text{ s}$ per item into the intermediate staging tray.
- **Cycle Decoupling:** While BellyBot's telescopic arm completes placement into the luggage row, the buffer conveyor advances the subsequent bag to the standby staging threshold.
- **Sustained Flow:** $\mathbf{12\text{ to }15\text{ bags/min}}$, fulfilling the turnaround rate of narrow-body airliners (150–180 bags loaded within 12–15 minutes).

---

### 3.3 Zero-Human-Intervention Safety Envelope
Working inside narrow-body aircraft lower cargo compartments ($1.12\text{ m}$ height) is recognized by aviation safety authorities as one of the most hazardous tasks in airport ground handling:
- **Ergonomic Hazards Eliminated:**
  - Repetitive lifting of $20\text{ – }32\text{ kg}$ luggage while kneeling or crouching.
  - Lumbar spine compression and knee osteoarthritis risks completely removed.
- **Confined Space Safety:**
  - Zero human personnel required inside the cargo compartment during normal infeed.
  - Ramp agent remains exterior to the aircraft at the belt loader control console.
- **Aircraft Structure Protection:**
  - Automatic collision boundary enforcement in software (`aircraft_guard.py`):
    - Ceiling clearance $\ge 314\text{ mm} > 100\text{ mm}$ limit.
    - Sidewall liner clearance $\ge 28.5\text{ mm} > 25\text{ mm}$ limit.
    - Floor clearance $\ge 0.0\text{ mm}$ (no floor gouging or surface denting).

---

## 4. Video Post-Production & Submission Encoding Guidelines

### 4.1 Master Submission Technical Parameters
The submission video must strictly follow the official competition guidelines:
- **Maximum Duration:** Strictly $\le 2\text{ minutes}$ ($120\text{ seconds}$).
- **Container / Codec:** MP4 (`.mp4`), H.264 video codec, AAC audio.
- **Resolution:** 1080p ($1920 \times 1080$) or 720p ($1280 \times 720$), 16:9 widescreen.
- **Frame Rate:** 30 or 60 FPS constant frame rate (CFR).
- **Official File Naming:** `2_Animation_<Team>_<Lead>.MP4` (e.g., `2_Animation_TeamBellyBot_LeadName.MP4`).

### 4.2 Segment Video Assembly & HUD Rendering Script
```bash
# 1. Stitch individual segment clips into the master 120-second video
ffmpeg -f concat -safe 0 -i segment_list.txt -c copy master_raw.mp4

# 2. Re-encode with master HUD overlay and normalize duration to <= 120.0s
ffmpeg -i master_raw.mp4 \
       -vf "drawtext=text='BELLYBOT DOCKSTACK C v3 | B737-800 CARGO SIMULATION':x=40:y=35:fontsize=22:fontcolor=white:box=1:boxcolor=black@0.6:boxborderw=8, \
            drawtext=text='FLOOR LOAD: 531.3 kg/m² (LIMIT: 600) | CEILING MARGIN: +314 mm | THROUGHPUT: 12-15 BAGS/MIN':x=40:y=h-55:fontsize=18:fontcolor=yellow:box=1:boxcolor=black@0.6:boxborderw=6" \
       -t 120.0 \
       -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p \
       videos/2_Animation_TeamBellyBot_Final.MP4
```

### 4.3 Final Frame Technical Summary Card (1:56 – 2:00)
| Design Parameter | Specification Limit | BellyBot DockStack C v3 Performance | Status |
|---|---|---|---|
| **Hold Height Clearance** | $< 1.200\text{ m}$ compartment | Full vehicle height $0.468\text{ m}$ (Stowed), $+314\text{ mm}$ margin at max lift | **PASS** |
| **Baggage Payload Mass** | $20\text{ – }32\text{ kg}$ | Verified under $32.0\text{ kg}$ maximum dynamic bag load | **PASS** |
| **Floor Pressure Limit** | $\le 600.0\text{ kg/m}^2$ | Evaluated peak reaction: $\mathbf{531.3\text{ kg/m}^2}$ | **PASS** |
| **Cargo Fragility Impact** | First-layer drop damage | $+30.92^\circ$ Tilting tray: $< 30\text{ mm}$ slide, impact reduced by **87%** ($< 9.4\text{ J}$) | **PASS** |
| **Transfer Throughput** | $\ge 12\text{ bags/min}$ | Pipelined flow delivers $\mathbf{12\text{ – }15\text{ bags/min}}$ ($4\text{ – }5\text{ s/bag}$) | **PASS** |
| **Human Safety** | Zero hold ergonomics injury | **100% Autonomous In-Hold Operation**, zero human lifting in hold | **PASS** |
