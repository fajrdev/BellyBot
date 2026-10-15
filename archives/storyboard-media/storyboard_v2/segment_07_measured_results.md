# Segment 07: Measured Results, Floor-Load Verification & Limitations (1:48–2:00)

## 1. Scene Overview & Judging Objective

| Attribute | Specification |
| :--- | :--- |
| **Video Timestamp** | `1:48–2:00` (12.0 seconds duration — concludes exactly at the `2:00` maximum video limit) |
| **Target World / Scene** | Split-screen: Live `transfer_cutaway.sdf` final loaded hold view + Quantitative Engineering Scorecard Table |
| **Playback Speed** | `1.0x Real-Time` (`12.0 s` duration) |
| **Primary Judging Criteria** | All 6 Judging Criteria (Final quantitative synthesis & engineering transparency) |
| **What It Establishes** | Summarizes demonstrated throughput (`score_events.py` output), measured geometric clearances (`resolution_checks.json` / `validation.json`), 4-patch floor-load calculation (`≤ 600 kg/m²`), and remaining hardware/simulation limitations. |

## 2. Quantitative Scorecard Data (Grounded in Workspace Evidence Files)

Display the following exact, verified metrics from `docs/validation.json`, `docs/resolution_checks.json`, `docs/timing.json`, and `/tmp/segment_03_throughput_score.json`:

| Hackathon Requirement | Target Threshold | Demonstrated / Measured Value | Verification Source & Method |
| :--- | :--- | :--- | :--- |
| **Working Envelope Height** | `< 1.20 m` (NB Cargo Hold) | **`1.120 m` hold ceiling; `0.538 m` stowed robot height** | Checked across `477` work poses & `2,432` entry samples (`resolution_checks.json`). |
| **Minimum Clearances** | No collision (`≥ 100 mm` roof, `≥ 25 mm` wall) | **`110.0 mm` roof clearance (at lift cap); `32.77 mm` min door-turn clearance; `28.5 mm` stowed liner clearance; `2.0 mm` floor-pickup nose clearance** | Triangle-mesh distance checks (`resolution_checks.json`) + `aircraft_guard.py` real-time enforcement. |
| **Baggage Mass & Variety** | `20–32 kg`, irregular / soft / fragile | **`20 kg` (`bag_hard_20`, `bag_fragile_20`), `23 kg` (`bag_duffel_23`), `26 kg` (`bag_hard_26`), `32 kg` (`bag_hard_32`)** | Free rigid-body dynamics in Gazebo Harmonic (`mu = 0.30..0.70`), zero artificial bag welding. |
| **Throughput Target** | `≥ 12 bags/minute` | **`6 bags placed in 28.41 s sim time = 12.67 bags/min` over the measured 30 s fixed-layer interval** (`4.66 s` nominal cycle) | Counted only after release & settle via `score_events.py` (pre-filled doorway feed, fixed layer height). |
| **Floor Load Limit** | `≤ 600 kg/m²` | **`594.31 kg/m²` peak screened patch pressure** (at `1.00 m` full reach with `32 kg` payload and `1.5x` dynamic factor; `332.8 kg/m²` static stowed) | `173 kg` robot + `32 kg` bag = `205 kg` across two `1.32 m × 0.35 m` tracks (four `0.66 × 0.35 m = 0.231 m²` half-track patches). |
| **Space Utilisation** | Maximize usable cargo hold volume | **`+2.83 m` of hold floor length recovered** (`36.8%` of `7.70 m` hold) by removing the two interior floor conveyors (`corner_transfer` & `longitudinal_buffer`) | Direct doorway threshold handoff (`Y = -0.61 m`) + longitudinal robot shuttle. |

## 3. Explicit Floor-Load Calculation Breakdown (Shown on Slide / Overlay)

The 4-patch floor-loading equation implemented in `aircraft_guard.py` (`evaluate()`) and `cad/validate.py` must be shown clearly on screen:
- **Total Mass ($M_{\text{total}}$)**: $173\text{ kg (BellyBot v3 assigned mass)} + 32\text{ kg (max bag)} = 205\text{ kg}$
- **Track Contact Geometry**: Two tracks of $1.320\text{ m} \times 0.350\text{ m}$ ($A_{\text{total}} = 0.924\text{ m}^2$), modeled as four half-track patches of $A_{\text{patch}} = 0.660\text{ m} \times 0.350\text{ m} = 0.231\text{ m}^2$ at longitudinal centers $x_p = \pm 0.330\text{ m}$ and lateral centers $y_p = \pm 0.375\text{ m}$.
- **Patch Reaction & Peak Screened Pressure ($P_{\text{peak}}$)** with $1.5\times$ dynamic/uneven-sharing multiplier:
  $$R_{sx, sy} = \frac{M_{\text{total}}}{4}\left(1 + sx\frac{x_{\text{CoM}}}{0.330} + sy\frac{y_{\text{CoM}}}{0.375}\right), \quad P_{\text{peak}} = 1.5 \times \frac{\max(R_{sx,sy})}{0.231\text{ m}^2} = 594.31\text{ kg/m}^2 \le 600\text{ kg/m}^2$$

## 4. Transparent Engineering Limitations Statement

Per the judging guidelines and evidence rules, display these four concise engineering boundaries on the closing card:
1. **Sustained vs. Interval Throughput**: The `12.67 bags/min` (`6 bags / 28.4 s`) rate is demonstrated over a pre-filled, fixed-layer batch interval. Whole-hold sustained throughput including `30.9°` floor pickup (`6.73 s` mechanism round-trip) and longitudinal shuttle repositioning is lower (`~6.5–8.5 bags/min`).
2. **Entry Deck Changeover**: Removing the `180 mm` temporary entry turn-deck/ramp and docking the cantilevered `door_belt` at `Y = -0.61 m` is a manual ground-crew setup changeover between ingress and loading, not an automated in-simulation transform.
3. **Soft-Bag & Strap Physics**: Gazebo Harmonic uses convex rigid-body proxies (`mu = 0.30..0.70`) and `TrackController` surface velocities; real soft-shell bag deformation, loose strap snagging, and compliant nose deflection require physical hardware trials.
4. **Localization & Perception**: Current commissioning uses Gazebo `PosePublisher` ground-truth poses (`/aircraft/robot_pose`, `/aircraft/world_poses`); production deployment requires onboard 3D depth-camera (`front_depth`) bag segmentation and SLAM/visual odometry.

## 5. Gazebo & ROS 2 Commands for Final Scene Capture

```bash
# Generate the final scored JSON summary from the recorded run logs
python3 tools/score_events.py   --events /tmp/event_log.csv   --start 0.0   --duration 30.0   --patches /tmp/floor_patch_log.csv   --output /tmp/final_storyboard_metrics.json

# Hold BellyBot at completed stack inspection pose in transfer_cutaway.sdf while camera orbits
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray '{data: [0.0, 0.0, 0.0, 0.0]}'
```
