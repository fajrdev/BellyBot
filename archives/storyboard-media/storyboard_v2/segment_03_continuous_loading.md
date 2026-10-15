# Segment 03: Continuous Loading — 30-Second Throughput Benchmark (0:23–0:53)

## 1. Scene Overview & Judging Objective

| Attribute | Specification |
| :--- | :--- |
| **Video Timestamp** | `0:23–0:53` (30.0 seconds uninterrupted sequence) |
| **Target World / Scene** | `transfer_cutaway.sdf` (or `batch_transfer.sdf` integrated into `nb_hold_cutaway` without interior floor conveyors) |
| **Playback Speed** | `1.0x Real-Time` (or explicitly labeled accelerated playback if commissioning speed `speed_scale < 1.0` is used; see Section 2) |
| **Primary Judging Criteria** | Functional Effectiveness & Baggage Handling Strategy (20%), Autonomy & Intelligent Operation (15%) |
| **What It Establishes** | One uninterrupted loading sequence with a live simulation clock (`0.00 s` to `30.00 s`) and a live completed-placement counter (`0/6` to `6/6`), demonstrating **6 completed placements in 30.0 seconds of simulation time** (**12.0 bags/min over the measured 30 s interval**). |

## 2. Strict Hackathon Throughput Rules & Architectural Setup

Per the storyboard rules, the HUD and report must state all four measurement conditions transparently:
1. **Counting Rule**: A bag is counted **only after successful placement on the target stack/floor and full release/retraction of `tray_belt`** (`VERIFY` state complete in `batch_cycle.py` / `score_events.py`).
2. **Simulation Time**: Elapsed time is measured strictly from ROS 2 `/clock` (`rosgraph_msgs/msg/Clock`), and any accelerated video playback is explicitly badged on screen.
3. **Pre-Filled Conveyor State Disclosure**: The HUD explicitly states: **`PRE-CONDITION: External loader & doorway belt pre-filled with 6 staged bags (20–32 kg) at t = 0.00 s; robot pre-aligned at receiving layer height (Z = 0.595 m).`**
4. **Honest Rate Reporting**: Report the result as **`6 bags in 28.0–30.0 s sim time = 12.0–12.86 bags/min over the measured fixed-layer batch interval (not claimed as sustained whole-hold average across chassis repositioning)`**.

### How Continuous Feeding Works Without the Two Removed Conveyors
With `corner_transfer` and `longitudinal_buffer` removed, how does BellyBot achieve the `≤ 5.0 s/bag` (`4.6566 s` nominal kinematic cycle) 6-bag continuous batch?
- **Direct Doorway-Zone Layer Batching**: BellyBot locks its tracks adjacent to the doorway handoff (`X = 1.95 m, Y = 0.00 m, yaw = 0.0 rad`) or uses the doorway-aligned staging feed where the shortened `door_belt` delivers pre-queued bags directly to the rear/side receiving interface of BellyBot's `tray_belt` at matched height (`Z = 0.595 m`, `lift = 0.10 m`).
- Because `lift = 0.10 m` and `yaw = 0.0 rad` remain constant throughout the 6-bag layer batch, BellyBot eliminates per-bag mast and yaw round-trips. Each bag executes the 4-phase high-speed layer batch (`speed_scale := 1.0`, `retract_belt_sync := true`, `deposit_belt_ratio := 1.78125`):
  1. **Phase 1 — Receive from Doorway Feed (`1.32 s`)**: `door_belt` and `tray_belt` run synchronously at peak `1.50 m/s` (`3.0 m/s²` quintic profile) to pull `bag_i` (`0.90 m` advance) onto `tray_belt`.
  2. **Phase 2 — High-Speed Telescopic Extend (`1.36 s`)**: `extension_mid_joint` and `extension_tip_joint` extend from `0.00 m` to `0.40 m` each (`0.80 m` total reach) at up to `0.70 m/s` per stage (`1.40 m/s` combined).
  3. **Phase 3 — Synchronized Deposit Retraction (`1.78 s`)**: While the telescopic slides retract from `0.80 m` to `0.00 m`, `tray_belt` drives forward (`+X`) with `deposit_belt_ratio = 1.78125` (`1.425 m` belt travel), keeping the bag stationary over the receiving bay while the tray slips cleanly out from underneath it.
  4. **Phase 4 — Placement Verification & Counter Increment (`0.20 s`)**: `score_events.py` / pose monitor confirms the bag is resting stably on the target stack (`speed < 0.02 m/s`, `Z` within supported band) and increments the completed-placement counter **after tray release**.

## 3. Bag-by-Bag 30-Second Timeline (`t_sim = 0.00 s` to `28.50 s`)

| Bag # | Entity Name | Model Type | Mass (kg) | Dimensions (L×W×H m) | Receive Start (Sim s) | Extend Peak (Sim s) | Released & Verified (Sim s) | Counter Display |
| :---: | :--- | :--- | :---: | :--- | :---: | :---: | :---: | :---: |
| **1** | `incoming_000` | `bag_hard_32` | `32.0 kg` | `0.78 × 0.50 × 0.32` | `0.00 s` | `2.68 s` | **`4.66 s`** | `PLACED: 1 / 6` |
| **2** | `incoming_001` | `bag_hard_26` | `26.0 kg` | `0.72 × 0.46 × 0.30` | `4.75 s` | `7.43 s` | **`9.41 s`** | `PLACED: 2 / 6` |
| **3** | `incoming_002` | `bag_duffel_23` | `23.0 kg` | `0.70 × 0.40 × 0.32` | `9.50 s` | `12.18 s` | **`14.16 s`** | `PLACED: 3 / 6` |
| **4** | `incoming_003` | `bag_hard_20` | `20.0 kg` | `0.62 × 0.42 × 0.26` | `14.25 s` | `16.93 s` | **`18.91 s`** | `PLACED: 4 / 6` |
| **5** | `incoming_004` | `bag_fragile_20` | `20.0 kg` | `0.64 × 0.44 × 0.38` | `19.00 s` | `21.68 s` | **`23.66 s`** | `PLACED: 5 / 6` |
| **6** | `incoming_005` | `bag_hard_32` | `32.0 kg` | `0.78 × 0.50 × 0.32` | `23.75 s` | `26.43 s` | **`28.41 s`** | `PLACED: 6 / 6 (12.67 bags/min)` |

*Note on Fallback / Bottleneck Reporting*: If contact physics in Gazebo Harmonic requires running at `speed_scale := 0.50` (`~9.3 s/bag`, completing 6 bags in `55.8 s` of simulation time played back at `2.0x` inside the `30 s` video window), the HUD must display: `SIM TIME ELAPSED: 55.8s (2.0x PLAYBACK) | MEASURED RATE: 6.45 bags/min (Physics Contact Speed Scale 0.50) | BOTTLENECK: Unrestrained Belt-to-Bag Friction Limit (mu=0.45..0.70) During Deposit Retraction`.

## 4. Gazebo Harmonic (`gz sim`) Commands & Spawning

```bash
# 1. Pre-fill the external belt loader and doorway belt with the 6 bags before starting the 30s timer
python3 tools/spawn_bags.py --world transfer_cutaway --count 6 --interval 1.0 --seed 42 --prefix incoming --execute

# 2. Verify all 6 bag entities (incoming_000 .. incoming_005) are publishing world poses
gz topic -e -t /world/transfer_cutaway/dynamic_pose/info -n 1
```

## 5. ROS 2 Jazzy Communications & Execution

### 5.1 Supervisor & Batch Cycle Parameters
```bash
# Configure supervisor for high-speed synchronized layer deposit
ros2 param set /dock_stack_supervisor speed_scale 1.0
ros2 param set /dock_stack_supervisor retract_belt_sync true
ros2 param set /dock_stack_supervisor deposit_belt_ratio 1.78125

# Pre-align robot tray to layer height (lift = 0.10 m, yaw = 0.0, reach = 0.0, pitch = 0.0)
ros2 topic pub --once /bellybot/pose_target std_msgs/msg/Float64MultiArray '{data: [0.10, 0.0, 0.0, 0.4, 0.0, 0.10]}'

# Start the automated batch cycle logger (logs every event to /tmp/bellybot_batch_results.csv)
ros2 run bellybot_dock_stack_description batch_cycle.py   --ros-args -p use_sim_time:=true -p speed_scale:=1.0 -p log_path:=/tmp/bellybot_batch_results.csv
```

### 5.2 Topics Monitored During the 30s Sequence
| Topic | Type | Publisher | Subscriber | Condition Checked |
| :--- | :--- | :--- | :--- | :--- |
| `/bellybot/bag_ready` | `std_msgs/msg/Bool` | Truth/Sensor Pose Node | `batch_cycle.py` | `True` when `incoming_00i` reaches doorway handoff datum |
| `/bellybot/belt_cmd` | `std_msgs/msg/Float64` | `batch_cycle.py` | `supervisor.py` | Quintic belt velocity profile up to `1.50 m/s` during `RECEIVE` |
| `/bellybot/bag_supported` | `std_msgs/msg/Bool` | Truth/Sensor Pose Node | `batch_cycle.py` | `True` when bag CoM is centered on `tray_belt` (`speed < 0.02 m/s`) |
| `/bellybot/pose_target` | `std_msgs/msg/Float64MultiArray` | `batch_cycle.py` | `supervisor.py` | Commands `[0.10, 0.0, 0.80, 0.4, 0.0, 0.10]` (`EXTEND`) then `reach=0.0` (`DEPOSIT_RETRACT`) |
| `/bellybot/motion_active` | `std_msgs/msg/Bool` | `supervisor.py` | `batch_cycle.py` | Handshake confirming trajectory start and completion (`< 5 mm` error) |
| `/bellybot/bag_placed` | `std_msgs/msg/Bool` | Truth/Sensor Pose Node | `batch_cycle.py` | `True` after tray retraction when bag settles on stack without dropping |

### 5.3 Post-Window Automated Scoring Verification
```bash
python3 tools/score_events.py   --events /tmp/event_log.csv   --start 0.0   --duration 30.0   --patches /tmp/floor_patch_log.csv   --output /tmp/segment_03_throughput_score.json
```

## 6. On-Screen HUD & Video Overlay Requirements

| Overlay Region | Exact Content |
| :--- | :--- |
| **Top-Left Banner** | `0:23–0:53 | CONTINUOUS LOADING BENCHMARK (UNINTERRUPTED)` |
| **Top-Right Counter Box** | `COMPLETED PLACEMENTS: [ X / 6 ] (Counted Only After Full Tray Release & Settle)` |
| **Bottom-Left Disclosure** | `Pre-Condition: External Conveyor Pre-Filled | Fixed-Layer Batch Rate: 12.0+ bags/min (30s Window)` |
| **Bottom-Right Timer** | `ELAPSED SIM TIME: [ 00.00 s / 30.00 s ] | PLAYBACK: 1.0x REAL-TIME` |
