# BellyBot Interactive Manual Control & Teleoperation Plan

**Document Version:** 1.0  
**Date:** 2026-10-18  
**Objective:** Provide an intuitive, responsive, and fun interactive teleoperation script (`scripts/play_bellybot.py`) allowing manual driving, multi-DOF joint manipulation, and baggage handling via keyboard or gamepad in either the authentic Boeing 737 cargo hold or an open testing arena.

---

## 1. System Architecture & Control Pipeline

```
+-------------------------------------------------------------------------+
|                  USER INPUT: KEYBOARD OR USB GAMEPAD                    |
|                                                                         |
|   Keyboard: WASD (drive), IK (lift), JL (yaw), UO (reach), TG (pitch)   |
|   Gamepad:  Left Stick (drive), Right Stick (lift), D-Pad/Triggers      |
+-------------------------------------------------------------------------+
                                    |
                                    v (100 Hz Async Input Loop)
+-------------------------------------------------------------------------+
|        INTERACTIVE TELEOPERATION NODE (scripts/play_bellybot.py)        |
|                                                                         |
|   - Real-time kinematic command smoothing & rate limiting               |
|   - Live joint feedback & pose monitoring (/aircraft/robot_pose)       |
|   - Dynamic Luggage Spawner (spawn suitcases on demand via hotkey)      |
|   - Live Telemetry Dashboard & OpenCV Camera HUD Window                 |
+-------------------------------------------------------------------------+
                                    |
                                    v (ROS 2 Actuation Topics)
+-------------------------------------------------------------------------+
|  ROS 2 / GAZEBO HARMONIC ACTUATION INTERFACES                           |
|  - Tracks:    /aircraft/actuation/cmd_vel (Twist) + wheel speed topics  |
|  - Mast:      /aircraft/actuation/lift_stage_target (0 to 0.180 m/stg)  |
|  - Turntable: /aircraft/actuation/yaw_target (-1.57 to +1.57 rad)       |
|  - Boom:      /aircraft/actuation/extension_stage_target (0 to 0.38 m)  |
|  - Pitch:     /aircraft/actuation/pitch_target (0 to 0.576 rad)         |
|  - Belts:     /aircraft/actuation/tray_belt_speed (+-0.45 m/s)          |
+-------------------------------------------------------------------------+
                                    |
                                    v (ros_gz_bridge)
+-------------------------------------------------------------------------+
|  GAZEBO HARMONIC PHYSICS RUNTIME (Interactive 3D GUI or Headless)       |
|  - World Options: "plane" (Boeing 737-800 Hold) OR "open" (Sandbox)     |
|  - Rigid-body contact friction, gravity, collisions, and baggage physics|
+-------------------------------------------------------------------------+
```

---

## 2. Dual Simulation Environment Options

The script will support two distinct simulation environments via a simple CLI argument:

### Environment A: `--world plane` (Boeing 737-800 Cargo Hold)
* **Description:** The authentic narrow-body aircraft forward cargo hold (`worlds/mission_shuttle.sdf`).
* **Features:**
  * Curved fuselage sidewall liners ($1.12\text{ m}$ ceiling, $1.22\text{ m}$ flat floor corridor).
  * Inclined doorway infeed conveyor ($5.48^\circ$) at the cargo door sill.
  * Real aircraft confined-space constraints to test clearance navigation and corridor driving.
* **Best For:** Practicing precision docking, corridor driving, and realistic hold stacking.

### Environment B: `--world open` (Open Playground / Sandbox Arena)
* **Description:** An unrestricted, open-air flat testing arena based on `bench.sdf`.
* **Features:**
  * Large open floor ($20\text{ m} \times 20\text{ m}$ flat terrain) with no ceiling or walls.
  * Unlocked speed limit ($V_x$ up to $1.2\text{ m/s}$) for high-speed driving, drift turns, and donuts.
  * Full mast elevation ($540\text{ mm}$) and reach ($760\text{ mm}$) without any ceiling boundary restrictions.
  * Pre-spawned test baggage models and ramps to push, scoop, and launch.
* **Best For:** Casual fun, stunt testing, and learning joint controls without fear of hitting airplane liners.

---

## 3. Comprehensive Control Mapping

### 3.1 Keyboard Controls

| Key | Actuator / Function | Action / Motion |
| :--- | :--- | :--- |
| **`W` / `S`** | Crawler Tracks | Drive Forward / Reverse (linear velocity $V_x$) |
| **`A` / `D`** | Crawler Tracks | Steer Left / Right (in-place pivot angular velocity $\omega_z$) |
| **`I` / `K`** | Telescopic Mast | Raise Lift Up / Lower Lift Down ($0\text{ to }540\text{ mm}$) |
| **`J` / `L`** | Slewing Turntable | Rotate Yaw Left (Port) / Right (Starboard) |
| **`U` / `O`** | Telescopic Boom | Extend Reach Forward / Retract Boom ($0\text{ to }760\text{ mm}$) |
| **`T` / `G`** | Conveyor Deck Hinge | Pitch Deck Tilt Down (Scoop) / Tilt Up (Level) ($0^\circ \text{--} 33^\circ$) |
| **`F` / `V`** | Conveyor Belt & Rollers | Run Belts Forward (Intake) / Run Belts Reverse (Discharge) |
| **`SPACE`** | Emergency Stop | **Instant Actuator Zero & Brake Lockout** |
| **`R`** | Supervised Reset | Reset all joints to stowed home travel configuration |
| **`B`** | Baggage Spawner | **Spawn a 32 kg dynamic suitcase directly in front of the robot!** |
| **`C`** | Camera Switch | Toggle camera views (Hero 3/4 cutaway $\leftrightarrow$ Aisle Chase $\leftrightarrow$ Deck Eye) |
| **`Q` / `ESC`**| Exit | Safely stop simulation and exit teleop |

### 3.2 Gamepad / Controller Mapping (Xbox / PlayStation / Generic USB)

If a USB or Bluetooth controller is connected, the script will automatically detect it:
* **Left Thumbstick ($Y$-axis):** Proportional Throttle (Forward / Reverse).
* **Left Thumbstick ($X$-axis):** Proportional Steering (Left / Right).
* **Right Thumbstick ($Y$-axis):** Smooth Mast Lift (Up / Down).
* **D-Pad ($X$-axis):** Turntable Yaw Rotation (Left / Right).
* **Right Trigger ($RT$) / Left Trigger ($LT$):** Telescopic Boom Extend / Retract.
* **Right Bumper ($RB$) / Left Bumper ($LB$):** Conveyor Deck Pitch Down / Up.
* **Button `A` / Button `Y`:** Conveyor Belt Feed / Reverse.
* **Button `B`:** Instant Emergency Stop.
* **Button `X`:** Stowed Home Reset.
* **Start Button:** Spawn dynamic suitcase.

---

## 4. Visual Experience & Telemetry Display

The script will offer two viewing options:
1. **Native Gazebo 3D GUI (`--gui`):**
   * Launches the full standard Gazebo window on `:0` where you can rotate the 3D camera with the mouse, view shadows, inspect mesh wireframes, and observe contacts directly.
2. **Interactive HUD Stream Window (`--hud`):**
   * Displays an OpenCV high-fps window with real-time HUD overlays:
     * Live chassis speed ($V_x$, $\omega_z$) and $(X, Y, Z)$ coordinates.
     * Joint status bars: Mast height (mm), Turntable yaw ($^\circ$), Boom reach (mm), Deck pitch ($^\circ$).
     * Conveyor belt speed and optical payload detection indicator.
     * Active state badge (`MANUAL TELEOP`, `E-STOP ENGAGED`, `SCOOP READY`).

---

## 5. Dynamic Baggage Spawning Feature

To make the script truly fun and interactive:
* Pressing **`B`** (or Gamepad Start) will dynamically inject a 32 kg passenger suitcase at $[X_{\text{robot}} + 1.20\text{ m}, Y_{\text{robot}}, Z = 0.16\text{ m}]$ using Gazebo's `/world/{world}/create` service.
* You can immediately drive BellyBot up to the suitcase, pitch the deck down ($29^\circ$), extend the boom ($680\text{ mm}$), run the intake belt, and scoop the suitcase up onto the tray!

---

## 6. Implementation Deliverables

1. `scripts/play_bellybot.py`: Complete, standalone executable Python script with CLI options:
   ```bash
   # Play in the Boeing 737 aircraft cargo hold:
   python3 scripts/play_bellybot.py --world plane --gui

   # Play in the open sandbox arena with full speed:
   python3 scripts/play_bellybot.py --world open --gui

   # Play with HUD camera stream:
   python3 scripts/play_bellybot.py --world plane --hud
   ```
2. Automated environment setup (`GZ_SIM_RESOURCE_PATH`, ROS 2 bridges, and automatic cleanup on exit).
