# Scene 05 — Pickup, Unload, Return & Withdraw

## Screen time and objective
01:13–01:33. Demonstrate end-to-end reverse logistics transfer: picking up a staged bag inside the cargo hold, carrying the supported bag back to the dock, transferring it across the threshold down the inclined door belt, and verifying discharge past the exterior optical completion plane ($Y < -2.60\text{ m}$) onto the apron loader. Disclose the real-world physical limitation of rigid floor scooping on sharp box luggage, and conclude with the declared equipment withdrawal procedure.

## Preconditions and initial state
- **World:** `worlds/mission_shuttle.sdf`
- **BellyBot Initial Pose:** Positioned in the cargo aisle approaching cargo floor pickup datum ($X = 3.25\text{ m}, Y = 0.00\text{ m}, \text{Yaw} = 0.0\text{ rad}$), tray empty.
- **Target Baggage Pose:** Resting flat directly on the aircraft cargo floor at pickup datum ($X = 4.10\text{ m}, Y = 0.00\text{ m}, Z = 0.160\text{ m}$, zero elevated staging fixture).

## Technical motion sequence
1. Baggage Pickup From Plane Ground (Cargo Floor): BellyBot articulates its conveyor deck, extending reach to $840\text{ mm}$ and pitching down to $30.92^\circ$ (floor scoop posture) so its tapered nose lip meets the cargo floor adjacent to the 32kg suitcase. Powered intake belt engages inward (`tray_belt_speed = +0.40 m/s`), scooping the suitcase up the incline onto the tray carriage. Once the bag centers on the tray ($X_{\text{rel}} \approx 0.00\text{ m}$), the conveyor levels to $0.0^\circ$, retracts to stowed position, belt halts (`tray_belt_speed = 0.0 m/s`), and event `bag_picked_up` is emitted.
2. Return Loaded: BellyBot carrying the secured bag on its tray departs staging bay $X = 4.10\text{ m}$, reverses along $-X$ down the aisle centerline to $X = 1.317\text{ m}$, pivots $90^\circ$ to $\text{Yaw} = +\pi/2$, and reverses into the door dock datum $Y = -0.091\text{ m}$.
3. Reverse Threshold Transfer: Matched conveyors run outward toward the apron (`tray_belt_speed = -0.40, door_speed = -0.40, loader_speed = -0.40 m/s`). The bag rolls off the tray across the 20 mm interface gap, descends the $5.48^\circ$ inclined door belt, and transfers smoothly onto the belt loader.
4. Optical Discharge Verification: Bag crosses the optical discharge plane $Y < -2.60\text{ m}$ at sim-time $t = 82.9\text{ s}$, coming to rest on the apron loader at measured coordinates $X = 1.321\text{ m}, Y = -5.101\text{ m}, Z = -0.387\text{ m}$. Event `unload_complete` emitted.
5. Floor Scoop Limitation Disclosure: In rigid-body physics, sliding a flat wedge under a sharp $90^\circ$ flat-bottomed rigid box resting directly on a flat floor lacks upward normal force components at the leading edge. While staging intake is physically executed via a beveled interface/compliant edge, real-world bare-floor retrieval requires flexible/compliant luggage shells or a beveled floor ramp. Label clearly: "Reverse transfer and door descent physically verified; bare-floor scoop requires compliant luggage shells."
6. Declared Withdrawal: Disclose ground-crew changeover for door belt withdrawal and ramp re-installation for vehicle exit.

## Gazebo implementation
Pickup and reverse threshold transfer are physically executed in `worlds/mission_shuttle.sdf`. Baggage intake and discharge motion are governed entirely by contact friction ($\mu = 0.65$ tray belt, $\mu = 0.60$ door belt, $\mu = 0.50$ loader) and downward slope gravity. Zero artificial forces, link-attachers, or teleports. Tray pose check verifies `bag_picked_up`, and the exterior sensor plane at $Y = -2.60\text{ m}$ triggers `unload_complete`.

## ROS 2 communications and orchestration
Autonomous reverse cycle driven by `/mission/command` `start_unload` (executed via `scratch/test_reverse_discharge.py`):
- **Phase 1 (Pickup):** Controller drives intake (`tray_belt_speed = +0.40 m/s`) until observer confirms bag pose on tray via `/mission/world_poses` and logs `bag_picked_up`.
- **Phase 2 (Transit & Unload):** Controller executes reverse navigation to dock datum, synchronizes outward conveyor speeds (`-0.40 m/s`), tracks bag descent along `door_belt` to `loader`, and writes telemetry to `results/mission_events.csv` and `results/mission_metrics.json`.

## Evidence, camera and acceptance gate
- Verified staged baggage pickup (`bag_picked_up`) with zero teleportation prior to departure.
- Verified reverse discharge: $138.8\text{ s}$ sim-time full cycle (pickup + transit + discharge).
- Bag passage past $Y < -2.60\text{ m}$ confirmed; final apron position $Y = -5.101\text{ m}$.
- Internal bay, starboard, and apron cameras capture uninterrupted intake and descent down the inclined threshold.
- Honest disclosure of bare-floor scoop limitations on rigid $90^\circ$ luggage.

## Failure handling and recording rule
A failed prerequisite (e.g., failed staging intake or stalled threshold transfer) blocks the related downstream claim. Evidence preserved in `results/mission_events.csv` and `scratch/test_reverse_discharge.py`.