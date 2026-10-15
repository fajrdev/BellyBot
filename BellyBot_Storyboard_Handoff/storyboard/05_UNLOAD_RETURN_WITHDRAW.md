# Scene 05 — Unload Return Withdraw

## Screen time and objective
01:13–01:33. Demonstrate reverse logistics transfer: carrying a supported bag back to the dock, transferring it across the threshold down the inclined door belt, and verifying discharge past the exterior optical completion plane ($Y < -2.60\text{ m}$) onto the apron loader. Disclose the real-world physical limitation of rigid floor scooping on sharp box luggage, and conclude with the declared equipment withdrawal procedure.

## Technical motion sequence
1. Return Loaded: BellyBot carrying bag on tray departs staging bay $X=4.10\text{ m}$, reverses along $-X$ along the aisle centerline to $X=1.317\text{ m}$, pivots $90^\circ$ to $\text{Yaw}=+\pi/2$, and reverses into dock datum $Y=-0.091\text{ m}$.
2. Reverse Threshold Transfer: Matched conveyors run outward toward apron (`tray_belt_speed = -0.40, door_speed = -0.40, loader_speed = -0.40 m/s`). Bag rolls off tray across the 20 mm interface gap, descends the $5.48^\circ$ inclined door belt, and transfers smoothly onto the belt loader.
3. Optical Discharge Verification: Bag crosses optical discharge plane $Y < -2.60\text{ m}$ at sim-time $t=82.9\text{ s}$, coming to rest on apron loader at measured coordinates $X=1.321\text{ m}, Y=-5.101\text{ m}, Z=-0.387\text{ m}$. Event `unload_complete` emitted.
4. Floor Scoop Limitation Disclosure: In rigid-body physics, sliding a wedge under a flat-bottomed rigid box resting on a flat floor lacks normal vertical force components at sharp 90° corners. Real-world floor retrieval requires flexible/compliant luggage shells or a beveled floor ramp. Label clearly: "Reverse transfer and door descent physically verified; floor scoop requires compliant luggage shells."
5. Declared Withdrawal: Disclose ground-crew changeover for door belt withdrawal and ramp re-installation for vehicle exit.

## Gazebo implementation
Reverse threshold transfer physically executed in `worlds/mission_shuttle.sdf`. Baggage motion governed entirely by contact friction ($\mu=0.60$ door belt, $\mu=0.50$ loader) and downward slope gravity. Zero artificial forces or teleports. Sensor plane at $Y=-2.60\text{ m}$ triggers `unload_complete`.

## ROS 2 communications and orchestration
Autonomous reverse cycle driven by `/mission/command` `start_unload`. Observer tracks bag transform on `/mission/world_poses`, verifies descent along `door_belt` to `loader`, and writes to `results/mission_events.csv` and `results/mission_metrics.json`.

## Evidence, camera and acceptance gate
- Verified reverse discharge: $138.8\text{ s}$ sim-time round trip.
- Bag passage past $Y < -2.60\text{ m}$ confirmed; final apron position $Y=-5.101\text{ m}$.
- Starboard and apron cameras capture uninterrupted descent down the inclined threshold.
- Honest disclosure of floor scoop limitations on rigid luggage.

## Failure handling and recording rule
A failed prerequisite blocks the related claim. Evidence preserved in `results/mission_events.csv` and `scratch/test_reverse_discharge.py`.

