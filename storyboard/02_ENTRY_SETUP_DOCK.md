# Scene 02 — Entry Setup Dock

## Screen time and objective
00:08–00:23. Show BellyBot entering the aircraft through the forward cargo doorway, identify the declared manual ground-crew changeover (removal of entry ramp deck and deployment of the inclined door-dock assembly), and demonstrate precision docking at the verified receiving datum.

## Technical motion sequence
1. Autonomous Entry: Robot spawns stowed at initial ground pose $(1.300, -3.290, 0.183\text{ m}, \text{Yaw}=+\pi/2)$, drives forward along $+Y$, ascends entry ramp to cargo floor level ($Z=0.06\text{ m}$), pivots clockwise at $(1.300, 0.000)$ to $\text{Yaw}=0.0$, and advances along $+X$ into the hold.
2. Declared Setup / Changeover: Disclose ground-crew removal of temporary entry ramp and placement/anchoring of the inclined doorway conveyor (`door_belt`) spanning $Z=0.300\text{ m} \to 0.498\text{ m}$ ($5.48^\circ$ pitch, $\mu=0.60$).
3. Docking Approach: Controller executes 3-phase dock approach: reverse along $-X$ to $X=1.300\text{ m}$, pivot to $\text{Yaw}=+\pi/2$, and reverse 95 mm along $-Y$ to the established dock pose $\mathbf{P}_{\text{dock}} = [1.300, -0.095, 0.003\text{ m}, \text{Yaw}=+1.5708\text{ rad}]$.

## Gazebo implementation
Preserve historical entry fixture (`aircraft.launch.py mode:=entry`) for regression tests. Active mission world (`worlds/mission_shuttle.sdf`) contains the self-contained `mission_door_belt` model meeting the robot rear infeed with a measured $20\text{ mm}$ longitudinal interface gap, $0\text{ mm}$ lateral offset, and $0\text{ mm}$ height step at zero mast lift.

## ROS 2 communications and orchestration
Controller runs state sequence `ENTRY` $\to$ `SETUP_REQUIRED` $\to$ `DOCK_APPROACH` $\to$ `DOCKED`. Dock convergence is verified through measured TF transform feedback from `/aircraft/robot_pose` ($|\Delta x| < 15\text{ mm}, |\Delta y| < 10\text{ mm}, |\Delta\text{yaw}| < 0.03\text{ rad}$). Upstream feed gate remains inhibited until `dock_reserved` event is emitted.

## Evidence, camera and acceptance gate
- Verified dock pose: $\mathbf{P}_{\text{dock}} = [1.300, -0.095, 0.003\text{ m}, \text{Yaw}=+1.5708\text{ rad}]$.
- Close-up camera inspection of the $20\text{ mm}$ interface gap showing zero vertical step to the robot receiving tray.
- Successful transfer trial: dynamic bag passes smoothly across threshold without collision or tipping.

## Failure handling and recording rule
A failed prerequisite blocks the related claim. Geometry decision record committed in `docs/docking_decision.md` (commit `ae56bd5`). Docking convergence and events logged to `results/mission_events.csv`.

