# Scene 01 — Concept

## Screen time and objective
00:00–00:08. Establish aircraft hold cutaway, exterior loading path, fixed door dock, mobile BellyBot shuttle, cargo placement zone, and clear center return corridor in one cutaway view. The two marked interior transfer conveyors (`corner_transfer` and `longitudinal_buffer`) are confirmed removed from the active mission variant (`worlds/mission_shuttle.sdf`). The 198 mm doorway threshold height difference is bridged by the reconfigured, friction-backed ($5.48^\circ$, $\mu=0.60$) `door_belt`.

## Technical motion sequence
Begin stationary at dock with empty level tray. Show one staged bag outside the hold on the exterior belt loader ($Y \le -2.60\text{ m}$), the dock reservation at $\mathbf{P}_{\text{dock}} = [1.300, -0.095, 0.003\text{ m}, \text{Yaw}=+1.5708\text{ rad}]$, and target cell 1 at $X=4.08\text{ m}$ to $5.144\text{ m}$. Explanatory route overlay: loading moves inward through door to BellyBot tray; mobile BellyBot travels along center aisle ($Y=0.000\text{ m}$) to placement cell; robot deposits bag and returns empty to dock.

## Gazebo implementation
Active world: `worlds/mission_shuttle.sdf` with full curved-hold collision geometry preserved. External `belt_loader` and inclined `door_belt` retained; internal transfer conveyors removed. Ground-truth poses published via `gz-sim-pose-publisher-system` on `/world/mission_shuttle/dynamic_pose/info`. Dynamic baggage models use realistic rigid-body contacts ($\mu=0.45$, mass 32.0 kg). Presentation camera framed from wide starboard cutaway.

## ROS 2 communications and orchestration
Controller and observer subscribe to `/clock`, `/mission/world_poses`, `/aircraft/robot_pose`, and `/aircraft/robot_joints`. Telemetry published on `/mission/status` (10 Hz) and `/mission/bags` (5 Hz). Actuation exclusively owned by `mission_controller.py` on `/aircraft/actuation/*`.

## Evidence, camera and acceptance gate
- Verified removal map: `docs/docking_decision.md` (commit `ae56bd5`).
- Labelled wide frame captured from starboard cutaway.
- Narration: "Door-side supply is metered; BellyBot shuttles autonomously along the aisle to place each bag." No claim of continuous umbilical feed.

## Failure handling and recording rule
A failed prerequisite blocks the related claim. Evidence logged to `results/mission_events.csv` with simulation timestamps. Mechanism facts match `BellyBot_DockStack_C_v3`; competition targets reference contest guidelines.

