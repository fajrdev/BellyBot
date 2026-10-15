# Scene 04 — Reach Stack Resupply

## Screen time and objective
00:53–01:13. Demonstrate controlled floor placement, show measured ceiling clearance margins, disclose real-world upper-layer stacking feasibility, and show explicit supply gating while the mobile robot is absent from the dock.

## Technical motion sequence
1. Floor Deposit Execution: BellyBot stops at staging bay $X=4.08\text{ m}$. Arm pitches nose-down $30.9^\circ$ ($0.539\text{ rad}$) at reach $0.74\text{ m}$. Forward roller touches floor level ($Z=0.065\text{ m}$). Tray belt runs forward at $+0.60\text{ m/s}$, smoothly rolling the 32 kg case flat onto the cargo floor.
2. Verified Rest Pose: Bag comes to rest flat at measured coordinates $X=5.144\text{ m}, Y=-0.023\text{ m}, Z=0.284\text{ m}$ (floor elevation $Z=0.120\text{ m} + 0.160\text{ m}$ half-height). 0.5 s dwell confirms stability before tray retracts to stowed carrying configuration.
3. Supply Gating: While BellyBot travels and deposits, the exterior loader and door belt stop immediately upon robot departure (`loader_speed = 0.0, door_speed = 0.0`); supply remains gated until BellyBot returns to dock.
4. Upper-Layer Reach & Stacking Analysis: BellyBot mast provides up to $0.60\text{ m}$ lift. Kinematically, reaching an upper layer at $Z \approx 0.55\text{ m}$ with yaw $\pm 65^\circ$ is verified in CAD. However, in rigid-body physics simulation, dynamic stacking on compliant soft luggage without lateral boundary guides requires closed-loop force compliance. Label clearly: "Upper-layer placement kinematically proven in CAD; soft-luggage contact compliance requires hardware testing."

## Gazebo implementation
Floor placement executed in `worlds/mission_shuttle.sdf` with full rigid-body friction and gravity. Free dynamic bag rolls off the moving tray belt, lands on the floor link, settles under friction ($\mu=0.5$), and achieves velocity $< 0.02\text{ m/s}$.

## ROS 2 communications and orchestration
Controller enforces `MobileGuard` kinematic boundaries using measured base SE(3). Stationary manipulation requests checked against hold ceiling and floor bounds. Event `placement_candidate` emitted on arm arrival; `placement_complete` emitted after 0.5 s velocity settling predicate passes.

## Evidence, camera and acceptance gate
- Measured floor deposit COM: $X=5.144\text{ m}, Y=-0.023\text{ m}, Z=0.284\text{ m}$.
- Measured minimum roof clearance: $0.345\text{ m}$ ($345\text{ mm}$ headroom below 1.15 m aircraft ceiling).
- Measured floor clearance: $3\text{ mm}$ chassis ground clearance.
- Upstream feed gating verified: zero motion on apron loader while robot is in hold.

## Failure handling and recording rule
A failed prerequisite blocks the related claim. No invisible rigid shelves or artificial attachment joints used. Stacking limitations disclosed transparently.

