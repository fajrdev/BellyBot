# Scene 04 — Reach Stack Resupply

## Screen time and objective
00:53–01:13. Demonstrate controlled floor placement, a genuinely reachable upper-layer placement if feasible, and explicit supply behavior while the robot relocates.

## Technical motion sequence
First choose a validated floor target. Extend level, then pitch nose-down only with allowed floor envelope (aircraft guard: lift≈0,yaw≈0,reach≥.72 m,pitch≤32°); transfer gently and avoid striking floor. Verify bag released and tray clear before retraction; level before retracting across forbidden envelopes. Next compute upper-layer target from actual lower bags, support polygon, bag height, local roof and tray underside clearance. Raise only within loaded caps; keep pitch level for yaw, retract before yaw, then extend within guard. Upper-layer bags must be supported by luggage, not an invisible rigid shelf. If unavailable, label “upper-layer placement not yet demonstrated” and show measured reach limits rather than fake a stack.

## Gazebo implementation
Use a saved legitimate state from loading or explicitly label a separate contact test. Existing batch_transfer.sdf has a rigid receiving fixture at .555 m, not an actual luggage stack; do not present it as stack validation. Keep all obstacle collisions active. During base reposition show upstream belt gate closed and bags physically queued at the fixed dock. Then show robot returning and receiving another bag; never animate a detached conveyor tracking the robot.

## ROS 2 communications and orchestration
Use mission pose_target only while stationary; observer and guard calculate local roof, payload envelope and occupied targets. /mission/status must expose WAIT_DOCK/queue or equivalent states so feed inhibit is visible. Total lift maps to three stages/3, reach to two stages/2. New mobile world guard uses measured base transform; old hard-coded Guard cannot be used unchanged.

## Evidence, camera and acceptance gate
Exit: one completed floor placement plus upper-layer result (pass or honest limitation); recorded tray-floor minimum, roof/wall minimum, target support and placement error. Confirm supply gated while absent and resumed only after re-docking. A 380 mm lower case top plus 380 mm upper case may fit below roof while the robot tray cannot reach its underside; check mechanism reach AND bag top clearance independently. No unconditional second-layer promise.

## Failure handling and recording rule
A failed prerequisite blocks the related claim, not just the camera shot. Save failure evidence and report the smallest fix; do not force readiness, bypass guards or substitute a kinematic animation without a “concept animation” label. Camera time is not a deadline for physics. New /mission interfaces in this brief require implementation; see technical/03_ROS2_GAZEBO_CONTRACT.md. Existing mechanism facts come from supplied archives [file:1][file:2]; contest targets come from [file:5].
