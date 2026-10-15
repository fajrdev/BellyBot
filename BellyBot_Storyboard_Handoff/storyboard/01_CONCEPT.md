# Scene 01 — Concept

## Screen time and objective
00:00–00:08. Establish aircraft, exterior loading path, fixed door dock, mobile robot, placement zone and clear return corridor in one cutaway. Show the two removed assemblies absent; do not hide the doorway height problem with the camera.

## Technical motion sequence
Begin stationary with empty level tray. Show one staged bag outside the hold, the dock reservation and target locations. Overlay directional arrows: loading toward hold; unloading toward door. Arrows are explanatory only, not evidence that bags moved. The route must include the robot's return to the dock, not an arrow from a missing conveyor to a remote robot.

## Gazebo implementation
Use the new mission world after the geometry gate. Preserve curved-hold collision geometry even if roof/wall visuals are hidden for filming. Use a presentation camera independent of physics. Establish unique bag IDs and physically supported source positions. Do not use old transfer_cutaway unmodified: it contains the removed assemblies.

## ROS 2 communications and orchestration
Read /clock, /mission/status and /mission/bags; do not command actuators for a concept pan. Overlay evidence_mode=sim_ground_truth if using model poses. On-screen route labels must match the geometry decision record and actual scene tree.

## Evidence, camera and acceptance gate
Entry: confirmed removal_map, selected supply architecture and collision-checked dock. Exit: viewer can trace apron→retained loader→door belt→robot→placement and the return path. Capture one labelled wide frame. Narration: “Door-side supply is metered; BellyBot returns to collect each bag.” Do not say autonomous continuous following-feed.

## Failure handling and recording rule
A failed prerequisite blocks the related claim, not just the camera shot. Save failure evidence and report the smallest fix; do not force readiness, bypass guards or substitute a kinematic animation without a “concept animation” label. Camera time is not a deadline for physics. New /mission interfaces in this brief require implementation; see technical/03_ROS2_GAZEBO_CONTRACT.md. Existing mechanism facts come from supplied archives [file:1][file:2]; contest targets come from [file:5].
