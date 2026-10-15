# Scene 05 — Unload Return Withdraw

## Screen time and objective
01:13–01:33. Show an accessible bag physically acquired, carried back, discharged through the retained door path and moving out toward apron. Include a concise withdrawal/setup caption or accelerated shot so the operation has an end.

## Technical motion sequence
Select accessible uppermost/frontmost bag with no supported bags above it. Stop base, approach with validated tray nose, run belt toward robot at low verified speed, confirm full supported acquisition; level and retract before travel. Return supported to the dock. Align tray rear with door belt; hold robot stationary; reverse the appropriate belts so bag travels robot→door belt→external loader. Observe bag passing external completion plane fully, not just disappearing. Stop supply/robot, clear transfer zone and declare required fixture changeover for empty exit. Use an actual validated reverse exit route only if implemented; otherwise label withdrawal procedure as planned/manual.

## Gazebo implementation
Retrieve bags actually deposited or label an independent retrieval setup. Never delete a buried bag or phase the tray through a stack. The original floor_pickup world is only a commissioning fixture. Belt reversal, slope descent and tipping must be physically tested; do not infer unloading success from loading success. Sensor plane at exterior discharge verifies full bag passage. Avoid removing a bag model until past that plane and outside view, with event logged. Show retained equipment withdrawal by declared crew procedure, not conveyor disappearance.

## ROS 2 communications and orchestration
New start_unload command initiates RETRIEVE_APPROACH→ACQUIRE→RETURN_LOADED→DOOR_DISCHARGE→VERIFY_UNLOAD. Observer provides bag IDs and support evidence. Mission emits unload_complete only after external crossing and release from robot. start_exit is NEW and gated by empty tray, stopped belts, correct deck arrangement and validated route. Existing start_entry has no reverse/exit API. Record manual intervention events and separate them from autonomous state transitions.

## Evidence, camera and acceptance gate
Exit: at least one full unload transaction with extraction order explained (“accessible top/front bags first; do not undermine supporting bags”), return-path evidence and operation termination. Show a short disclosed acceleration or caption for manual withdrawal if 20 s screen budget is insufficient. Do not count a bag merely removed from pile as an aircraft-unloaded bag. State retrieval limitations for flexible/fragile baggage.

## Failure handling and recording rule
A failed prerequisite blocks the related claim, not just the camera shot. Save failure evidence and report the smallest fix; do not force readiness, bypass guards or substitute a kinematic animation without a “concept animation” label. Camera time is not a deadline for physics. New /mission interfaces in this brief require implementation; see technical/03_ROS2_GAZEBO_CONTRACT.md. Existing mechanism facts come from supplied archives [file:1][file:2]; contest targets come from [file:5].
