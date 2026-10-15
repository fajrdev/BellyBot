# Scene 02 — Entry Setup Dock

## Screen time and objective
00:08–00:23. Show robot entry, identify any manual fixture changeover, then show the actual receiving dock. Do not imply the old entry and transfer worlds already form one continuous autonomous sequence.

## Technical motion sequence
Use empty fully stowed robot for original entry: spawn (1.30,-3.29,.183), yaw +π/2; drive toward Y=0; pivot near (1.30,0); drive +X on ramp toward X=4.90. These are original scene datums, not a new docking solution. Existing code uses low speeds and entry can exceed this 15 s screen budget; accelerate with visible speed label. After entry stop, isolate the area and disclose removal of entry deck / installation or repositioning of retained supply. New dock motion follows computed route and world-frame guards. Do not set base pose to hop between fixtures during a purported continuous run.

## Gazebo implementation
Keep original entry_resolved scene as regression evidence. Either implement real changeover in a unified mission world or cut to a separately initialized work scene with a clear “Manual ground-crew changeover / separate setup” caption. Installation is conceptual if not contact-simulated. Save setup before/after states and any support/anchorage assumptions. Reconfigured door belt must physically meet robot rear infeed with acceptable gap, overlap, height and chassis clearance. Show this joint close-up.

## ROS 2 communications and orchestration
Original entry trigger: /aircraft/start_entry Empty; /aircraft/status and /aircraft/robot_pose supply feedback. For mission use new ENTRY→SETUP_REQUIRED→DOCK_APPROACH→DOCKED states and measured pose. Do not use both controllers at once. New dock readiness must require stationary base, level tray, valid empty-bag state, fresh feedback, upstream gate closed and measured transfer geometry. Require acknowledgement of manual setup; a film cut cannot set readiness true.

## Evidence, camera and acceptance gate
Exit only when dock and loader transition pass one empty-belt and one free-bag trial. Record minimum doorway/turn clearance and tracking error; old offline nominal clearances are not new measured results. Save joint/pose logs, dock transform, revised support layout and intervention description. Fail scene if bag must jump 198 mm or collision geometry is disabled.

## Failure handling and recording rule
A failed prerequisite blocks the related claim, not just the camera shot. Save failure evidence and report the smallest fix; do not force readiness, bypass guards or substitute a kinematic animation without a “concept animation” label. Camera time is not a deadline for physics. New /mission interfaces in this brief require implementation; see technical/03_ROS2_GAZEBO_CONTRACT.md. Existing mechanism facts come from supplied archives [file:1][file:2]; contest targets come from [file:5].
