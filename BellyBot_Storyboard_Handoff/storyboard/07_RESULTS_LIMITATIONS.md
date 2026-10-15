# Scene 07 — Results Limitations

## Screen time and objective
01:48–02:00. Summarize measured operation, compatibility and limitations without substituting analytical calculations for runtime evidence.

## Technical motion sequence
End with robot stationary, bags supported and transfer path clear. No new mechanism movement needed. Results overlay reads run artifacts, not hard-coded success numbers.

## Gazebo implementation
Export stable camera and final scene state. Keep render overlays separate from physics. Check final MP4 duration≤120 s, labels legible and all acceleration/cuts clearly disclosed. Do not trim the middle of the measured loading segment to meet runtime.

## ROS 2 communications and orchestration
Read metrics generated from event log and /clock; stop further task requests. Store run IDs next to every displayed metric. Prefer a generated metrics JSON consumed by overlay with explicit null/not_measured entries. Runtime RTF is distinct from playback speed and simulated throughput.

## Evidence, camera and acceptance gate
Display: actual N/T/rate with scope; loading and unloading transaction status; measured minimum roof/wall clearance and measurement method; floor-load SCREEN versus measured contact reactions/patches; stop/recovery result; autonomous versus manual steps. Guidelines ≥12 bags/min and ≤600 kg/m² are targets, not automatic passes. List unresolved supply deployment, loaded-navigation, stacking, soft/fragile-bag and real-aircraft-approval limitations that remain. If N=6,T=30, write “12 bags/min over 30 s”; no sustained-performance statement. Do not say “guaranteed winner” or “fully autonomous” with manual setup.

## Failure handling and recording rule
A failed prerequisite blocks the related claim, not just the camera shot. Save failure evidence and report the smallest fix; do not force readiness, bypass guards or substitute a kinematic animation without a “concept animation” label. Camera time is not a deadline for physics. New /mission interfaces in this brief require implementation; see technical/03_ROS2_GAZEBO_CONTRACT.md. Existing mechanism facts come from supplied archives [file:1][file:2]; contest targets come from [file:5].
