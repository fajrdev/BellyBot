# Scene 03 — Measured Loading

## Screen time and objective
00:23–00:53. Show one uninterrupted 30 s simulation-time loading interval with timer and completed-placement count. Six placements is optional; retain the test even if actual throughput is lower.

## Technical motion sequence
At t0 document queue fill, robot pose, any bag already on tray and target occupancy. Prefer empty robot and queued supply outside the aircraft. Dock/reserve one bag→transfer with matched belts→verify full support→stop belts→retract/level carrying pose→travel along validated route→stop→approach target→deposit by tested belt/retraction sequence→verify release and stable target support→return empty. Repeat. No concurrent base motion and tray reach/yaw/pitch unless separately proven; default sequential. Deposits must occupy distinct nonintersecting targets, not recycle a bag or pile at the same coordinate.

## Gazebo implementation
Use free dynamic bags, gravity and actual contact-driven belts. Spawn source bags only at external supply zone with no overlap and enough spacing for gate stopping distance. Create additional bags outside view only if logged source-in events and physically supported; no spawning on the robot. No pose resets, time jumps, pause/edit or hidden holds during the measured interval. Tune speed only after support/force trials. If 30 s shows zero placements, report zero and add a clearly labelled separate full-cycle insert elsewhere; do not cut it into this interval.

## ROS 2 communications and orchestration
Issue one start_load request through /mission/command, then let the controller run feedback-driven state transitions. Observer emits received, supported, placement_candidate, placement_complete or placement_failed using unique IDs and simulation stamps. Counter listens only to placement_complete. /mission/metrics computes 60*N/(t1-t0). Stop requests cancel feeding and motion safely. Existing batch_cycle inputs cannot be forced true just to drive the count.

## Evidence, camera and acceptance gate
Acceptance: continuous recording, /clock/log correspondence, unique bag target records, support/release evidence and no disallowed state mutation. Count after the 0.5 s provisional settling test completes INSIDE the interval, not retroactively. Tune that dwell only with justification and disclose it. Display “N completed / T simulation s = R bags/min over this interval”. State “prefilled queue” if applicable. Keep 1× if T=30 s fits 30 screen seconds. If another T is used, show T and any playback factor explicitly; no sustained-rate claim from a short burst. Log travel, docking, receive and deposit durations to identify bottlenecks.

## Failure handling and recording rule
A failed prerequisite blocks the related claim, not just the camera shot. Save failure evidence and report the smallest fix; do not force readiness, bypass guards or substitute a kinematic animation without a “concept animation” label. Camera time is not a deadline for physics. New /mission interfaces in this brief require implementation; see technical/03_ROS2_GAZEBO_CONTRACT.md. Existing mechanism facts come from supplied archives [file:1][file:2]; contest targets come from [file:5].
