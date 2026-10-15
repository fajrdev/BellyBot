# Ordered agent work plan

## Phase A — preserve and inspect
Save git diff/status and baseline launch/logs; no blanket reset. Resolve active package duplication and SDF differences. Record installed ROS/Gazebo/plugin versions. Copy only needed source changes into selected live workspace. Read upstream source before adapting it. Run available offline tests and runtime smoke tests; no evidence here establishes baseline runtime success.

## Phase B — identify and gate architecture
Produce labelled removal overlay. Confirm target IDs. Prove retained door-belt height/loader transition and a full-body dock pose. Compare possible approaches geometrically before coding navigation. Produce docking_decision.md with transforms, heights, clearances, bag envelope, fixture supports, entry/changeover and withdrawal route. If direct dock is impossible, stop this phase and present evidence plus minimum alternative; do not spend time polishing a fake loading video.

## Phase C — variant and communications
Create new mission launch/world/bridge/display using the preserved robot. Remove confirmed conveyor models and dependencies from active variant. Update generator variant switches and build/install entries. Adapt robot pure planning to measured base transform, real obstacles and bag support state. One actuator owner; do not fork old lock logic and delete all safety checks. Implement /mission messages, observer, latched stop and recovery, wall-time watchdog and event logger. Test convergence-based completion, not the old timeout-as-success path.

## Phase D — commission contact operations
Verify empty movement and belt signs first; one dynamic bag transfer; loaded transport; one deposit; retrieve; external discharge. Save actual commands and selected poses into each scene MD. Enable multiple bags only after all single-bag gates pass. Use conservative speeds first; speed changes require repeated loaded/contact tests. No optimization bypassing safeguards.

## Phase E — full mission and measurements
Run complete loading/unloading transactions with source queue management and stable placement observer. Add reachable stacking or document the boundary. Record throughput and full mission metrics. Inject one known implemented fault; recover with explicit operator authorization. Validate fixture withdrawal/exit or label as planned/manual. Run original scenes again for regression comparison.

## Phase F — film
Capture all seven scenes only from valid states. Keep scene 03 continuous with actual simulation timer and counter. Edit other scenes with clear playback/changeover labels. Generate results overlay from artifacts. Export ≤120 s MP4 and verify playback, total duration, readability, evidence claims and source revision. Preserve raw captures/logs so a judge's question can be answered without rerunning everything.

## Completion report required
- Implemented versus planned per scene.
- Exact build/launch commands tested, package paths and revisions.
- Removed object mapping and active-world collision verification.
- Geometry/supply decision and interventions still manual.
- Per-test evidence paths, actual rates and failure/bottleneck discussion.
- Original functionality retained and regressions, if any.
- Limitations and minimum remaining work; never generic “all tests pass” without logs.
