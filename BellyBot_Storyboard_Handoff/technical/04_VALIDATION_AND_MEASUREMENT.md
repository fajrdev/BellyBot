# Evidence and acceptance tests

## Placement definition
Observer uses unique bag ID and fixed target ID. Provisional completed placement predicate: bag released from robot; no robot support/contact; footprint inside target tolerance; supported by intended floor/lower bags; no forbidden collision or roof strike; translation speed<.02 m/s and angular speed<.05 rad/s continuously for .5 simulation s. These thresholds are proposed commissioning settings, not verified hardware requirements. Derive contact/support from backend instrumentation plus geometry. Store raw evidence and tune thresholds transparently. Pose-only settling is a proxy and must be labelled.

Emit placement_complete at END of verification dwell. Count each bag once per loading run and only within [t0,t1]. Retries do not add completions. A retrieved/replaced bag does not count as new throughput in the same loading run. Unloading has a separate ledger and definition: full passage of external discharge plane, no robot support, intended external support.

## Rate
R=60*N/(t1-t0), where clock is Gazebo simulation time and t1>t0. Include every robot trip/return/docking wait occurring in the chosen interval. Freeze interval boundaries before evaluating results. Report initial queue, initial tray contents, sample duration, case types, failures and retries. A prefilled fixed-layer test cannot be called complete-mission throughput. Preserve full mission recording from setup through last bag; report end-to-end elapsed time separately, including disclosed manual interventions and setup where relevant. Analytical budget and accelerated playback are not a measured rate.

## Geometry/space
Measure actual meshes or conservative collision bounds including bag and robot at every sampled trajectory pose, then quantify sampling resolution/error bound. Identify minima for roof, wall, doorway, floor nose and obstacle contacts separately. Intentional transfer gaps are not wall-clearance violations but must pass contact transfer tests. Default aircraft safety margins from source: roof .10 m, wall .025 m; do not reduce silently. Original nominal entry sample distances are not uncertainty-aware measured clearance. Track friction, slip, pose error and bag drift, not only CAD dimensions.

## Floor load
Report assigned robot and payload mass, COM, actual support locations and loaded pose. A screen uses mass-equivalent reaction/area in kg/m²; physical contact reaction Fz/(g*A) with justified effective area is a separate estimate. Do not average over the robot's outline or conveyor length. Include retained belt supports, entry platform/ramp supports, baggage piles, dynamic transfer loads and loaded turns. Static world anchors supply ideal reaction and do not establish aircraft anchorage strength. Log unsupported/negative-reaction/tipping cases. Preserve failed cases, not only passing poses. ≤600 kg/m² requirement is not established by a single average number.

## Minimum test matrix
1. Baseline original entry and original single-bag transfer; save runtime failures without claiming prior authors ran them.
2. New mission world: names/resources/plugins load, no removed collision geometry, one actuator owner and one TF authority.
3. Empty dock: physical alignment, height, chassis clearance, source metering, both belt directions.
4. One free 20 kg and one free 32 kg case transfer in/out; vary friction; observe slip and force spikes. Soft/fragile models remain rigid-body approximations.
5. Loaded travel with level retracted tray; braking/turns/clearance/support; independent stale-feedback and node-loss tests.
6. One floor deposit and retrieval; actual upper-layer test only if reachable and stable.
7. Multi-bag unique placement and reverse unloading order; source starvation, occupied dock and blocked target behavior.
8. Continuous throughput interval and full-operation recording. Repositioning and return trips cannot be edited out of mission metrics.
9. Manual setup declaration, fixture withdrawal and empty exit if implemented.
10. Stop latch, rejected unsafe reset, safe reset and no duplicate count after recovery.

Record pass/fail/not_implemented/not_measured, artifacts and exact revision for each test. Runtime absence is not a passing test. Final video may contain explicitly labelled concept/manual operations, but its results slide cannot claim these are simulated autonomous achievements.
