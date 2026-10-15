# Coding-agent handoff — revision 2

## Use these assets

`ros2/` contains both `bellybot_dock_stack_description` (the original, unchanged robot) and `bellybot_aircraft_environment` (updated environment and mechanisms). Use `aircraft.launch.py`, not the old standalone robot supervisor. No new robot-base shrink or track modification was made.

| Item | Entry configuration | Work configuration |
|---|---|---|
| World | `entry_resolved.sdf` | `transfer_resolved.sdf` or `transfer_cutaway.sdf` |
| Robot initial pose | (1.30, -3.29, 0.183), yaw +90° | (4.10, 0, 0.003), yaw 0° |
| Robot state | Empty and fully stowed | Receive or operate inside guarded joint envelope |
| Fixtures | Docked exterior platform, narrow bridge, 180 mm turn deck, ramp | Inclined loader, level door belt, corner table, new buffer |
| Exit/transfer datum | Entry ends at X=4.90 | Receiving tray surface Z=0.498 + total mast lift |

The ground-service platform is already aligned and elevated in the entry scene; raising it from apron level is outside scope. Deck installation/removal and conveyor installation are setup operations, not an automated changeover. Do not run both fixture sets together or claim the two scenes form one uninterrupted simulation. The new station pieces have ≤0.96 m base widths and can be handled as installation modules, but their own deployment route and aircraft-approved attachment are not modeled.

## Door route

Straight ingress: X=1.30, Y=-3.29 → 0, Z≈0.183, yaw +90°. Pivot at (1.30,0) to yaw 0°. Drive +X to 4.90 m; Gazebo contact, not a base-pose setter, determines ramp pitch and vertical position. The controller sends tracked drive commands and matching wheel-index speeds, using simulator model-pose feedback. A real system needs calibrated localization.

Turning deck: X=0.34..2.26, Y=-0.89..0.89, top Z=0.180. Its spreader sits on the original floor. Ramp: X=2.26..4.06, width 1.18, slope 5.71°. Ingress is empty only: a tall bag on the raised robot would not have the same door clearance.

Nominal whole-mesh distance checks: 60 mm to the jambs, 32.77 mm minimum during rotation. With the sample spacing bound, the nominal continuous ingress/pivot path retains >30.7 mm. This excludes tracking error, dimensional uncertainty and ground-contact dynamics. Entry controller aborts at >8 mm path/turn-centre error. Its ramp envelope check is sampled geometry, not a stability proof.

## Connected transfer mechanism

| Surface or joint | Geometry / behavior |
|---|---|
| External loader | Ends at Y=-2.60, surface Z=0.300; incline is outside doorway |
| Level door belt | Y=-2.588..-0.512, X=1.30, width 0.60, surface Z=0.300 |
| Corner table | Center (1.30,0); 1.00 × 0.60 belt; receive heading +Y |
| Corner lift | Three telescopic stages, each 0..0.113333 m; 170 mm tubes preserve overlap |
| Corner yaw | 0..π/2 about local -Z; zero receives +Y, π/2 discharges +X |
| Corner receiving lift | 0.198 + robot mast lift; table surface matches robot surface |
| Buffer lift | Two stages 0..0.129 m; home belt Z=0.370; delivery lift 0.128 + mast lift |
| Folding nose | 228 mm long; joint -π/2 parked downward, 0 horizontal; folds before rotation |
| Buffer belt | X=2.044..3.644, width 0.60; 10 mm tail plate ends at X=3.656 |
| Robot belt entry | X≈3.658, surface Z=0.498 + mast lift |

Nominal transfer gaps: 12 mm inlet-to-table, 8 mm table-to-folding nose, 8 mm nose-to-main buffer, 2 mm buffer-to-tail plate and 2 mm plate-to-robot. The rotating deck clears the door belt by lifting first. It clears the output by folding the nose first. These are intentional small transfer gaps, not 25 mm obstacle clearances. Bag straps, soft-shell sag and impact need physical/contact tests.

Belts use Gazebo TrackController contact velocity. Rollers are continuous joints with angular speeds matched to belt speed. No bag is welded to a link. The temporary deck and conveyor fixtures are anchored/static simulation fixtures: aircraft-floor loads through these supports and anchorage strength are not validated by that representation.

## Commands and feedback

| ROS interface | Type | Meaning |
|---|---|---|
| `/aircraft/start_entry` | `std_msgs/Empty` | Begin checked entry from the supplied platform pose |
| `/aircraft/start_transfer` | `std_msgs/Empty` | One selected tracked-bag commissioning transfer |
| `/aircraft/pose_target` | `std_msgs/Float64MultiArray` | [total lift m, upper yaw rad, reach m, tray pitch rad] |
| `/aircraft/stop` | `std_msgs/Bool` | `true` latches stop |
| `/aircraft/status` | `std_msgs/String` | JSON phase, readiness and fault report |
| `/aircraft/robot_pose` | `tf2_msgs/TFMessage` | Gazebo PosePublisher model pose |
| `/aircraft/world_poses` | `tf2_msgs/TFMessage` | Selected `tracked_bag` PosePublisher, in world coordinates |
| `/aircraft/*_joints` | `sensor_msgs/JointState` | Robot, corner and buffer joint feedback |

The single-bag controller checks alignment, settling, lift completion, rotation completion and bridge state before running belts. It stops for missing bag pose, drift, timeout or misaligned transfer heights. All relevant joint/model feedback must be fresher than 0.35 s. Raw actuator bridge topics are internal; launch exactly one controller.

No general baggage perception, navigation stack, collision-aware placement into a growing pile or worker detection is supplied. The simulator truth-pose adapter is an explicit commissioning aid. Current bag limits use the conservative 780 × 500 × 380 mm, 32 kg envelope. Any height increase above 380 mm is rejected. Use the documented lower cap for receiving even if the bag allows a higher placement lift. Supply measured dimensions with launch arguments `bag_height` and `bag_mass`; defaults are 0.38 m and 32 kg. The controller does not infer height from the truth-pose adapter.

## Host validation order

1. Build both packages; run `tools/runtime_smoke_check.sh` and inspect all logs. The scripts were not executed here because ROS/Gazebo are unavailable.
2. Verify joint signs and belt surface directions with an empty fixture. Run entry slowly, inspect track slip, side-wall clearance and ramp pitch. Do not set the robot pose during this test.
3. Start a fresh transfer world; unpause; spawn one bag using `tools/spawn_transfer_bag.py --execute`. Confirm root pose messages use world coordinates, then trigger the transfer. Verify actual bag support at every transition.
4. Test the different free bag models and jam/pose-loss stops. Measure pressure from physical contact forces and actual supported patch areas; the controller's assigned-mass screen is only a conservative rejection heuristic.
5. Only after contact trials, profile the cycle. The supplied serial commissioning sequence at 0.25 m/s is not a 12 bags/min solution. Reaching that target requires measured optimization, buffering/overlap and potentially a continuous corner-transfer redesign; no throughput compliance is claimed.

## Evidence files

- `resolution_checks.json`: 2,432 entry/pivot samples, 901 ramp envelope samples, 253 mechanism samples, 477 robot/buffer poses checked with conservative per-part bounds; full triangle meshes at finite doorway walls.
- `controller_contract_checks.json`: meaningful limit/interlock failure checks using synthetic feedback.
- `validation.json`: offline file/geometry/inertia checks, explicitly not a Gazebo result.
- `Entry_Dock_R2_check.json`, `Corner_Transfer_R2_check.json`: STEP reimport validity.
- `Resolution_Review_R2.pdf`: six-page explanation and requirement status.

The entry/transfer GLBs are colored CAD previews. The URDF is for articulated RViz inspection, while SDF carries physics/contact plugins. Use metres for simulation and millimetres for STEP. The default `aircraft_system.urdf` is fixed at the work datum; use the dynamic model pose if extending RViz to entry mode.
