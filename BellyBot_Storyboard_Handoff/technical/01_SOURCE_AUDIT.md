# Source audit — inspected files, not runtime certification

Paths below are relative to `BellyBot_Aircraft_Environment_v2/` unless prefixed ROBOT. ROBOT means `BellyBot_DockStack_C_v3/bellybot_dock_stack_C_v3/`. Package-relative paths are under `ros2/<package>/`.

## Existing architecture
- `ros2/bellybot_aircraft_environment/launch/aircraft.launch.py`: only modes entry/transfer; starts Gazebo without `-r` (paused); uses aircraft_bridge.yaml and aircraft_controller.py. Work cutaway is selected by `cutaway:=true`. No mission/shuttle mode exists.
- `worlds/entry_resolved.sdf`: dedicated entry fixtures. `worlds/transfer_resolved.sdf` and `transfer_cutaway.sdf`: robot plus loader, door_belt, corner_transfer, longitudinal_buffer. The robot is embedded, not simply included from a robot model file.
- `docs/AGENT_HANDOFF.md`: explicitly says entry/work fixtures are separate; deployment route and approved attachment of work conveyors are not modelled. Existing single-bag commissioning transfer is not a 12/min solution.
- `scripts/aircraft_controller.py`: requires corner and buffer feedback in transfer mode. Base remains locked at nominal work datum. start_transfer handles a selected `tracked_bag`, not a general continuous stream.
- `scripts/aircraft_guard.py`: Guard.evaluate adds `[4.1,0,.003]` to kinematic vertices. It is NOT a mobile world-frame collision checker. Guard.plan is sampled staged planning. Add measured base SE(3) and real scene obstacles for a new mobile mode.
- `scripts/transfer_core.py`: old station sequencing; retain for legacy tests, not as a fake replacement feeder.
- `config/aircraft_bridge.yaml`: station feedback and actuator bridges are coupled to those models. Remove their requirements from the new variant, not from unrelated legacy launches.
- `urdf/aircraft_system.urdf`: fixed robot_work_datum, includes door/corner/buffer URDF trees. It is a work-mode visual assembly, not mobile-base physics. Replace fixed visualization attachment with a single dynamic world/odom→base TF path for mission RViz.

## Preserve robot implementation
In `bellybot_dock_stack_description`: `scripts/control_core.py`, `scripts/supervisor.py`, `scripts/batch_cycle.py`, `urdf/bellybot.urdf`, `gazebo/bellybot_dock_stack.sdf`, `config/bridge.yaml`. Preserve existing plugin orientation/sign conventions until tested. Standalone `sim.launch.py` launches a separate infeed and supervisor; it is not the aircraft mission launcher.

Uploaded embedded versus standalone copies: supervisor.py, control_core.py and bellybot.urdf are byte-identical; bellybot_dock_stack.sdf is not. Audit differences before selecting a source. Do not install two packages with the same name into the active overlay.

## Names and limits found in source
- Three lift_stage_{1,2,3}_joint, each 0..0.2 m; command total lift/3.
- extension_mid_joint and extension_tip_joint, each 0..0.5 m; command reach/2.
- yaw_joint raw URDF ±95°; aircraft operating guard restricts ±65°. Raw URDF limits are not a safe operating envelope.
- tray_pitch_joint 0..32°; positive nose-down. Floor pitch requires yaw≈0, lift≈0, reach≥0.72 m under aircraft guard.
- Guarded yaw requires lift≥0.08 m and reach≤0.50 m; preserve level-before-yaw and retract-before-yaw sequencing.
- Aircraft nominal roof 1.12 m, roof margin .10 m, wall margin .025 m; receive lift cap .13 m for .38 m bag. Other bag-height caps must be recomputed from guard and local roof, not raw lift travel.
- Payload screen: up to 32 kg and conservative .78×.50×.38 m envelope. Guidelines allow 20–32 kg; some existing models have lower heights than the conservative maximum.
- Archived floor-load/timing checks are analytical/offline. The standalone validation report gives a 4.6566 s prepared-layer estimate, NOT measured throughput. Reactions at a fixed datum are not a moving-floor-load proof.

## Implementation hazards observed directly
The source currently includes timeout-based arm segment completion after duration+2 s even if joint error remains. New mission acceptance must require measured joint convergence, not trust arm_busy alone. Status `joint_feedback_fresh` is based on bool(self.q), not per-joint age: calculate freshness independently. Documentation and controller entry tracking values differ; inspect actual code and baseline behavior instead of copying prose thresholds. Do not certify an 8 mm path abort from the old handoff text.

Sources: supplied archives [file:1][file:2]. See evidence/source_inventory.json for original hashes and actual model/joint names.
