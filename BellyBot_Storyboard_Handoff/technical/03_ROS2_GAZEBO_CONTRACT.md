# ROS 2 / Gazebo communications contract

## Actual existing public interfaces
| Interface | ROS type | Meaning / restrictions |
|---|---|---|
| /aircraft/start_entry | std_msgs/msg/Empty | Start original empty entry only |
| /aircraft/start_transfer | std_msgs/msg/Empty | Original one tracked-bag corner/buffer sequence only |
| /aircraft/pose_target | std_msgs/msg/Float64MultiArray | [total_lift, upper_yaw, total_reach, tray_pitch]; m,rad,m,rad |
| /aircraft/stop | std_msgs/msg/Bool | true latches; false does not reset |
| /aircraft/status | std_msgs/msg/String | Existing JSON report; verify fields against code |
| /aircraft/robot_pose | tf2_msgs/msg/TFMessage | Model-level Gazebo pose adapter |
| /aircraft/world_poses | tf2_msgs/msg/TFMessage | Original selected tracked_bag only |
| /aircraft/robot_joints | sensor_msgs/msg/JointState | Real simulated robot feedback |
| /bellybot/pose_target | std_msgs/msg/Float64MultiArray | Standalone [lift,yaw,reach,infeed_extension,pitch,infeed_lift] |
| /bellybot/belt_cmd | std_msgs/msg/Float64 | Standalone requested tray speed, verify contact sign |
| /bellybot/cmd_vel | geometry_msgs/msg/Twist | Standalone tracked drive, guarded supervisor |
| /bellybot/stop | std_msgs/msg/Bool | Standalone stop; not a safety-rated physical device |
| /bellybot/motion_active | std_msgs/msg/Bool | Standalone supervisor acknowledgement |
| /bellybot/bag_ready, bag_supported, bag_placed | std_msgs/msg/Bool | batch_cycle inputs; require an observer, not arbitrary true flags |

Do not use the standalone infeed's six-value pose examples in the four-value aircraft interface. Retired infeed parameters are not the removed corner/buffer's joints. Prefer a new four-value mission manipulation API and reuse compatible planning primitives behind an adapter.

## Actual retained Gazebo bridge routes in aircraft config
- /clock ← /clock: rosgraph_msgs/msg/Clock ↔ gz.msgs.Clock, GZ_TO_ROS.
- /aircraft/robot_joints ← /model/bellybot_dock_stack/joint_states: JointState ↔ gz.msgs.Model.
- /aircraft/robot_pose ← /model/bellybot_dock_stack/pose: TFMessage ↔ gz.msgs.Pose_V.
- /aircraft/actuation/cmd_vel → /model/bellybot_dock_stack/cmd_vel: Twist ↔ gz.msgs.Twist.
- Float64 ↔ gz.msgs.Double commands /aircraft/actuation/{lift_stage_target,extension_stage_target,yaw_target,pitch_target,tray_belt_speed,tray_roller_speed,left_wheel_speed,right_wheel_speed} → /model/bellybot_dock_stack/<same suffix>.
- /aircraft/actuation/loader_speed → /environment/loader/speed.
- /aircraft/actuation/door_speed → /environment/door_belt_contact/speed.
- /aircraft/actuation/door_roller → /environment/door_belt_contact/roller_speed.

Keep private actuation routes or consistently rename them in ONE adapter/bridge. The storyboard node must never publish directly alongside an actuator owner. Multiple independent /joint_states publishers need a coherent aggregation strategy; do not duplicate state authority.

## Proposed NEW mission interfaces — implement before use
These are specifications, not existing ROS commands. Default to reliable QoS depth 10; sensors can use host-appropriate sensor QoS if both ends match. Stop must latch in controller state even if its transport is not transient-local. All records carry simulation timestamps; watchdog uses independent monotonic wall time too.

| Interface | Type | Proposed semantics |
|---|---|---|
| /mission/command | std_msgs/msg/String | JSON {request_id, command}; command=start_load,start_unload,stop,request_recovery,start_exit |
| /mission/pose_target | std_msgs/msg/Float64MultiArray | [total_lift_m,yaw_rad,total_reach_m,pitch_rad]; only accepted in stationary manipulation state |
| /mission/status | std_msgs/msg/String | JSON {run_id,sim_time_s,state,active_bag_id,fault,stop_latched,feedback_ages_s} |
| /mission/events | std_msgs/msg/String | JSON schema in config/event_schema.json; append-only transition/measurement events |
| /mission/bags | std_msgs/msg/String | JSON array of observations: id, stamp, frame, pose, dimensions, mass, velocity, supporting_surface_ids, evidence_mode |
| /mission/metrics | std_msgs/msg/String | JSON current N,T,rate,placement IDs, failed/retried counts, evidence scope |
| /mission/stop | std_msgs/msg/Bool | true latches globally; false NEVER clears |
| /mission/reset | std_srvs/srv/Trigger | Accept only after cause cleared, stationary supported state and fresh valid feedback; respond success+reason |

Only mission controller owns base and joint/belt output. Storyboard orchestrator submits command requests; observer publishes bag evidence; logger records; overlay reads. Proposed core states: IDLE, ENTRY, SETUP_REQUIRED, DOCK_APPROACH, DOCKED, RECEIVE, TRANSPORT, PLACE_APPROACH, DEPOSIT, VERIFY_PLACEMENT, RETURN_EMPTY, RETRIEVE_APPROACH, ACQUIRE, RETURN_LOADED, DOOR_DISCHARGE, VERIFY_UNLOAD, STOP_LATCHED, RECOVERY_CHECK, EXIT, COMPLETE.

Motion has measured completion predicates and a timeout fault, never timeout=success. Do not invent action servers. If the agent chooses custom messages/actions instead of JSON, update all briefs and install/build/test the interfaces before recording.

## Feedback and sensors
Original bridge sees one tracked_bag; add uniquely named multi-bag observation routes or an aggregate world pose adapter. Confirm root model poses are world coordinates; frame strings for new world will differ from original transfer_resolved filters. PosePublisher alone does not prove contact/support. Add Gazebo contact sensors/system output as supported by the installed backend, and bridge its actual discovered message type after introspection. Require known support geometry + contact evidence + settled motion. If contact data cannot be obtained, label pose-only proxy and do not promote it to validated support/force sensing.

Joint feedback freshness target from old controller is .35 s; independently verify timestamps and publish per-source age. Missing bag pose, base pose, required joint/contact evidence or transform rejects new movement and commands a controlled stop. A ROS node using sim-time timers alone cannot detect a frozen /clock; use a wall-time transport/watchdog path.

## Baseline commands (existing, run in original configuration only)
```bash
ros2 pkg prefix bellybot_aircraft_environment
ros2 pkg prefix bellybot_dock_stack_description
ros2 launch bellybot_aircraft_environment aircraft.launch.py mode:=entry
# Unpause Gazebo, confirm fresh feedback, then:
ros2 topic pub --once /aircraft/start_entry std_msgs/msg/Empty '{}'
ros2 topic echo /aircraft/status
# Emergency stop demonstration; restart required in this original controller:
ros2 topic pub --once /aircraft/stop std_msgs/msg/Bool '{data: true}'
```
Run one mode/controller at a time. Do not invoke start_transfer in the conveyor-removed world. New mission.launch.py is not supplied or implemented by this document.

## Host introspection and recording
```bash
printenv ROS_DISTRO
ros2 topic list -t
ros2 node list
ros2 topic info /clock -v
gz topic -l
gz service -l
```
After implementing mission topics, record `/clock /mission/status /mission/events /mission/bags /mission/metrics /aircraft/robot_joints /aircraft/robot_pose /tf /tf_static` plus actual contact topics and actuator commands. Use `ros2 bag record -o <new_run_directory> <resolved_topic_list>` only after resolving real names; do not use literal angle-bracket placeholders. Store launch args, seed, physics backend, timestep, RTF, package revision, bag masses/friction and observer mode with each run.

Existing contracts are grounded in supplied code [file:1][file:2]; /mission interfaces are proposed additions.
