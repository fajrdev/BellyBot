# Scene 06 — Stop Recovery

## Screen time and objective
01:33–01:48. Demonstrate a real implemented software emergency stop and a controlled, supervised recovery procedure. Verify immediate actuator zeroing, state latching, rejection of subsequent motion commands, and recovery to `IDLE` through the supervised `/mission/reset` Trigger service.

## Technical motion sequence
1. Stop Command Injection: While operating, publish `Bool(data=True)` to `/mission/stop`.
2. Immediate Actuator Shutdown: Controller cuts all base wheel speeds and conveyor belt speeds to 0.0 immediately ($< 0.02\text{ s}$ latency), freezes arm joints at current positions, latches state to `STOP_LATCHED`, and emits `stop_commanded` event.
3. False Reset Rejection: Publish `Bool(data=False)` to `/mission/stop`; verify latch remains locked and false value is ignored per contract.
4. Motion Command Rejection: Attempt to publish new motion commands via `/mission/command`; verify controller rejects with logged error while `STOP_LATCHED`.
5. Supervised Reset: Operator calls `/mission/reset` (`std_srvs/srv/Trigger`); controller verifies fresh feedback ($< 0.40\text{ s}$) and stationary robot pose before clearing the latch, resetting fault string, emitting `reset_accepted`, and returning state cleanly to `IDLE`.

## Gazebo implementation
Stop and recovery tested live under active physics in `worlds/mission_shuttle.sdf`. Sim clock `/clock` remains running throughout the event to record real reaction latencies. Background wall-clock watchdog thread monitors `/clock` and triggers `STOP_LATCHED` if simulation time freezes for $> 2.5\text{ s}$ wall time.

## ROS 2 communications and orchestration
- Stop input: `/mission/stop` (`std_msgs/msg/Bool`).
- Reset service: `/mission/reset` (`std_srvs/srv/Trigger`).
- Controller telemetry updates: `/mission/status` immediately publishes `stop_latched = True` on stop, and `stop_latched = False` on reset.
- Observer audit trail: `results/mission_events.csv` logs `stop_commanded` and `reset_accepted`.

## Evidence, camera and acceptance gate
- Verified test run: `scratch/test_stop_recovery.py` (exit code 0).
- Telemetry confirms immediate zeroing of wheel and conveyor velocities.
- Verified recovery to `IDLE` with full system operational integrity.
- Narration: "Missing telemetry or an operator stop signal globally latches the controller; recovery requires supervised reset validation."

## Failure handling and recording rule
A failed prerequisite blocks the related claim. Evidence preserved in `results/mission_events.csv`.

