# BellyBot A — Known Issues & Design Limitations

This document records observed runtime behavior, simulation approximations, and core physical/mechanical limitations for Concept A v0.2. It explicitly separates simulation runtime issues from mechanical design limitations.

---

## 1. Simulation & Runtime Issues

### 1.1 Ideal Actuators & Position Control
- The Gazebo model uses `gz::sim::systems::JointPositionController` configured with `use_velocity_commands: true`.
- **Limitation:** This commands joint velocities directly to match position targets rather than simulating motor torque, lead-screw / ball-screw dynamics, cable tension, gearbox friction, or motor back-EMF.
- **Consequence:** It cannot establish whether real physical actuators have adequate torque, force, or holding brake capacity to lift or hold 32 kg payloads under acceleration.

### 1.2 Self-Collision Disabled
- In `drive_world.sdf` and `bellybot_A.sdf`, `<self_collide>false</self_collide>` is configured.
- **Consequence:** Telescoping stages, chassis links, and infeed structures do not generate internal contact forces. Geometry clearances must be verified geometrically (e.g. via CadQuery / collision checking in `validate.py`) rather than relying on Gazebo collision detection.

### 1.3 Track Contact Approximation
- Ground traction is simulated using Gazebo's `TrackController` and `TrackedVehicle` plugins operating on fixed track link contact envelopes (`left_track_link`, `right_track_link`).
- The 12 road wheels rotate via independent `JointController` velocity topics purely for visual animation.
- **Limitation:** The tracks are not continuous compliant rubber or segmented belts wrapping around idlers and sprockets. Track surface contact motion is an idealized kinematic contact-surface slip model (`steering_efficiency = 1.0`).
- **Consequence:** Actual turning scrub, track slip on smooth aircraft hold aluminum panels, and localized track pressure distributions are not captured by the contact solver.

### 1.4 Coarse Collision Geometries
- Robot link collisions are approximated using simplified bounding shapes and convex shells. Fine structural features and fastener heads are excluded.

### 1.5 Lack of Real-Time Safety Watchdog / Timeout
- The simulation topics (`cmd_vel`, `left_wheel_speed`, `right_wheel_speed`) maintain their last commanded velocity until explicitly commanded to zero.
- An application-level watchdog node must be implemented in ROS 2 to send zero velocity on command dropouts.

---

## 2. Mechanical Design & Architectural Limitations

### 2.1 Minimum Tray Height vs Floor-Level Handling
- **Status:** The current tray surface sits at a minimum height of approximately **410 mm** above ground level.
- **Brief Requirement:** Aircraft belly holds require loading from the floor level up to the hold ceiling (~1.2 m) without damaging luggage.
- **Measured Simulation Findings in B737-800 Hold:**
  - For Layer 1 (floor placement), the bag drops **230 mm** from the tray deck to the floor.
  - Impact velocity: **2.12 m/s**.
  - Impact kinetic energy on a 32 kg suitcase: **72.2 J**.
  - This violates gentle handling criteria for fragile bags.
  - In contrast, Layer 2 (top stack) placement drops only **40 mm** (energy **12.6 J**), confirming gentle placement.
- **Limitation:** The current two-stage vertical telescope cannot reach floor level (0 mm) for first-layer bag placement or last-layer unloading. An articulated tilting nose or lowering ramp mechanism is required in Concept A v0.3.

### 2.2 Lack of Active Conveyor Contact & Baggage Transfer
- The tray and infeed CAD meshes feature visual roller and belt geometry, but no moving belt surface or friction transfer controller is implemented in simulation.
- **Simulation Surrogate:** The B737 hold simulation utilizes Gazebo's `DetachableJoint` plugin to simulate pickup from infeed and release at the stack. While functional for kinematic validation, physical belt friction and irregular soft-shell luggage deformation require a specialized contact solver plugin.

### 2.3 Throughput Target: 12 Bags / Minute
- **Status:** The hackathon brief requires an average throughput of at least 12 bags per minute (nominal cycle time <= 5.0 seconds per bag, or 4.0 seconds at 80% duty cycle).
- **Measured Single-Robot Cycle Time:** In the B737 hold simulation, average cycle time was measured at **~22 seconds per bag** (effective rate: **~2.7 bags/minute**).
- **Physical Feasibility Finding:** A single autonomous mobile robot reciprocating back and forth along a 7-meter aircraft hold cannot physically achieve 12 bags/min with safe acceleration ($a \le 0.3\text{ m/s}^2$) and single-bag carriage.
- **Required Architecture for 12 bags/min:**
  1. Multi-bag buffer / carousel carrying 3–4 bags per trip.
  2. Indexing conveyor floor in the hold to minimize vehicle travel distance.
  3. High-speed shuttle gantry (> 1.5 m/s).

### 2.4 Floor Loading Margin (< 600 kg/m²)
- **Status:** Total robot mass budget is 152 kg; payload is 32 kg (total 184 kg).
- **Ideal Uniform Static Load:** Over the 0.836 m² track area, uniform static loading is ~220.1 kg/m² (or 330.1 kg/m² with a 1.5 dynamic factor).
- **Asymmetric Extended Reach:** At maximum lateral extension (yaw 90°, extension 550 mm), the shifted center of mass increases the heavy track reaction to ~478.2 kg/m² factored.
- **Four-Patch Compliance Screening:** In the worst-case yaw angle (~37.4°), the maximum patch pressure reaches **574.0 kg/m² factored**.
- **Limitation:** While 574 kg/m² is below the 600 kg/m² threshold, the safety margin is narrow (< 5%). Transient dynamics, uneven aircraft hold floor panels, or cargo door sills could cause localized patch loads exceeding 600 kg/m².

### 2.5 Cargo Hold Envelope & Roof Curvature
- At maximum lift with a 400 mm tall bag, the bag top reaches **1050 mm**, leaving ~130 mm clearance under an assumed 1180 mm flat ceiling.
- Real narrow-body aircraft holds (e.g. A320, B737) have significant lateral curvature towards the fuselage walls. The robot must plan lift and extension trajectories that account for hold curvature.

### 2.6 Interlocks and Motion Sequencing
- Hardware interlocks are currently not enforced at the controller level:
  1. Base must be stationary during infeed transfer.
  2. Lift must raise >= 80 mm before initiating yaw to avoid hitting chassis members.
  3. Telescopic extension must remain fully retracted during yaw rotation.
  4. Yaw and extension must return to zero before descending to home position.
- These sequences must be implemented in a dedicated state machine (e.g. Nav2 / BehaviorTree.CPP / ROS 2 lifecycle node).

---

## 3. Summary of Development Priorities for Next Milestone

1. **Active Conveyor & Payload Handling:** Implement dynamic conveyor belts on the infeed and tray for realistic bag transfer.
2. **Floor-Level Bag Infeed/Placing:** Redesign the front attachment to lower to hold floor level (~50 mm).
3. **Automated Cycle Executive:** Build a ROS 2 behavior tree to execute the 5-second pickup-lift-yaw-extend-place sequence.
4. **Sensor Perception Integration:** Add simulated depth cameras / LiDAR for cargo wall detection and bag pose estimation.
5. **Dynamic Load Validation:** Test real-time track contact reactions with 20 kg and 32 kg payloads under full dynamic acceleration.
