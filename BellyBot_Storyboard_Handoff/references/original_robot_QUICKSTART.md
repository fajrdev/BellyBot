# Ubuntu / ROS 2 / Gazebo handoff

Target stack: Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic. Inspect the computer first. Follow current official ROS installation instructions if ROS is not installed; do not apply Jazzy packages to another Ubuntu release without checking compatibility.

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
https://gazebosim.org/docs/harmonic/ros_installation/

After the ROS repository is configured:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-joint-state-publisher-gui python3-colcon-common-extensions python3-rosdep
source /opt/ros/jazzy/setup.bash
# Initialize rosdep only if it is not already initialized.
rosdep update
rosdep install --from-paths ros2 --ignore-src -r -y
bash scripts/build.sh
bash scripts/run.sh rviz
```

Open only one package revision/workspace overlay at a time. This revision uses the same ROS package name as v1/v2. Do not build multiple revisions in the same workspace.

```bash
bash scripts/run.sh demo    # Unloaded joint-motion preview in RViz
bash scripts/run.sh bench   # Robot + independently supported infeed in Gazebo
bash scripts/run.sh floor   # A free 32 kg case placed directly on the floor
bash scripts/run.sh batch   # One staged bag and a rigid receiving fixture
bash scripts/run.sh cargo   # Illustrative curved hold, not a specific aircraft
```

The generic viewer files use explicit joints and relative mesh paths. Preserve the extracted directory structure. The display URDF includes a fixed relative conveyor mount only for convenient viewing. Gazebo has a separate conveyor with a world anchor.

## New six-value command interface

`/bellybot/pose_target` is `std_msgs/msg/Float64MultiArray`:

`[total_lift_m, yaw_rad, total_extension_m, infeed_extension_m, tray_pitch_rad, infeed_lift_m]`

This replaces the previous four-value interface. All three lift joints receive total lift / 3. Both extension joints receive total extension / 2. The new pitch joint is a rear +Y axis: positive angle slopes the nose down.

Source ROS and `ros2_ws/install/setup.bash` in every terminal. With physics feedback fresh and the base stopped, request the floor approach:

```bash
ros2 topic pub --once /bellybot/pose_target std_msgs/msg/Float64MultiArray '{data: [0.0, 0.0, 0.84, 0.4, 0.5396, 0.0]}'
```

Use the exact angle in `docs/poses.json` for a 2 mm nominal clearance. The rounded-down 0.5396 rad command gives slightly more clearance. The planner extends with the tray level first, then pitches. After it settles:

```bash
ros2 topic pub --once /bellybot/pose_target std_msgs/msg/Float64MultiArray '{data: [0.0, 0.0, 1.0, 0.4, 0.5396, 0.0]}'
```

Test acquisition by running the tray belt **toward the robot**, beginning at low speed:

```bash
ros2 topic pub -r 10 /bellybot/belt_cmd std_msgs/msg/Float64 '{data: -0.10}'
```

Verify the physical belt direction in the installed backend. This model reverses TrackController orientation for top-contact conveyor motion; support depends on the physics backend. Stop belt requests before changing a pose. Returning to home causes the planner to level the tray before retracting:

```bash
ros2 topic pub --once /bellybot/belt_cmd std_msgs/msg/Float64 '{data: 0.0}'
ros2 topic pub --once /bellybot/pose_target std_msgs/msg/Float64MultiArray '{data: [0.0, 0.0, 0.0, 0.4, 0.0, 0.0]}'
```

The planner does not know whether a bag has been successfully acquired. Use measured support/pose/force observations before issuing a loaded movement. There is no artificial bag attachment.

## Sideways yaw

The tray swings left/right about `yaw_joint`; it does not translate sideways. V3 has a bolted rotary bearing mount, continuous cranked frame and retained horizontal guides. The roller joints spin only the belt drums. `tray_pitch_joint` slopes the whole cassette down.

Before yaw, the planner levels the cassette, retracts the infeed to at most 150 mm stroke, raises the mast at least 80 mm and retracts both horizontal stages. A yawed target must request infeed stroke at most 0.15 m. Example:

```bash
ros2 topic pub --once /bellybot/pose_target std_msgs/msg/Float64MultiArray '{data: [0.1, 1.134464, 0.5, 0.15, 0.0, 0.0]}'
```

Keep pitch at zero during yaw. Independent viewer sliders bypass these multi-joint rules; limits alone do not prevent all collisions. Gazebo collision geometry is simplified and the prismatic/revolute joints impose the bearing constraints; the guide clearances are not a simulation of bearing compliance.

## Layer batching

Use the batch world for a single free-bag contact test. It has a rigid receiving fixture starting at x = 0.780 m with a top at z = 0.555 m and a width of 0.550 m, and one bag on the home infeed. The 40 mm belt-to-receiver drop must be included in contact/fragility testing. It does not automatically provide a stream of bags.

Prepare the robot and infeed at the same height; allow the free bag to settle and verify its staging position before starting:

```bash
ros2 topic pub --once /bellybot/pose_target std_msgs/msg/Float64MultiArray '{data: [0.1, 0.0, 0.0, 0.4, 0.0, 0.1]}'
```

For commissioning, the supervisor's `speed_scale` defaults to 0.25. The analytical fast-cycle budget uses 1.0. Raise speeds only after checking unloaded operation, loaded support and measured forces.

For a level deposit test:

```bash
ros2 param set /dock_stack_supervisor retract_belt_sync true
ros2 param set /dock_stack_supervisor deposit_belt_ratio 1.78125
```

This moves the belt 1.425 m during a 0.8 m tray retraction. It deliberately advances the bag relative to the floor as well as compensating tray withdrawal. The original pure-compensation ratio is 1.0. Do not enable the 1.78125 ratio for retrieval or a floor-pitched tray. Changing the ratio changes the target placement distance.

`batch_cycle.py` is a test state machine that requires `/bellybot/bag_ready`, `/bellybot/bag_supported` and `/bellybot/bag_placed` Bool decisions from the team's sensing/validation nodes. It also waits for the supervisor's motion-active acknowledgement and per-joint feedback before issuing the next move. It does not synthesize these decisions. With layer alignment and the supervisor prepared:

```bash
ros2 run bellybot_dock_stack_description batch_cycle.py --ros-args -p use_sim_time:=true -p speed_scale:=0.25 -p log_path:=/tmp/bellybot_batch_results.csv
```

Keep its speed_scale equal to the supervisor's. The node requests receive → support confirmation → extend → synchronized retract → placement confirmation. It records simulation timestamps and completed-bag counts. Missing support/placement confirmation stops the test; it does not count a timed animation as a successful bag transfer. The 4.66 s planning value is not the measured result of this node.

## Track driving and wheel motion

`/bellybot/cmd_vel` accepts `geometry_msgs/msg/Twist`. Drive only in a clear bench with the mechanism stowed. Positive X is forward; differential left/right track velocities turn the base. TrackController supplies traction. Each side's five wheel joints rotate at track-command speed / 0.068 m. They are not steering joints, and spinning their meshes alone does not propel the model. Real slip and odometry must be observed in Gazebo.

The software stop is `/bellybot/stop` (`std_msgs/msg/Bool`). Input watchdogs stop stale requests while the supervisor is running. These are not a safety-rated stop, independent process-failure watchdog or physical holding brake.

## Checks to run before performance claims

1. Launch/reset; check mesh paths, plugins, positive inertias, gravity and one publisher per TF relationship.
2. Exercise pitch, lift, extension and infeed height independently at low speed; then the provided guarded sequence.
3. Confirm free 20/32 kg floor pickup, deposition and retrieval, with varied friction and bag shape. Record nose and baggage contact forces.
4. Verify nose clearance under deflection and floor variation. Add compliant nose/force-feedback handling before real-world use.
5. Run the actual fixed-layer batch with sensing flags; log completed transfers, failures, retries and all timing.
6. Include floor operations, layer changes and repositioning in a complete mission test. The slow floor route cannot be omitted from a mission-wide 12/min claim.
7. Check local floor reactions and actual contact patches, particularly at 1 m reach. Review any failed screening cases in validation.json.

Gazebo and RViz are not installed in the authoring environment; no runtime success is claimed. SDF XML/reference checks are not a full runtime test. Rigid-body physics is insufficient to establish soft-bag/fragility performance.

## Regenerate from source

Use a separate CAD environment:

```bash
python3 -m venv .cad-venv
source .cad-venv/bin/activate
pip install -r cad/requirements.txt
python cad/regenerate.py
```

`python cad/regenerate.py --no-video` skips MP4 regeneration. Individual check scripts remain available in `cad/`; `python cad/render.py --video-only` regenerates only the unloaded kinematic MP4. Its duration/playback does not establish baggage throughput. Keep model.py, parameters.json, control_core.py and exporter constants consistent when changing mechanical dimensions or control limits.
