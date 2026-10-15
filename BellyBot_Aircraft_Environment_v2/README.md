# BellyBot aircraft environment — revision 2

This revision resolves the **geometric door-entry route** and replaces the disconnected infeed with a **connected, articulated corner-transfer station**. BellyBot's original size, track contact area and robot mechanism are preserved. Aircraft dimensions are unchanged.

- Entry: empty, stowed robot drives onto a 180 mm raised turn deck, turns 90°, and descends a 1.8 m ramp. Checked nominal jamb clearance is 60 mm per side; minimum triangle-mesh turning clearance is 32.77 mm. The external platform is already docked at cargo-floor height +180 mm.
- Transfer: a 300 mm-high horizontal door belt, lifting rotary belt table, folding bridge and height-matched longitudinal buffer feed the original robot tray. This replaces the original separate v3 infeed in the new worlds.
- Limits: a supplied ROS controller rejects roof, wall, joint and assigned-mass pressure-screen violations. It locks the chassis during work and requires measured feedback to sequence the transfer. These limits are active when using `aircraft.launch.py`.

**Evidence boundary:** CAD, mesh sweeps, local resource checks and pure controller checks were run. Gazebo, RViz, contact dynamics, structural ratings and sustained 12 bags/min were not run or demonstrated here. The delivered controller uses slow commissioning speeds. Do not label the geometric preview as a successful physical loading trial.

Read `docs/AGENT_HANDOFF.md` first. `docs/Resolution_Review_R2.pdf` explains the mechanisms, limits and evidence in six pages. CAD and colored GLB assemblies are in `exports/`. Start with `BellyBot_Aircraft_Work_Assembly_R2_mm.step` for the complete work layout. `Entry_and_Transfer_Kinematic_Preview.mp4` is a labelled 30-second geometric illustration. All sources and local meshes are included. The unchanged robot CAD source is under `vendor/bellybot_v3/cad/`; the new mechanism source is `src/build_resolution.py`.

## Run on ROS 2 Jazzy / Gazebo Harmonic

Extract the ZIP, then place the two directories in `ros2/` under a colcon workspace's `src/` directory. Install the declared dependencies using rosdep, build, and source the workspace.

```bash
colcon build --symlink-install --packages-select bellybot_dock_stack_description bellybot_aircraft_environment
source install/setup.bash
ros2 launch bellybot_aircraft_environment aircraft.launch.py mode:=entry
```

The world starts paused. Unpause, wait for fresh feedback, then:

```bash
ros2 topic echo /aircraft/status
ros2 topic pub --once /aircraft/start_entry std_msgs/msg/Empty '{}'
```

Entry ends at X=4.90 m, with the robot completely clear of the temporary ramp. **Entry and baggage handling are separate fixture setups:** remove the temporary dock/deck/ramp before installing the conveyor station. This setup/changeover is not an automated mechanism or a demonstrated in-simulation process. The transfer world starts the unchanged robot at X=4.10 m on the floor and contains no entry deck.

```bash
ros2 launch bellybot_aircraft_environment aircraft.launch.py mode:=transfer
# In another terminal, from this extracted folder, after unpausing:
python3 tools/spawn_transfer_bag.py --execute
ros2 topic pub --once /aircraft/start_transfer std_msgs/msg/Empty '{}'
```

The free bag is named `tracked_bag`; the pose publisher is added by the spawn tool. The commissioning controller uses that bag's measured simulator pose. It does not attach, teleport or script the bag's motion. Vision-based detection and arbitrary bag selection remain integration work. Stop and remove that bag before starting a new independent trial.

Live RViz work-mode inspection:

```bash
ros2 launch bellybot_aircraft_environment aircraft_display.launch.py
```

Once the transfer is `DONE`, the buffer is lowered and the bridge is folded, a conservative example arm command is:

```bash
# [total lift m, upper yaw rad, total reach m, tray pitch rad]
ros2 topic pub --once /aircraft/pose_target std_msgs/msg/Float64MultiArray '{data: [0.13, 0.8, 0.4, 0.0]}'
```

`/aircraft/stop` (`std_msgs/Bool`, `data: true`) latches a stop; investigate and restart the controller to clear it. Never launch the original v3 supervisor alongside the aircraft controller: both would command the same robot joints. Original launch files are retained only for the independent robot package.

## What “operating limits” means

The robot fits stowed: 1.10 m track width on the modeled 1.22 m floor, and about 0.538 m overall height including the spawn gap. Its **full 0.60 m mast stroke is too tall** in the 1.12 m hold. Its chassis must stay lengthwise on the normal floor. Useful upper-joint motion remains available.

| Bag height | Maximum total mast lift | Roof clearance at that cap |
|---|---:|---:|
| 380 mm or unknown up to 380 mm | 130 mm | about 110 mm |
| 320 mm | 190 mm | about 110 mm |
| 280 mm | 230 mm | about 110 mm |

These are total lift, divided across the three mast joints. The default assumes an unknown 380 mm-high, 32 kg bag. For a measured shorter bag, pass e.g. `bag_height:=0.28 bag_mass:=32.0` to `aircraft.launch.py`; height is supplied, not inferred by a vision system. Receiving from the new conveyor station is limited to 130 mm lift for all bags. After receipt, a shorter bag can use a higher checked placement pose. Upper yaw is limited to ±65° in this commissioning profile, with at least 80 mm lift and at most 500 mm reach. Straight, level reach can extend to 1 m when the full pose and pressure screen pass. Mechanical joint limits remain in the robot URDF/SDF; the controller imposes the tighter environment-dependent limits.

The entry deck allows a chassis turn only during empty, fully stowed entry. It is not permission to turn the chassis anywhere in the hold. The aft-taper legacy world is outside this controller's approved work profile.

## Rebuild and evidence

Install `requirements-build.txt` in a virtual environment. The artifact contains the bundled original robot package, so no missing external ZIP is needed.

```bash
python3 src/build_environment.py
python3 src/build_resolution.py
python3 src/build_guard_data.py
python3 src/build_integration.py
python3 src/build_display.py
python3 tools/check_resolution.py
python3 tools/check_controller_contracts.py
python3 tools/validate_package.py
python3 src/render_resolution.py
python3 src/make_resolution_review.py
```

The enclosure profile between published Boeing dimensions, doorway pocket, and fixture details are assumptions. This is a reference-based hackathon model, not an as-built aircraft survey or manufacturing approval. Changing the geometry requires rechecking the paths and adapting the controller profile. Older worlds remain as reference/stress tests; `entry_resolved`, `transfer_resolved`, and `transfer_cutaway` are the revision 2 scenarios.
