# Offline verification — C-DockStack-v3.0

## Corrected support structure

| Rigid subassembly | Separate solids checked | Connected components |
|---|---:|---:|
| yaw_carrier | 42 | 1 |
| extension_mid | 13 | 1 |
| tray_carriage | 13 | 1 |
| tray | 18 | 1 |

Each separate solid is a node; actual contact within 0.005 mm joins nodes. Compound membership does not count as a connection. Decorative painted tray stripes are excluded. All four corrected groups form one connected component each. This proves geometric continuity only, not joint strength, preload, weld quality or stiffness. The bearing internals remain a mounting envelope.

Both 54 mm captive blocks on each side remain on their 830 mm guide at 0, 125, 250, 375 and 500 mm stage travel. The least guide-end margin is 8 mm. Linear paths make these endpoint inequalities valid throughout the stroke; wear, deflection and bearing capacity are not tested.

## CAD, URDF and controls

- 202 CAD part groups / 511 solids passed shape validity checks.
- Exported home STEP reimported successfully with 495 valid solids; the free example bag is omitted from that assembly.
- Five articulated URDF variants passed independent forward-kinematics comparisons at 40 random configurations each. Mesh paths, millimetre-to-metre scale and positive physical inertia tensors were checked.
- 30 named CAD configurations showed no cross-link solid intersection above 0.25 mm³. They include independent lift/extension positions, floor pitch, low yaw sweeps with the infeed retracted and maximum infeed height. See `validation.json` for exact poses/results.
- 2641 samples of the unloaded 44-second demonstration obey individual joint/velocity limits. Lowest nose height is 2.000 mm and maximum robot height on that demonstration is 635.0 mm. This demonstration does not use the full lift stroke.
- The separate full-stroke unloaded CAD pose is 1135.0 mm tall, below the 1,200 mm height target. Loaded and curved-roof clearances depend on payload and location.
- 17 control checks passed, including the new infeed-before-yaw guard, floor pitch sequence, invalid-pose rejection and differential wheel speeds.
- The 555 mm receiving fixture clears the CAD at 0, 400 and 800 mm reach with 100 mm lift and zero yaw. Cassette underside clearance is 15 mm; the belt-to-fixture drop is 40 mm.

These are sampled interference checks, not a continuous collision proof. Independent viewer sliders and direct joint topics can bypass the supervisor's multi-joint rules. Collision primitives omit small bearing/guide details; simulator joints impose the ideal constraints.

## Recalculated floor-load screen

Assigned robot mass is 173 kg; the rigid bag is 32 kg. Contact is 1,320 × 350 mm per track. The screen assumes four equal-compliance half-track patches with uniform pressure and a 1.5 multiplier.

| Named pose | Factored peak kg/m² | Against 600 kg/m² |
|---|---:|---|
| home | 342.98 | Within screen |
| placement | 490.40 | Within screen |
| floor_insert | 594.31 | Within screen |
| batch_reach | 544.57 | Within screen |
| full_reach | 470.64 | Within screen |

Peak is 594.31 kg/m², leaving only 5.69 kg/m² in this approximation. Mass and load distribution uncertainty can consume that margin. Actual local wheel/track pressures, floor flexibility, acceleration, bag offsets and contact loss require measurement. No aircraft-floor compliance claim is made.

## Timing and runtime boundary

The prepared layer-batch estimate remains 4.6566 seconds, or 12.89 bags/min before additional delay. The revised yaw route is about 16.81 seconds before a separate deposit allowance; floor mechanism approach/return is about 6.73 seconds before bag transfer. These are analytical motion budgets.

Gazebo, ROS 2 and RViz were not run in the authoring environment. SDF XML references and plugin targets were checked; the SDFormat schema/backend, tracking, contact, conveyor direction and free-bag behavior require runtime commissioning. Structural capacity, final drives/brakes, free-bag pickup, fragile/soft baggage and sustained throughput remain unverified.

Machine-readable evidence: `structure_checks.json`, `validation.json`, `control_checks.json`, `batch_fixture_checks.json`, `timing.json` and `motion_profile.json`. The source checks are in `cad/`.
