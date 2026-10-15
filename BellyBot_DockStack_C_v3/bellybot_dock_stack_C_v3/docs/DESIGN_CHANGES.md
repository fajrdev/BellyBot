# Revision 3: connected yaw mechanism

## What was wrong

The old yaw disk, side rail frame and central support were assigned to the same rigid CAD/URDF group, but the solids did not all meet. A rigid-link animation concealed the missing physical connection. The correction establishes a continuous geometric load path and captured sliding interfaces.

## Corrected load path

**Bag → belt cassette/pan → hinge ears and shafts → carriage runners → retained blocks → middle guide beams → retained blocks → fixed rails → mounting pedestals/crossbeams → cranked ribs → rotating flange → bearing → mast mount plate → telescopic column → chassis/tracks.**

| Assembly | Geometry in V3 |
|---|---|
| Mast bearing seat | 330 × 330 × 12 mm plate, central output-shaft bore and outer-ring bolt pattern |
| Rotary bearing envelope | 115 mm bore, 240 mm outside diameter, 28 mm width; 139 / 217 mm inner / outer bolt-circle diameters |
| Ring fasteners | 12 M8 nominal positions per ring; inner tapped-hole envelopes and outer counterbored clearances |
| Rotating frame | 380 mm diameter × 10 mm carrier flange, four 16 mm-wide cranked ribs, two 50 × 20 mm lower crossbeams and four rail pedestals |
| Fixed / middle / tip beams | Hollow rectangular sections 60 / 54 / 48 mm deep |
| Captured sliding guides | Dedicated 830 mm guide profiles, two 54 mm blocks per side per stage, 160 mm block spacing and end stops |
| Rear cross ties | Separate middle and carriage ties placed behind the swept inner runners and tilted cassette |
| Pitch connection | Two hinge ears, driven/idler stub shafts, bearing housings and a motor support bridge |

The bearing mounting dimensions follow the THK RU178X catalog envelope. This is not supplier production CAD: raceways, rolling elements, preload and threads are simplified. The motor and reduction drive are packaging envelopes. The dovetail/gib guides are custom concept geometry with nominal 0.2 mm profile clearance, not selected/rated catalog linear bearings. Joining faces and bearing mounts now exist; detailed welds/fasteners, lubrication, wear adjustment, linear drives, brakes and tolerances remain engineering work.

The solid-connectivity checker treats each separate solid as a graph node. Merely grouping disconnected pieces in a compound cannot pass. It checks yaw carrier, middle slide, tip carriage and tray separately; decorative painted stripes are excluded. Contacting geometry does not establish weld strength, bolt preload, fatigue or stiffness.

## Motion and joint meaning

- Roller joints rotate conveyor drums continuously. The belt contact surface is driven separately in Gazebo.
- Yaw rotates the complete upper frame about the vertical axis, limited to ±95°. It does not provide lateral translation.
- Both horizontal slides translate forward/back independently by up to 500 mm. Captive blocks retain them vertically and laterally in the proposed mechanism.
- The rear pitch hinge rotates the cassette downward by up to 32°. Its driven shaft, idler shaft and ears rotate with the cassette.
- The base turns through differential crawler speeds; the ten wheel meshes rotate about their axle axes.

A real bearing constrains unwanted degrees of freedom. “No mimic dependency” means explicit serial joints in the URDF; it does not mean removing the bearing constraints needed for a functioning mechanism.

## Clearance changes

The deck rises 20 mm to give the continuous frame and moving cross ties space. Home belt height is 495 mm, and the pitch axis is 503.5 mm above the nominal floor. The infeed moves up by the same amount. At 30.922° pitch and zero lift/yaw, the nose underside is 2 mm above the floor. Its forward position is approximately 1.182 m at 840 mm reach and 1.342 m at 1,000 mm reach.

Floor mode extends at least 840 mm before pitching; pitch stays unchanged during the last 160 mm insertion/withdrawal. Level the tray before retracting over the chassis. A 2 mm nominal gap needs compliance/height sensing once floor variation and structural sag are included.

The new lower crossbeams can sweep through an extended infeed during yaw. The guarded sequence therefore retracts the infeed to at most 150 mm stroke, levels and retracts the tray, and raises at least 80 mm before turning. Yawed target commands require the infeed to remain retracted. Direct viewer sliders bypass these multi-joint rules.

## Faster layer transfer remains conditional

The infeed has 250 mm vertical adjustment. At a prepared layer, robot and infeed remain at the same height and yaw stays zero, avoiding a mast/yaw round trip for each bag. The analytical batch budget remains 4.6566 s, or 12.89 bags/min before additional delays. It assumes one staged rigid bag, sufficient traction, 0.8 m extension, 1.425 m belt advance during withdrawal and only 0.20 s verification/command allowance. More than 93.132% productive time is needed to exceed 12/min under that exact model.

The 550 mm-wide receiving fixture begins at x = 780 mm and has a 555 mm top. At 100 mm lift the belt is 595 mm high, leaving a 40 mm drop and 15 mm nominal cassette-underside clearance. Clear channels beside the bag column are required. Bag slip, deformation, fragile contents and settling are not solved by a kinematic timing model.

With the new infeed retraction sequence, the example 65° yaw round trip takes about 16.81 s before a separate deposit allowance. The floor mechanism approach/return takes about 6.73 s before bag transfer. Neither is a 12-bags/min mode. Gazebo belt acceleration limits now match the stated 3 m/s² analytical ceiling; this is a command ceiling, not demonstrated drive capability.

## Loads and remaining requirements

Assigned mobile mass is 173 kg, or 205 kg with the rigid 32 kg bag. These are mass budgets and uniform-density CAD inertias, not a material-resolved or measured bill of materials. Track contact is 1,320 × 350 mm per side; the four-patch floor screen and actual computed results are in `validation.json`. It assumes uniform pressure within each half-track patch, four contacts of equal compliance and a 1.5 multiplier. It does not qualify an aircraft floor.

At the new floor angle, a 32 kg bag needs about 161 N uphill gravity traction and a friction coefficient of at least 0.599 at constant speed, before insertion losses or acceleration. The bag alone gives approximately 135 N·m about a hinge 430 mm from its centre. The provisional 450 N·m pitch effort is not a selected or verified drive. Structural bending, bearing moment capacity, mounting stiffness, actuator torque and braking remain unverified.

The source sketch's teal body, central mast, circular yaw stage, thin belt cassette and five-wheel-per-side tracks remain. The hackathon height, payload, throughput and floor-load requirements are tracked in the report with their verification status. Offline geometric corrections are complete; free-bag handling and sustained performance require runtime and hardware tests.

## Primary references

- THK Cross Roller Ring RU catalog and mounting dimensions: https://www.thk.com/jp/en/products/cross_roller_ring/cross_roller_ring/ru/
- THK RU dimensional specification: https://tech.thk.com/en/products/thkdlink.php?id=4697
- ROS / Gazebo installation: https://gazebosim.org/docs/harmonic/ros_installation/
- TrackController: https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1TrackController.html
- JointPositionController: https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointPositionController.html
- User-supplied hackathon guideline PDF: `reference/Hackathon_Guidelines_Oct_2.pdf`.
