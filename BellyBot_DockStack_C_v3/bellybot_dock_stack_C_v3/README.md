# BELLYBOT — Dock & Stack, revision 3

Corrected CAD for the sketch-derived robot, with STEP, editable CadQuery source, URDF meshes and a ROS 2 / Gazebo handoff.

## The highlighted joint is corrected

The tray can **swing left/right about a vertical yaw bearing**. This is rotation, not sideways translation. In revision 2 the yaw disk and rail frame were grouped as one rigid link despite gaps between the solids. V3 replaces that connection with a bolted bearing mount, a continuous cranked frame and crossmembers that physically meet the rails.

- **Yaw mount:** stationary mast plate, separate inner/outer bearing envelopes, 12 mounting screws per ring, rotating flange and four connected support ribs. The mounting envelope follows THK RU178X dimensions; raceways, threads and drive internals are simplified.
- **Retained extension:** two dovetail/gib guide interfaces, four captive blocks per stage, end stops and connected rear cross ties. Each stage travels 500 mm; both 54 mm blocks remain engaged throughout the stroke.
- **Tray hinge:** driven and idler stub shafts, hinge ears and a connected motor mounting bridge. The side guides now meet their supporting rails.
- **Clearance:** the tray mechanism and infeed belt rise 20 mm. Home belt height is now 495 mm. Floor access is retained at 30.922° downward pitch, with a 2 mm nominal nose clearance.
- **Yaw sequence:** level the tray, retract the infeed to at most 150 mm stroke, raise at least 80 mm, retract the horizontal stages, then turn. This is included in the supervisor. Independent viewer sliders bypass these rules.

The tray roller joints only spin the conveyor drums. `yaw_joint` swings the whole upper assembly; `tray_pitch_joint` tilts the cassette. The two extension joints move it forward/back.

## Start here

| File | Purpose |
|---|---|
| `exports/BellyBot_DockStack_yaw_mechanism.step` | Corrected bearing, frame and slide assembly with tray removed for inspection |
| `exports/BellyBot_DockStack_home.step` | Robot and infeed in receiving pose |
| `exports/BellyBot_DockStack_placement.step` | Raised 65° sketch-like pose with example bag |
| `exports/BellyBot_DockStack_floor_access.step` | Floor-access pose |
| `exports/BellyBot_DockStack_robot.step` | Mobile robot alone |
| `ros2/bellybot_dock_stack_description/urdf/display_portable.urdf` | Relative mesh paths for generic viewers; robot and infeed |
| `ros2/bellybot_dock_stack_description/urdf/bellybot_portable.urdf` | Mobile robot alone |
| `docs/BellyBot_DockStack_v3_overview.png` | Preview rendered from the actual CAD |
| `docs/BellyBot_DockStack_v3_motion.mp4` | 44-second unloaded joint-motion demonstration |
| `docs/BellyBot_DockStack_v3_report.pdf` | Seven-page hackathon concept report draft |
| `docs/QUICKSTART.md` | Build, RViz, Gazebo and guarded motion commands |
| `docs/VALIDATION.md` | Results and scope of the offline checks |

STEP exports are in millimetres; URDF joint values are in metres/radians. Keep the extracted package structure intact. STEP contains solids; articulation is defined by the URDF and source joint tree.

## Joint limits

| Joint(s) | Travel | Provisional speed ceiling |
|---|---|---|
| Three lift joints | 0–200 mm each | 0.12 m/s each |
| `yaw_joint` | −95° to +95° | 1.0 rad/s |
| Two extension joints | 0–500 mm each | 0.70 m/s each |
| `tray_pitch_joint` | 0–32° downward | 0.60 rad/s |
| Infeed extension | 0–400 mm | 0.12 m/s |
| Infeed lift | 0–250 mm | 0.12 m/s |
| Ten road wheels | Continuous rotation | 10 rad/s |
| Four belt drums | Continuous rotation | 165 rad/s |

Explicit serial joints require no mimic linkage or artificial bag attachment. Travel limits still need the operating sequence: for floor mode, lift/yaw zero and reach at least 840 mm; the floor-height guard limits usable pitch below the mechanical maximum. Continuous wheel/drum joints intentionally have no angular end stop. Differential crawler speeds turn the base.

## Verification and handoff

The corrected rigid subassemblies pass a solid-level connectivity check. Guide engagement, exact CAD interference samples, independent URDF kinematics and control guards are recorded in `docs/`. Collision bodies in the simulators are simplified; joints supply the bearing constraints. This is not a bearing-contact or structural-strength simulation.

The two earlier improvements remain: geometric floor reach and a height-matched infeed for repeated layer transfers. The prepared straight-ahead batch is analytically estimated at 4.6566 s, or 12.89 bags/min before extra delays. Floor pickup and yaw routes are slower; sustained hackathon throughput is unverified.

ROS 2, Gazebo and RViz were not run here. Final actuator/linear-drive selection, rail fastening details, brake sizing, deflection, free-bag acquisition, fragile luggage and actual aircraft-floor loading require engineering and tests. The seven-page report and 44-second video meet the requested document-length formats; complete the team details and prescribed submission filenames before submitting. No submission has been made.
