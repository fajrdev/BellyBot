# BellyBot replacement storyboard — implementation handoff

Purpose: replace the video plan, not rebuild BellyBot. Read `HANDOFF_PROMPT.md` first, then the audit, removal plan and communications contract. Seven scene briefs specify motion, Gazebo work, ROS 2 messages, recording and acceptance gates.

This archive is an instruction/evidence package, not a commissioned ROS workspace, edited CAD release or completed video. No ROS/Gazebo runtime was available here. Existing uploaded files were inspected as text/XML; their historical validation reports are not newly verified results. Original archives remain unchanged.

## Read order
1. `HANDOFF_PROMPT.md` — paste into your Antigravity agent.
2. `technical/01_SOURCE_AUDIT.md` — actual assets and implementation hazards.
3. `technical/02_CONVEYOR_REMOVAL_AND_SUPPLY.md` — identify marked objects, remove dependencies, close supply path.
4. `technical/03_ROS2_GAZEBO_CONTRACT.md` — existing interfaces versus proposed additions.
5. `storyboard/01_*.md` through `07_*.md`.
6. `technical/04_VALIDATION_AND_MEASUREMENT.md` and `05_IMPLEMENTATION_ORDER.md`.

## Revised operating concept
Remove the two marked interior transfer assemblies, provisionally `corner_transfer` and `longitudinal_buffer`, after a camera-matched ID check. Preserve the exterior `belt_loader`, unmarked doorway `door_belt`, complete robot including its tray belt, aircraft and entry equipment. The drawing is not an object-ID map: verify the left marked assembly against `door_belt` before any deletion.

Preferred minimal concept is fixed door-side supply plus robot docking/shuttling, NOT a magical feeder following the robot. Meter bags onto the retained door belt, stop supply while the robot is absent, receive only at a verified dock, transport a supported bag to a placement station, deposit, and return empty. Unloading reverses the logistics path but requires its own support and removal-order checks. This is a proposed software/fixture adaptation with real feasibility gates, not an existing capability.

Critical gate: the old door belt is at Z=0.300 m and robot receiving surface is nominally Z=0.498 m at zero mast lift. Simply moving the robot next to it does not close the 198 mm mismatch. Re-height/reposition the retained doorway belt and adjust the external loader transition, or stop and present an alternative. Do not add another long interior conveyor or silently restore the deleted pair. Preserve the original installation for regression tests.

## Film timeline (120 seconds maximum)
| File | Screen interval | Content |
|---|---|---|
| 01 | 00:00–00:08 | Whole operation and supply path |
| 02 | 00:08–00:23 | Entry, disclosed setup/changeover and docking |
| 03 | 00:23–00:53 | Uninterrupted measured loading interval |
| 04 | 00:53–01:13 | Floor/upper-layer reach and resupply during repositioning |
| 05 | 01:13–01:33 | Retrieval, return transfer and withdrawal |
| 06 | 01:33–01:48 | Implemented stop and controlled restart |
| 07 | 01:48–02:00 | Evidence, limitations and honest results |

Screen durations are editorial budgets, not motion deadlines. Slow entry, changeover or recovery may use disclosed acceleration/cuts. Scene 03 is one continuous measured simulation interval; default 30 s at 1×. Six completed placements is a target, not a mandated outcome. Use actual N and elapsed simulation time. Report fixed-station and shuttle/mission rates separately.

## Sources
Supplied aircraft archive [file:1], robot archive [file:2], marked/unmarked reference images [file:3][file:4], and hackathon guidelines [file:5]. The guidelines specify height <1.2 m, baggage 20–32 kg, ≥12 bags/min, floor-load limit ≤600 kg/m² and MP4 ≤2 minutes. The seven-scene plan and proposed interfaces in this handoff are design instructions, not statements from the guidelines.
