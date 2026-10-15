# Conveyor removal and replacement supply logistics

## Identification gate — no blind deletion
The marked image appears to identify the short interior corner-transfer assembly and long horizontal buffer between the door and robot. Expected model IDs: `corner_transfer` and `longitudinal_buffer`. This mapping remains provisional until camera-matched overlay. The door_belt is a separate, collinear doorway element and may overlap the left red mark in projection. If the left marked object proves to be door_belt, stop and ask; the retained-door-belt shuttle concept then needs revision.

Render original scene and label these IDs with contrasting colors. Compare endpoints, supports, yellow rails and relative orientation with both supplied images. Save confirmed mapping to removal_map.json with original world path and model/link lists. Never identify an object by mesh color alone.

## Reversible removal in a mission variant
1. Keep original transfer/entry worlds and launch files available for regression. Add `worlds/mission_shuttle.sdf` and `launch/mission.launch.py` (new proposed names).
2. Remove `<include>` entries with model://corner_transfer and model://longitudinal_buffer from the new world. Handle embedded copies only by confirmed model scope, not global string replacement. Remove associated bridge/nose with the buffer.
3. Remove corresponding URDF link/joint subtrees from a new mission display assembly. Preserve robot tray, rear/front rollers, mast, yaw, tracks and external loader. Remove a joint if its child was removed; validate one parent per link, no dangling references and no duplicate names.
4. New bridge config omits /aircraft/corner_joints and /aircraft/buffer_joints plus their merged /joint_states entries, stage/angle/bridge/buffer/corner commands. Retain required robot/loader/door commands. New controller must not wait for absent station joints or issue station commands.
5. Keep legacy aircraft_controller and transfer_core for original scenes. New controller imports/reuses pure robot logic where appropriate but must not inherit transfer-mode station checks or datum lock.
6. Update generators: src/build_resolution.py has corner(), buffer(), door_belt(), worlds(), step_export() and main(); src/build_display.py explicitly merges all three; src/build_integration.py regenerates station bridges. Add a variant parameter or separate variant generator so regeneration cannot resurrect deleted geometry. Do not simply edit generated SDF and then regenerate.
7. Regenerate mission-only STEP/GLB, screenshots and CAD assembly bill of materials after physical design is chosen. Old Corner_Transfer_R2 and Work_Assembly_R2 exports are historical, not final mission CAD. No full CAD rebuild of unchanged robot parts is required.
8. Check GUI scene tree AND collisions for absent assemblies. Search names in active world/URDF/bridge/controller and generator branches. Legacy references are allowed outside active mission. Verify launch reset and installed-share paths, not just source files.

## Preferred architecture: fixed doorway supply + mobile shuttle
Supply chain: apron bag source → retained external loader → retained doorway belt → robot receiving tray at verified dock → supported low-speed travel → placement. Robot returns empty. No following conveyor and no claim of continuously feeding a robot while it travels. The supply CONTINUES as a managed queue, pausing upstream while the dock is unoccupied.

### Height and docking are hard gates
Old door belt: center X=1.30, Y=-1.55, points +Y, surface Z=.300; inner end Y≈-.512. Nominal robot tray at zero mast: .498 m world surface at old datum. Gap=.198 m BEFORE any docking changes. A dock pose is not supplied because no collision-checked solution has been established.

Preferred investigation: re-height/reposition retained door_belt so discharge matches the measured robot rear-entry surface at the dock; recompute exterior-loader slope, endpoint, supports and floor reactions. Maintain doorway/roof envelope and prevent interference with chassis, trays and entry platform. This modifies a retained fixture, not a new interior conveyor. Avoid a bare 198 mm ramp that assumes bags climb without powered contact. If this cannot work, present the smallest feasible alternative and its deployment route; do not hide the gap or replace robot geometry without approval.

At initial dock investigation the robot could face +Y with its rear tray entrance toward the -Y supply; this is only a direction hypothesis. Its chassis may collide with the doorway belt or curved wall. Solve full mesh transforms, not just tray-center alignment. A 90° loaded base turn is not the same as guarded upper yaw; validate all swept envelopes and stability.

### Dock transaction
WAIT_EMPTY_DOCK → reserve one bag ID → gate upstream → align and stop base → confirm pose, support geometry and height → match belt speeds → observe full transfer and supported bag → stop belts → release dock reservation. No feed if robot absent, moving, yawing, pitching, stale, obstructed or full. With one-bag buffer capacity, stop loader before a second bag can overlap; use measured spacing and stopping distance. Source creation occurs outside aircraft on a physically supported belt, never at the receiving tray.

### Transport transaction
Tray level, mechanism retracted to validated carrying pose, bag fully supported and settled, belts zero; release dock and follow verified path. No bag weld/teleport. On support loss or slip stop in a controlled fashion. Maintain a free return/egress corridor; fill far positions first only when accessible and stable. Every trip returns to the fixed dock; do not pretend the supply endpoint moves with the robot.

### Setup/withdrawal
Preserve empty entry on existing entry deck. Entry deck and reconfigured door supply may require manual changeover. Show or explicitly label that intervention; do not splice separate worlds as autonomous deployment. Specify fixture installation path, crew access/exclusion and support locations in CAD. After unloading clear bags from dock, stop belts, remove/withdraw retained supply by declared procedure, restore entry/exit configuration and validate empty reverse route. Existing start_entry does not implement exit. New reverse/exit logic is required before presenting withdrawal as simulated.

## Throughput consequences
A one-bag shuttle adds round-trip travel/docking and will likely underperform the prepared-layer analytical estimate. Measure it. Do not assume six deliveries/30 s is achievable. If it fails, keep an honest throughput scene and quantify bottlenecks. Optimization cannot mean hidden preloading, omitted return trips, disabled roof checks or an approved-looking but nonphysical transfer.

Sources for existing geometry/dependencies: [file:1][file:2]. Shuttle architecture and modifications above are proposed, uncommissioned work.
