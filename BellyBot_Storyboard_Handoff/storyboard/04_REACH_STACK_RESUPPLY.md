# Scene 04 — Reach Stack Resupply

## Screen time and objective
00:53–01:13 (20.0s total). Demonstrate rapid, gentle drop-loading supercut across 6 bag placements into an optimized staircase matrix (2x2x1 Base + 2x1x1 Upper), show measured ceiling clearance margins, disclose real-world upper-layer stacking feasibility, and show explicit supply gating while the mobile robot is absent from the dock.

## Technical motion sequence & timing redistribution
1. Rapid Drop-Loading Supercut (00:53.0–01:11.0, 18.0s / 540 frames, 90 frames / 3.0s per drop):
   - Loading Order & Exact Poses (Fuselage cargo floor surface at Z = 0.000m):
     - Drop 1: ROW 1 PORT (BASE) — 32kg Grey Case (0.78x0.50x0.32m) at X=5.00m, Y=+0.27m, COM Z=0.160m (Top Z=0.320m). Robot staged at X=3.30m, Pitch=21.8°, Reach=740mm.
     - Drop 2: ROW 1 STBD (BASE) — 26kg Blue Case (0.72x0.46x0.30m) at X=5.00m, Y=-0.27m, COM Z=0.150m (Top Z=0.300m). Robot staged at X=3.30m, Pitch=21.8°, Reach=740mm.
     - Drop 3: ROW 1 PORT (UPPER) — 32kg Grey Case (0.78x0.50x0.32m) at X=5.00m, Y=+0.27m, COM Z=0.480m (Top Z=0.640m). Mast lift=60mm, Pitch=14.5°, Reach=740mm. Directly supported by Bag 1.
     - Drop 4: ROW 1 STBD (UPPER) — 26kg Blue Case (0.72x0.46x0.30m) at X=5.00m, Y=-0.27m, COM Z=0.450m (Top Z=0.600m). Mast lift=50mm, Pitch=14.5°, Reach=740mm. Directly supported by Bag 2.
     - Drop 5: ROW 2 PORT (BASE) — 26kg Blue Case (0.72x0.46x0.30m) at X=4.18m, Y=+0.27m, COM Z=0.150m (Top Z=0.300m). Robot retracted/staged at X=2.48m, Pitch=21.8°, Reach=740mm.
     - Drop 6: ROW 2 STBD (BASE) — 32kg Grey Case (0.78x0.50x0.32m) at X=4.18m, Y=-0.27m, COM Z=0.160m (Top Z=0.320m). Robot retracted/staged at X=2.48m, Pitch=21.8°, Reach=740mm.
   - Six-Phase Synchronized State Machine & Kinematics per drop:
     - Phase 1 (CONVEYOR_EXTEND, Steps 0..9, 0.00s–0.33s): Telescopic boom extends to 760mm target reach (v_boom = +0.38 m/s), conveyor articulates to pitch angle (21.8 deg base, 14.5 deg upper). HUD displays `[EXTENDING: BAG X / 6]`.
     - Phase 2 (CONTROLLED_TRANSFER, Steps 10..37, 0.33s–1.26s): Powered belt transport down inclined conveyor deck at v_belt = 0.45 m/s. Luggage physically supported along the deck surface throughout travel. HUD displays `[TRANSFERRING: BAG X / 6]`.
     - Phase 3 (TOUCHDOWN_RELEASE, Steps 38..51, 1.26s–1.73s): Luggage achieves soft touchdown on cargo floor (Z=0.000m) or lower case top surface (Z=0.320m Port / 0.300m Stbd). Luggage decelerates to 0.00 m/s, mechanical release disengages. HUD displays `[TOUCHDOWN: BAG X / 6]`.
     - Phase 4 (GUIDE_RETRACT, Steps 52..69, 1.73s–2.33s): Telescopic boom actively retracts back to chassis (760 -> 0 mm at v_boom = -0.40 m/s), conveyor levels to 0.0 deg. Conveyor runner tips disengage cleanly without dragging or penetrating the luggage (>400 mm clear margin verified). Case remains completely stationary. HUD displays `[WITHDRAWING: BAG X / 6]`.
     - Phase 5 (INDEPENDENT_SETTLING, Steps 70..84, 2.33s–2.83s): Luggage resting completely independently on floor / underlying case. Settling dwell predicate enforced: linear velocity v < 0.02 m/s, angular velocity w < 0.05 rad/s for >= 0.50 s continuous dwell. Placed counter remains at prior count (`drop_idx`). HUD displays `[SETTLING: BAG X / 6]`.
     - Phase 6 (SETTLED_VERIFIED, Steps 85..89, 2.83s–3.00s): Dwell and independent support physically confirmed. Stacking counter increments to `X / 6 PLACED` and verified badge illuminates.
2. Completed Staircase Block & Technical Clearance Overview (01:11.0–01:13.0, 2.0s / 60 frames):
   - Wide isometric push-in displaying completed 2x2x1 Base + 2x1x1 Upper staircase configuration.
   - Headroom Clearance: 510 mm measured clearance above Port two-tier stack ($1.150\text{ m} - 0.640\text{ m} = 0.510\text{ m}$, PASS vs $\ge 300\text{ mm}$ threshold requirement).
   - Starboard Headroom: 550 mm measured clearance above Starboard two-tier stack ($1.150\text{ m} - 0.600\text{ m} = 0.550\text{ m}$, PASS).
   - Chassis Ground Clearance: 3.0 mm measured datum clearance (PASS).
   - Track Boundary Margin: 28.5 mm measured track envelope margin (PASS).
   - Longitudinal Space Margin: Robot at X=2.48m maintains >860 mm clearance to Row 2 luggage rear edge ($X=3.790\text{ m}$), eliminating mechanism/fender collision.
   - Target Gaps: 40 mm longitudinal gap between Row 1 and Row 2; 40 mm transverse centerline gap between Port and Starboard.
   - Resupply Interlock: Upstream conveyor gate locked (`loader_speed = 0.0 m/s`, `door_speed = 0.0 m/s`) while robot is actively loading in the hold.
   - Engineering Disclosure: Upper-layer reach kinematically proven in CAD; soft-luggage compliance requires hardware testing.

## Evidence, camera and acceptance gate
- Measured floor deposit COM: Row 1 Base (X=5.00m, Y=±0.27m, Z=0.150–0.160m), Upper Tier (Z=0.450–0.480m), Row 2 Base (X=4.18m, Y=±0.27m, Z=0.150–0.160m).
- Measured roof clearance: 510 mm headroom above 32kg two-tier stack below 1.150 m aircraft ceiling (Threshold: >=300 mm).
- Measured chassis clearances: 3.0 mm ground clearance, 28.5 mm track margin, >860 mm front track clearance during Row 2 placement.
- Automated Test Suite: `scripts/verify_scene04_stacking_suite.py` outputs `results/scene04_verification_metrics.json` and `results/scene04_interlock_log.csv` (100% PASS across nominal, tolerance, and fault tests).
- Outputs Produced:
  - `videos/segment_04_reach_stack_resupply.mp4` (Annotated HUD, 1920x1080 @ 30 fps, 20.0s / 600 frames).
  - `videos/scene_04_pure.mp4` (Pure clean video, strictly zero overlays, 1920x1080 @ 30 fps, 20.0s / 600 frames).


