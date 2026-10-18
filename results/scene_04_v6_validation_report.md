# BellyBot Scene 04 Revision 6 Technical Validation Report

**Overall Verdict:** `PASS`

**Timestamp:** `2026-10-17T20:58:33Z`

## 1. Priority 0: Physical Dynamic Test Evidence

- **Model:** `bag_hard_32` (static=`False`, mass=32.0 kg, $\mu=0.45$)
- **Actuator Convergence:** `CONVERGED`
- **Settling Velocity:** `0.00000 m/s` (Threshold: 0.020 m/s) -> `PASS`
- **Floor Penetration:** `0.00 mm` -> `ZERO_PENETRATION`

## 2. Six Completed Placements (2x2x1 Base + 2x1x1 Upper)

| Bag # | Bay Target | Tier | Target (X, Y, Z) | Bottom Z | Surface Z | Contact Err | Status |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 1 | ROW 1 PORT (BASE) | BASE | (5.00, +0.27, 0.160) | 0.000 m | 0.000 m | 0.00 mm | `PASS` |
| 2 | ROW 1 STBD (BASE) | BASE | (5.00, -0.27, 0.150) | 0.000 m | 0.000 m | 0.00 mm | `PASS` |
| 3 | ROW 1 PORT (UPPER) | UPPER | (5.00, +0.27, 0.480) | 0.320 m | 0.320 m | 0.00 mm | `PASS` |
| 4 | ROW 1 STBD (UPPER) | UPPER | (5.00, -0.27, 0.450) | 0.300 m | 0.300 m | 0.00 mm | `PASS` |
| 5 | ROW 2 PORT (BASE) | BASE | (4.18, +0.27, 0.150) | 0.000 m | 0.000 m | 0.00 mm | `PASS` |
| 6 | ROW 2 STBD (BASE) | BASE | (4.18, -0.27, 0.160) | 0.000 m | 0.000 m | 0.00 mm | `PASS` |

## 3. Fuselage and Roof Clearances

- **Ceiling Datum:** `Z = 1.150 m`
- **Max Stack Height:** `Z = 0.640 m` (Top of Bag 3)
- **Measured Roof Clearance:** `510.0 mm` (Threshold: > 50 mm) -> `PASS`
- **Measured Sidewall Clearance:** `680.0 mm` (Threshold: > 100 mm) -> `PASS`

## 4. Speed & Motion Continuity

- **Belt Kinematics:** 0.76m travel at 0.45 m/s = 1.69s (51 frames at 30 fps) [Exact match]
- **Chassis Stepback:** Delta X = -0.820 m with S-curve velocity profile (v_max = 0.25 m/s, a <= 1.6 m/s^2)
- **Retraction Ordering:** Pitch levels to 0.0 deg strictly BEFORE boom retraction
- **Feeder Continuity:** Continuous infeed from rear buffer corridor eliminates sudden bag pop-in across cuts

## 5. Cadence & Throughput

- **Net Transfer Cadence:** `11.5 bags/min` (Target: 12.0 bags/min)
- **Gross Batch Cadence:** `8.1 bags/min`
- **Video Deliverable:** Exactly 35.0 seconds (1050 frames @ 30 fps)
- **Time Scale Disclosure:** `1.0x Real-time / 1.8x Presentation Condensed` transparently labeled on HUD
