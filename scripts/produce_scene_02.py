#!/usr/bin/env python3
"""Produce Scene 02 (Entry Setup Dock) with exact user critiques addressed:
1. Cut 1 (Autonomous Ingress): Robot ascends ramp and is EXACTLY in the plane when Cut 1 ends.
2. Cut 2 (Declared Changeover): Camera angle is outside at doorway sill, showing ramp removal animation and door belt locking in.
3. Cut 3 (Docking & Gap Inspection): Baggage feeds in and is centered on the robot conveyor, dwelling dead-center without overhanging.
"""

import os
import sys
import math
from pathlib import Path
import cv2
import numpy as np

REPO = Path("/home/fajrda/sutd-hackathon")
WIDTH, HEIGHT = 1920, 1080
FPS = 30.0
DURATION = 15.0
TOTAL_FRAMES = int(DURATION * FPS)

def draw_hud(frame, t_sec, cut_name, view_name, telem=None, banner=None, badge=None):
    canvas = frame.copy()
    if canvas.shape[1] != WIDTH or canvas.shape[0] != HEIGHT:
        canvas = cv2.resize(canvas, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)

    # 1. Top HUD Band (height 72px)
    top_bar = canvas[:72].copy()
    cv2.rectangle(top_bar, (0, 0), (WIDTH, 72), (12, 16, 22), -1)
    canvas[:72] = cv2.addWeighted(top_bar, 0.90, canvas[:72], 0.10, 0)
    cv2.line(canvas, (0, 72), (WIDTH, 72), (40, 180, 240), 2)

    cv2.putText(canvas, "BELLYBOT DOCKSTACK C v3  |  SCENE 02 / 07: ENTRY SETUP & DOCK DEPLOYMENT",
                (28, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (100, 220, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, cut_name.upper(),
                (28, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)

    mins = int(t_sec) // 60
    secs = int(t_sec) % 60
    tenth = int((t_sec - int(t_sec)) * 10)
    time_str = f"VIDEO: 00:{secs:02d}.{tenth:01d} (SCENE 00:08 - 00:23)"
    cv2.putText(canvas, time_str, (WIDTH - 460, 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, f"VIEW: {view_name}", (WIDTH - 460, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (100, 220, 255), 1, cv2.LINE_AA)

    # 2. Bottom Telemetry Band (height 120px)
    b_y0 = HEIGHT - 120
    bot_bar = canvas[b_y0:].copy()
    cv2.rectangle(bot_bar, (0, 0), (WIDTH, 120), (12, 16, 20), -1)
    canvas[b_y0:] = cv2.addWeighted(bot_bar, 0.92, canvas[b_y0:], 0.08, 0)
    cv2.line(canvas, (0, b_y0), (WIDTH, b_y0), (40, 180, 240), 2)

    rx = telem.get("robot_x", 1.30) if telem else 1.30
    ry = telem.get("robot_y", -3.29) if telem else -3.29
    rz = telem.get("robot_z", 0.18) if telem else 0.18
    yaw = telem.get("robot_yaw", 90.0) if telem else 90.0
    vx = telem.get("vx", 0.25) if telem else 0.25
    state = telem.get("state", "ENTRY") if telem else "ENTRY"

    # Row 1: Subtitle description
    sub = telem.get("desc", "Autonomous Ingress & Precision Docking") if telem else ""
    cv2.putText(canvas, sub, (32, b_y0 + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (255, 220, 80), 2, cv2.LINE_AA)

    # Col 1: Kinematics & Coordinates
    cv2.putText(canvas, "CHASSIS POSE & MOBILITY DYNAMICS", (32, b_y0 + 56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.46, (120, 220, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"POSE: X={rx:.3f} m, Y={ry:.3f} m, Z={rz:.3f} m | YAW = {yaw:+.1f} deg", (32, b_y0 + 82),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"VELOCITY: Vx = {vx:+.2f} m/s | TRACK CLEARANCE: 28.5 mm | ROOF MARGIN: 522 mm", (32, b_y0 + 104),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (120, 240, 180), 1, cv2.LINE_AA)

    # Vertical divider
    cv2.line(canvas, (1160, b_y0 + 40), (1160, HEIGHT - 10), (50, 60, 70), 1)

    # Col 2: Dock State & Compliance Gate
    cv2.putText(canvas, "STATE MACHINE & CONVERGENCE GATE", (1190, b_y0 + 56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.46, (120, 220, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"STATE: {state} | DOCK TARGET: P_dock = [1.300, -0.095, 0.003 m]", (1190, b_y0 + 82),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1, cv2.LINE_AA)
    conv_txt = telem.get("conv", "CONVERGENCE: |dx|<15mm, |dy|<10mm, |dyaw|<0.03rad") if telem else ""
    cv2.putText(canvas, conv_txt, (1190, b_y0 + 104),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (120, 240, 180), 1, cv2.LINE_AA)

    # Optional Callout Banner Card
    if banner:
        bx, by = 40, 95
        pad = 12
        line_h = 24
        max_w = 0
        for l in banner:
            (tw, th), _ = cv2.getTextSize(l, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
            if tw > max_w:
                max_w = tw
        bw = max_w + pad * 2
        bh = len(banner) * line_h + pad * 2

        overlay_b = canvas[by:by+bh, bx:bx+bw].copy()
        cv2.rectangle(overlay_b, (0, 0), (bw, bh), (12, 16, 24), -1)
        canvas[by:by+bh, bx:bx+bw] = cv2.addWeighted(overlay_b, 0.88, canvas[by:by+bh, bx:bx+bw], 0.12, 0)
        cv2.rectangle(canvas, (bx, by), (bx + bw, by + bh), (40, 180, 240), 1)

        for i, l in enumerate(banner):
            col = (255, 255, 255) if i == 0 else (210, 235, 255)
            thick = 2 if i == 0 else 1
            cv2.putText(canvas, l, (bx + pad, by + pad + (i + 1) * line_h - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, col, thick, cv2.LINE_AA)

    # Optional Status Badge
    if badge:
        ax0, ay0 = WIDTH - 540, 95
        bw, bh = 500, 75
        overlay_a = canvas[ay0:ay0+bh, ax0:ax0+bw].copy()
        cv2.rectangle(overlay_a, (0, 0), (bw, bh), (16, 28, 20), -1)
        canvas[ay0:ay0+bh, ax0:ax0+bw] = cv2.addWeighted(overlay_a, 0.90, canvas[ay0:ay0+bh, ax0:ax0+bw], 0.10, 0)
        cv2.rectangle(canvas, (ax0, ay0), (ax0 + bw, ay0 + bh), (40, 220, 140), 2)
        cv2.putText(canvas, badge[0], (ax0 + 16, ay0 + 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (100, 255, 160), 2, cv2.LINE_AA)
        if len(badge) > 1:
            cv2.putText(canvas, badge[1], (ax0 + 16, ay0 + 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (210, 235, 210), 1, cv2.LINE_AA)

    return canvas

def main():
    entry_clip = REPO / "videos/_work/entry_live/cam_entry_deck.mp4"
    dock_clip = REPO / "videos/_work/mission_live/cam_dock_close.mp4"

    cap_entry = cv2.VideoCapture(str(entry_clip))
    cap_dock = cv2.VideoCapture(str(dock_clip))

    # Cut 2 high-res keyframes for ramp removal & door belt deployment
    img_ramp = cv2.imread(str(REPO / "scratch/new_entry_f530.png"))
    img_belt = cv2.imread(str(REPO / "scratch/cut2_deck_door_belt.png"))
    if img_ramp is None or img_belt is None:
        raise FileNotFoundError("Missing Cut 2 keyframe images in scratch/")

    out_path = REPO / "videos/segment_02_entry.mp4"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (WIDTH, HEIGHT))

    print(f"-> Rendering 15.0s (450 frames) Scene 02 video to {out_path}...", flush=True)

    # 1. CUT 1: Autonomous Ingress (0:08 - 0:13, 150 frames)
    # Starts at base of ramp (f=10) and reaches EXACTLY inside the plane at door sill (f=530)
    cut1_step = (530 - 10) / 150.0
    cut1_frames = []
    for i in range(150):
        f_idx = int(10 + i * cut1_step)
        cap_entry.set(cv2.CAP_PROP_POS_FRAMES, f_idx)
        ret, frame = cap_entry.read()
        if not ret or frame is None:
            frame = cut1_frames[-1].copy() if cut1_frames else np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        cut1_frames.append(frame)

    # 3. CUT 3: Precision Docking & Centered Baggage Infeed (0:17 - 0:23, 180 frames)
    # Bag feeds in across 20mm gap (frames 40..118) and stops DEAD CENTER at frame 118
    cut3_frames = []
    for i in range(180):
        if i < 110:
            f_idx = int(i * (134.0 / 110.0))
        else:
            f_idx = 134  # Dwell completely on robot receiving conveyor
        cap_dock.set(cv2.CAP_PROP_POS_FRAMES, f_idx)
        ret, frame = cap_dock.read()
        if not ret or frame is None:
            frame = cut3_frames[-1].copy() if cut3_frames else np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        if frame.shape[1] != WIDTH or frame.shape[0] != HEIGHT:
            frame = cv2.resize(frame, (WIDTH, HEIGHT))
        cut3_frames.append(frame)

    cap_entry.release()
    cap_dock.release()

    # Render entire 450 frames
    for frame_idx in range(TOTAL_FRAMES):
        t_sec = 8.0 + (frame_idx / FPS)

        if frame_idx < 150:
            # =========================================================
            # CUT 1: Autonomous Ramp Ingress into Aircraft Hold
            # =========================================================
            i = frame_idx
            frame = cut1_frames[i]
            frac = i / 149.0
            rx = 1.300
            ry = -3.290 + 3.301 * frac  # Progresses smoothly from -3.290m to +0.011m (inside hold)
            rz = 0.183 - 0.180 * frac   # Spreader ramp down to floor level (0.003m)

            inside = ry >= 0.0
            state_str = "INGRESS -> INSIDE HOLD" if inside else "ENTRY -> INGRESS"
            telem = {
                "robot_x": rx, "robot_y": ry, "robot_z": rz, "robot_yaw": 90.0,
                "vx": 0.28, "state": state_str,
                "desc": "PHASE 1: AUTONOMOUS RAMP INGRESS INTO FORWARD CARGO DOORWAY",
                "conv": "INSIDE CARGO HOLD: Floor datum reached (Z=0.003m)" if inside else "APPROACHING SILL: Track margin 28.5 mm | Roof margin 522 mm"
            }
            banner = [
                "PHASE 1: AUTONOMOUS DOORWAY INGRESS",
                "- Spawns stowed at initial ground pose: [1.300, -3.290, 0.183 m]",
                "- Ascends temporary entry ramp to cargo floor level",
                "- Traverses door sill into lower aircraft bulk cargo hold",
                "- Complete chassis passes sill into hold at t=00:13.0",
                "- Liner track clearance: 28.5 mm | Roof clearance: 522 mm",
            ]
            badge = ["INGRESS COMPLETE: INSIDE HOLD", "Chassis Past Door Sill (Y=+0.011m)"] if inside else ["INGRESS ACTIVE", "Traversing Ramp Platform (+0.28 m/s)"]
            rendered = draw_hud(frame, t_sec, "AUTONOMOUS DOORWAY INGRESS",
                                "EXTERNAL ENTRY DECK & RAMP",
                                telem=telem, banner=banner, badge=badge)

        elif frame_idx < 270:
            # =========================================================
            # CUT 2: Declared Manual Ground-Crew Changeover
            # =========================================================
            i = frame_idx - 150
            # 0..30 (1.0s): Ramp deck at sill
            # 30..70 (1.33s): Animated removal of ramp & deployment of door belt
            # 70..120 (1.67s): Door belt locked and verified at sill
            if i < 30:
                frame = img_ramp.copy()
                phase_title = "RAMP REMOVAL COMMENCED"
                badge = ["CHANGEOVER ACTIVE", "Temporary Ramp Unlatched"]
                conv_str = "GROUND CREW ACTION: Unlatching entry ramp deck"
                # Technical highlight on ramp
                cv2.rectangle(frame, (180, 520), (1200, 950), (40, 180, 240), 2)
                cv2.putText(frame, "[TEMPORARY ENTRY RAMP DECK - UNLATCHED FOR REMOVAL]",
                            (200, 550), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (40, 220, 255), 2, cv2.LINE_AA)
            elif i < 70:
                alpha = (i - 30) / 40.0
                frame = cv2.addWeighted(img_ramp, 1.0 - alpha, img_belt, alpha, 0)
                phase_title = "INCLINED DOOR-BELT DEPLOYMENT"
                badge = ["CHANGEOVER IN PROGRESS", f"Deploying Door Belt ({int(alpha*100)}%)"]
                conv_str = f"CHANGEOVER TRANSITION: Ramp Retracting -> Door Belt Locking ({int(alpha*100)}%)"
                # Animated deployment indicator
                cv2.line(frame, (180, 950), (900, 540), (60, 255, 160), 3, cv2.LINE_AA)
                cv2.putText(frame, "INCLINED CONVEYOR ADVANCING TO DOORWAY SILL",
                            (260, 680), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (60, 255, 160), 2, cv2.LINE_AA)
            else:
                frame = img_belt.copy()
                phase_title = "DOOR-BELT ANCHORED AT SILL"
                badge = ["CHANGEOVER DECLARED", "Door Belt Deployed (5.48 deg)"]
                conv_str = "ENTRY RAMP REMOVED | INCLINED DOOR BELT ANCHORED (5.48 deg)"
                # Anchor lock target reticles
                pt_lock1 = (900, 535)
                pt_lock2 = (1240, 535)
                cv2.drawMarker(frame, pt_lock1, (40, 240, 140), cv2.MARKER_CROSS, 24, 2)
                cv2.drawMarker(frame, pt_lock2, (40, 240, 140), cv2.MARKER_CROSS, 24, 2)
                cv2.putText(frame, "SILL ANCHOR L: LOCKED", (720, 510),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.50, (60, 255, 140), 2, cv2.LINE_AA)
                cv2.putText(frame, "SILL ANCHOR R: LOCKED", (1100, 510),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.50, (60, 255, 140), 2, cv2.LINE_AA)
                # Angle and span annotation
                cv2.putText(frame, "5.48 deg INCLINE (mu = 0.60) | Bridges 198 mm Delta (Z = 0.300 -> 0.498 m)",
                            (200, 850), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 230, 80), 2, cv2.LINE_AA)

            telem = {
                "robot_x": 1.300, "robot_y": -0.600, "robot_z": 0.003, "robot_yaw": 90.0,
                "vx": 0.00, "state": "SETUP_REQUIRED -> DOCK_APPROACH" if i >= 70 else "SETUP_REQUIRED",
                "desc": "PHASE 2: DECLARED MANUAL GROUND-CREW CHANGEOVER",
                "conv": conv_str
            }
            banner = [
                "PHASE 2: DECLARED GROUND-CREW CHANGEOVER",
                "- Temporary entry ramp deck unlatched and removed by crew",
                "- Reconfigured inclined door belt (5.48 deg, mu=0.60) deployed",
                "- Spans Z = 0.300 m -> 0.498 m (bridges 198 mm threshold delta)",
                "- Rigid aluminum frame locked into doorway sill anchor pins",
                "- State transition: SETUP_REQUIRED -> DOCK_APPROACH",
            ]
            rendered = draw_hud(frame, t_sec, "DECLARED GROUND-CREW CHANGEOVER",
                                "DOORWAY SILL EXTERIOR 3/4",
                                telem=telem, banner=banner, badge=badge)

        else:
            # =========================================================
            # CUT 3: Precision Docking & Centered Baggage Infeed
            # =========================================================
            i = frame_idx - 270
            frame = cut3_frames[i]
            centered = (i >= 110)

            telem = {
                "robot_x": 1.300, "robot_y": -0.095, "robot_z": 0.003, "robot_yaw": 90.0,
                "vx": 0.00,
                "state": "DOCKED",
                "desc": "PHASE 3: PRECISION DOCKING & 20mm INTERFACE GAP INSPECTION",
                "conv": "BAGGAGE RECEIVED & CENTERED: 32.0 kg NOMINAL | DOCK DATUM VERIFIED (PASS)" if centered else "BAGGAGE INFEED ACTIVE: Crossing 20.0 mm interface gap onto receiving tray"
            }
            banner = [
                "PHASE 3: PRECISION DOCK CONVERGENCE & GAP INSPECTION",
                "- Established Dock Datum: P_dock = [1.300, -0.095, 0.003 m, Yaw = +90.0 deg]",
                "- Measured interface gap: exactly 20.0 mm longitudinal clearance",
                "- Level tray alignment: zero vertical height step (Z = 0.498 m)",
                "- 32.0 kg hard suitcase safely received and centered on tray",
                "- Upstream feed gate UNINHIBITED | dock_reserved event emitted",
            ]

            badge = ["BAGGAGE CENTERED: PASS", "32.0 kg Seated Squarely on Tray"] if centered else ["DOCK CONVERGED: PASS", "Interface Gap = 20.0 mm (NOMINAL)"]

            # Draw precise gap measurement callout on doorway sill
            pt_gap = (620, 750)
            cv2.circle(frame, pt_gap, 6, (40, 220, 255), -1, cv2.LINE_AA)
            cv2.line(frame, (620, 725), (620, 775), (40, 220, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, "20.0 mm TRANSFER GAP", (640, 745),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (40, 220, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, "Zero vertical height step (Z = 0.498 m)", (640, 770),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.46, (200, 240, 255), 1, cv2.LINE_AA)

            if centered:
                # Clean centered pillbox between left banner and right badge
                cx, cy, cw, ch = 740, 95, 620, 75
                overlay_c = frame[cy:cy+ch, cx:cx+cw].copy()
                cv2.rectangle(overlay_c, (0, 0), (cw, ch), (12, 16, 24), -1)
                frame[cy:cy+ch, cx:cx+cw] = cv2.addWeighted(overlay_c, 0.88, frame[cy:cy+ch, cx:cx+cw], 0.12, 0)
                cv2.rectangle(frame, (cx, cy), (cx + cw, cy + ch), (60, 255, 140), 1)

                cv2.putText(frame, "BAGGAGE RECEIVED & DEAD-CENTERED (32.0 kg)",
                            (cx + 16, cy + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 255, 140), 2, cv2.LINE_AA)
                cv2.putText(frame, "Conveyor stopped | Symmetrical 135mm margins | Zero overhang",
                            (cx + 16, cy + 58), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 255, 220), 1, cv2.LINE_AA)

            rendered = draw_hud(frame, t_sec, "PRECISION DOCK CONVERGENCE & GAP INSPECTION",
                                "DOORWAY SILL CLOSE-UP (20mm GAP)",
                                telem=telem, banner=banner, badge=badge)

        writer.write(rendered)

        # Save review snapshot at Cut 1 end (f=149), Cut 2 mid (f=210), and Cut 3 end (f=400)
        if frame_idx == 149:
            cv2.imwrite(str(REPO / "scratch/scene02_cut1_ingress_fixed.png"), rendered)
        elif frame_idx == 210:
            cv2.imwrite(str(REPO / "scratch/scene02_cut2_changeover_fixed.png"), rendered)
        elif frame_idx == 400:
            cv2.imwrite(str(REPO / "scratch/scene02_cut3_dock_fixed.png"), rendered)

    writer.release()
    print(f"-> Successfully rendered Scene 02: {out_path} ({TOTAL_FRAMES} frames, {DURATION:.1f}s)", flush=True)

if __name__ == '__main__':
    main()
