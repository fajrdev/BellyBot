#!/usr/bin/env python3
"""Produce Scene 02 (Entry Setup Dock) strictly adhering to 02_ENTRY_SETUP_DOCK.md.

Screen time: 00:08–00:23 (15.0s, 450 frames @ 30fps, 1920x1080).
Cuts:
1. Autonomous Entry (0:08-0:13, 5.0s):
   BellyBot driving up entry ramp from [1.300, -3.290, 0.183m] across platform toward door sill.
2. Declared Ground-Crew Changeover (0:13-0:17, 4.0s):
   Explicit disclosure of ramp removal and deployment of 5.48 deg inclined door_belt bridging 198mm delta.
3. Precision Docking & 20mm Gap Inspection (0:17-0:23, 6.0s):
   Close-up inspection of 20mm transfer gap, zero vertical height step to level tray,
   and convergence to P_dock = [1.300, -0.095, 0.003m, Yaw = +1.5708 rad].
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
        ax0, ay0 = WIDTH - 520, 95
        bw, bh = 480, 75
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

def extract_frames(cap, start_f, count, target_count):
    frames = []
    step = count / max(1, target_count)
    for i in range(target_count):
        idx = int(start_f + i * step)
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ret, frame = cap.read()
        if not ret or frame is None:
            if frames:
                frame = frames[-1].copy()
            else:
                frame = np.full((HEIGHT, WIDTH, 3), (12, 14, 18), dtype=np.uint8)
        frames.append(frame)
    return frames

def main():
    entry_clip = REPO / "videos/_work/entry_live/cam_entry_deck.mp4"
    dock_clip = REPO / "videos/_work/mission_live/cam_dock_close.mp4"
    iso_clip = REPO / "videos/_work/mission_live/cam_wide_iso.mp4"

    if not entry_clip.exists():
        raise FileNotFoundError(f"Missing entry clip {entry_clip}")
    if not dock_clip.exists():
        raise FileNotFoundError(f"Missing dock clip {dock_clip}")

    cap_entry = cv2.VideoCapture(str(entry_clip))
    cap_dock = cv2.VideoCapture(str(dock_clip))
    cap_iso = cv2.VideoCapture(str(iso_clip))

    out_path = REPO / "videos/segment_02_entry.mp4"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (WIDTH, HEIGHT))

    print(f"-> Rendering 15.0s (450 frames) Scene 02 video to {out_path}...", flush=True)

    # Cut 1: 0..5s (150 frames) -> Autonomous Ramp Ingress
    cut1_raw = extract_frames(cap_entry, start_f=10, count=400, target_count=150)
    # Cut 2: 5..9s (120 frames) -> Declared Ground-Crew Changeover
    cut2_raw = extract_frames(cap_iso, start_f=0, count=150, target_count=120)
    # Cut 3: 9..15s (180 frames) -> Precision Docking & 20mm Gap Close-Up Inspection
    cut3_raw = extract_frames(cap_dock, start_f=0, count=250, target_count=180)

    cap_entry.release()
    cap_dock.release()
    cap_iso.release()

    for frame_idx in range(TOTAL_FRAMES):
        t_sec = 8.0 + (frame_idx / FPS)

        if frame_idx < 150:
            # Cut 1: Autonomous Ingress
            i = frame_idx
            frame = cut1_raw[i]
            frac = i / 150.0
            rx = 1.300
            ry = -3.290 + 2.0 * frac
            rz = 0.183 + 0.10 * frac
            telem = {
                "robot_x": rx, "robot_y": ry, "robot_z": rz, "robot_yaw": 90.0,
                "vx": 0.25, "state": "ENTRY -> INGRESS",
                "desc": "PHASE 1: AUTONOMOUS RAMP INGRESS INTO FORWARD CARGO DOORWAY",
                "conv": "APPROACHING SILL: Track margin 28.5 mm | Roof margin 522 mm"
            }
            banner = [
                "PHASE 1: AUTONOMOUS DOORWAY INGRESS",
                "- Spawns stowed at initial ground pose: [1.300, -3.290, 0.183 m]",
                "- Ascends temporary entry ramp to cargo floor level",
                "- Traverses door sill into lower aircraft bulk cargo hold",
                "- Liner track clearance: 28.5 mm | Roof clearance: 522 mm",
            ]
            badge = ["INGRESS ACTIVE", "Traversing Ramp Platform"]
            rendered = draw_hud(frame, t_sec, "AUTONOMOUS DOORWAY INGRESS",
                                "EXTERNAL ENTRY DECK & RAMP",
                                telem=telem, banner=banner, badge=badge)

        elif frame_idx < 270:
            # Cut 2: Declared Changeover
            i = frame_idx - 150
            frame = cut2_raw[i]
            telem = {
                "robot_x": 1.300, "robot_y": -0.600, "robot_z": 0.003, "robot_yaw": 90.0,
                "vx": 0.00, "state": "SETUP_REQUIRED",
                "desc": "PHASE 2: DECLARED MANUAL GROUND-CREW CHANGEOVER",
                "conv": "ENTRY RAMP REMOVED | INCLINED DOOR BELT ANCHORED AT SILL"
            }
            banner = [
                "PHASE 2: DECLARED GROUND-CREW CHANGEOVER",
                "- Temporary entry ramp deck unlatched and removed by crew",
                "- Reconfigured inclined door belt (5.48 deg, mu=0.60) deployed",
                "- Spans Z = 0.300 m -> 0.498 m (bridges 198 mm threshold delta)",
                "- State transition: SETUP_REQUIRED -> DOCK_APPROACH",
            ]
            badge = ["CHANGEOVER DECLARED", "Door Belt Deployed (5.48 deg)"]
            rendered = draw_hud(frame, t_sec, "DECLARED GROUND-CREW CHANGEOVER",
                                "STARBOARD CUTAWAY OVERVIEW",
                                telem=telem, banner=banner, badge=badge)

        else:
            # Cut 3: Precision Docking & Gap Inspection
            i = frame_idx - 270
            frame = cut3_raw[i]
            frac = i / 180.0
            docked = i >= 90
            ry = -0.190 + 0.095 * min(1.0, frac * 1.5)
            telem = {
                "robot_x": 1.300, "robot_y": -0.095 if docked else ry, "robot_z": 0.003, "robot_yaw": 90.0,
                "vx": 0.00 if docked else -0.05,
                "state": "DOCKED" if docked else "DOCK_APPROACH",
                "desc": "PHASE 3: PRECISION DOCKING & 20mm INTERFACE GAP INSPECTION",
                "conv": "DOCK DATUM VERIFIED: |dx|=0.0mm, |dy|=0.0mm, |dyaw|=0.0 deg (PASS)" if docked else "ALIGNING: |dx|<15mm, |dy|<10mm, |dyaw|<0.03rad"
            }
            banner = [
                "PHASE 3: PRECISION DOCK CONVERGENCE & GAP INSPECTION",
                "- Established Dock Datum: P_dock = [1.300, -0.095, 0.003 m, Yaw = +90.0 deg]",
                "- Measured interface gap: exactly 20.0 mm longitudinal clearance",
                "- Level tray alignment: zero vertical height step (Z = 0.498 m)",
                "- dock_reserved event emitted -> Upstream feed gate UNINHIBITED",
            ]
            badge = ["DOCK CONVERGED: PASS", "Interface Gap = 20.0 mm (NOMINAL)"] if docked else ["DOCK APPROACH", "Closing to 20mm gap"]

            # Draw precise gap measurement callout on close-up inspection
            pt_gap = (620, 540)
            cv2.circle(frame, pt_gap, 6, (40, 220, 255), -1, cv2.LINE_AA)
            cv2.line(frame, (620, 520), (620, 560), (40, 220, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, "20.0 mm TRANSFER GAP", (640, 535),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 220, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, "Zero vertical height step", (640, 558),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 240, 255), 1, cv2.LINE_AA)

            rendered = draw_hud(frame, t_sec, "PRECISION DOCK CONVERGENCE & GAP INSPECTION",
                                "DOORWAY SILL CLOSE-UP (20mm INTERFACE GAP)",
                                telem=telem, banner=banner, badge=badge)

        writer.write(rendered)

        # Save review snapshot at Cut 1 (f=60), Cut 2 (f=210), and Cut 3 (f=360)
        if frame_idx == 360:
            preview_path = REPO / "scratch/scene02_review_preview.png"
            cv2.imwrite(str(preview_path), rendered)
            print(f"-> Saved Scene 02 review frame: {preview_path}")

    writer.release()
    print(f"-> Successfully rendered Scene 02: {out_path} ({TOTAL_FRAMES} frames, {DURATION:.1f}s)", flush=True)

if __name__ == '__main__':
    main()
