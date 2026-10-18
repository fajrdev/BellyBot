#!/usr/bin/env python3
"""Render pure cinematic video cuts for Scene 01, Scene 02, and Scene 03.

Strictly zero text, zero graphics, zero HUD overlays, zero banners, zero datum lines.
Pure high-definition 1080p camera recordings:
1. videos/scene_01_pure.mp4: (00:00 - 00:08, 8.0s, 240 frames @ 30fps) - Concept & Bulk Hold Architecture
2. videos/scene_02_pure.mp4: (00:08 - 00:23, 15.0s, 450 frames @ 30fps) - Autonomous Ingress, Changeover, Centered Handoff
3. videos/scene_03_pure.mp4: (00:23 - 00:53, 30.0s, 900 frames @ 30fps) - Agile Pivot, Corridor Cruise, Soft Touchdown Deposit
"""

import os
import sys
from pathlib import Path
import cv2
import numpy as np

REPO = Path("/home/fajrda/sutd-hackathon")
WIDTH, HEIGHT = 1920, 1080
FPS = 30.0

def render_scene_01():
    print("=== RENDERING SCENE 01 (PURE) ===")
    raw_path = REPO / "scratch/images/scene_01/scene01_precise_raw.png"
    if not raw_path.exists():
        raw_path = REPO / "scratch/scene01_precise_raw.png"
    if not raw_path.exists():
        raise FileNotFoundError(f"Missing {raw_path}")
    base_frame = cv2.imread(str(raw_path))
    if base_frame.shape[1] != WIDTH or base_frame.shape[0] != HEIGHT:
        base_frame = cv2.resize(base_frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)

    out_path = REPO / "videos/scene_01_pure.mp4"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (WIDTH, HEIGHT))

    frames = int(8.0 * FPS)
    for _ in range(frames):
        writer.write(base_frame)
    writer.release()
    print(f"-> Saved: {out_path} ({frames} frames, 8.0s)")

def render_scene_02():
    print("=== RENDERING SCENE 02 (PURE) ===")
    clip_entry = REPO / "videos/_work/entry_live/cam_entry_deck.mp4"
    clip_dock = REPO / "videos/_work/dock_infeed/cam_dock_close.mp4"

    p_ramp = REPO / "scratch/images/scene_02/new_entry_f530.png"
    if not p_ramp.exists(): p_ramp = REPO / "scratch/new_entry_f530.png"
    p_belt = REPO / "scratch/images/scene_02/cut2_deck_door_belt.png"
    if not p_belt.exists(): p_belt = REPO / "scratch/cut2_deck_door_belt.png"

    img_ramp = cv2.imread(str(p_ramp))
    img_belt = cv2.imread(str(p_belt))

    if not clip_entry.exists() or not clip_dock.exists():
        raise FileNotFoundError("Missing source clips for Scene 02")
    if img_ramp is None or img_belt is None:
        raise FileNotFoundError("Missing Cut 2 keyframe images for Scene 02")

    if img_ramp.shape[1] != WIDTH or img_ramp.shape[0] != HEIGHT:
        img_ramp = cv2.resize(img_ramp, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)
    if img_belt.shape[1] != WIDTH or img_belt.shape[0] != HEIGHT:
        img_belt = cv2.resize(img_belt, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)

    cap_entry = cv2.VideoCapture(str(clip_entry))
    cap_dock = cv2.VideoCapture(str(clip_dock))

    # Cut 1: 150 frames (ingress)
    cut1_step = (530 - 10) / 150.0
    cut1_frames = []
    for i in range(150):
        f_idx = int(10 + i * cut1_step)
        cap_entry.set(cv2.CAP_PROP_POS_FRAMES, f_idx)
        ret, frame = cap_entry.read()
        if not ret or frame is None:
            frame = cut1_frames[-1].copy() if cut1_frames else np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        if frame.shape[1] != WIDTH or frame.shape[0] != HEIGHT:
            frame = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)
        cut1_frames.append(frame)

    # Cut 3: 180 frames (dock & centered bag)
    cut3_frames = []
    for i in range(180):
        if i < 110:
            f_idx = int(i * (134.0 / 110.0))
        else:
            f_idx = 134
        cap_dock.set(cv2.CAP_PROP_POS_FRAMES, f_idx)
        ret, frame = cap_dock.read()
        if not ret or frame is None:
            frame = cut3_frames[-1].copy() if cut3_frames else np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        if frame.shape[1] != WIDTH or frame.shape[0] != HEIGHT:
            frame = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)
        cut3_frames.append(frame)

    cap_entry.release()
    cap_dock.release()

    out_path = REPO / "videos/scene_02_pure.mp4"
    writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (WIDTH, HEIGHT))

    total_frames = 450
    for frame_idx in range(total_frames):
        if frame_idx < 150:
            # Cut 1: Autonomous Ingress
            frame = cut1_frames[frame_idx]
        elif frame_idx < 270:
            # Cut 2: Pure Changeover Transition (Ramp -> Door Belt)
            i = frame_idx - 150
            if i < 30:
                frame = img_ramp
            elif i < 70:
                alpha = (i - 30) / 40.0
                frame = cv2.addWeighted(img_ramp, 1.0 - alpha, img_belt, alpha, 0)
            else:
                frame = img_belt
        else:
            # Cut 3: Precision Docking & Centered Handoff
            frame = cut3_frames[frame_idx - 270]

        writer.write(frame)

    writer.release()
    print(f"-> Saved: {out_path} ({total_frames} frames, 15.0s)")

def render_scene_03():
    print("=== RENDERING SCENE 03 (PURE) ===")
    clip_iso = REPO / "videos/_work/mission_live/cam_wide_iso.mp4"
    clip_runway = REPO / "videos/_work/mission_live/cam_deposit_low.mp4"

    if not clip_iso.exists() or not clip_runway.exists():
        raise FileNotFoundError("Missing source clips for Scene 03")

    cap_iso = cv2.VideoCapture(str(clip_iso))
    cap_runway = cv2.VideoCapture(str(clip_runway))

    # Cut 1: 300 frames (10.0s, dock departure & agile pivot) from cam_wide_iso (frames 0..270)
    frames_cut1 = []
    for i in range(300):
        f_in = int(i * (270.0 / 300.0))
        cap_iso.set(cv2.CAP_PROP_POS_FRAMES, f_in)
        ret, frame = cap_iso.read()
        if not ret or frame is None:
            frame = frames_cut1[-1].copy() if frames_cut1 else np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        if frame.shape[1] != WIDTH or frame.shape[0] != HEIGHT:
            frame = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)
        frames_cut1.append(frame)

    # Cut 2: 600 frames (20.0s, corridor cruise, extension, soft touchdown deposit, and settlement)
    frames_cut2 = []
    for i in range(600):
        f_in = int(260 + i * (470.0 / 600.0))
        cap_runway.set(cv2.CAP_PROP_POS_FRAMES, f_in)
        ret, frame = cap_runway.read()
        if not ret or frame is None:
            frame = frames_cut2[-1].copy() if frames_cut2 else np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        if frame.shape[1] != WIDTH or frame.shape[0] != HEIGHT:
            frame = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)
        frames_cut2.append(frame)

    cap_iso.release()
    cap_runway.release()

    out_path = REPO / "videos/scene_03_pure.mp4"
    writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (WIDTH, HEIGHT))

    total_frames = 900
    for frame_idx in range(total_frames):
        if frame_idx < 300:
            frame = frames_cut1[frame_idx]
        else:
            frame = frames_cut2[frame_idx - 300]
        writer.write(frame)

    writer.release()
    print(f"-> Saved: {out_path} ({total_frames} frames, 30.0s)")

def main():
    render_scene_01()
    render_scene_02()
    render_scene_03()
    print("\n*** ALL 3 PURE SCENES SUCCESSFULLY RENDERED ***")

if __name__ == '__main__':
    main()
