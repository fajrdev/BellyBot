#!/usr/bin/env python3
"""Compose a storyboard segment clip from per-camera recordings, with HUD overlays.

No ffmpeg on this host, so OpenCV does the letterboxing, title/footer bands and
per-cut captions. Driven by a JSON plan so text can be tuned without code edits.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def letterbox(frame, w, h, bg=(18, 20, 22)):
    fh, fw = frame.shape[:2]
    scale = min(w / fw, h / fh)
    nw, nh = int(fw * scale), int(fh * scale)
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = np.full((h, w, 3), bg, np.uint8)
    x = (w - nw) // 2
    y = (h - nh) // 2
    canvas[y:y + nh, x:x + nw] = resized
    return canvas


def band(canvas, y0, y1, color=(0, 0, 0), alpha=0.55):
    overlay = canvas[y0:y1].copy()
    cv2.rectangle(overlay, (0, 0), (canvas.shape[1], y1 - y0), color, -1)
    canvas[y0:y1] = cv2.addWeighted(overlay, alpha, canvas[y0:y1], 1 - alpha, 0)


def text(canvas, s, x, y, size, color, thickness=2):
    cv2.putText(canvas, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, (0, 0, 0),
                thickness + 3, cv2.LINE_AA)
    cv2.putText(canvas, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color,
                thickness, cv2.LINE_AA)


def solid_card(w, h, lines):
    card = np.full((h, w, 3), (16, 26, 32), np.uint8)
    y = h // 2 - (len(lines) - 1) * 22
    for i, (s, size, col) in enumerate(lines):
        (tw, th), _ = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, size, 2)
        text(card, s, (w - tw) // 2, y + i * 46, size, col)
    return card


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("plan", help="JSON plan path")
    args = ap.parse_args()
    plan = json.loads(Path(args.plan).read_text())

    w, h, fps = plan["width"], plan["height"], plan["fps"]
    out = plan["out"]
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))

    title_card = solid_card(w, h, [(plan["title"], 1.1, (120, 220, 255)),
                                   (plan.get("subtitle", ""), 0.6, (230, 230, 230))])
    for _ in range(int(fps * plan.get("title_seconds", 1.5))):
        writer.write(title_card)

    for cut in plan["cuts"]:
        cap = cv2.VideoCapture(cut["src"])
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        s = int(n * cut["start_frac"])
        e = int(n * cut["end_frac"])
        out_frames = int(cut["dur"] * fps)
        frames = 0
        for i in range(out_frames):
            fidx = s + int((e - s - 1) * i / max(1, out_frames - 1))
            cap.set(cv2.CAP_PROP_POS_FRAMES, fidx)
            ok, fr = cap.read()
            if not ok:
                break
            canvas = letterbox(fr, w, h)
            band(canvas, 0, 54)
            text(canvas, plan["title"], 24, 36, 0.9, (120, 220, 255), 2)
            band(canvas, h - 92, h)
            text(canvas, plan.get("footer", ""), 24, h - 60, 0.62, (230, 230, 230), 1)
            if cut.get("label"):
                text(canvas, cut["label"], 24, h - 22, 0.66, (90, 230, 160), 1)
            cv2.putText(canvas, f"VIEW: {cut['camera']}", (w - 360, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            writer.write(canvas)
            frames += 1
        cap.release()
        print(f"  cut {cut['camera']}: {frames} frames ({cut['dur']}s)")

    end_card = solid_card(w, h, [(plan.get("end_title", ""), 0.95, (120, 220, 255))])
    for _ in range(int(fps * plan.get("end_seconds", 0.8))):
        writer.write(end_card)

    writer.release()
    total = cv2.VideoCapture(out).get(cv2.CAP_PROP_FRAME_COUNT) / fps
    print(f"wrote {out}  ({total:.1f}s, {w}x{h}@{fps})")


if __name__ == "__main__":
    main()
