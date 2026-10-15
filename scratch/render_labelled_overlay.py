import cv2
import numpy as np
import math
from pathlib import Path

# Camera parameters
el = 35.0
az = -67.0
e = math.radians(el)
a = math.radians(az)
nx = math.cos(e) * math.cos(a)
ny = math.cos(e) * math.sin(a)
nz = math.sin(e)
tx, ty, tz = 2.95, -0.75, 0.45
cam_pos = np.array([tx + 9.0 * nx, ty + 9.0 * ny, tz + 9.0 * nz])

pitch = math.asin(nz)
yaw = math.atan2(-ny, -nx)
forward = (np.array([tx, ty, tz]) - cam_pos)
forward /= np.linalg.norm(forward)

up_world = np.array([0, 0, 1])
right = np.cross(forward, up_world)
right /= np.linalg.norm(right)
up = np.cross(right, forward)
up /= np.linalg.norm(up)

width, height = 1280, 720
hfov = 0.65

def project(p):
    d = p - cam_pos
    xc = np.dot(d, right)
    yc = np.dot(d, -up)
    zc = np.dot(d, forward)
    f = (width / 2.0) / math.tan(hfov / 2.0)
    u = width / 2.0 + f * (xc / zc)
    v = height / 2.0 + f * (yc / zc)
    return int(round(u)), int(round(v))

clean = cv2.imread('videos/_work/idea/zoom_D9_fov65.png')
marked = cv2.imread('videos/_work/idea/bellybot revision.png')

canvas = clean.copy()

# Semi-transparent overlay layer
overlay = canvas.copy()

# Model 1: belt_loader (outside aircraft hold) - Cyan
# Belt loader coordinates approx: X=1.3, Y=[-6.0, -2.6], Z=[-0.8, 0.3]
bl_pts = [
    project(np.array([0.9, -6.0, -1.0])),
    project(np.array([1.7, -6.0, -1.0])),
    project(np.array([1.7, -2.5, 0.25])),
    project(np.array([0.9, -2.5, 0.25])),
]
cv2.fillPoly(overlay, [np.array(bl_pts)], (255, 200, 0))

# Model 2: door_belt (retained doorway conveyor) - Green
# X=1.3, Y=[-2.588, -0.512], Z=[0.0, 0.35]
db_pts = [
    project(np.array([0.95, -2.588, 0.0])),
    project(np.array([1.65, -2.588, 0.0])),
    project(np.array([1.65, -0.512, 0.35])),
    project(np.array([0.95, -0.512, 0.35])),
]
cv2.fillPoly(overlay, [np.array(db_pts)], (0, 220, 100))

# Model 3: corner_transfer (marked for removal) - Red / Orange
# X=[0.82, 1.78], Y=[-0.48, 0.48], Z=[0.0, 0.45]
ct_pts = [
    project(np.array([0.82, -0.48, 0.0])),
    project(np.array([1.78, -0.48, 0.0])),
    project(np.array([1.78, 0.48, 0.45])),
    project(np.array([0.82, 0.48, 0.45])),
]
cv2.fillPoly(overlay, [np.array(ct_pts)], (0, 0, 255))

# Model 4: longitudinal_buffer + folding bridge (marked for removal) - Magenta
# X=[1.80, 3.70], Y=[-0.43, 0.43], Z=[0.0, 0.42]
lb_pts = [
    project(np.array([1.80, -0.43, 0.0])),
    project(np.array([3.70, -0.43, 0.0])),
    project(np.array([3.70, 0.43, 0.42])),
    project(np.array([1.80, 0.43, 0.42])),
]
cv2.fillPoly(overlay, [np.array(lb_pts)], (200, 0, 220))

# Model 5: bellybot_dock_stack (retained robot) - Gold / Yellow
# X=[3.65, 4.90], Y=[-0.50, 0.50], Z=[0.0, 0.75]
bb_pts = [
    project(np.array([3.65, -0.50, 0.0])),
    project(np.array([4.90, -0.50, 0.0])),
    project(np.array([4.90, 0.50, 0.75])),
    project(np.array([3.65, 0.50, 0.75])),
]
cv2.fillPoly(overlay, [np.array(bb_pts)], (0, 220, 255))

# Blend overlay with 35% opacity
cv2.addWeighted(overlay, 0.35, canvas, 0.65, 0, canvas)

# Draw outlines
cv2.polylines(canvas, [np.array(bl_pts)], True, (255, 200, 0), 2)
cv2.polylines(canvas, [np.array(db_pts)], True, (0, 255, 120), 2)
cv2.polylines(canvas, [np.array(ct_pts)], True, (0, 50, 255), 3)
cv2.polylines(canvas, [np.array(lb_pts)], True, (255, 50, 255), 3)
cv2.polylines(canvas, [np.array(bb_pts)], True, (0, 255, 255), 2)

# Draw Callout Labels and Leader Lines
def callout(img, text, subtext, pt, callout_pos, color, box_w=280, box_h=44):
    u, v = pt
    cx, cy = callout_pos
    cv2.circle(img, (u, v), 5, color, -1)
    cv2.line(img, (u, v), (cx, cy), color, 2)
    # Background box
    cv2.rectangle(img, (cx - 4, cy - 28), (cx + box_w, cy + box_h - 28), (15, 18, 22), -1)
    cv2.rectangle(img, (cx - 4, cy - 28), (cx + box_w, cy + box_h - 28), color, 2)
    cv2.putText(img, text, (cx + 6, cy - 6), cv2.FONT_HERSHEY_DUPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, subtext, (cx + 6, cy + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

callout(canvas, "[RETAINED] belt_loader", "Apron baggage feed conveyor", project(np.array([1.3, -3.2, 0.0])), (40, 600), (255, 200, 0), box_w=290)
callout(canvas, "[RETAINED] door_belt", "Doorway feed (re-height for dock)", project(np.array([1.3, -1.5, 0.25])), (70, 160), (0, 255, 120), box_w=310)
callout(canvas, "[CANDIDATE 1] corner_transfer", "REMOVE: 90 deg rotary deck (matches left mark)", project(np.array([1.3, 0.0, 0.25])), (330, 80), (0, 80, 255), box_w=360)
callout(canvas, "[CANDIDATE 2] longitudinal_buffer", "REMOVE: Buffer & folding bridge (right mark)", project(np.array([2.8, 0.0, 0.35])), (720, 110), (255, 50, 255), box_w=360)
callout(canvas, "[RETAINED] bellybot_dock_stack", "Mobile robot with tilting tray", project(np.array([4.2, 0.0, 0.45])), (920, 520), (0, 255, 255), box_w=300)

# Critical boundary markers:
# 1. Door belt to Corner transfer junction:
pt_junc = project(np.array([1.3, -0.512, 0.30]))
cv2.drawMarker(canvas, pt_junc, (0, 255, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
cv2.putText(canvas, "Junction Y=-0.512m", (pt_junc[0] - 80, pt_junc[1] + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)

# 2. Buffer bridge hinge to robot tray lip:
pt_tail = project(np.array([3.646, 0.0, 0.37]))
cv2.drawMarker(canvas, pt_tail, (0, 255, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
cv2.putText(canvas, "Dock Lip X=3.65m", (pt_tail[0] - 50, pt_tail[1] + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)

# Header banner
cv2.rectangle(canvas, (0, 0), (width, 50), (12, 14, 18), -1)
cv2.line(canvas, (0, 50), (width, 50), (0, 180, 255), 2)
cv2.putText(canvas, "BELLYBOT SYSTEM IDENTIFICATION: CAMERA-MATCHED MODEL OVERLAY", (24, 34), cv2.FONT_HERSHEY_DUPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)

cv2.imwrite('scratch/removal_candidates_labelled.png', canvas)
print('Saved scratch/removal_candidates_labelled.png')

# Now create 3-panel comparison:
# Panel 1: Marked reference (1280x720)
# Panel 2: Clean reference (1280x720)
# Panel 3: Labelled overlay (1280x720)
# Subsample each to 640x360 and stack horizontally or 2x2
p1 = cv2.resize(marked, (640, 360))
p2 = cv2.resize(clean, (640, 360))
p3 = cv2.resize(canvas, (640, 360))

# Add title to p1, p2, p3
cv2.putText(p1, "(A) Reference 1: Marked Ground Truth", (15, 30), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 0, 255), 2)
cv2.putText(p2, "(B) Reference 2: Clean Simulation Scene", (15, 30), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 0), 2)
cv2.putText(p3, "(C) Model ID & Removal Boundary Map", (15, 30), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 220, 255), 2)

top_row = np.hstack([p1, p2])
# Bottom row: p3 and a legend/metrics card
p4 = np.zeros((360, 640, 3), dtype=np.uint8)
p4[:] = (18, 22, 28)
cv2.putText(p4, "REMOVAL CONFIRMATION SUMMARY", (20, 35), cv2.FONT_HERSHEY_DUPLEX, 0.65, (255, 255, 255), 2)
cv2.line(p4, (20, 45), (620, 45), (100, 100, 100), 1)

lines = [
    ("Candidate 1 [RED]:", "corner_transfer (rotary deck + telescoping legs)"),
    ("  -> Matches:", "Left-hand red loop at X=1.30m, Y=0.0m"),
    ("Candidate 2 [MAGENTA]:", "longitudinal_buffer (conveyor + folding bridge)"),
    ("  -> Matches:", "Right-hand diagonal stroke X=1.80m..3.65m"),
    ("Retained Fixture 1 [GREEN]:", "door_belt (doorway sill, Y=-2.59m..-0.51m)"),
    ("  -> Clearance:", "Does NOT overlap marked removal; kept for dock"),
    ("Retained Fixture 2 [CYAN]:", "belt_loader (external apron feed, Y < -2.6m)"),
    ("Retained Robot [YELLOW]:", "bellybot_dock_stack (100% mechanism retained)"),
]
y_text = 75
for k, v in lines:
    cv2.putText(p4, k, (20, y_text), cv2.FONT_HERSHEY_DUPLEX, 0.45, (0, 220, 255) if "Candidate" in k or "Retained" in k else (200, 200, 200), 1)
    cv2.putText(p4, v, (230, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255) if "Matches" in k else (180, 240, 180) if "Clearance" in k else (220, 220, 220), 1)
    y_text += 32

bottom_row = np.hstack([p3, p4])
comp_4panel = np.vstack([top_row, bottom_row])
cv2.imwrite('scratch/removal_comparison_3panel.png', comp_4panel)
print('Saved scratch/removal_comparison_3panel.png')
