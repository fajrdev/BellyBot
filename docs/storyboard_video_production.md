# Storyboard Video Production — pipeline, findings, deviations

How the 2-minute submission video is produced from the ROS 2 / Gazebo simulation,
plus the constraints discovered while capturing Segment 2.

## Pipeline (headless render -> ROS -> encode)

No `ffmpeg` is installed on this host and `sudo` needs a password, so the capture
path avoids both:

1. `gz sim -s --headless-rendering <world>.sdf` renders SDF `camera` sensors offscreen.
2. `ros_gz_bridge` bridges each `/shot/<name>/image` (`gz.msgs.Image` -> `sensor_msgs/Image`).
3. A Python recorder (`cv_bridge` + OpenCV `VideoWriter`) writes one MP4 per camera.
4. `scripts/render_segment.py` composites the per-camera clips into the segment clip
   with title/footer bands and per-cut captions (OpenCV, no ffmpeg).

Authoring `ffmpeg`'s `x11grab`/`drawtext` recipe from the storyboard is not possible here;
GStreamer (`ximagesrc`+`x264enc`+`textoverlay`) is available as a GUI-screen fallback.

### Mandatory host setting

`GZ_IP=127.0.0.1`. The LAN interface drops multicast ("Network is unreachable"), which
stalls gz-transport discovery; binding to loopback fixes it. `capture_lib.ros_env()` sets it.

### Commands

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 scripts/capture_segment_02.py --segment 2 --width 1280 --height 720 \
    --outdir videos/_work/seg02
python3 scripts/render_segment.py <plan>.json
```

## Findings that change the storyboard's camera plan

1. **The storyboard camera poses do not frame the action.** The world files define only a
   single `<camera name="inspection">`; the 15 named shot cameras exist only in the docs.
   The documented `<gui><camera>` poses use the GUI *orbit* controller (look-at), so the
   raw yaw/pitch does not correspond to the sensor's +X forward axis. Dropped in verbatim,
   they produce mostly empty white frames.
2. **The hold interior renders near-white**, so wide shots wash out. Only close framing on
   the robot / conveyor / payload yields contrast.
3. **The payload is a white mesh with a forced white material** (`bag_hard_32`: ambient and
   diffuse `1 1 1`), i.e. invisible against the hold. `capture_lib.spawn_tracked_bag()`
   spawns `tracked_bag` with a solid coloured **box visual** (physics collision unchanged)
   so the suitcase reads on camera.
4. **The `-y` side views are obstructed** at working height; near-vertical top-down views
   over each conveyor station reliably show the belt and payload (verified by scanning for
   payload-coloured pixels across the clips).

## Segment status

| # | Segment | Status |
|---|---|---|
| 1 | Access & Ingress | not started — needs `entry_resolved` cameras (roof occludes top-down; use the door/ramp angles) |
| 2 | Continuous Infeed Transfer | **captured + rendered** (`videos/segment_02_continuous_infeed_transfer.mp4`, 30 s, 1920x1080) |
| 3 | Zero-Drop Floor Placement | not started (`/aircraft/pose_target [0,0,0.86,0.5397]`) |
| 4 | 2nd-Layer Stacking | not started (`/aircraft/pose_target [0.13,0.50,0.40,0.0]`) |
| 5 | Specs & Safety | not started (mostly HUD/graphics + auto-egress) |

## Segment 2 camera deviations (storyboard pose -> used pose)

The storyboard names are kept; the poses are the verified working equivalents
(`videos/_work/seg02/*.mp4`).

| Storyboard camera | Storyboard pose (x y z r p y) | Used pose | Reason |
|---|---|---|---|
| camera_corner_lift | 0.40 -1.80 1.25 0 0.42 1.12 | 1.30 -0.35 1.55 0 1.25 1.5708 | original framed only white |
| camera_buffer_travel | 2.10 -1.10 1.10 0 0.35 1.5708 | 2.10 -0.35 1.55 0 1.25 1.5708 | original washes out; keep station x |
| (new) camera_buffer_mid | — | 3.30 -0.35 1.55 0 1.25 1.5708 | covers mid-buffer transit |
| camera_tray_handoff | 4.40 -0.85 0.95 0 0.30 2.45 | 4.20 -0.35 1.55 0 1.25 1.5708 | original framed white; roof-side obstructed |

## Known caveats

- Throughput >= 12 bags/min is **not** demonstrated; the controller runs a slow
  commissioning sequence. HUD text is limited to verified geometry/timings.
- Segment 3 header in the storyboard says 0:45-1:15 while the index says 0:55-1:15; the
  index (20 s) is treated as authoritative.
- The final submission filename convention (`2_Animation_<Team>_<Lead>.MP4`) and the
  5-8 page report are still outstanding.
