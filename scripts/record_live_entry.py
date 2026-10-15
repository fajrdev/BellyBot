#!/usr/bin/env python3
"""Record live simulation of Entry & Deployment (Segment 02) in entry_resolved.sdf.

Captures synchronized multi-camera MP4s of BellyBot:
- Starting on the external entry deck [1.3, -3.29, 0.183m]
- Driving forward along ramp towards door platform
- Pivoting 90 degrees on the deck
- Traversing through the doorway sill into the aircraft hold.
"""

import os
import sys
import time
import json
import math
import subprocess
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge

REPO = Path("/home/fajrda/sutd-hackathon")
sys.path.insert(0, str(REPO / "scripts"))
import capture_lib as cl

PKG = REPO / "BellyBot_Aircraft_Environment_v2/ros2/bellybot_aircraft_environment"

CAMERAS = {
    "cam_entry_deck": (3.2, -4.8, 1.8, 0.0, 0.35, 2.15),
    "cam_entry_sill": (1.30, -0.40, 2.20, 0.0, 1.5708, 0.0),
}

class EntryLiveRecorder(Node):
    def __init__(self, outdir, fps=30.0, width=1920, height=1080):
        super().__init__('entry_live_recorder')
        self.bridge = CvBridge()
        self.fps = fps
        self.width = width
        self.height = height
        self.outdir = Path(outdir)
        self.outdir.mkdir(parents=True, exist_ok=True)

        self.recording = False
        self.writers = {}
        self.frames = {cam: 0 for cam in CAMERAS}

        for cam in CAMERAS:
            path = str(self.outdir / f"{cam}.mp4")
            self.writers[cam] = cv2.VideoWriter(
                path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
            )
            self.create_subscription(
                Image, f"/shot/{cam}/image",
                lambda msg, c=cam: self.on_image(c, msg), 10
            )

        self.status = {}
        self.robot_pose = None
        self.sub_status = self.create_subscription(String, '/aircraft/status', self.on_status, 10)
        self.sub_pose = self.create_subscription(TFMessage, '/aircraft/robot_pose', self.on_robot_pose, 10)
        self.pub_entry = self.create_publisher(Empty, '/aircraft/start_entry', 10)

    def on_image(self, cam, msg):
        if not self.recording:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        self.writers[cam].write(frame)
        self.frames[cam] += 1

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except Exception:
            pass

    def on_robot_pose(self, msg):
        for tr in msg.transforms:
            if 'bellybot_dock_stack' in tr.child_frame_id:
                p = tr.transform.translation
                self.robot_pose = (p.x, p.y, p.z)

    def close(self):
        self.recording = False
        for cam, w in self.writers.items():
            w.release()

def main():
    world = "entry_resolved"
    outdir = REPO / "videos/_work/entry_live"
    logs = cl.SCRATCH

    cl.kill_gazebo()
    env = os.environ.copy()
    env['GZ_IP'] = '127.0.0.1'

    aircraft = cl.share_dir("bellybot_aircraft_environment")
    desc = cl.share_dir("bellybot_dock_stack_description")
    res_env = cl.resource_env(aircraft, desc)

    print("=== Starting Live Entry Simulation Capture ===", flush=True)
    sim = cl.run_bg(
        ["gz", "sim", "-s", "--headless-rendering", "-v", "3",
         str(aircraft / "worlds" / f"{world}.sdf")],
        res_env, logs / "entry_sim.log")

    bridge = cl.run_bg(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
         "--ros-args", "-p", f"config_file:={aircraft / 'config' / 'aircraft_bridge.yaml'}"],
        res_env, logs / "entry_bridge.log")

    ctrl = cl.run_bg(
        ["ros2", "run", "bellybot_aircraft_environment", "aircraft_controller.py",
         "--ros-args", "-p", "use_sim_time:=true", "-p", "mode:=entry"],
        res_env, logs / "entry_ctrl.log")

    rclpy.init()
    node = EntryLiveRecorder(outdir, fps=30.0)
    cam_bridge = None

    try:
        cl.wait_and_unpause(world, res_env, timeout=20.0)
        print("-> Spawning Entry cameras...", flush=True)
        cam_topics = cl.spawn_shot_cameras(world, CAMERAS, res_env, 1920, 1080, hfov=1.0)
        cam_bridge = cl.run_bg(
            ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
            + [f"{t}@sensor_msgs/msg/Image[gz.msgs.Image" for t in cam_topics.values()],
            res_env, logs / "entry_cambridge.log"
        )

        # Wait for controller ready
        t0 = time.time()
        while time.time() - t0 < 15.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.status and node.status.get('joint_feedback_fresh'):
                break

        print(f"-> Controller ready. Triggering entry...", flush=True)
        node.recording = True
        node.pub_entry.publish(Empty())

        # Record entry for 25s sim time
        t_start = time.time()
        last_phase = ""
        while time.time() - t_start < 45.0:
            rclpy.spin_once(node, timeout_sec=0.033)
            phase = node.status.get('entry', '')
            if phase != last_phase:
                print(f"   [ENTRY PHASE] -> {phase} @ {time.time()-t_start:.1f}s | Robot: {node.robot_pose}", flush=True)
                last_phase = phase
            if phase == 'DONE':
                print(f"*** Entry navigation complete! ***", flush=True)
                time.sleep(2.0)
                break
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
        cl.stop_proc(cam_bridge)
        cl.stop_proc(ctrl)
        cl.stop_proc(bridge)
        cl.stop_proc(sim)
        cl.kill_gazebo()

    print("=== ENTRY RECORDING COMPLETE ===")
    for cam, fcount in node.frames.items():
        print(f"   - {cam}.mp4: {fcount} frames ({fcount/30.0:.1f}s)")

if __name__ == '__main__':
    main()
