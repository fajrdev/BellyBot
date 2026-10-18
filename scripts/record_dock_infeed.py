#!/usr/bin/env python3
"""Record pristine physical infeed for Scene 02 Cut 3 from cam_dock_close.

Spawns test_bag_32kg on the inclined door belt (y_spawn=-1.60).
BellyBot is docked at [1.300, -0.095, 0.003, Yaw=1.5708].
Matched conveyors (door=0.35 m/s, tray=0.35 m/s) transfer the bag across
the 20mm interface gap until it sits squarely centered on the receiving tray.
Records pure 1920x1080 frames to videos/_work/dock_infeed/cam_dock_close.mp4.
"""

import os
import sys
import time
import json
import math
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge

REPO = Path("/home/fajrda/sutd-hackathon")
sys.path.insert(0, str(REPO / "scripts"))
import capture_lib as cl

PKG = REPO / "BellyBot_Aircraft_Environment_v2/ros2/bellybot_aircraft_environment"

CAMERA = {
    "cam_dock_close": (1.3, -1.8, 1.1, 0.0, 0.35, 1.5708),
}

class DockInfeedRecorder(Node):
    def __init__(self, outdir, fps=30.0, width=1920, height=1080):
        super().__init__('dock_infeed_recorder')
        self.bridge = CvBridge()
        self.fps = fps
        self.width = width
        self.height = height
        self.outdir = Path(outdir)
        self.outdir.mkdir(parents=True, exist_ok=True)

        self.recording = False
        self.frame_count = 0
        self.bag_pose = None
        path = str(self.outdir / "cam_dock_close.mp4")
        self.writer = cv2.VideoWriter(
            path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
        )

        self.create_subscription(
            Image, "/shot/cam_dock_close/image", self.on_image, 10
        )
        self.create_subscription(String, '/mission/status', self.on_status, 10)
        self.create_subscription(String, '/mission/events', self.on_event, 10)
        self.create_subscription(TFMessage, '/mission/world_poses', self.on_world_poses, 10)
        self.pub_cmd = self.create_publisher(String, '/mission/command', 10)
        self.status = {}

    def on_world_poses(self, msg):
        for tr in msg.transforms:
            if 'bag' in tr.child_frame_id:
                p = tr.transform.translation
                self.bag_pose = (p.x, p.y, p.z)

    def on_event(self, msg):
        try:
            evt = json.loads(msg.data)
            print(f"   [EVENT] {evt.get('event')} @ {evt.get('sim_time_s', 0):.1f}s: {evt.get('details')}", flush=True)
        except Exception:
            pass

    def on_image(self, msg):
        if not self.recording:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        self.writer.write(frame)
        self.frame_count += 1

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except Exception:
            pass

    def close(self):
        self.writer.release()
        print(f"-> Saved {self.frame_count} frames to {self.outdir / 'cam_dock_close.mp4'}", flush=True)

def spawn_bag(world_name, bag_model="bag_hard_32", bag_id="test_bag_32kg", y_spawn=-1.60, env=None):
    model_sdf = PKG / "models" / bag_model / "model.sdf"
    tree = ET.parse(model_sdf)
    root = tree.getroot()
    m = root.find("model")
    m.set("name", bag_id)

    size = [float(x) for x in m.findtext(".//collision/geometry/box/size").split()]
    z_spawn = 0.398 + size[2] / 2.0 + 0.005
    pose_elem = ET.SubElement(m, "pose")
    pose_elem.text = f"1.300 {y_spawn} {z_spawn:.3f} 0 -0.0956 1.570796327"

    pl = ET.SubElement(m, "plugin", filename="gz-sim-pose-publisher-system", name="gz::sim::systems::PosePublisher")
    for k, v in [
        ('topic', f'/world/{world_name}/dynamic_pose/info'),
        ('publish_model_pose', 'true'),
        ('publish_link_pose', 'true'),
        ('publish_visual_pose', 'false'),
        ('publish_collision_pose', 'false'),
        ('use_pose_vector_msg', 'true'),
        ('update_frequency', '50')
    ]:
        ET.SubElement(pl, k).text = v

    for u in m.findall(".//mesh/uri"):
        if u.text.startswith("model://"):
            u.text = (PKG / "models" / u.text[8:]).resolve().as_uri()

    req = f"sdf: {json.dumps(ET.tostring(root, encoding='unicode'))} allow_renaming: false"
    cmd = [
        'gz', 'service', '-s', f'/world/{world_name}/create',
        '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '4000', '--req', req
    ]
    print(f"-> Spawning {bag_id} at Y={y_spawn}, Z={z_spawn:.3f}m on door belt...", flush=True)
    subprocess.run(cmd, check=True, env=env)

def main():
    world = "mission_shuttle"
    outdir = REPO / "videos/_work/dock_infeed"

    cl.kill_gazebo()
    env = os.environ.copy()
    env['GZ_IP'] = '127.0.0.1'

    print("=== Recording Live Dock Infeed Clip ===", flush=True)
    launch_cmd = [
        'ros2', 'launch', 'bellybot_aircraft_environment', 'mission.launch.py',
        'headless:=true', 'run_id:=infeed_capture'
    ]
    proc = subprocess.Popen(launch_cmd, env=env)
    print(f"-> Launched mission.launch.py (PID {proc.pid})", flush=True)

    rclpy.init()
    node = DockInfeedRecorder(outdir, fps=30.0, width=1920, height=1080)
    bridge_proc = None

    try:
        print("-> Waiting 4s for Gazebo initialization...", flush=True)
        time.sleep(4)

        print("-> Unpausing Gazebo Sim...", flush=True)
        subprocess.run([
            'gz', 'service', '-s', f'/world/{world}/control',
            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', 'pause: false'
        ], check=True, env=env)

        print("-> Spawning cam_dock_close...", flush=True)
        cam_topics = cl.spawn_shot_cameras(world, CAMERA, env, 1920, 1080, hfov=1.0)

        bridge_proc = subprocess.Popen(
            ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
            + [f"{t}@sensor_msgs/msg/Image[gz.msgs.Image" for t in cam_topics.values()],
            env=env
        )
        print("-> Camera image bridge active", flush=True)

        # Wait for IDLE controller state
        t0 = time.time()
        while time.time() - t0 < 10.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.status and node.status.get('state') == 'IDLE':
                break

        print(f"-> Initial controller state: {node.status.get('state')}", flush=True)

        # Spawn bag on inclined door_belt
        spawn_bag(world, bag_model="bag_hard_32", bag_id="test_bag_32kg", y_spawn=-1.60, env=env)
        time.sleep(1.0)

        # Start recording
        node.recording = True
        print("-> RECORDING STARTED!", flush=True)

        # Send start_load command WITHOUT skip_infeed (triggers natural belt infeed)
        print("-> Publishing /mission/command: start_load...", flush=True)
        cmd_msg = String()
        cmd_msg.data = json.dumps({'command': 'start_load', 'bag_id': 'test_bag_32kg'})
        node.pub_cmd.publish(cmd_msg)

        # Record until infeed completes and dwells
        t_start = time.time()
        transport_frame = None
        while time.time() - t_start < 30.0:
            rclpy.spin_once(node, timeout_sec=0.033)
            if node.status.get('state') == 'TRANSPORT' and transport_frame is None:
                transport_frame = node.frame_count
                print(f"-> State -> TRANSPORT at frame {transport_frame}! Recording dwell...", flush=True)
            if (transport_frame is not None and node.frame_count >= transport_frame + 30) or node.frame_count >= 250:
                print(f"-> Collected {node.frame_count} frames! Infeed transfer complete.", flush=True)
                break

    finally:
        node.close()
        if bridge_proc:
            bridge_proc.terminate()
        proc.terminate()
        cl.kill_gazebo()
        rclpy.shutdown()

    print("=== Infeed Recording Successfully Finished ===")

if __name__ == '__main__':
    main()
