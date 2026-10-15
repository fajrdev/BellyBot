#!/usr/bin/env python3
"""Record live physical simulation of Reverse Discharge (Unloading) in mission_shuttle.sdf.

Captures synchronized multi-camera MP4s + kinematics telemetry for:
- Bag on robot tray at corridor
- Loaded return to doorway dock [1.300, -0.095, 0.003m, Yaw=+1.5708]
- Reverse threshold discharge (tray=-0.40, door=-0.40, loader=-0.40 m/s)
- Luggage descending the 5.48 deg inclined door_belt past optical plane Y < -2.60m onto apron loader.
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
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, JointState
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge

REPO = Path("/home/fajrda/sutd-hackathon")
sys.path.insert(0, str(REPO / "scripts"))
import capture_lib as cl

PKG = REPO / "BellyBot_Aircraft_Environment_v2/ros2/bellybot_aircraft_environment"

CAMERAS = {
    "cam_wide_iso": (5.5, -4.5, 2.8, 0.0, 0.42, 1.85),
    "cam_dock_close": (1.3, -1.8, 1.1, 0.0, 0.35, 1.5708),
    "cam_reverse_exit": (1.3, -4.2, 0.8, 0.0, 0.25, 1.5708),
}

class DischargeLiveRecorder(Node):
    def __init__(self, outdir, fps=30.0, width=1280, height=720):
        super().__init__('discharge_live_recorder')
        self.bridge = CvBridge()
        self.fps = fps
        self.width = width
        self.height = height
        self.outdir = Path(outdir)
        self.outdir.mkdir(parents=True, exist_ok=True)

        self.recording = False
        self.writers = {}
        self.frames = {cam: 0 for cam in CAMERAS}
        self.telemetry_log = []

        self.status = {}
        self.events = []
        self.bag_pose = None
        self.robot_pose = None
        self.robot_yaw = 0.0
        self.joint_states = {}

        for cam in CAMERAS:
            path = str(self.outdir / f"{cam}.mp4")
            self.writers[cam] = cv2.VideoWriter(
                path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
            )
            self.create_subscription(
                Image, f"/shot/{cam}/image",
                lambda msg, c=cam: self.on_image(c, msg), 10
            )

        self.create_subscription(String, '/mission/status', self.on_status, 10)
        self.create_subscription(String, '/mission/events', self.on_event, 10)
        self.create_subscription(TFMessage, '/mission/world_poses', self.on_world_poses, 10)
        self.create_subscription(TFMessage, '/aircraft/robot_pose', self.on_robot_pose, 10)
        self.create_subscription(JointState, '/aircraft/robot_joints', self.on_joints, 10)

        self.pub_cmd = self.create_publisher(String, '/mission/command', 10)

    def on_image(self, cam, msg):
        if not self.recording:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        self.writers[cam].write(frame)
        self.frames[cam] += 1

    def on_joints(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joint_states[n] = p

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except Exception:
            pass

    def on_event(self, msg):
        try:
            evt = json.loads(msg.data)
            self.events.append(evt)
            print(f"   [EVENT] {evt.get('event')} @ {evt.get('sim_time_s', 0):.1f}s: {evt.get('details')}", flush=True)
        except Exception:
            pass

    def on_robot_pose(self, msg):
        for tr in msg.transforms:
            if 'bellybot_dock_stack' in tr.child_frame_id:
                p = tr.transform.translation
                q = tr.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self.robot_pose = (p.x, p.y, p.z)
                self.robot_yaw = yaw

    def on_world_poses(self, msg):
        for tr in msg.transforms:
            if 'bag' in tr.child_frame_id:
                p = tr.transform.translation
                self.bag_pose = (p.x, p.y, p.z)

    def record_telemetry_point(self, sim_t):
        if not self.recording:
            return
        self.telemetry_log.append({
            "t": sim_t,
            "state": self.status.get("state", ""),
            "robot_x": self.robot_pose[0] if self.robot_pose else 0.0,
            "robot_y": self.robot_pose[1] if self.robot_pose else 0.0,
            "robot_z": self.robot_pose[2] if self.robot_pose else 0.0,
            "robot_yaw_deg": math.degrees(self.robot_yaw),
            "bag_x": self.bag_pose[0] if self.bag_pose else None,
            "bag_y": self.bag_pose[1] if self.bag_pose else None,
            "bag_z": self.bag_pose[2] if self.bag_pose else None,
        })

    def close(self):
        self.recording = False
        for cam, w in self.writers.items():
            w.release()
        telem_file = self.outdir / "telemetry.json"
        telem_file.write_text(json.dumps(self.telemetry_log, indent=2))

def spawn_tray_bag(world_name, bag_model="bag_hard_32", bag_id="tray_discharge_bag", x=4.10, y=0.00, env=None):
    model_sdf = PKG / "models" / bag_model / "model.sdf"
    tree = ET.parse(model_sdf)
    root = tree.getroot()
    m = root.find("model")
    m.set("name", bag_id)
    z_spawn = 0.660
    pose_elem = ET.SubElement(m, "pose")
    pose_elem.text = f"{x} {y} {z_spawn:.3f} 0 0 0"

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
    print(f"-> Spawning {bag_id} on robot tray at X={x}, Y={y}, Z={z_spawn:.3f}m...", flush=True)
    subprocess.run(cmd, check=True, env=env)

def main():
    world = "mission_shuttle"
    outdir = REPO / "videos/_work/discharge_live"
    logs = cl.SCRATCH

    cl.kill_gazebo()
    env = os.environ.copy()
    env['GZ_IP'] = '127.0.0.1'

    print("=== Starting Live Reverse Discharge Simulation Capture ===", flush=True)
    launch_cmd = [
        'ros2', 'launch', 'bellybot_aircraft_environment', 'mission.launch.py',
        'headless:=true', 'run_id:=live_discharge'
    ]
    proc = subprocess.Popen(launch_cmd, env=env)

    rclpy.init()
    node = DischargeLiveRecorder(outdir, fps=30.0)
    bridge_proc = None

    try:
        time.sleep(4)
        subprocess.run([
            'gz', 'service', '-s', f'/world/{world}/control',
            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', 'pause: false'
        ], check=True, env=env)

        cam_topics = cl.spawn_shot_cameras(world, CAMERAS, env, 1280, 720, hfov=1.0)
        bridge_proc = subprocess.Popen(
            ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
            + [f"{t}@sensor_msgs/msg/Image[gz.msgs.Image" for t in cam_topics.values()],
            env=env
        )

        t0 = time.time()
        while time.time() - t0 < 10.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.status and node.status.get('state') == 'IDLE':
                break

        # Spawn bag on tray at X=4.10, Y=0.00
        spawn_tray_bag(world, bag_model="bag_hard_32", bag_id="tray_discharge_bag", x=4.10, y=0.00, env=env)
        time.sleep(1.0)

        node.recording = True
        print("-> RECORDING STARTED ON DISCHARGE CAMERAS!", flush=True)

        cmd_msg = String()
        cmd_msg.data = json.dumps({'command': 'start_unload', 'bag_id': 'tray_discharge_bag'})
        node.pub_cmd.publish(cmd_msg)

        t_start = time.time()
        last_state = ""
        while time.time() - t_start < 200.0:
            rclpy.spin_once(node, timeout_sec=0.033)
            sim_t = node.status.get('sim_time_s', 0.0)
            node.record_telemetry_point(sim_t)

            state = node.status.get('state', '')
            if state != last_state:
                p = node.robot_pose or (0, 0, 0)
                print(f"   [TRANSITION] -> {state:18s} @ sim {sim_t:.2f}s | Bag: {node.bag_pose}", flush=True)
                last_state = state

            if state == 'IDLE' and any(e.get('event') == 'discharge_complete' for e in node.events):
                print(f"*** Discharge cycle successfully completed! ***", flush=True)
                t_dwell = time.time()
                while time.time() - t_dwell < 2.0:
                    rclpy.spin_once(node, timeout_sec=0.033)
                break
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
        if bridge_proc:
            bridge_proc.terminate()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        subprocess.run(['pkill', '-9', '-f', 'gz sim'], stderr=subprocess.DEVNULL)
        subprocess.run(['killall', '-9', 'gz'], stderr=subprocess.DEVNULL)

    print("=== DISCHARGE RECORDING COMPLETE ===")

if __name__ == '__main__':
    main()
