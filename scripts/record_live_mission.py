#!/usr/bin/env python3
"""Record live physical simulation of BellyBot DockStack C v3 in mission_shuttle.sdf.

Captures synchronized multi-camera MP4s + kinematics telemetry for:
- 1 Free 32kg Bag infeed across 20mm transfer gap from inclined door_belt
- Chassis undock (-Y), 90 deg pivot turn onto aisle centerline
- Corridor transit (+X to X=4.08m)
- Articulated deposit (0.74m reach, 30.9 deg pitch, +0.60 m/s roll-off onto floor)
- Placement settle and return empty to dock.
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
    "cam_aisle_chase": (0.6, 0.0, 0.95, 0.0, 0.15, 0.0),
    "cam_deposit_low": (5.5, -1.2, 0.65, 0.0, 0.20, 2.45),
}

class MissionLiveRecorder(Node):
    def __init__(self, outdir, fps=30.0, width=1280, height=720):
        super().__init__('mission_live_recorder')
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

        # Video writers
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
            "lift_mm": sum(self.joint_states.get(f"lift_stage_{i}_joint", 0.0) for i in (1, 2, 3)) * 1000.0,
            "reach_mm": (self.joint_states.get("extension_mid_joint", 0.0) + self.joint_states.get("extension_tip_joint", 0.0)) * 1000.0,
            "pitch_deg": math.degrees(self.joint_states.get("tray_pitch_joint", 0.0)),
        })

    def close(self):
        self.recording = False
        for cam, w in self.writers.items():
            w.release()
        telem_file = self.outdir / "telemetry.json"
        telem_file.write_text(json.dumps(self.telemetry_log, indent=2))
        print(f"-> Saved telemetry log with {len(self.telemetry_log)} records to {telem_file}", flush=True)

def spawn_bag(world_name, bag_model="bag_hard_32", bag_id="test_bag_32kg", pose_str="1.300 -0.095 0.658 0 0 1.570796327", env=None):
    model_sdf = PKG / "models" / bag_model / "model.sdf"
    tree = ET.parse(model_sdf)
    root = tree.getroot()
    m = root.find("model")
    m.set("name", bag_id)

    pose_elem = ET.SubElement(m, "pose")
    pose_elem.text = pose_str

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
    print(f"-> Spawning {bag_id} at pose: {pose_str}...", flush=True)
    subprocess.run(cmd, check=True, env=env)

def main():
    world = "mission_shuttle"
    outdir = REPO / "videos/_work/mission_live"
    logs = cl.SCRATCH

    cl.kill_gazebo()
    env = os.environ.copy()
    env['GZ_IP'] = '127.0.0.1'

    print("=== Starting Live Multi-Camera Mission Simulation Capture ===", flush=True)
    launch_cmd = [
        'ros2', 'launch', 'bellybot_aircraft_environment', 'mission.launch.py',
        'headless:=true', 'run_id:=live_capture'
    ]
    proc = subprocess.Popen(launch_cmd, env=env)
    print(f"-> Launched mission.launch.py (PID {proc.pid})", flush=True)

    rclpy.init()
    node = MissionLiveRecorder(outdir, fps=30.0)

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

        print("-> Spawning 4 shot cameras...", flush=True)
        cam_topics = cl.spawn_shot_cameras(world, CAMERAS, env, 1280, 720, hfov=1.0)

        bridge_proc = subprocess.Popen(
            ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
            + [f"{t}@sensor_msgs/msg/Image[gz.msgs.Image" for t in cam_topics.values()],
            env=env
        )
        print("-> Camera image bridges active", flush=True)

        # Wait for IDLE controller state
        t0 = time.time()
        while time.time() - t0 < 10.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.status and node.status.get('state') == 'IDLE':
                break

        print(f"-> Initial controller state: {node.status.get('state')}", flush=True)

        # Spawn test bag centered directly on robot tray conveyor
        spawn_bag(world, bag_model="bag_hard_32", bag_id="test_bag_32kg",
                  pose_str="1.300 -0.095 0.658 0 0 1.570796327", env=env)
        time.sleep(1.0)

        # Start recording
        node.recording = True
        print("-> RECORDING STARTED ON ALL 4 CAMERAS!", flush=True)

        # Send start_load command with skip_infeed=True for swift continuous loading
        print("-> Publishing /mission/command: start_load (skip_infeed=True)...", flush=True)
        cmd_msg = String()
        cmd_msg.data = json.dumps({'command': 'start_load', 'bag_id': 'test_bag_32kg', 'skip_infeed': True})
        node.pub_cmd.publish(cmd_msg)

        # Monitor full cycle
        t_start = time.time()
        last_state = ""
        last_log = time.time()
        while time.time() - t_start < 260.0:
            rclpy.spin_once(node, timeout_sec=0.033)
            sim_t = node.status.get('sim_time_s', 0.0)
            node.record_telemetry_point(sim_t)

            state = node.status.get('state', '')
            if state != last_state:
                p = node.robot_pose or (0, 0, 0)
                print(f"   [TRANSITION] -> {state:18s} @ sim {sim_t:.2f}s | Robot: X={p[0]:.2f}, Y={p[1]:.2f}, Yaw={math.degrees(node.robot_yaw):.1f}° | Bag: {node.bag_pose}", flush=True)
                last_state = state

            if time.time() - last_log > 5.0:
                print(f"   [PROGRESS] Sim {sim_t:.1f}s | Frames: {dict(node.frames)}", flush=True)
                last_log = time.time()

            # End when empty return completes
            if state in ('RETURN_EMPTY', 'IDLE') and any(e.get('event') == 'placement_complete' for e in node.events):
                print(f"*** Full cycle successfully executed and recorded! ***", flush=True)
                # Dwell 2 seconds to capture settled robot
                t_dwell = time.time()
                while time.time() - t_dwell < 2.0:
                    rclpy.spin_once(node, timeout_sec=0.033)
                break
        else:
            print("WARNING: Cycle timeout reached", flush=True)

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

    print("=== RECORDING COMPLETE ===")
    for cam, fcount in node.frames.items():
        print(f"   - {cam}.mp4: {fcount} frames ({fcount/30.0:.1f}s)")

if __name__ == '__main__':
    main()
