#!/usr/bin/env python3
"""Run Option A with the wide isometric camera and save progress snapshots of each tier.
"""
import os
import sys
import time
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import json
import subprocess
import cv2

REPO = Path("/home/fajrda/sutd-hackathon")
sys.path.insert(0, str(REPO / "scripts"))
import capture_lib as cl

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge

def smooth(u):
    return 10 * u**3 - 15 * u**4 + 6 * u**5

# Option A: 2 Columns (Port: Y=+0.27, Starboard: Y=-0.27) x 3 Tiers (Floor, Mid, Top)
BAG_SPECS_OPTION_A = [
    # Tier 1 (Floor)
    {"type": "bag_hard_32", "name": "bag_1_t1_port", "color": "0.95 0.45 0.10 1",
     "label": "Bag #1 [Tier 1 Port]", "slot": [0.00,  0.22, 0.65, 0.10]},
    {"type": "bag_hard_26", "name": "bag_2_t1_stbd", "color": "0.20 0.55 0.90 1",
     "label": "Bag #2 [Tier 1 Starboard]", "slot": [0.00, -0.22, 0.65, 0.10]},
    # Tier 2 (Middle, Lift = +110mm)
    {"type": "bag_hard_20", "name": "bag_3_t2_port", "color": "0.85 0.20 0.35 1",
     "label": "Bag #3 [Tier 2 Port]", "slot": [0.11,  0.22, 0.60, 0.08]},
    {"type": "bag_fragile_20", "name": "bag_4_t2_stbd", "color": "0.95 0.85 0.15 1",
     "label": "Bag #4 [Tier 2 Starboard]", "slot": [0.11, -0.22, 0.60, 0.08]},
    # Tier 3 (Top, Lift = +175mm)
    {"type": "bag_duffel_23", "name": "bag_5_t3_port", "color": "0.25 0.75 0.45 1",
     "label": "Bag #5 [Tier 3 Port]", "slot": [0.175,  0.22, 0.56, 0.06]},
    {"type": "bag_hard_32", "name": "bag_6_t3_stbd", "color": "0.65 0.30 0.80 1",
     "label": "Bag #6 [Tier 3 Starboard]", "slot": [0.175, -0.22, 0.56, 0.06]},
]

CAMERAS = {
    "camera_wide_iso": (6.601, -8.090, 6.186, 0.0, 0.6109, 1.9722),
    "camera_macro_stack": (5.75, -2.15, 2.30, 0.0, 0.42, 2.25),
}

def spawn_bag(world, spec, env, aircraft, x=3.70, y=0.0, z=0.66):
    model_sdf = aircraft / "models" / spec["type"] / "model.sdf"
    root = ET.parse(model_sdf).getroot()
    m = root.find("model")
    m.set("name", spec["name"])
    size = [float(v) for v in m.findtext(".//collision/geometry/box/size").split()]
    
    pose = m.find("pose")
    if pose is None:
        pose = ET.SubElement(m, "pose")
    pose.text = f"{x} {y} {z} 0 0 0"
    
    vis = m.find(".//visual")
    geo = vis.find("geometry")
    for ch in list(geo):
        geo.remove(ch)
    box = ET.SubElement(geo, "box")
    ET.SubElement(box, "size").text = f"{size[0]} {size[1]} {size[2]}"
    mat = ET.SubElement(vis, "material")
    diff = ET.SubElement(mat, "diffuse")
    diff.text = spec["color"]
    amb = ET.SubElement(mat, "ambient")
    amb.text = "0.3 0.3 0.3 1"
    
    col = m.find(".//collision")
    surf = col.find("surface")
    if surf is None:
        surf = ET.SubElement(col, "surface")
    fric = surf.find("friction/ode")
    if fric is not None:
        fric.find("mu").text = "0.85"
        fric.find("mu2").text = "0.85"
    bounce = surf.find("bounce")
    if bounce is not None:
        bounce.find("restitution_coefficient").text = "0.0"
            
    req = "sdf: " + json.dumps(ET.tostring(root, encoding="unicode")) + " allow_renaming: false"
    res = subprocess.run(
        ["gz", "service", "-s", f"/world/{world}/create",
         "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
         "--timeout", "4000", "--req", req],
        check=True, env=env, capture_output=True, text=True
    )
    return "data: true" in res.stdout

class OptionARecorderNode(Node):
    def __init__(self, outdir, fps=30.0, width=1280, height=720):
        super().__init__('option_a_recorder_node')
        self.bridge = CvBridge()
        self.fps = fps
        self.width = width
        self.height = height
        self.outdir = Path(outdir)
        self.outdir.mkdir(parents=True, exist_ok=True)
        
        self.writers = {}
        self.latest_frames = {}
        self.joint_states = {}
        self.recording = False
        
        for cam in CAMERAS:
            mp4_path = str(self.outdir / f"{cam}.mp4")
            self.writers[cam] = cv2.VideoWriter(
                mp4_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
            )
            self.create_subscription(
                Image, f"/shot/{cam}/image",
                lambda msg, c=cam: self.on_image(c, msg), 10
            )

        self.create_subscription(JointState, '/aircraft/robot_joints', self.on_joints, 10)

        # High-speed infeed actuation (0.75 m/s)
        self.pubs = {
            'corner_stage_1': self.create_publisher(Float64, '/aircraft/actuation/corner_stage_1', 10),
            'corner_stage_2': self.create_publisher(Float64, '/aircraft/actuation/corner_stage_2', 10),
            'corner_stage_3': self.create_publisher(Float64, '/aircraft/actuation/corner_stage_3', 10),
            'corner_angle': self.create_publisher(Float64, '/aircraft/actuation/corner_angle', 10),
            'buffer_stage_1': self.create_publisher(Float64, '/aircraft/actuation/buffer_stage_1', 10),
            'buffer_stage_2': self.create_publisher(Float64, '/aircraft/actuation/buffer_stage_2', 10),
            'bridge_angle': self.create_publisher(Float64, '/aircraft/actuation/bridge_angle', 10),
            'door_speed': self.create_publisher(Float64, '/aircraft/actuation/door_speed', 10),
            'corner_speed': self.create_publisher(Float64, '/aircraft/actuation/corner_speed', 10),
            'bridge_speed': self.create_publisher(Float64, '/aircraft/actuation/bridge_speed', 10),
            'buffer_speed': self.create_publisher(Float64, '/aircraft/actuation/buffer_speed', 10),
            'lift': self.create_publisher(Float64, '/aircraft/actuation/lift_stage_target', 10),
            'yaw': self.create_publisher(Float64, '/aircraft/actuation/yaw_target', 10),
            'extension': self.create_publisher(Float64, '/aircraft/actuation/extension_stage_target', 10),
            'pitch': self.create_publisher(Float64, '/aircraft/actuation/pitch_target', 10),
            'tray_belt_speed': self.create_publisher(Float64, '/aircraft/actuation/tray_belt_speed', 10),
            'tray_roller_speed': self.create_publisher(Float64, '/aircraft/actuation/tray_roller_speed', 10),
            'buffer_roller': self.create_publisher(Float64, '/aircraft/actuation/buffer_roller', 10),
            'bridge_roller': self.create_publisher(Float64, '/aircraft/actuation/bridge_roller', 10),
        }

    def on_image(self, cam, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if cv_img.shape[1] != self.width or cv_img.shape[0] != self.height:
            cv_img = cv2.resize(cv_img, (self.width, self.height))
        self.latest_frames[cam] = cv_img
        if self.recording:
            self.writers[cam].write(cv_img)

    def on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_states[name] = pos

    def set_system_posture(self, lift=0.0, yaw=0.0, reach=0.0, pitch=0.0, tray_belt=0.0, conveyor_speed=0.75):
        # Conveyor station docking targets
        for s in ['1', '2', '3']:
            m = Float64(); m.data = 0.198 / 3.0; self.pubs[f'corner_stage_{s}'].publish(m)
        m = Float64(); m.data = 1.5708; self.pubs['corner_angle'].publish(m)
        for s in ['1', '2']:
            m = Float64(); m.data = 0.130 / 2.0; self.pubs[f'buffer_stage_{s}'].publish(m)
        m = Float64(); m.data = 0.0; self.pubs['bridge_angle'].publish(m)

        # High-speed infeed conveyors (0.75 m/s)
        for k in ['door_speed', 'corner_speed', 'bridge_speed', 'buffer_speed']:
            m = Float64(); m.data = conveyor_speed; self.pubs[k].publish(m)
        m = Float64(); m.data = conveyor_speed / 0.011; self.pubs['buffer_roller'].publish(m)
        m = Float64(); m.data = conveyor_speed / 0.005; self.pubs['bridge_roller'].publish(m)

        # Active robot DOF
        m = Float64(); m.data = lift / 3.0; self.pubs['lift'].publish(m)
        m = Float64(); m.data = yaw; self.pubs['yaw'].publish(m)
        m = Float64(); m.data = reach / 2.0; self.pubs['extension'].publish(m)
        m = Float64(); m.data = pitch; self.pubs['pitch'].publish(m)
        m = Float64(); m.data = tray_belt; self.pubs['tray_belt_speed'].publish(m)
        m = Float64(); m.data = tray_belt / 0.01; self.pubs['tray_roller_speed'].publish(m)

    def close(self):
        for cam, w in self.writers.items():
            w.release()

def step(node, start, end, duration, fps=30):
    steps = int(duration * fps)
    dt = duration / max(1, steps)
    t0 = time.time()
    for i in range(steps + 1):
        u = smooth(i / max(1, steps))
        p = [s + (e - s) * u for s, e in zip(start, end)]
        node.set_system_posture(lift=p[0], yaw=p[1], reach=p[2], pitch=p[3], tray_belt=p[4])
        rclpy.spin_once(node, timeout_sec=0.002)
        tgt = t0 + (i + 1) * dt
        rem = tgt - time.time()
        if rem > 0:
            time.sleep(rem)

def main():
    world = "transfer_cutaway"
    aircraft = cl.share_dir("bellybot_aircraft_environment")
    desc = cl.share_dir("bellybot_dock_stack_description")
    env = cl.resource_env(aircraft, desc)
    logs = cl.SCRATCH
    outdir = REPO / "videos/_work/seg02_opt_a"

    cl.kill_gazebo()
    sim = cl.run_bg(
        ["gz", "sim", "-s", "--headless-rendering", "-v", "3",
         str(aircraft / "worlds" / f"{world}.sdf")],
        env, logs / "opt_a_rec_sim.log")

    bridge = cl.run_bg(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
         "-p", f"config_file:={aircraft / 'config' / 'aircraft_bridge.yaml'}"],
        env, logs / "opt_a_rec_bridge.log")

    rclpy.init()
    node = OptionARecorderNode(outdir)

    try:
        cl.wait_and_unpause(world, env)
        cam_topics = cl.spawn_shot_cameras(world, CAMERAS, env, 1280, 720, hfov=0.64)
        image_bridge = cl.run_bg(
            ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
            + [f"{t}@sensor_msgs/msg/Image[gz.msgs.Image" for t in cam_topics.values()],
            env, logs / "opt_a_rec_imgbridge.log"
        )

        stow = [0.0, 0.0, 0.0, 0.0, 0.0]
        node.set_system_posture(*stow)
        time.sleep(2.5)

        node.recording = True
        print("-> RECORDING STARTED (Option A: 2-Column x 3-Tier Stacking)...", flush=True)
        t_start = time.time()

        for idx, spec in enumerate(BAG_SPECS_OPTION_A):
            slot = spec["slot"]
            t_bag = time.time()

            # 1. Spawn on tray threshold with high-speed buffer running (x=3.70)
            spawn_bag(world, spec, env, aircraft, x=3.70, y=0.0, z=0.66)

            # 2. Crisp Infeed & Centering Datum (0.8s)
            intake = [0.0, 0.0, 0.0, 0.0, 0.65]
            step(node, stow, intake, duration=0.8)

            # 3. Precision Datum Settling Hold (0.2s) - ensures bag is fully centered & stationary
            settle = [0.0, 0.0, 0.0, 0.0, 0.0]
            step(node, intake, settle, duration=0.2)

            # 4. Controlled S-Curve Reach, Yaw Swivel & Mast Lift (1.4s)
            reach_pose = [slot[0], slot[1], slot[2], slot[3], 0.0]
            step(node, settle, reach_pose, duration=1.4)

            # Sample peak telemetry
            actual_lift = node.joint_states.get('lift_stage_1_joint', 0.0) * 3.0
            actual_yaw = math.degrees(node.joint_states.get('yaw_joint', 0.0))
            actual_ext = node.joint_states.get('extension_stage_1_joint', 0.0) * 2.0

            # 5. Gentle Belt-Driven Placement (0.7s)
            deposit_pose = [slot[0], slot[1], slot[2], slot[3], 0.65]
            step(node, reach_pose, deposit_pose, duration=0.7)

            # 6. Anti-Drag Retraction (1.1s)
            retract_pose = [0.0, 0.0, 0.0, 0.0, 0.50]
            step(node, deposit_pose, retract_pose, duration=1.1)

            # 7. Neutral Reset (0.2s)
            stow = [0.0, 0.0, 0.0, 0.0, 0.0]
            step(node, retract_pose, stow, duration=0.2)

            elapsed = time.time() - t_bag
            print(f"[Cycle {idx+1}/6] {spec['label']} placed in {elapsed:.2f}s | Telemetry: Lift={actual_lift*1000:.0f}mm, Yaw={actual_yaw:.1f}deg, Reach={actual_ext:.2f}m", flush=True)

            # Save snapshot of each tier completion
            if idx in [1, 3, 5] and 'camera_wide_iso' in node.latest_frames:
                snap_path = outdir / f"tier_{(idx//2)+1}_complete.png"
                cv2.imwrite(str(snap_path), node.latest_frames['camera_wide_iso'])
                print(f"  --> Saved snapshot: {snap_path}", flush=True)

        # Hold final completed posture
        while time.time() - t_start < 30.0:
            node.set_system_posture(*stow)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.04)

        total_time = time.time() - t_start
        throughput = (6.0 / total_time) * 60.0
        print(f"===================================================================", flush=True)
        print(f" [SUCCESS] 6 bags actively organized in {total_time:.2f}s | Rate: {throughput:.1f} bags/min", flush=True)
        print(f"===================================================================", flush=True)

    finally:
        node.recording = False
        node.close()
        node.destroy_node()
        rclpy.shutdown()
        for p in [image_bridge, bridge, sim]:
            cl.stop_proc(p)
        cl.kill_gazebo()

if __name__ == "__main__":
    main()
