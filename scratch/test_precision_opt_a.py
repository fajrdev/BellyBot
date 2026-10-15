#!/usr/bin/env python3
"""Test precision controlled deposit for Option A with full bag roll-off and fast conveyors.
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

# Option A: 2 Columns (Port: Y=+0.26, Starboard: Y=-0.26) x 3 Tiers (Floor, Mid, Top)
# Using reach=0.72m and tilt=0.18 rad (10.3 deg) for guaranteed roll-off
BAG_SPECS = [
    # Tier 1 (Floor Level: Z_floor = 0.003)
    {"type": "bag_hard_32", "name": "bag_1_t1_p", "color": "0.95 0.45 0.10 1",
     "label": "Bag #1 [Tier 1 Port]", "slot": [0.00,  0.22, 0.72, 0.20]},
    {"type": "bag_hard_26", "name": "bag_2_t1_s", "color": "0.20 0.55 0.90 1",
     "label": "Bag #2 [Tier 1 Starboard]", "slot": [0.00, -0.22, 0.72, 0.20]},
    # Tier 2 (Middle Tier: Lift = +120mm)
    {"type": "bag_hard_20", "name": "bag_3_t2_p", "color": "0.85 0.20 0.35 1",
     "label": "Bag #3 [Tier 2 Port]", "slot": [0.12,  0.22, 0.68, 0.16]},
    {"type": "bag_fragile_20", "name": "bag_4_t2_s", "color": "0.95 0.85 0.15 1",
     "label": "Bag #4 [Tier 2 Starboard]", "slot": [0.12, -0.22, 0.68, 0.16]},
    # Tier 3 (Top Tier: Lift = +180mm)
    {"type": "bag_duffel_23", "name": "bag_5_t3_p", "color": "0.25 0.75 0.45 1",
     "label": "Bag #5 [Tier 3 Port]", "slot": [0.18,  0.22, 0.64, 0.14]},
    {"type": "bag_hard_32", "name": "bag_6_t3_s", "color": "0.65 0.30 0.80 1",
     "label": "Bag #6 [Tier 3 Starboard]", "slot": [0.18, -0.22, 0.64, 0.14]},
]

CAMERAS = {
    "camera_wide_iso": (6.601, -8.090, 6.186, 0.0, 0.6109, 1.9722),
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
    
    # Inelastic contact and realistic fabric friction
    col = m.find(".//collision")
    surf = col.find("surface")
    if surf is None:
        surf = ET.SubElement(col, "surface")
    fric = surf.find("friction/ode")
    if fric is not None:
        fric.find("mu").text = "0.90"
        fric.find("mu2").text = "0.90"
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

class PrecisionNode(Node):
    def __init__(self):
        super().__init__('precision_node')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.joint_states = {}
        
        self.create_subscription(
            Image, "/shot/camera_wide_iso/image", self.on_image, 10
        )
        self.create_subscription(JointState, '/aircraft/robot_joints', self.on_joints, 10)

        # Fast conveyors (0.85 m/s)
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

    def on_image(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_states[name] = pos

    def set_system_posture(self, lift=0.0, yaw=0.0, reach=0.0, pitch=0.0, tray_belt=0.0, conveyor_speed=0.85):
        for s in ['1', '2', '3']:
            m = Float64(); m.data = 0.198 / 3.0; self.pubs[f'corner_stage_{s}'].publish(m)
        m = Float64(); m.data = 1.5708; self.pubs['corner_angle'].publish(m)
        for s in ['1', '2']:
            m = Float64(); m.data = 0.130 / 2.0; self.pubs[f'buffer_stage_{s}'].publish(m)
        m = Float64(); m.data = 0.0; self.pubs['bridge_angle'].publish(m)

        # Fast brisk conveyor flow
        for k in ['door_speed', 'corner_speed', 'bridge_speed', 'buffer_speed']:
            m = Float64(); m.data = conveyor_speed; self.pubs[k].publish(m)
        m = Float64(); m.data = conveyor_speed / 0.011; self.pubs['buffer_roller'].publish(m)
        m = Float64(); m.data = conveyor_speed / 0.005; self.pubs['bridge_roller'].publish(m)

        m = Float64(); m.data = lift / 3.0; self.pubs['lift'].publish(m)
        m = Float64(); m.data = yaw; self.pubs['yaw'].publish(m)
        m = Float64(); m.data = reach / 2.0; self.pubs['extension'].publish(m)
        m = Float64(); m.data = pitch; self.pubs['pitch'].publish(m)
        m = Float64(); m.data = tray_belt; self.pubs['tray_belt_speed'].publish(m)
        m = Float64(); m.data = tray_belt / 0.01; self.pubs['tray_roller_speed'].publish(m)

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
    outdir = REPO / "videos/_work/precision_test"
    outdir.mkdir(parents=True, exist_ok=True)

    cl.kill_gazebo()
    sim = cl.run_bg(
        ["gz", "sim", "-s", "--headless-rendering", "-v", "3",
         str(aircraft / "worlds" / f"{world}.sdf")],
        env, logs / "prec_sim.log")

    bridge = cl.run_bg(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
         "-p", f"config_file:={aircraft / 'config' / 'aircraft_bridge.yaml'}"],
        env, logs / "prec_bridge.log")

    rclpy.init()
    node = PrecisionNode()

    try:
        cl.wait_and_unpause(world, env)
        cam_topics = cl.spawn_shot_cameras(world, CAMERAS, env, 1280, 720, hfov=0.64)
        image_bridge = cl.run_bg(
            ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
            + [f"{t}@sensor_msgs/msg/Image[gz.msgs.Image" for t in cam_topics.values()],
            env, logs / "prec_imgbridge.log"
        )

        stow = [0.0, 0.0, 0.0, 0.0, 0.0]
        node.set_system_posture(*stow)
        time.sleep(2.5)

        print("===================================================================", flush=True)
        print(" Starting Precision Option A Cycle Test with Brisk Infeed & Datum Settle", flush=True)
        print("===================================================================", flush=True)

        t_start = time.time()
        for idx, spec in enumerate(BAG_SPECS):
            slot = spec["slot"]
            t_bag = time.time()

            # 1. Fast buffer infeed (x=3.70)
            spawn_bag(world, spec, env, aircraft, x=3.70, y=0.0, z=0.66)

            # 2. Crisp Intake (0.7s) - pulls bag securely onto tray
            intake = [0.0, 0.0, 0.0, 0.0, 0.70]
            step(node, stow, intake, duration=0.7)

            # 3. Precision Datum Settle (0.3s) - halts belt, centers bag firmly
            settle = [0.0, 0.0, 0.0, 0.0, 0.0]
            step(node, intake, settle, duration=0.3)

            # 4. Graceful S-Curve Kinematic Reach & Swivel (1.4s)
            reach_pose = [slot[0], slot[1], slot[2], slot[3], 0.0]
            step(node, settle, reach_pose, duration=1.4)

            # 5. Full Roll-Off Deposit (1.0s) - belt drives forward at 0.85 m/s to fully clear bag
            deposit_pose = [slot[0], slot[1], slot[2], slot[3], 0.85]
            step(node, reach_pose, deposit_pose, duration=1.0)

            # 6. Anti-Drag Retraction (1.0s) - tray pulls back cleanly
            retract_pose = [0.0, 0.0, 0.0, 0.0, 0.50]
            step(node, deposit_pose, retract_pose, duration=1.0)

            # 7. Reset to Neutral Stow (0.1s)
            stow = [0.0, 0.0, 0.0, 0.0, 0.0]
            step(node, retract_pose, stow, duration=0.1)

            elapsed = time.time() - t_bag
            print(f"[Cycle {idx+1}/6] {spec['label']} placed in {elapsed:.2f}s", flush=True)

            if node.latest_frame is not None:
                snap_path = outdir / f"bag_{idx+1}_placed.png"
                cv2.imwrite(str(snap_path), node.latest_frame)

        print(f"Total sequence time: {time.time()-t_start:.2f}s")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for p in [image_bridge, bridge, sim]:
            cl.stop_proc(p)
        cl.kill_gazebo()

if __name__ == "__main__":
    main()
