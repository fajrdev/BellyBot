#!/usr/bin/env python3
"""BellyBot Interactive Manual Control & Teleoperation (Keyboard / Gamepad).

Allows human operators to manually drive BellyBot, jog all multi-DOF joints,
and interact with dynamic baggage inside either:
  1. The authentic Boeing 737-800 forward cargo hold (--world plane)
  2. An open, unrestricted testing sandbox arena (--world open)

Controls:
  - W / S       : Drive Forward / Reverse (chassis tracks)
  - A / D       : Steer Left / Right (in-place pivot)
  - I / K       : Telescopic Mast Lift Up / Down (0 to 540 mm)
  - J / L       : Turntable Yaw Rotate Left / Right (+-30 deg fine / +-90 deg dock)
  - U / O       : Telescopic Boom Reach Extend / Retract (0 to 760 mm)
  - T / G       : Conveyor Deck Pitch Down (Scoop) / Up (Level) (0 to 33 deg)
  - F / V       : Conveyor Belts & Rollers Feed Forward (Intake) / Reverse (Discharge)
  - B           : Spawn a 32 kg test suitcase in front of BellyBot
  - SPACE       : Emergency Stop (Instant zero-cutoff & brake)
  - R           : Supervised Reset to stowed travel configuration
  - Q / ESC     : Exit and clean up simulation

Gamepad / Controller (Auto-detected):
  - Left Stick  : Drive & Steer
  - Right Stick : Mast Lift Up / Down
  - D-Pad       : Turntable Yaw
  - RT / LT     : Boom Reach Extend / Retract
  - RB / LB     : Pitch Down / Up
  - A / Y       : Conveyor Belt Forward / Reverse
  - B           : Emergency Stop
  - X           : Reset to stowed pose
  - Start       : Spawn dynamic suitcase
"""

import os
import sys
import time
import math
import json
import argparse
import subprocess
import threading
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64, Bool
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))
import capture_lib as cl

PKG = REPO / "BellyBot_Aircraft_Environment_v2/ros2/bellybot_aircraft_environment"
SCRATCH = REPO / "scratch"
SCRATCH.mkdir(parents=True, exist_ok=True)

# Try importing pygame for smooth keyboard & gamepad input
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False


def create_plane_world(start_in_aisle=True):
    """Build the Boeing 737 hold world."""
    shuttle_sdf = PKG / "worlds" / "mission_shuttle.sdf"
    tree = ET.parse(shuttle_sdf)
    root = tree.getroot()
    world = root.find("world")

    if start_in_aisle:
        robot = world.find("./model[@name='bellybot_dock_stack']")
        if robot is not None:
            p_elem = robot.find("pose")
            if p_elem is not None:
                p_elem.text = "2.500 0.000 0.003 0 0 0"

    out_path = SCRATCH / "play_plane_world.sdf"
    tree.write(str(out_path), encoding="utf-8", xml_declaration=True)
    return "mission_shuttle", out_path


def create_open_world():
    """Build an open testing sandbox world with expanded floor and no ceiling."""
    shuttle_sdf = PKG / "worlds" / "mission_shuttle.sdf"
    tree = ET.parse(shuttle_sdf)
    root = tree.getroot()
    world = root.find("world")

    # Remove the aircraft hold include
    for inc in list(world.findall("include")):
        uri = inc.findtext("uri", "")
        if "nb_hold_cutaway" in uri or "mission_door_belt" in uri or "belt_loader" in uri:
            world.remove(inc)

    # Position robot at center of playground
    robot = world.find("./model[@name='bellybot_dock_stack']")
    if robot is not None:
        p_elem = robot.find("pose")
        if p_elem is not None:
            p_elem.text = "0.000 0.000 0.003 0 0 0"

    # Expand apron to a large 40x40m testing floor
    apron = world.find("./model[@name='apron']")
    if apron is not None:
        box_size = apron.find(".//collision/geometry/box/size")
        if box_size is not None:
            box_size.text = "40.0 40.0 0.05"
        vis_size = apron.find(".//visual/geometry/box/size")
        if vis_size is not None:
            vis_size.text = "40.0 40.0 0.05"

    out_path = SCRATCH / "play_open_world.sdf"
    tree.write(str(out_path), encoding="utf-8", xml_declaration=True)
    return "mission_shuttle", out_path


def make_bag_sdf(name, x, y, z, yaw, mass=32.0, color=(0.15, 0.45, 0.85)):
    """Generate dynamic rigid-body passenger suitcase SDF."""
    w, l, h = 0.52, 0.76, 0.32
    ixx = (1.0 / 12.0) * mass * (w**2 + h**2)
    iyy = (1.0 / 12.0) * mass * (l**2 + h**2)
    izz = (1.0 / 12.0) * mass * (l**2 + w**2)
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
    <link name="link">
      <inertial>
        <mass>{mass:.1f}</mass>
        <inertia>
          <ixx>{ixx:.5f}</ixx>
          <iyy>{iyy:.5f}</iyy>
          <izz>{izz:.5f}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>{l} {w} {h}</size></box>
        </geometry>
        <surface>
          <friction>
            <ode><mu>0.45</mu><mu2>0.45</mu2></ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{l} {w} {h}</size></box>
        </geometry>
        <material>
          <ambient>{color[0]} {color[1]} {color[2]} 1</ambient>
          <diffuse>{color[0]} {color[1]} {color[2]} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class BellyBotTeleopNode(Node):
    """ROS 2 Node handling live teleoperation input and actuator publishing."""

    def __init__(self, max_speed=0.45):
        super().__init__('bellybot_play_teleop')
        self.bridge = CvBridge()
        self.sim_time = 0.0
        self.max_speed = max_speed

        # State targets
        self.vx = 0.0
        self.wz = 0.0
        self.lift_target = 0.0       # 0.0 to 0.540 m (total across 3 stages)
        self.yaw_target = 0.0        # -1.57 to +1.57 rad (-90 to +90 deg)
        self.reach_target = 0.0      # 0.0 to 0.760 m (total across 2 stages)
        self.pitch_target = 0.0      # 0.0 to 0.576 rad (0 to 33 deg)
        self.tray_belt_speed = 0.0   # +-0.45 m/s

        self.estop = False
        self.robot_pose = (0.0, 0.0, 0.003, 0.0, 0.0, 0.0)
        self.latest_cam_frame = None
        self.bag_count = 0

        # Subscriptions
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Clock, '/clock', self.on_clock, 20)
        self.create_subscription(TFMessage, '/aircraft/robot_pose', self.on_robot_pose, 20)
        self.create_subscription(Image, '/shot/cam_teleop/image', self.on_cam_image, qos)

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/aircraft/actuation/cmd_vel', 10)
        self.pub_lift = self.create_publisher(Float64, '/aircraft/actuation/lift_stage_target', 10)
        self.pub_yaw = self.create_publisher(Float64, '/aircraft/actuation/yaw_target', 10)
        self.pub_ext = self.create_publisher(Float64, '/aircraft/actuation/extension_stage_target', 10)
        self.pub_pitch = self.create_publisher(Float64, '/aircraft/actuation/pitch_target', 10)
        self.pub_belt = self.create_publisher(Float64, '/aircraft/actuation/tray_belt_speed', 10)
        self.pub_roller = self.create_publisher(Float64, '/aircraft/actuation/tray_roller_speed', 10)
        self.pub_left_wheel = self.create_publisher(Float64, '/aircraft/actuation/left_wheel_speed', 10)
        self.pub_right_wheel = self.create_publisher(Float64, '/aircraft/actuation/right_wheel_speed', 10)

        # 50 Hz control loop
        self.create_timer(0.02, self.control_loop)

    def on_clock(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_robot_pose(self, msg: TFMessage):
        for tr in msg.transforms:
            if 'bellybot_dock_stack' in tr.child_frame_id:
                p = tr.transform.translation
                q = tr.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
                roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
                self.robot_pose = (p.x, p.y, p.z, roll, pitch, yaw)

    def on_cam_image(self, msg: Image):
        try:
            self.latest_cam_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

    def control_loop(self):
        if self.estop:
            tw = Twist()
            self.pub_cmd_vel.publish(tw)
            self.pub_left_wheel.publish(Float64(data=0.0))
            self.pub_right_wheel.publish(Float64(data=0.0))
            self.pub_belt.publish(Float64(data=0.0))
            self.pub_roller.publish(Float64(data=0.0))
            return

        # Continuous Actuation
        tw = Twist()
        tw.linear.x = float(self.vx)
        tw.angular.z = float(self.wz)
        self.pub_cmd_vel.publish(tw)

        wheel_rad = 0.068
        w_left = float((self.vx - self.wz * 0.44) / wheel_rad)
        w_right = float((self.vx + self.wz * 0.44) / wheel_rad)
        self.pub_left_wheel.publish(Float64(data=w_left))
        self.pub_right_wheel.publish(Float64(data=w_right))

        # Joint stages: 3-stage mast, 2-stage boom
        self.pub_lift.publish(Float64(data=float(self.lift_target / 3.0)))
        self.pub_yaw.publish(Float64(data=float(self.yaw_target)))
        self.pub_ext.publish(Float64(data=float(self.reach_target / 2.0)))
        self.pub_pitch.publish(Float64(data=float(self.pitch_target)))
        self.pub_belt.publish(Float64(data=float(self.tray_belt_speed)))
        roller_spd = (self.tray_belt_speed / 0.01) if self.tray_belt_speed != 0.0 else 0.0
        self.pub_roller.publish(Float64(data=float(roller_spd)))

    def reset_stowed(self):
        self.vx = 0.0
        self.wz = 0.0
        self.lift_target = 0.0
        self.yaw_target = 0.0
        self.reach_target = 0.0
        self.pitch_target = 0.0
        self.tray_belt_speed = 0.0
        self.estop = False


def run_interactive_app(args):
    print("=" * 80)
    print("BELLYBOT INTERACTIVE MANUAL CONTROLLER")
    print(f"-> Selected Environment: {args.world.upper()}")
    print(f"-> Gazebo 3D GUI: {'ENABLED' if args.gui else 'DISABLED'}")
    print("=" * 80)

    # Clean existing instances
    cl.kill_gazebo()
    time.sleep(1.0)

    aircraft_share = cl.share_dir("bellybot_aircraft_environment")
    desc_share = cl.share_dir("bellybot_dock_stack_description")
    env = cl.resource_env(aircraft_share, desc_share)

    # 1. Create World
    if args.world == "open":
        world_name, world_path = create_open_world()
        cam_pose = (4.50, -4.50, 2.80, 0.0, 0.45, 2.35)
        max_speed = 0.90
    else:
        world_name, world_path = create_plane_world(start_in_aisle=True)
        cam_pose = (3.50, -3.20, 1.70, 0.0, 0.28, 1.78)
        max_speed = 0.45

    print(f"-> Generated world SDF: {world_path}")

    # 2. Launch Gazebo
    gz_args = ['gz', 'sim', '-v', '3', '-r']
    if not args.gui:
        gz_args.extend(['-s', '--headless-rendering'])
    gz_args.append(str(world_path))

    gz_proc = subprocess.Popen(gz_args, env=env)
    print(f"-> Launched Gazebo (PID {gz_proc.pid})")

    # 3. Launch ros_gz_bridge
    bridge_config = str(aircraft_share / "config" / "mission_bridge.yaml")
    bridge_cmd = [
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '--ros-args', '-p', f'config_file:={bridge_config}'
    ]
    bridge_proc = subprocess.Popen(bridge_cmd, env=env)
    print(f"-> Launched ros_gz_bridge (PID {bridge_proc.pid})")

    # 4. Initialize ROS 2
    rclpy.init()
    node = BellyBotTeleopNode(max_speed=max_speed)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(3.5)
    cl.wait_and_unpause(world_name, env, timeout=30.0)

    # Spawn shot camera
    cameras = {"cam_teleop": cam_pose}
    cam_topics = cl.spawn_shot_cameras(world_name, cameras, env, 960, 540, hfov=1.1)
    cam_bridge_proc = subprocess.Popen(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
        + [f"{t}@sensor_msgs/msg/Image[gz.msgs.Image" for t in cam_topics.values()],
        env=env
    )
    print("-> Spawned interactive HUD camera and bridge")

    # Initialize Pygame for window & inputs
    if PYGAME_AVAILABLE:
        pygame.init()
        pygame.joystick.init()
        joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
        for j in joysticks:
            j.init()
            print(f"-> Connected Gamepad: {j.get_name()}")

        win_w, win_h = 1000, 680
        screen = pygame.display.set_mode((win_w, win_h))
        pygame.display.set_caption("BellyBot Interactive Teleoperation Dashboard")
        clock = pygame.time.Clock()
        font_large = pygame.font.SysFont("Arial", 20, bold=True)
        font_medium = pygame.font.SysFont("Arial", 14, bold=True)
        font_small = pygame.font.SysFont("Courier New", 12, bold=True)

    def spawn_bag_in_front():
        node.bag_count += 1
        name = f"bag_play_{node.bag_count}"
        rx, ry, rz, _, _, ryaw = node.robot_pose
        # Spawn 1.15m in front of robot
        bx = rx + 1.15 * math.cos(ryaw)
        by = ry + 1.15 * math.sin(ryaw)
        bz = 0.16
        bag_sdf = make_bag_sdf(name, bx, by, bz, ryaw)
        try:
            cl.gz_create(world_name, bag_sdf, env)
            print(f"\n[SPAWN] Created 32 kg test suitcase '{name}' at ({bx:.2f}, {by:.2f})!")
        except Exception as e:
            print(f"\n[SPAWN ERROR] Could not spawn bag: {e}")

    running = True
    print("\n" + "=" * 80)
    print("CONTROLS ACTIVE! Focus on the Dashboard Window to drive.")
    print("  [W/S] Drive | [A/D] Steer | [I/K] Mast Lift | [J/L] Yaw | [U/O] Reach")
    print("  [T/G] Pitch | [F/V] Belt  | [B] Spawn Bag   | [SPACE] E-Stop | [R] Reset | [Q] Quit")
    print("=" * 80 + "\n")

    try:
        while running and rclpy.ok():
            if not PYGAME_AVAILABLE:
                time.sleep(0.05)
                continue

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        node.estop = not node.estop
                        print(f"\n-> E-STOP {'ENGAGED' if node.estop else 'RELEASED'}")
                    elif event.key == pygame.K_r:
                        node.reset_stowed()
                        print("\n-> Joints Reset to Stowed Home Pose")
                    elif event.key == pygame.K_b:
                        spawn_bag_in_front()

            keys = pygame.key.get_pressed()

            # Joystick input if present
            gp_drive = 0.0
            gp_steer = 0.0
            gp_lift = 0.0
            if joysticks:
                js = joysticks[0]
                gp_drive = -js.get_axis(1) # Left stick Y inverted
                gp_steer = -js.get_axis(0) # Left stick X
                gp_lift = -js.get_axis(4) if js.get_numaxes() > 4 else 0.0
                if js.get_button(1): # B button
                    node.estop = True
                if js.get_button(0): # A button -> belt forward
                    node.tray_belt_speed = 0.40
                elif js.get_button(3): # Y button -> belt reverse
                    node.tray_belt_speed = -0.40
                if js.get_button(7): # Start -> spawn bag
                    spawn_bag_in_front()
                    time.sleep(0.3)

            # Keyboard Drive Input
            target_vx = 0.0
            target_wz = 0.0
            if keys[pygame.K_w] or keys[pygame.K_UP]:
                target_vx = node.max_speed
            elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
                target_vx = -node.max_speed

            if keys[pygame.K_a] or keys[pygame.K_LEFT]:
                target_wz = 0.50
            elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
                target_wz = -0.50

            # Merge Gamepad & Keyboard
            if abs(gp_drive) > 0.15:
                target_vx = gp_drive * node.max_speed
            if abs(gp_steer) > 0.15:
                target_wz = gp_steer * 0.60

            # Smooth linear velocity ramping
            node.vx = float(np.clip(target_vx, -node.max_speed, node.max_speed))
            node.wz = float(np.clip(target_wz, -0.60, 0.60))

            # Manipulator Jog Controls
            # 1. Mast Lift (I / K) -> 0.0 to 0.540 m
            if keys[pygame.K_i] or gp_lift > 0.2:
                node.lift_target = min(0.540, node.lift_target + 0.006)
            elif keys[pygame.K_k] or gp_lift < -0.2:
                node.lift_target = max(0.000, node.lift_target - 0.006)

            # 2. Turntable Yaw (J / L) -> -1.57 to +1.57 rad
            if keys[pygame.K_j]:
                node.yaw_target = max(-1.5708, node.yaw_target - 0.015)
            elif keys[pygame.K_l]:
                node.yaw_target = min(1.5708, node.yaw_target + 0.015)

            # 3. Telescopic Reach (U / O) -> 0.0 to 0.760 m
            if keys[pygame.K_u]:
                node.reach_target = min(0.760, node.reach_target + 0.008)
            elif keys[pygame.K_o]:
                node.reach_target = max(0.000, node.reach_target - 0.008)

            # 4. Conveyor Deck Pitch (T / G) -> 0.0 to 33.0 deg (0.576 rad)
            if keys[pygame.K_t]:
                node.pitch_target = min(0.576, node.pitch_target + 0.010)
            elif keys[pygame.K_g]:
                node.pitch_target = max(0.000, node.pitch_target - 0.010)

            # 5. Conveyor Belt Feed (F / V)
            if keys[pygame.K_f]:
                node.tray_belt_speed = 0.40
            elif keys[pygame.K_v]:
                node.tray_belt_speed = -0.40
            elif not joysticks or not (js.get_button(0) or js.get_button(3)):
                if not (keys[pygame.K_f] or keys[pygame.K_v]):
                    node.tray_belt_speed = 0.0

            # Render Pygame Dashboard
            screen.fill((15, 23, 42)) # Deep navy background

            # Live Camera Feed on Left
            if node.latest_cam_frame is not None:
                cam_rgb = cv2.cvtColor(node.latest_cam_frame, cv2.COLOR_BGR2RGB)
                cam_surface = pygame.surfarray.make_surface(np.rot90(cam_rgb))
                cam_surface = pygame.transform.scale(cam_surface, (600, 338))
                screen.blit(cam_surface, (20, 50))
                pygame.draw.rect(screen, (56, 189, 248), (20, 50, 600, 338), 2)
            else:
                pygame.draw.rect(screen, (30, 41, 59), (20, 50, 600, 338))
                t_load = font_medium.render("Waiting for Gazebo Camera Stream...", True, (148, 163, 184))
                screen.blit(t_load, (160, 200))

            # Title Bar
            title_text = font_large.render(f"BELLYBOT TELEOPERATION CONSOLE — [{args.world.upper()} WORLD]", True, (56, 189, 248))
            screen.blit(title_text, (20, 15))

            # Status Pill (E-Stop vs Active)
            if node.estop:
                pill_col = (239, 68, 68)
                pill_txt = "E-STOP LATCHED"
            else:
                pill_col = (34, 197, 94)
                pill_txt = "SYSTEM ACTIVE"
            pygame.draw.rect(screen, pill_col, (win_w - 200, 15, 180, 26), border_radius=4)
            p_surf = font_medium.render(pill_txt, True, (255, 255, 255))
            screen.blit(p_surf, (win_w - 185, 20))

            # Right Side: Telemetry Gauges
            x_bar = 640
            y_bar = 60

            def draw_gauge(label, val_str, norm_val, y_pos, color=(56, 189, 248)):
                txt = font_medium.render(f"{label}: {val_str}", True, (241, 245, 249))
                screen.blit(txt, (x_bar, y_pos))
                pygame.draw.rect(screen, (51, 65, 85), (x_bar, y_pos + 18, 330, 12), border_radius=3)
                w_fill = int(max(0.0, min(1.0, norm_val)) * 330)
                if w_fill > 0:
                    pygame.draw.rect(screen, color, (x_bar, y_pos + 18, w_fill, 12), border_radius=3)

            rx, ry, rz, _, _, ryaw = node.robot_pose
            txt_pos = font_medium.render(f"CHASSIS POSE: X={rx:.2f}m, Y={ry:.2f}m, Yaw={math.degrees(ryaw):.1f}°", True, (250, 204, 21))
            screen.blit(txt_pos, (x_bar, y_bar))

            draw_gauge("SPEED (Vx)", f"{node.vx:+.2f} m/s", (node.vx + node.max_speed) / (2 * node.max_speed), y_bar + 35)
            draw_gauge("MAST LIFT", f"{node.lift_target*1000:.0f} mm / 540 mm", node.lift_target / 0.540, y_bar + 75, (74, 222, 128))
            draw_gauge("TURNTABLE YAW", f"{math.degrees(node.yaw_target):+.1f}°", (math.degrees(node.yaw_target) + 90.0) / 180.0, y_bar + 115, (251, 146, 60))
            draw_gauge("TELESCOPIC REACH", f"{node.reach_target*1000:.0f} mm / 760 mm", node.reach_target / 0.760, y_bar + 155, (192, 132, 252))
            draw_gauge("DECK PITCH", f"{math.degrees(node.pitch_target):.1f}° / 33.0°", math.degrees(node.pitch_target) / 33.0, y_bar + 195, (244, 114, 182))
            draw_gauge("CONVEYOR BELT", f"{node.tray_belt_speed:+.2f} m/s", (node.tray_belt_speed + 0.45) / 0.90, y_bar + 235, (56, 189, 248))

            # Bottom Help & Controls Overlay
            pygame.draw.rect(screen, (30, 41, 59), (20, 410, win_w - 40, 240), border_radius=6)
            h_title = font_medium.render("INTERACTIVE KEYBOARD & GAMEPAD CONTROLS GUIDE", True, (250, 204, 21))
            screen.blit(h_title, (40, 422))

            ctrl_lines = [
                "[W / S] Drive Forward / Reverse       | [A / D] Steer Differential Pivot      | [SPACE] Emergency Stop",
                "[I / K] Mast Lift Up / Down (0-540mm) | [J / L] Turntable Yaw (-90 to +90 deg) | [R] Reset to Stowed Home",
                "[U / O] Boom Reach Extend / Retract   | [T / G] Deck Pitch Down / Level        | [B] SPAWN 32KG SUITCASE IN FRONT!",
                "[F / V] Conveyor Belt Forward / Rev   | [GAMEPAD] Left Stick (Drive)           | [Q / ESC] Quit Teleop",
                "Tip for Ground Scooping: Drive near suitcase -> Press [T] to pitch 29 deg -> Press [U] to reach -> Press [F] to scoop!"
            ]
            for idx, cl_text in enumerate(ctrl_lines):
                c_surf = font_small.render(cl_text, True, (226, 232, 240))
                screen.blit(c_surf, (40, 452 + idx * 24))

            pygame.display.flip()
            clock.tick(50) # 50 FPS

    finally:
        print("\n-> Cleaning up teleoperation processes...")
        if PYGAME_AVAILABLE:
            pygame.quit()
        cl.kill_gazebo()
        try:
            bridge_proc.terminate()
            cam_bridge_proc.terminate()
            gz_proc.terminate()
        except Exception:
            pass
        rclpy.shutdown()
        print("-> BellyBot Teleop session closed safely. Thank you for driving!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="BellyBot Interactive Manual Teleop")
    parser.add_argument("--world", choices=["plane", "open"], default="plane",
                        help="Simulation environment: 'plane' (Boeing 737 hold) or 'open' (Sandbox playground)")
    parser.add_argument("--gui", action="store_true", default=True,
                        help="Launch native Gazebo 3D GUI window (default: True)")
    parser.add_argument("--no-gui", dest="gui", action="store_false",
                        help="Run Gazebo headless and use only the HUD window")
    args = parser.parse_args()
    run_interactive_app(args)
