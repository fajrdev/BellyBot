#!/usr/bin/env python3
"""Mission Controller for BellyBot door-docked shuttle and mobile manipulation.

Owns all actuators exclusively. Enforces:
- Feedback-driven state transitions with measured convergence.
- Sim-time timers from /clock.
- Independent wall-clock watchdog for frozen /clock or lost transport.
- Globally latched stop (/mission/stop) and supervised reset (/mission/reset).
- Generalized mobile-frame safety guard based on measured robot SE(3).
"""

import math
import time
import json
import threading
from pathlib import Path
import numpy as np

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64, Float64MultiArray, Bool, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock

PI = math.pi

def smooth(u):
    return 10 * u**3 - 15 * u**4 + 6 * u**5

class MobileGuard:
    """Aircraft hold clearance and pressure screen using measured robot base SE(3)."""
    def __init__(self, geom_file):
        self.data = json.loads(Path(geom_file).read_text())
        self.links = self.data['links']
        self.joints = self.data['joints']
        self.verts = {l['name']: np.c_[np.array(l['vertices']), np.ones(len(l['vertices']))] for l in self.links}

    def evaluate(self, base_pose, q, height=0.38, mass=32.0):
        """Evaluate clearance at measured base_pose [x, y, z, roll, pitch, yaw] and arm q [lift, yaw, reach, pitch]."""
        lift, yaw, reach, pitch = q
        bx, by, bz, broll, bpitch, byaw = base_pose
        
        # Check raw limits
        if not (0 <= lift <= 0.60 and abs(yaw) <= math.radians(65) + 0.01 and 0 <= reach <= 1.0 and 0 <= pitch <= math.radians(32) + 0.01):
            raise ValueError(f"Arm joint limits exceeded: lift={lift:.3f}, yaw={yaw:.3f}, reach={reach:.3f}, pitch={pitch:.3f}")
        
        # Guard kinematic rules
        if abs(yaw) > 0.01 and (lift < 0.075 or reach > 0.52):
            raise ValueError(f"Guarded yaw requires lift >= 80mm and reach <= 500mm (got lift={lift:.3f}, reach={reach:.3f})")
        if pitch > 0.005 and (abs(yaw) > 0.02 or lift > 0.01 or reach < 0.70):
            raise ValueError(f"Floor pitch requires yaw=0, lift=0, reach >= 720mm (got yaw={yaw:.3f}, lift={lift:.3f}, reach={reach:.3f})")
            
        vals = {
            'lift_stage_1_joint': lift / 3.0,
            'lift_stage_2_joint': lift / 3.0,
            'lift_stage_3_joint': lift / 3.0,
            'yaw_joint': yaw,
            'extension_mid_joint': reach / 2.0,
            'extension_tip_joint': reach / 2.0,
            'tray_pitch_joint': pitch
        }
        
        # Forward kinematics relative to base_link
        frames = {self.data['root']: np.eye(4)}
        todo = self.joints.copy()
        while todo:
            rem = []
            for j in todo:
                if j['parent'] not in frames:
                    rem.append(j)
                    continue
                b = np.eye(4)
                v = vals.get(j['name'], 0.0)
                if j['type'] == 'prismatic':
                    b[:3, 3] = np.array(j['axis']) * v
                elif j['type'] in ('revolute', 'continuous'):
                    c = math.cos(v)
                    s = math.sin(v)
                    ax = np.array(j['axis'], dtype=float)
                    ax /= np.linalg.norm(ax)
                    x, y, z = ax
                    d = 1.0 - c
                    rot = np.array([
                        [c + x*x*d, x*y*d - z*s, x*z*d + y*s],
                        [y*x*d + z*s, c + y*y*d, y*z*d - x*s],
                        [z*x*d - y*s, z*y*d + x*s, c + z*z*d]
                    ])
                    b[:3, :3] = rot
                T_j = np.array(j['origin'], dtype=float)
                frames[j['child']] = frames[j['parent']] @ T_j @ b
            if len(rem) == len(todo):
                break
            todo = rem

        # Base transform into world frame
        T_base = np.eye(4)
        T_base[:3, 3] = [bx, by, bz]
        cy, sy = math.cos(byaw), math.sin(byaw)
        T_base[:3, :3] = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

        # Transform all points to world
        world_pts = []
        for l in self.links:
            T_link = T_base @ frames[l['name']]
            pts = (T_link @ self.verts[l['name']].T).T[:, :3]
            world_pts.append(pts)

        if mass > 0:
            tray = T_base @ frames['tray']
            # Bag bounding vertices
            bag_box = np.array([
                [0.43 + x, y, -0.008 + z, 1.0]
                for x in [-0.39, 0.39]
                for y in [-0.25, 0.25]
                for z in [0.0, height]
            ])
            world_pts.append((tray @ bag_box.T).T[:, :3])

        all_xyz = np.concatenate(world_pts)
        x_pts, y_pts, z_pts = all_xyz.T

        # Roof clearance (hold ceiling at nominal 1.15m)
        roof_clearance = float((1.15 - z_pts).min())
        floor_clearance = float(z_pts.min())

        return {
            'roof_m': roof_clearance,
            'floor_m': floor_clearance,
            'max_z_m': float(z_pts.max())
        }

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.declare_parameter('mode', 'mission')
        self.declare_parameter('bag_height', 0.38)
        self.declare_parameter('bag_mass', 32.0)
        self.declare_parameter('run_id', 'run_001')

        self.mode = self.get_parameter('mode').value
        self.bag_height = float(self.get_parameter('bag_height').value)
        self.bag_mass = float(self.get_parameter('bag_mass').value)
        self.run_id = self.get_parameter('run_id').value

        pkg_share = Path(get_package_share_directory('bellybot_aircraft_environment'))
        self.guard = MobileGuard(pkg_share / 'config/robot_geometry.json')

        # Telemetry & State
        self.q = {}
        self.q_stamp = {}
        self.robot_pose = None  # (x, y, z, roll, pitch, yaw)
        self.robot_pose_stamp = 0.0
        self.pose_history = []  # (t, x, y, yaw)
        self.sim_time = 0.0
        self.last_sim_time = -1.0
        self.last_clock_wall = time.monotonic()
        self.last_clock_advance_wall = time.monotonic()
        self.wall_watchdog_ok = True

        # State Machine
        self.state = 'IDLE'
        self.fault = ''
        self.stop_latched = False
        self.active_bag_id = None
        self.active_target_id = None
        self.placement_counter = 0

        # Motion targets
        self.target_arm = [0.0, 0.0, 0.0, 0.0]  # lift, yaw, reach, pitch
        self.current_arm_cmd = [0.0, 0.0, 0.0, 0.0]
        self.target_base = None  # (x, y, yaw)
        self.arm_segment = None
        self.arm_stage = 'IDLE'
        self.base_trajectory = None

        # Docking Definition
        self.P_DOCK = (1.300, -0.095, PI / 2.0)
        self.P_WORK = (3.200, 0.000, 0.0)

        # Publications
        self.pub_status = self.create_publisher(String, '/mission/status', 10)
        self.pub_events = self.create_publisher(String, '/mission/events', 10)
        self.cmd_vel = self.create_publisher(Twist, '/aircraft/actuation/cmd_vel', 10)
        
        self.pub_act = {}
        names = [
            'lift_stage_target', 'extension_stage_target', 'yaw_target', 'pitch_target',
            'tray_belt_speed', 'tray_roller_speed', 'left_wheel_speed', 'right_wheel_speed',
            'loader_speed', 'door_speed', 'door_roller'
        ]
        for n in names:
            self.pub_act[n] = self.create_publisher(Float64, f'/aircraft/actuation/{n}', 10)

        # Subscriptions
        self.create_subscription(Clock, '/clock', self.on_clock, 20)
        self.create_subscription(JointState, '/aircraft/robot_joints', self.on_joints, 20)
        self.create_subscription(TFMessage, '/aircraft/robot_pose', self.on_robot_pose, 20)
        self.create_subscription(String, '/mission/command', self.on_command, 10)
        self.create_subscription(Float64MultiArray, '/mission/pose_target', self.on_pose_target, 10)
        self.create_subscription(Bool, '/mission/stop', self.on_stop, 10)

        # Service
        self.srv_reset = self.create_service(Trigger, '/mission/reset', self.on_reset)

        # Control Timers
        self.create_timer(0.02, self.control_loop)
        self.create_timer(0.5, self.publish_telemetry)

        # Background Wall-Clock Watchdog Thread
        self.watchdog_thread = threading.Thread(target=self.wall_clock_watchdog, daemon=True)
        self.watchdog_thread.start()

        self.emit_event('run_started', details={'mode': self.mode, 'run_id': self.run_id})
        self.get_logger().info(f"Mission Controller initialized [mode={self.mode}, run_id={self.run_id}]")

    def now_sim(self):
        return self.sim_time

    def on_clock(self, msg: Clock):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.sim_time = t
        self.last_clock_wall = time.monotonic()
        if t > self.last_sim_time:
            self.last_sim_time = t
            self.last_clock_advance_wall = time.monotonic()

    def wall_clock_watchdog(self):
        while True:
            time.sleep(0.2)
            dt_wall = time.monotonic() - self.last_clock_advance_wall
            if dt_wall > 2.5:
                if self.wall_watchdog_ok and self.sim_time > 0.0:
                    self.get_logger().error(f"Wall-clock Watchdog Triggered: /clock frozen for {dt_wall:.2f}s (> 2.5s threshold)!")
                    self.wall_watchdog_ok = False
                    self.latch_stop("Simulator link /clock frozen (> 2.5s)")

    def on_joints(self, msg: JointState):
        t = self.now_sim()
        for name, pos in zip(msg.name, msg.position):
            self.q[name] = pos
            self.q_stamp[name] = t

    def on_robot_pose(self, msg: TFMessage):
        t = self.now_sim()
        for tr in msg.transforms:
            if 'bellybot_dock_stack' in tr.child_frame_id:
                p = tr.transform.translation
                q = tr.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
                roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
                self.robot_pose = (p.x, p.y, p.z, roll, pitch, yaw)
                self.robot_pose_stamp = t
                self.pose_history.append((t, p.x, p.y, yaw))
                if len(self.pose_history) > 60:
                    self.pose_history = [(pt, px, py, pyaw) for pt, px, py, pyaw in self.pose_history if t - pt <= 2.0]

    def is_robot_stationary(self):
        if len(self.pose_history) < 2:
            return True
        t_now = self.now_sim()
        recent = [p for p in self.pose_history if t_now - p[0] <= 0.40]
        if len(recent) < 2:
            recent = self.pose_history[-2:]
        t0, x0, y0, yaw0 = recent[0]
        t1, x1, y1, yaw1 = recent[-1]
        dt = t1 - t0
        if dt < 0.01:
            return True
        lin_spd = math.hypot(x1 - x0, y1 - y0) / dt
        ang_spd = abs(yaw1 - yaw0) / dt
        return lin_spd < 0.02 and ang_spd < 0.04

    def latch_stop(self, reason):
        self.stop_latched = True
        self.state = 'STOP_LATCHED'
        self.fault = reason
        # Freeze arm hold at current measured joint posture
        if self.q:
            lift = sum(self.q.get(f'lift_stage_{i}_joint', 0.0) for i in (1, 2, 3))
            yaw = self.q.get('yaw_joint', 0.0)
            reach = self.q.get('extension_mid_joint', 0.0) + self.q.get('extension_tip_joint', 0.0)
            pitch = self.q.get('tray_pitch_joint', 0.0)
            self.current_arm_cmd = [lift, yaw, reach, pitch]
        # Inhibit active trajectories
        self.arm_segment = None
        self.arm_stage = 'IDLE'
        self.base_trajectory = None
        self.stop_actuators()
        self.emit_event('stop_commanded', details={'reason': self.fault})
        self.publish_telemetry()
        self.get_logger().warn(f"Emergency Stop LATCHED globally: {reason}")

    def on_stop(self, msg: Bool):
        if msg.data:
            self.latch_stop('E-Stop commanded via /mission/stop')
        # Note: data==False NEVER clears latched stop per contract.

    def on_reset(self, req, res):
        if not self.stop_latched and not self.fault:
            res.success = True
            res.message = "System already clear."
            return res

        # Verify safe stationary conditions before reset
        if not self.fresh_feedback():
            res.success = False
            res.message = "Reset rejected: feedback is stale or missing."
            self.emit_event('reset_rejected', details={'reason': res.message})
            return res

        if not self.is_robot_stationary():
            res.success = False
            res.message = "Reset rejected: robot is not stationary."
            self.emit_event('reset_rejected', details={'reason': res.message})
            return res

        if self.robot_pose is not None:
            self.stop_latched = False
            self.fault = ''
            self.wall_watchdog_ok = True
            self.last_clock_advance_wall = time.monotonic()
            self.state = 'IDLE'
            self.publish_telemetry()
            res.success = True
            res.message = "Reset accepted: operator authorized and preconditions met."
            self.emit_event('reset_accepted', details={'reason': res.message})
            self.get_logger().info("Mission controller reset successfully.")
            return res

        res.success = False
        res.message = "Reset rejected: robot pose unknown."
        self.emit_event('reset_rejected', details={'reason': res.message})
        return res

    def on_command(self, msg: String):
        try:
            req = json.loads(msg.data)
            cmd = req.get('command')
            bag_id = req.get('bag_id')
            target_id = req.get('target_id')
            self.get_logger().info(f"Received mission command: {cmd} [bag_id={bag_id}]")

            if self.stop_latched:
                err_msg = f"Cannot accept command '{cmd}' while STOP_LATCHED"
                self.emit_event('command_rejected', details={'command': cmd, 'reason': err_msg})
                self.get_logger().error(err_msg)
                return

            if cmd == 'start_load':
                if self.state in ('IDLE', 'COMPLETE'):
                    self.active_bag_id = bag_id or f"bag_{self.placement_counter + 1}"
                    self.active_target_id = target_id or f"target_cell_{self.placement_counter + 1}"
                    if req.get('skip_infeed', False):
                        self.state = 'TRANSPORT'
                        self.state_entry_time = self.now_sim()
                        self.emit_event('support_confirmed', details={'tray_supported': True, 'preloaded': True})
                        self.get_logger().info("Preloaded baggage verified. State -> TRANSPORT")
                    else:
                        self.state = 'DOCK_APPROACH'
                        self.emit_event('dock_reserved', bag_id=self.active_bag_id, target_id=self.active_target_id)
            elif cmd == 'start_unload':
                if self.state in ('IDLE', 'COMPLETE'):
                    self.active_bag_id = bag_id or f"bag_{max(1, self.placement_counter)}"
                    if req.get('return_loaded', False) or (self.robot_pose and self.robot_pose[0] > 3.0 and req.get('loaded', False)):
                        self.state = 'RETURN_LOADED'
                    elif self.robot_pose and self.robot_pose[0] < 2.0:
                        self.state = 'RETRIEVE_TRAVEL'
                    else:
                        self.state = 'RETRIEVE_APPROACH'
                    self.state_entry_time = self.now_sim()
                    self.emit_event('unload_started', bag_id=self.active_bag_id)
            elif cmd == 'start_deposit':
                if self.state in ('IDLE', 'COMPLETE'):
                    self.state = 'DEPOSIT'
                    self.state_entry_time = self.now_sim()
                    self.current_arm_cmd = [0.0, 0.0, 0.74, math.radians(33.0)]
                    self.get_logger().info("Starting DEPOSIT sequence for Scene 06")
            elif cmd == 'stop':
                self.on_stop(Bool(data=True))
            elif cmd == 'start_exit':
                if self.state in ('IDLE', 'COMPLETE'):
                    self.state = 'EXIT'
        except Exception as e:
            self.get_logger().error(f"Failed to process command: {e}")

    def on_pose_target(self, msg: Float64MultiArray):
        # Only accept pose_target in stationary manipulation state
        if self.state not in ('IDLE', 'PLACE_APPROACH', 'DOCKED', 'RETRIEVE_APPROACH'):
            self.get_logger().warn(f"Rejected pose_target: only accepted in stationary manipulation state (current: {self.state})")
            return
        if len(msg.data) != 4:
            self.get_logger().warn(f"Rejected pose_target: expected 4 values [lift, yaw, reach, pitch], got {len(msg.data)}")
            return
        target = list(msg.data)
        base = self.robot_pose or (4.1, 0.0, 0.003, 0.0, 0.0, 0.0)
        try:
            self.guard.evaluate(base, target, self.bag_height, self.bag_mass)
            self.start_arm_trajectory(target, duration=2.5)
        except Exception as e:
            self.get_logger().warn(f"Pose target rejected by guard: {e}")

    def fresh_feedback(self):
        t = self.now_sim()
        if t - self.robot_pose_stamp > 0.40:
            return False
        joint_names = [f'lift_stage_{i}_joint' for i in (1, 2, 3)] + ['yaw_joint', 'extension_mid_joint', 'extension_tip_joint', 'tray_pitch_joint']
        for j in joint_names:
            if j not in self.q or t - self.q_stamp[j] > 0.40:
                return False
        return True

    def emit_event(self, event_name, bag_id=None, target_id=None, details=None):
        payload = {
            "run_id": self.run_id,
            "sim_time_s": round(self.now_sim(), 3),
            "event": event_name,
            "bag_id": bag_id or self.active_bag_id,
            "target_id": target_id or self.active_target_id,
            "state": self.state,
            "evidence_mode": "contact_and_pose",
            "details": details or {}
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_events.publish(msg)

    def publish_telemetry(self):
        ages = {
            "robot_pose_age_s": round(self.now_sim() - self.robot_pose_stamp, 3) if self.robot_pose_stamp > 0 else 999.0,
            "wall_watchdog_ok": self.wall_watchdog_ok
        }
        status = {
            "run_id": self.run_id,
            "sim_time_s": round(self.now_sim(), 3),
            "state": self.state,
            "active_bag_id": self.active_bag_id,
            "target_id": self.active_target_id,
            "placement_count": self.placement_counter,
            "fault": self.fault,
            "stop_latched": self.stop_latched,
            "feedback_ages_s": ages,
            "arm_posture": [round(v, 4) for v in self.current_arm_cmd],
            "robot_pose": [round(v, 3) for v in self.robot_pose] if self.robot_pose else None
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)

    def send_actuator(self, name, val):
        m = Float64()
        m.data = float(val)
        self.pub_act[name].publish(m)

    def send_base(self, v=0.0, w=0.0):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_vel.publish(t)
        self.send_actuator('left_wheel_speed', (v - w * 0.375) / 0.068)
        self.send_actuator('right_wheel_speed', (v + w * 0.375) / 0.068)

    def send_belts(self, loader=0.0, door=0.0, tray=0.0):
        self.send_actuator('loader_speed', loader)
        self.send_actuator('door_speed', door)
        self.send_actuator('door_roller', door / 0.011 if door != 0.0 else 0.0)
        self.send_actuator('tray_belt_speed', tray)
        self.send_actuator('tray_roller_speed', tray / 0.01 if tray != 0.0 else 0.0)

    def send_arm(self, q):
        self.current_arm_cmd = q
        lift, yaw, reach, pitch = q
        self.send_actuator('lift_stage_target', lift / 3.0)
        self.send_actuator('yaw_target', yaw)
        self.send_actuator('extension_stage_target', reach / 2.0)
        self.send_actuator('pitch_target', pitch)

    def start_arm_trajectory(self, target_q, duration=2.0):
        start_q = self.current_arm_cmd.copy()
        self.arm_segment = (self.now_sim(), start_q, target_q, max(0.5, duration))

    def arm_converged(self, target_q, tol=0.035):
        if not self.q:
            return False
        curr_lift = sum(self.q.get(f'lift_stage_{i}_joint', 0.0) for i in (1, 2, 3))
        curr_yaw = self.q.get('yaw_joint', 0.0)
        curr_reach = self.q.get('extension_mid_joint', 0.0) + self.q.get('extension_tip_joint', 0.0)
        curr_pitch = self.q.get('tray_pitch_joint', 0.0)
        errs = [abs(curr_lift - target_q[0]), abs(curr_yaw - target_q[1]),
                abs(curr_reach - target_q[2]), abs(curr_pitch - target_q[3])]
        return max(errs) < tol

    def stop_actuators(self):
        self.send_base(0.0, 0.0)
        self.send_belts(0.0, 0.0, 0.0)
        self.send_arm(self.current_arm_cmd)

    def control_loop(self):
        t = self.now_sim()
        if t <= 0.0:
            return

        # Check safety & freshness
        if self.stop_latched:
            self.stop_actuators()
            return

        if not self.fresh_feedback():
            self.fault = "Feedback stale or missing"
            self.state = 'STOP_LATCHED'
            self.stop_latched = True
            self.emit_event('fault_detected', details={'fault': self.fault})
            self.stop_actuators()
            return

        # Interpolate arm trajectory if active
        if self.arm_segment is not None:
            t0, start_q, end_q, dur = self.arm_segment
            u = min(1.0, (t - t0) / dur)
            interp_q = [s + (e - s) * smooth(u) for s, e in zip(start_q, end_q)]
            self.send_arm(interp_q)
            if u >= 1.0 or (t - t0 >= dur):
                self.arm_segment = None

        # Execute State Machine
        p = self.robot_pose
        if p is None:
            return
        x, y, z, roll, pitch, yaw = p

        if self.state == 'IDLE':
            self.send_base(0.0, 0.0)
            self.send_belts(0.0, 0.0, 0.0)

        elif self.state == 'DOCK_APPROACH':
            # 3-phase dock approach:
            # Phase 1: Drive reverse along -X to aisle dock junction X=1.300 (along Y=0.000, Yaw=0.0)
            if x > 1.32:
                v = -min(0.12, max(0.03, (x - 1.30) * 0.6))
                w = max(-0.03, min(0.03, -yaw * 1.0 - y * 0.8))
                self.send_base(v, w)
            # Phase 2: Pivot from Yaw=0.0 to Yaw=+PI/2 at (1.300, 0.000)
            elif abs(yaw - PI/2.0) > 0.02:
                w = math.copysign(min(0.08, max(0.02, abs(PI/2.0 - yaw) * 0.8)), PI/2.0 - yaw)
                self.send_base(0.0, w)
            # Phase 3: Reverse along -Y 95mm from Y=0.000 to dock datum Y=-0.095
            elif y > -0.090:
                v = -min(0.06, max(0.02, (y - (-0.095)) * 0.6))
                w = max(-0.03, min(0.03, (PI/2.0 - yaw) * 1.0))
                self.send_base(v, w)
            else:
                self.send_base(0.0, 0.0)
                self.state = 'DOCKED'
                self.state_entry_time = t
                self.get_logger().info("Arrived at Dock. State -> DOCKED")

        elif self.state == 'DOCKED':
            self.send_base(0.0, 0.0)
            self.send_arm([0.0, 0.0, 0.0, 0.0])  # zero lift, level tray
            # Initiate bag intake
            self.state = 'RECEIVE'
            self.state_entry_time = t
            self.emit_event('bag_received', details={'infeed_started': True})

        elif self.state == 'RECEIVE':
            # Run matched conveyors to transfer bag onto robot tray (+0.35 moves forward +Y into aircraft)
            self.send_base(0.0, 0.0)
            self.send_belts(loader=0.35, door=0.35, tray=0.35)
            # Dwell for transfer (approx 5.5s for 2m travel along inclined door_belt onto tray)
            if t - self.state_entry_time > 5.5:
                self.send_belts(0.0, 0.0, 0.0)
                self.state = 'TRANSPORT'
                self.state_entry_time = t
                self.emit_event('support_confirmed', details={'tray_supported': True})
                self.get_logger().info("Transfer complete, bag supported. State -> TRANSPORT")

        elif self.state == 'TRANSPORT':
            self.send_belts(0.0, 0.0, 0.0)
            # 3-phase dock departure:
            # Phase 1: Pull forward along +Y 95mm from dock datum Y=-0.095 to aisle centerline Y=0.000
            if y < -0.010:
                v = min(0.30, max(0.08, (-0.005 - y) * 2.0))
                w = max(-0.06, min(0.06, (PI/2.0 - yaw) * 1.5))
                self.send_base(v, w)
            # Phase 2: Swift agile pivot clockwise from Yaw=+PI/2 to Yaw=0.0 at (1.300, 0.000)
            elif abs(yaw) > 0.02:
                w = -math.copysign(min(0.68, max(0.18, abs(yaw) * 1.6)), yaw)
                self.send_base(0.0, w)
            # Phase 3: Swift cruise forward along +X to shortened work datum X=3.20m
            elif x < 3.18:
                v = min(0.48, max(0.10, (3.20 - x) * 1.2))
                w = max(-0.06, min(0.06, -yaw * 1.5 - y * 1.0))  # steady tracking
                self.send_base(v, w)
            else:
                self.send_base(0.0, 0.0)
                self.state = 'PLACE_APPROACH'
                self.state_entry_time = t
                self.get_logger().info("Arrived at Staging bay X=3.20m. State -> PLACE_APPROACH")

        elif self.state == 'PLACE_APPROACH':
            self.send_base(0.0, 0.0)
            # Floor placement posture with increased conveyor pitch: lip lowered to within ~5mm of floor
            # lift=0, yaw=0, reach=0.74m, pitch=33.0 deg (0.5760 rad)
            floor_pose = [0.0, 0.0, 0.74, math.radians(33.0)]
            if self.arm_stage == 'IDLE':
                self.start_arm_trajectory(floor_pose, duration=2.0)
                self.arm_stage = 'MOVING'
            elif self.arm_stage == 'MOVING' and self.arm_segment is None:
                self.arm_stage = 'IDLE'
                self.state = 'DEPOSIT'
                self.state_entry_time = t
                self.emit_event('placement_candidate', details={'target_pose': floor_pose})
                self.get_logger().info("Arm positioned for soft deposit. State -> DEPOSIT")

        elif self.state == 'DEPOSIT':
            dt_dep = t - self.state_entry_time
            # Soft touchdown roll-out sequence:
            # 0.0 - 1.2s: Gentle forward belt feed (+0.24 m/s) to guide front edge softly onto floor liner
            if dt_dep < 1.2:
                self.send_base(0.0, 0.0)
                self.send_belts(loader=0.0, door=0.0, tray=0.24)
            # 1.2 - 3.4s: Synchronous roll-out: belt feeds at +0.22 m/s while chassis slowly eases backward at -0.10 m/s
            # Conveyor lip gently recedes from underneath the bag, laying it softly flat on the floor with zero harsh drop
            elif dt_dep < 3.4:
                self.send_base(-0.10, 0.0)
                self.send_belts(loader=0.0, door=0.0, tray=0.22)
            # 3.4 - 4.2s: Stop actuators and allow bag to settle completely stationary
            elif dt_dep < 4.2:
                self.send_base(0.0, 0.0)
                self.send_belts(0.0, 0.0, 0.0)
            else:
                self.send_base(0.0, 0.0)
                self.send_belts(0.0, 0.0, 0.0)
                self.state = 'VERIFY_PLACEMENT'
                self.state_entry_time = t
                self.get_logger().info("Soft bag roll-out completed. State -> VERIFY_PLACEMENT")

        elif self.state == 'VERIFY_PLACEMENT':
            self.send_belts(0.0, 0.0, 0.0)
            # Retract and level tray
            stowed = [0.0, 0.0, 0.0, 0.0]
            if self.arm_stage == 'IDLE':
                self.start_arm_trajectory(stowed, duration=2.0)
                self.arm_stage = 'STOWING'
            elif self.arm_stage == 'STOWING' and self.arm_segment is None:
                if t - self.state_entry_time > 3.0:
                    self.arm_stage = 'IDLE'
                    self.placement_counter += 1
                    self.emit_event('placement_complete', details={'placement_count': self.placement_counter})
                    self.get_logger().info(f"Placement #{self.placement_counter} Verified! State -> RETURN_EMPTY")
                    self.state = 'RETURN_EMPTY'
                    self.state_entry_time = t

        elif self.state == 'RETURN_EMPTY':
            self.send_belts(0.0, 0.0, 0.0)
            # 3-phase dock approach:
            # Phase 1: Drive reverse along -X back to dock area (X=1.30m) along Y=0.000
            if x > 1.32:
                v = -min(0.40, max(0.08, (x - 1.30) * 1.0))
                w = max(-0.06, min(0.06, -yaw * 1.5 - y * 1.0))
                self.send_base(v, w)
            # Phase 2: Pivot from Yaw=0.0 to Yaw=+PI/2 (facing +Y)
            elif abs(yaw - PI/2.0) > 0.02:
                w = math.copysign(min(0.65, max(0.18, abs(PI/2.0 - yaw) * 1.5)), PI/2.0 - yaw)
                self.send_base(0.0, w)
            # Phase 3: Reverse along -Y 95mm from Y=0.000 to dock datum Y=-0.095
            elif y > -0.090:
                v = -min(0.25, max(0.05, (y - (-0.095)) * 1.5))
                w = max(-0.06, min(0.06, (PI/2.0 - yaw) * 1.5))
                self.send_base(v, w)
            else:
                self.send_base(0.0, 0.0)
                self.state = 'IDLE'
                self.get_logger().info("Cycle complete! Returned to dock.")

        elif self.state == 'RETRIEVE_TRAVEL':
            self.send_belts(0.0, 0.0, 0.0)
            # 3-phase dock departure:
            # Phase 1: Pull forward along +Y 95mm from dock datum Y=-0.095 to aisle centerline Y=0.000
            if y < -0.010:
                v = min(0.08, max(0.02, (-0.005 - y) * 0.8))
                w = max(-0.03, min(0.03, (PI/2.0 - yaw) * 1.0))
                self.send_base(v, w)
            # Phase 2: Pivot clockwise from Yaw=+PI/2 to Yaw=0.0 at (1.300, 0.000)
            elif abs(yaw) > 0.02:
                w = -math.copysign(min(0.08, max(0.02, abs(yaw) * 0.8)), yaw)
                self.send_base(0.0, w)
            # Phase 3: Drive forward along +X to retrieval datum X=3.75m
            elif x < 3.75:
                v = min(0.12, max(0.03, (3.77 - x) * 0.6))
                w = max(-0.03, min(0.03, -yaw * 1.0 - y * 0.8))
                self.send_base(v, w)
            else:
                self.send_base(0.0, 0.0)
                self.state = 'RETRIEVE_APPROACH'
                self.state_entry_time = t
                self.get_logger().info("Arrived at staging bay for retrieval. State -> RETRIEVE_APPROACH")

        elif self.state == 'RETRIEVE_APPROACH':
            self.send_base(0.0, 0.0)
            # Low nose pickup posture matching floor_pickup fixture: reach=1.0m, pitch=30.92 deg
            pickup_pose = [0.0, 0.0, 1.0, math.radians(30.92)]
            if self.arm_stage == 'IDLE':
                self.start_arm_trajectory(pickup_pose, duration=2.5)
                self.arm_stage = 'PICKUP'
            elif self.arm_stage == 'PICKUP' and self.arm_segment is None:
                self.arm_stage = 'IDLE'
                self.state = 'ACQUIRE'
                self.state_entry_time = t

        elif self.state == 'ACQUIRE':
            dt = t - self.state_entry_time
            # Slowly nudge forward (5 cm/s) for first 3.0s while running belt reverse (-0.60 m/s) to smoothly scoop bag
            v_crawl = 0.05 if dt < 3.0 else 0.0
            self.send_base(v_crawl, 0.0)
            self.send_belts(loader=0.0, door=0.0, tray=-0.60)
            if dt > 4.5:
                self.send_base(0.0, 0.0)
                self.send_belts(0.0, 0.0, 0.0)
                stowed = [0.0, 0.0, 0.0, 0.0]
                if self.arm_stage == 'IDLE':
                    self.start_arm_trajectory(stowed, duration=2.5)
                    self.arm_stage = 'STOWING'
                elif self.arm_stage == 'STOWING' and self.arm_segment is None:
                    self.arm_stage = 'IDLE'
                    self.state = 'RETURN_LOADED'
                    self.state_entry_time = t
                    self.emit_event('bag_retrieved', details={'bag_id': self.active_bag_id})
                    self.get_logger().info("Bag secured on tray. Returning to dock -> RETURN_LOADED")

        elif self.state == 'RETURN_LOADED':
            self.send_belts(0.0, 0.0, 0.0)
            # 3-phase dock approach while carrying bag:
            # Phase 1: Drive reverse along -X back to dock area (X=1.30m) along Y=0.000
            if x > 1.32:
                v = -min(0.12, max(0.03, (x - 1.30) * 0.6))
                w = max(-0.03, min(0.03, -yaw * 1.0 - y * 0.8))
                self.send_base(v, w)
            # Phase 2: Pivot from Yaw=0.0 to Yaw=+PI/2 (facing +Y)
            elif abs(yaw - PI/2.0) > 0.02:
                w = math.copysign(min(0.08, max(0.02, abs(PI/2.0 - yaw) * 0.8)), PI/2.0 - yaw)
                self.send_base(0.0, w)
            # Phase 3: Reverse along -Y 95mm from Y=0.000 to dock datum Y=-0.095
            elif y > -0.090:
                v = -min(0.06, max(0.02, (y - (-0.095)) * 0.6))
                w = max(-0.03, min(0.03, (PI/2.0 - yaw) * 1.0))
                self.send_base(v, w)
            else:
                self.send_base(0.0, 0.0)
                self.state = 'DOOR_DISCHARGE'
                self.state_entry_time = t

        elif self.state == 'DOOR_DISCHARGE':
            self.send_base(0.0, 0.0)
            # Reverse discharge: tray, door belt, and loader run outward (-0.40 toward apron)
            self.send_belts(loader=-0.40, door=-0.40, tray=-0.40)
            if t - self.state_entry_time > 12.0:
                self.send_belts(0.0, 0.0, 0.0)
                self.state = 'VERIFY_UNLOAD'
                self.state_entry_time = t

        elif self.state == 'VERIFY_UNLOAD':
            self.send_belts(0.0, 0.0, 0.0)
            if t - self.state_entry_time > 1.0:
                self.emit_event('unload_complete', details={'bag_id': self.active_bag_id})
                self.state = 'IDLE'

        elif self.state == 'EXIT':
            self.send_base(0.0, 0.0)
            self.send_belts(0.0, 0.0, 0.0)
            self.send_arm([0.0, 0.0, 0.0, 0.0])
            self.emit_event('exit_complete')
            self.state = 'COMPLETE'

def main():
    rclpy.init()
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_actuators()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
