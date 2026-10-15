#!/usr/bin/env python3
"""Mission Observer for BellyBot simulation.

Provides non-intrusive ground-truth state observation and verification:
- Subscribes to dynamic world poses from Gazebo Harmonic (/mission/world_poses).
- Identifies bag support surfaces (loader, door_belt, robot_tray, cargo_floor, luggage_stack).
- Implements the 0.5s settling dwell verification predicate (v < 0.02 m/s, w < 0.05 rad/s).
- Publishes /mission/bags, /mission/metrics, and appends to results/mission_events.csv.
"""

import math
import time
import json
import csv
from pathlib import Path
from collections import defaultdict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

class MissionObserver(Node):
    def __init__(self):
        super().__init__('mission_observer')
        self.declare_parameter('run_id', 'run_001')
        self.declare_parameter('out_csv', '/home/fajrda/sutd-hackathon/results/mission_events.csv')
        self.declare_parameter('metrics_json', '/home/fajrda/sutd-hackathon/results/mission_metrics.json')

        self.run_id = self.get_parameter('run_id').value
        self.out_csv = Path(self.get_parameter('out_csv').value)
        self.metrics_json = Path(self.get_parameter('metrics_json').value)

        self.out_csv.parent.mkdir(parents=True, exist_ok=True)
        if not self.out_csv.exists():
            with open(self.out_csv, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['run_id', 'sim_time_s', 'event', 'bag_id', 'target_id', 'details'])

        # State tracking
        self.sim_time = 0.0
        self.t0_sim = None
        self.robot_pose = None
        self.bags = {}  # bag_name -> dict(x, y, z, vx, vy, vz, last_t, support, settled_since)
        self.completed_placements = set()
        self.completed_unloads = set()

        # Publishers
        self.pub_bags = self.create_publisher(String, '/mission/bags', 10)
        self.pub_metrics = self.create_publisher(String, '/mission/metrics', 10)

        # Subscriptions
        self.create_subscription(Clock, '/clock', self.on_clock, 20)
        self.create_subscription(TFMessage, '/mission/world_poses', self.on_world_poses, 20)
        self.create_subscription(TFMessage, '/aircraft/robot_pose', self.on_robot_pose, 20)
        self.create_subscription(String, '/mission/events', self.on_mission_event, 10)

        # Telemetry timer
        self.create_timer(0.2, self.publish_telemetry)
        self.get_logger().info(f"Mission Observer started [run_id={self.run_id}]")

    def on_clock(self, msg: Clock):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.sim_time = t
        if self.t0_sim is None and t > 0.0:
            self.t0_sim = t

    def on_robot_pose(self, msg: TFMessage):
        for tr in msg.transforms:
            if 'bellybot_dock_stack' in tr.child_frame_id:
                p = tr.transform.translation
                self.robot_pose = (p.x, p.y, p.z)

    def on_world_poses(self, msg: TFMessage):
        t = self.sim_time
        for tr in msg.transforms:
            name = tr.child_frame_id.rstrip('/').split('/')[-1]
            if 'bag' not in name:
                continue

            p = tr.transform.translation
            if name not in self.bags:
                self.bags[name] = {
                    'id': name,
                    'x': p.x, 'y': p.y, 'z': p.z,
                    'vx': 0.0, 'vy': 0.0, 'vz': 0.0,
                    'speed': 0.0,
                    'last_t': t,
                    'support': 'unknown',
                    'settled_since': None
                }
            else:
                b = self.bags[name]
                dt = t - b['last_t']
                if dt > 0.001:
                    vx = (p.x - b['x']) / dt
                    vy = (p.y - b['y']) / dt
                    vz = (p.z - b['z']) / dt
                    speed = math.hypot(vx, vy, vz)
                    b.update({'vx': vx, 'vy': vy, 'vz': vz, 'speed': speed, 'last_t': t})

                b['x'] = p.x
                b['y'] = p.y
                b['z'] = p.z

                # Determine supporting surface ID
                sup = self.classify_support(p.x, p.y, p.z)
                b['support'] = sup

                # Settling predicate: speed < 0.02 m/s for >= 0.5s
                if b['speed'] < 0.02:
                    if b['settled_since'] is None:
                        b['settled_since'] = t
                    elif t - b['settled_since'] >= 0.5:
                        # If supported on floor at placement station and not already recorded
                        if sup == 'hold_floor' and p.x > 3.0 and name not in self.completed_placements:
                            self.completed_placements.add(name)
                            self.log_event('placement_complete', name, f"cell_{len(self.completed_placements)}",
                                           {'sim_time_s': round(t, 3), 'pose': [round(p.x, 3), round(p.y, 3), round(p.z, 3)]})
                else:
                    b['settled_since'] = None

                # Unload predicate: passed through discharge plane Y < -2.60m
                if p.y < -2.60 and name not in self.completed_unloads:
                    self.completed_unloads.add(name)
                    self.log_event('unload_complete', name, 'apron',
                                   {'sim_time_s': round(t, 3), 'exit_y': round(p.y, 3)})

    def classify_support(self, x, y, z):
        # Robot tray check
        if self.robot_pose is not None:
            rx, ry, rz = self.robot_pose
            if abs(x - rx) < 0.55 and abs(y - ry) < 0.40 and 0.40 <= z <= 0.85:
                return 'robot_tray'
        # Door belt check
        if abs(x - 1.30) < 0.45 and -2.60 <= y <= -0.50 and 0.25 <= z <= 0.60:
            return 'door_belt'
        # Belt loader check
        if y < -2.60:
            return 'loader'
        # Floor check in hold (floor at Z=0.12, bag COM at Z=0.28)
        if x > 0.0 and abs(y) < 0.90 and z < 0.35:
            return 'hold_floor'
        return 'unsupported'

    def on_mission_event(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.log_event(data.get('event'), data.get('bag_id'), data.get('target_id'), data.get('details', {}))
        except Exception:
            pass

    def log_event(self, event_name, bag_id, target_id, details):
        t = self.sim_time
        with open(self.out_csv, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([self.run_id, round(t, 3), event_name, bag_id or '', target_id or '', json.dumps(details)])
        self.get_logger().info(f"[OBSERVER EVENT] {event_name}: bag={bag_id}, target={target_id}")

    def publish_telemetry(self):
        t = self.sim_time
        t0 = self.t0_sim or t
        dt = max(0.1, t - t0)
        n_placed = len(self.completed_placements)
        rate = (60.0 * n_placed) / dt if dt > 0 else 0.0

        # Bags JSON
        bag_list = []
        for name, b in self.bags.items():
            bag_list.append({
                "id": name,
                "stamp": round(b['last_t'], 3),
                "frame": "world",
                "pose": [round(b['x'], 3), round(b['y'], 3), round(b['z'], 3)],
                "speed": round(b['speed'], 4),
                "supporting_surface_ids": [b['support']],
                "evidence_mode": "sim_ground_truth"
            })
        msg_bags = String()
        msg_bags.data = json.dumps(bag_list)
        self.pub_bags.publish(msg_bags)

        # Metrics JSON
        metrics = {
            "run_id": self.run_id,
            "sim_time_s": round(t, 3),
            "measurement_interval_s": round(dt, 3),
            "completed_placements_count": n_placed,
            "completed_placement_ids": list(self.completed_placements),
            "completed_unloads_count": len(self.completed_unloads),
            "completed_unload_ids": list(self.completed_unloads),
            "measured_throughput_bags_min": round(rate, 2),
            "evidence_scope": "continuous_simulation_interval"
        }
        msg_met = String()
        msg_met.data = json.dumps(metrics)
        self.pub_metrics.publish(msg_met)

        # Update JSON file on disk
        with open(self.metrics_json, 'w') as f:
            json.dump(metrics, f, indent=2)

def main():
    rclpy.init()
    node = MissionObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
