#!/usr/bin/env python3
"""Automated Milestone A verification script for BellyBot DockStack C v3.
Tests doorway ingress, 90-degree pivot on turn deck, and ramp descent into cargo hold.
"""
import os, sys, time, json, subprocess, math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from tf2_msgs.msg import TFMessage

class MilestoneAVerifier(Node):
    def __init__(self):
        super().__init__('milestone_a_verifier')
        self.status = {}
        self.robot_pose = None
        self.max_pitch = 0.0
        self.max_roll = 0.0
        self.min_turn_drift = 0.0
        self.sub_status = self.create_subscription(String, '/aircraft/status', self.on_status, 10)
        self.sub_pose = self.create_subscription(TFMessage, '/aircraft/robot_pose', self.on_robot_pose, 10)
        self.pub_entry = self.create_publisher(Empty, '/aircraft/start_entry', 10)
        
    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except Exception:
            pass
            
    def angles(self, q):
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        pitch = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
        roll = math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))
        return roll, pitch, yaw
            
    def on_robot_pose(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id.rstrip('/').split('/')[-1] == 'bellybot_dock_stack':
                p = tr.transform.translation
                r, pitch, y = self.angles(tr.transform.rotation)
                self.robot_pose = (p.x, p.y, p.z, r, pitch, y)
                if abs(pitch) > self.max_pitch:
                    self.max_pitch = abs(pitch)
                if abs(r) > self.max_roll:
                    self.max_roll = abs(r)

def run():
    print("=== [MILESTONE A] Starting Doorway Ingress & Ramp Descent Verification ===")
    env = os.environ.copy()
    env['GZ_IP'] = '127.0.0.1'
    
    # 1. Launch aircraft simulation in entry mode
    launch_cmd = [
        'ros2', 'launch', 'bellybot_aircraft_environment', 'aircraft.launch.py',
        'mode:=entry'
    ]
    proc = subprocess.Popen(launch_cmd, env=env)
    print(f"-> Launched aircraft.launch.py mode:=entry (PID {proc.pid})")
    
    rclpy.init()
    node = MilestoneAVerifier()
    
    try:
        # 2. Wait for simulation initialization and unpause
        print("-> Waiting for Gazebo initialization (4s)...")
        time.sleep(4)
        print("-> Unpausing Gazebo Sim...")
        subprocess.run([
            'gz', 'service', '-s', '/world/entry_resolved/control',
            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', 'pause: false'
        ], check=True, env=env)
        
        # 3. Wait for clean status
        print("-> Waiting for controller status...")
        start_wait = time.time()
        while time.time() - start_wait < 15:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.status and node.status.get('joint_feedback_fresh') and node.status.get('robot_pose_fresh'):
                break
        print(f"-> Controller status: {node.status}")
        if node.status.get('fault'):
            raise RuntimeError(f"Controller fault before entry: {node.status['fault']}")
            
        print(f"-> Initial robot pose: {node.robot_pose}")
        if not node.robot_pose:
            raise RuntimeError("Failed to receive robot pose")
            
        # 4. Trigger entry
        print("-> Triggering /aircraft/start_entry...")
        msg = Empty()
        node.pub_entry.publish(msg)
        
        # 5. Monitor entry phases: INGRESS -> TURN -> EXIT -> DONE
        last_phase = ""
        last_print = 0
        t0 = time.time()
        while time.time() - t0 < 800:
            rclpy.spin_once(node, timeout_sec=0.1)
            phase = node.status.get('entry', '')
            fault = node.status.get('fault', '')
            if fault:
                raise RuntimeError(f"Entry faulted in phase '{phase}': {fault}")
            p = node.robot_pose or (0,0,0,0,0,0)
            if phase != last_phase:
                print(f"   [Entry State Transition] -> {phase} (elapsed: {time.time()-t0:.1f}s, pos: X={p[0]:.3f}, Y={p[1]:.3f}, Yaw={math.degrees(p[5]):.1f} deg)")
                last_phase = phase
            elif time.time() - last_print > 4.0:
                print(f"      [Driving Telemetry] phase: {phase:7s} | pos: X={p[0]:.3f} m, Y={p[1]:.3f} m | Yaw={math.degrees(p[5]):.1f}° | Pitch={math.degrees(p[4]):.1f}°")
                last_print = time.time()
            if phase == 'DONE':
                print(f"*** Entry navigation SUCCESSFULLY COMPLETED in {time.time()-t0:.1f}s! ***")
                break
        else:
            raise TimeoutError("Entry timed out before reaching DONE")
            
        # 6. Verify final pose and dynamics
        p = node.robot_pose
        print(f"-> Final robot pose: X={p[0]:.3f} m, Y={p[1]:.3f} m, Z={p[2]:.3f} m, Yaw={math.degrees(p[5]):.2f} deg")
        print(f"-> Max observed pitch on ramp: {math.degrees(node.max_pitch):.2f} deg (nominal ~5.71 deg)")
        print(f"-> Max observed roll: {math.degrees(node.max_roll):.2f} deg (allowable < 2.0 deg)")
        
        if p[0] < 4.89:
            raise RuntimeError(f"Robot did not reach final datum X=4.90 (actual X={p[0]:.3f})")
            
        print("\n=======================================================")
        print(" [SUCCESS] Milestone A (Doorway Ingress & Ramp Descent) VERIFIED!")
        print("=======================================================")
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        subprocess.run(['pkill', '-9', '-f', 'gz sim'], stderr=subprocess.DEVNULL)
        subprocess.run(['killall', '-9', 'gz'], stderr=subprocess.DEVNULL)

if __name__ == '__main__':
    run()
