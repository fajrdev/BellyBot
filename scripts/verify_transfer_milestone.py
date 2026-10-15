#!/usr/bin/env python3
"""Automated Milestone B verification script for BellyBot DockStack C v3.
Tests connected baggage transfer and articulated tilting tray motion.
"""
import os, sys, time, json, subprocess, math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String, Float64MultiArray
from tf2_msgs.msg import TFMessage

class MilestoneBVerifier(Node):
    def __init__(self):
        super().__init__('milestone_b_verifier')
        self.status = {}
        self.tracked_bag_pose = None
        self.sub_status = self.create_subscription(String, '/aircraft/status', self.on_status, 10)
        self.sub_poses = self.create_subscription(TFMessage, '/aircraft/world_poses', self.on_world_poses, 10)
        self.pub_transfer = self.create_publisher(Empty, '/aircraft/start_transfer', 10)
        self.pub_pose_target = self.create_publisher(Float64MultiArray, '/aircraft/pose_target', 10)
        
    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except Exception:
            pass
            
    def on_world_poses(self, msg):
        for tr in msg.transforms:
            if 'tracked_bag' in tr.child_frame_id:
                p = tr.transform.translation
                self.tracked_bag_pose = (p.x, p.y, p.z)

def run():
    print("=== [MILESTONE B] Starting Transfer & Tilting Tray Verification ===")
    env = os.environ.copy()
    env['GZ_IP'] = '127.0.0.1'
    
    # 1. Launch aircraft simulation in background
    launch_cmd = [
        'ros2', 'launch', 'bellybot_aircraft_environment', 'aircraft.launch.py',
        'mode:=transfer', 'cutaway:=true'
    ]
    proc = subprocess.Popen(launch_cmd, env=env)
    print(f"-> Launched aircraft.launch.py (PID {proc.pid})")
    
    rclpy.init()
    node = MilestoneBVerifier()
    
    try:
        # 2. Wait for simulation initialization and unpause
        print("-> Waiting for Gazebo initialization (4s)...")
        time.sleep(4)
        print("-> Unpausing Gazebo Sim...")
        subprocess.run([
            'gz', 'service', '-s', '/world/transfer_cutaway/control',
            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', 'pause: false'
        ], check=True, env=env)
        
        # 3. Wait for clean status
        print("-> Waiting for controller status...")
        start_wait = time.time()
        while time.time() - start_wait < 10:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.status and node.status.get('joint_feedback_fresh') and node.status.get('robot_pose_fresh'):
                break
        print(f"-> Controller status: {node.status}")
        if node.status.get('fault'):
            raise RuntimeError(f"Controller fault before transfer: {node.status['fault']}")
            
        # 4. Spawn tracked bag
        print("-> Spawning 32 kg test bag on door belt...")
        spawn_script = '/home/fajrda/sutd-hackathon/BellyBot_Aircraft_Environment_v2/tools/spawn_transfer_bag.py'
        subprocess.run([
            'python3', spawn_script, '--world', 'transfer_cutaway', '--bag', 'bag_hard_32', '--execute'
        ], check=True, env=env)
        
        # 5. Wait for tracked_bag pose
        print("-> Waiting for tracked_bag pose...")
        start_wait = time.time()
        while time.time() - start_wait < 8:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.tracked_bag_pose:
                break
        print(f"-> Tracked bag detected at: {node.tracked_bag_pose}")
        if not node.tracked_bag_pose:
            raise RuntimeError("Failed to detect tracked_bag pose in world")
            
        # 6. Trigger transfer
        print("-> Triggering /aircraft/start_transfer...")
        msg = Empty()
        node.pub_transfer.publish(msg)
        
        # 7. Monitor transfer phases
        last_phase = ""
        t0 = time.time()
        transfer_start_sim_time = time.time()
        last_print_sec = -1
        while time.time() - t0 < 450:
            rclpy.spin_once(node, timeout_sec=0.1)
            phase = node.status.get('transfer', '')
            fault = node.status.get('fault', '')
            if fault:
                raise RuntimeError(f"Transfer faulted in phase '{phase}': {fault}")
            if phase != last_phase:
                print(f"   [Transfer State] -> {phase} (elapsed: {time.time()-t0:.1f}s)")
                last_phase = phase
            if phase == 'TRANSFER' and int(time.time() - t0) != last_print_sec and int(time.time() - t0) % 5 == 0 and node.tracked_bag_pose:
                last_print_sec = int(time.time() - t0)
                print(f"      [TRANSFER progress] bag x={node.tracked_bag_pose[0]:.3f}, y={node.tracked_bag_pose[1]:.3f}, z={node.tracked_bag_pose[2]:.3f}", flush=True)
            if phase == 'DONE':
                print(f"*** Baggage transfer SUCCESSFULLY COMPLETED in {time.time()-t0:.1f}s! ***")
                break
        else:
            raise TimeoutError("Transfer timed out before reaching DONE")
            
        # Check bag pose on robot tray
        print(f"-> Final bag pose on robot: {node.tracked_bag_pose}")
        
        def wait_arm_trajectory(name, timeout=300):
            print(f"-> Monitoring {name} motion...", flush=True)
            t_start = time.time()
            # 1. Wait for controller to register trajectory (arm_busy -> True)
            while time.time() - t_start < 10:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.status.get('fault'):
                    raise RuntimeError(f"{name} faulted: {node.status['fault']}")
                if node.status.get('arm_busy'):
                    break
            # 2. Wait for trajectory completion (arm_busy -> False)
            completed = False
            last_p = 0
            while time.time() - t_start < timeout:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.status.get('fault'):
                    raise RuntimeError(f"{name} faulted: {node.status['fault']}")
                if not node.status.get('arm_busy', True):
                    completed = True
                    break
                if int(time.time() - t_start) // 10 != last_p:
                    last_p = int(time.time() - t_start) // 10
                    print(f"   [{name}] running: elapsed {time.time()-t_start:.1f}s, arm_busy={node.status.get('arm_busy')}", flush=True)
            if not completed:
                raise TimeoutError(f"{name} timed out after {timeout}s (arm_busy={node.status.get('arm_busy')})")
            print(f"-> {name} completed successfully in {time.time()-t_start:.1f}s!", flush=True)

        # 8. Test Articulated Stacking Motion: Tilting Tray Floor Access
        print("-> Testing Stacking Arm & Articulated Tray Pitch (Floor Pickup at 30.92 deg)...")
        # Target: lift=0, yaw=0, reach=0.86m, pitch=0.5397 rad (~30.92 deg)
        target = Float64MultiArray()
        target.data = [0.0, 0.0, 0.86, 0.5397]
        node.pub_pose_target.publish(target)
        wait_arm_trajectory("Floor pickup posture")
        time.sleep(1)
        
        # Target: Stacking lift test (lift=0.13m cap, yaw=0.5 rad (~28 deg), reach=0.4m, pitch=0)
        print("-> Testing Stacking Mast Lift & Yaw Swing...")
        target_stack = Float64MultiArray()
        target_stack.data = [0.13, 0.50, 0.40, 0.0]
        node.pub_pose_target.publish(target_stack)
        wait_arm_trajectory("Stacking mast lift & yaw swing")
        
        print("\n=======================================================")
        print(" [SUCCESS] Milestone B (Connected Transfer + Stacking) VERIFIED!")
        print("=======================================================")
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
