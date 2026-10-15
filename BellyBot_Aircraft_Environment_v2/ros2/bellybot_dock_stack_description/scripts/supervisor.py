#!/usr/bin/env python3
"""Commissioning controls, smooth targets, roof limit and command watchdog.

Input /bellybot/pose_target: Float64MultiArray [lift_total,yaw,reach_total,infeed,pitch,infeed_height].
Input /bellybot/cmd_vel: Twist. /bellybot/belt_cmd: Float64 (manual feed speed).
Feedback holds on fault; this is not a safety-rated controller or bag policy.
"""
import math,time
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64,Float64MultiArray,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from control_core import clamp,lift_limit,plan,duration_for,smoothstep,wheel_speeds,segment_duration

class Supervisor(Node):
    def __init__(self):
        super().__init__('dock_stack_supervisor')
        self.declare_parameter('bag_height',.28);self.declare_parameter('local_roof',1.18)
        self.declare_parameter('retract_belt_sync',False)
        self.declare_parameter('speed_scale',.25)
        self.declare_parameter('deposit_belt_ratio',1.78125)
        self.q=[0.,0.,0.,.4,0.,0.];self.measured={};self.feedback_times={};self.last_feedback=0.;self.last_drive=0.;self.last_belt=0.
        self.target_drive=[0.,0.];self.drive=[0.,0.];self.belt=0.;self.estop=False
        self.queue=deque();self.segment=None;self.last_sim=None
        names=['lift_stage_target','extension_stage_target','yaw_target','pitch_target','tray_belt_speed','tray_roller_speed','left_wheel_speed','right_wheel_speed']
        self.pub={n:self.create_publisher(Float64,'/bellybot/gz/'+n,10) for n in names}
        for n in ['extension_target','lift_target','belt_speed','roller_speed']:
            self.pub['infeed_'+n]=self.create_publisher(Float64,'/bellybot/gz/infeed_'+n,10)
        self.drive_pub=self.create_publisher(Twist,'/bellybot/gz/cmd_vel',10)
        self.motion_pub=self.create_publisher(Bool,'/bellybot/motion_active',10)
        self.create_subscription(Float64MultiArray,'/bellybot/pose_target',self.pose,10)
        self.create_subscription(Twist,'/bellybot/cmd_vel',self.velocity,10)
        self.create_subscription(Float64,'/bellybot/belt_cmd',self.belt_command,10)
        self.create_subscription(Bool,'/bellybot/stop',self.stop,10)
        self.create_subscription(JointState,'/joint_states',self.feedback,20)
        self.create_subscription(JointState,'/infeed_joint_states',self.feedback,20)
        self.create_timer(.02,self.tick)
    def feedback(self,msg):
        if len(msg.name)!=len(msg.position) or not all(math.isfinite(x) for x in msg.position):return
        self.measured.update(zip(msg.name,msg.position));now=time.monotonic()
        self.feedback_times.update({n:now for n in msg.name})
        required=[f'lift_stage_{i}_joint' for i in [1,2,3]]+['yaw_joint','extension_mid_joint','extension_tip_joint','infeed_extension_joint','tray_pitch_joint','infeed_lift_joint']
        self.last_feedback=min(self.feedback_times.get(n,0) for n in required)
    def current(self):
        m=self.measured
        return [sum(m.get(f'lift_stage_{i}_joint',self.q[0]/3) for i in [1,2,3]),m.get('yaw_joint',self.q[1]),
                m.get('extension_mid_joint',self.q[2]/2)+m.get('extension_tip_joint',self.q[2]/2),m.get('infeed_extension_joint',self.q[3]),m.get('tray_pitch_joint',self.q[4]),m.get('infeed_lift_joint',self.q[5])]
    def pose(self,msg):
        if self.estop:return
        if self.segment or self.queue:
            self.get_logger().warning('Motion active; wait for completion before another pose target');return
        if time.monotonic()-self.last_feedback>.5:
            self.get_logger().error('No fresh simulation joint feedback');return
        try:
            self.q=self.current()
            if abs(self.belt)>.001 and time.monotonic()-self.last_belt<.4:raise ValueError('Stop belts before a pose command')
            targets=plan(self.q,list(msg.data),self.get_parameter('bag_height').value,self.get_parameter('local_roof').value)
            self.queue=deque(targets)
        except ValueError as e:self.get_logger().error(str(e))
    def velocity(self,msg):
        if not all(math.isfinite(x) for x in [msg.linear.x,msg.angular.z]):
            self.target_drive=[0,0];return
        self.target_drive=[clamp(msg.linear.x,-.2,.2),clamp(msg.angular.z,-.25,.25)];self.last_drive=time.monotonic()
    def belt_command(self,msg):
        if math.isfinite(msg.data):self.belt=clamp(msg.data,-1.5,1.5);self.last_belt=time.monotonic()
    def stop(self,msg):
        self.estop=msg.data
        if self.estop:self.queue.clear();self.segment=None;self.q=self.current();self.target_drive=[0,0];self.belt=0
    def send(self,name,value):m=Float64();m.data=float(value);self.pub[name].publish(m)
    def tick(self):
        now=self.get_clock().now().nanoseconds/1e9;wall=time.monotonic()
        dt=0 if self.last_sim is None else clamp(now-self.last_sim,0,.1)
        if self.last_sim is not None and now<self.last_sim:
            self.queue.clear();self.segment=None;self.q=self.current()
        self.last_sim=now;stale=wall-self.last_feedback>.5
        if stale or self.estop:
            self.target_drive=[0,0];self.belt=0;self.queue.clear();self.segment=None;self.q=self.current()
        if wall-self.last_drive>.4:self.target_drive=[0,0]
        # Drive only while the handling mechanism is stowed.
        actual=self.current()
        if self.segment or self.queue or actual[0]>.01 or abs(actual[1])>.02 or actual[2]>.01 or actual[4]>.01:self.target_drive=[0,0]
        for i,a in enumerate([.15,.3]):self.drive[i]+=clamp(self.target_drive[i]-self.drive[i],-a*dt,a*dt)
        drive=Twist();drive.linear.x=self.drive[0];drive.angular.z=self.drive[1];self.drive_pub.publish(drive)
        left,right=wheel_speeds(*self.drive);self.send('left_wheel_speed',left);self.send('right_wheel_speed',right)
        # Wait for a stopped base command before advancing a pose trajectory.
        if self.segment is None and self.queue and max(abs(x) for x in self.drive)<.002:
            target=self.queue.popleft()
            duration=segment_duration(self.q,target)
            if self.get_parameter('retract_belt_sync').value and target[2]<self.q[2] and target[4]<.001:
                ratio=clamp(float(self.get_parameter('deposit_belt_ratio').value),1.,2.)
                duration=max(duration,duration_for((self.q[2]-target[2])*ratio,1.5,3.))
            duration=duration/clamp(float(self.get_parameter("speed_scale").value),.05,1.)
            self.segment=(list(self.q),target,now,duration)
        reach_speed=0.
        if self.segment:
            start,target,began,duration=self.segment;u=clamp((now-began)/duration,0,1)
            blend=smoothstep(u);self.q=[s+(t-s)*blend for s,t in zip(start,target)]
            reach_speed=(target[2]-start[2])*(30*u*u-60*u**3+30*u**4)/duration
            if u>=1:
                if all(abs(a-b)<tol for a,b,tol in zip(actual,target,[.008,.02,.008,.01,.01,.008])):self.segment=None
                elif now-began>duration+3:
                    self.get_logger().error('Joint tracking timeout; stopping sequence');self.queue.clear();self.segment=None;self.q=actual
        lift,yaw,reach,feed,pitch,fh=self.q
        self.send('lift_stage_target',lift/3);self.send('yaw_target',yaw);self.send('extension_stage_target',reach/2)
        self.send('infeed_extension_target',feed);self.send('pitch_target',pitch);self.send('infeed_lift_target',fh)
        belt=self.belt if wall-self.last_belt<.4 and not stale and not self.estop else 0.
        if self.get_parameter('retract_belt_sync').value and reach_speed<0 and pitch<.001 and not stale and not self.estop:belt=-reach_speed*clamp(float(self.get_parameter("deposit_belt_ratio").value),1.,2.)
        if (self.segment or self.queue) and not (self.get_parameter('retract_belt_sync').value and reach_speed<0 and pitch<.001):belt=0.
        self.send('tray_belt_speed',belt);self.send('tray_roller_speed',belt/.01)
        receive=abs(lift-fh)<.008 and pitch<.001 and abs(yaw)<.02 and reach<.01 and max(abs(x) for x in self.drive)<.002
        feedbelt=belt if receive else 0.
        self.send('infeed_belt_speed',feedbelt);self.send('infeed_roller_speed',feedbelt/.01)
        self.motion_pub.publish(Bool(data=bool(self.segment or self.queue)))

def main():
    rclpy.init();n=Supervisor()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.target_drive=[0,0];n.drive=[0,0];n.estop=True;n.belt=0;n.tick()
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
