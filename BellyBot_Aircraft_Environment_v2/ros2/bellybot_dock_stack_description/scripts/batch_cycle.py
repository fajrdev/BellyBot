#!/usr/bin/env python3
"""Fixed-layer test sequence; requires real sensor decisions, no fake bag flags.
Precondition: base stopped, yaw=0, pitch=0, extension=0, tray/infeed height=.1m.
Supervisor must have retract_belt_sync=true; commissioning default runs slower.
"""
import csv,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool,Float64,Float64MultiArray
from sensor_msgs.msg import JointState
from control_core import duration_for,clamp
class Batch(Node):
 def __init__(self):
  super().__init__('dock_stack_batch_test');self.declare_parameter('speed_scale',.25);self.declare_parameter('log_path','batch_results.csv')
  self.stage='WAIT_READY';self.began=0.;self.q={};self.q_stamps={};self.flags={};self.count=0;self.started=None
  self.motion_active=True;self.motion_seen=False;self.motion_stamp=-1e9
  self.goal=self.create_publisher(Float64MultiArray,'/bellybot/pose_target',10)
  self.belt=self.create_publisher(Float64,'/bellybot/belt_cmd',10);self.stop=self.create_publisher(Bool,'/bellybot/stop',10)
  self.create_subscription(JointState,'/joint_states',self.feedback,10)
  self.create_subscription(JointState,'/infeed_joint_states',self.feedback,10)
  self.create_subscription(Bool,'/bellybot/motion_active',self.motion,10)
  for n in ['bag_ready','bag_supported','bag_placed']:
   self.create_subscription(Bool,'/bellybot/'+n,lambda m,n=n:self.flags.update({n:(m.data,self.now())}),10)
  path=Path(str(self.get_parameter('log_path').value)).expanduser();path.parent.mkdir(parents=True,exist_ok=True)
  self.file=path.open('a',newline='');self.csv=csv.writer(self.file)
  if path.stat().st_size==0:self.csv.writerow(['sim_time_s','event','bags_completed','elapsed_s'])
  self.create_timer(.02,self.tick)
 def now(self):return self.get_clock().now().nanoseconds*1e-9
 def feedback(self,m):
  if len(m.name)!=len(m.position):return
  self.q.update(zip(m.name,m.position));self.q_stamps.update({n:self.now() for n in m.name})
 def motion(self,m):
  self.motion_active=m.data;self.motion_stamp=self.now()
  if self.stage in ['EXTEND','DEPOSIT_RETRACT'] and m.data:self.motion_seen=True
 def ready(self,flag):
  v,stamp=self.flags.get(flag,(False,-1e9));return v and stamp>=self.began
 def transition(self,s):
  self.stage=s;self.began=self.now();self.motion_seen=False;self.csv.writerow([self.began,s,self.count,'' if self.started is None else self.began-self.started]);self.file.flush()
 def pose(self,e):self.goal.publish(Float64MultiArray(data=[.1,0.,e,.4,0.,.1]))
 def halt(self,reason):
  self.belt.publish(Float64(data=0.));self.stop.publish(Bool(data=True));self.get_logger().error(reason);self.transition('FAULT')
 def tick(self):
  t=self.now();scale=clamp(float(self.get_parameter('speed_scale').value),.05,1.)
  required=[f'lift_stage_{i}_joint' for i in [1,2,3]]+['yaw_joint','extension_mid_joint','extension_tip_joint','tray_pitch_joint','infeed_lift_joint','infeed_extension_joint']
  if self.stage=='FAULT':return
  if any(k not in self.q or t-self.q_stamps.get(k,-1e9)>.5 for k in required) or t-self.motion_stamp>.5:
   if self.stage!='WAIT_READY':self.halt('Joint feedback lost during batch')
   return
  e=self.q['extension_mid_joint']+self.q['extension_tip_joint']
  level=sum(self.q[f'lift_stage_{i}_joint'] for i in [1,2,3])
  aligned=abs(level-.1)<.008 and abs(self.q['infeed_lift_joint']-.1)<.008 and abs(self.q['infeed_extension_joint']-.4)<.01 and abs(self.q['yaw_joint'])<.01 and abs(self.q['tray_pitch_joint'])<.01
  if not aligned:
   if self.stage!='WAIT_READY':self.halt('Batch alignment lost')
   return
  age=t-self.began
  if self.stage=='WAIT_READY':
   if self.ready('bag_ready') and e<.005 and not self.motion_active:
    if self.started is None:self.started=t
    self.transition('RECEIVE')
  elif self.stage=='RECEIVE':
   T=duration_for(.9,1.5,3.)/scale;u=clamp(age/T,0,1)
   v=.9*(30*u*u-60*u**3+30*u**4)/T;self.belt.publish(Float64(data=v))
   if age>=T:
    self.belt.publish(Float64(data=0.));self.transition('CONFIRM_SUPPORTED')
  elif self.stage=='CONFIRM_SUPPORTED':
   # Sensor confirmation must be produced now; a stale readiness message is insufficient.
   if self.ready('bag_supported') and age>.02 and not self.motion_active:self.pose(.8);self.transition('EXTEND')
   elif age>5:self.halt('No full-support confirmation')
  elif self.stage=='EXTEND':
   if self.motion_seen and not self.motion_active and abs(e-.8)<.005:self.pose(0.);self.transition('DEPOSIT_RETRACT')
   elif age>15/scale:self.halt('Extension timeout')
  elif self.stage=='DEPOSIT_RETRACT':
   if self.motion_seen and not self.motion_active and e<.005:self.transition('VERIFY')
   elif age>15/scale:self.halt('Retraction timeout')
  elif self.stage=='VERIFY':
   if self.ready('bag_placed') and age>=.2:self.count+=1;self.transition('WAIT_READY')
   elif age>5:self.halt('No stable-placement confirmation')
def main():
 rclpy.init();n=Batch()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.belt.publish(Float64(data=0.));n.stop.publish(Bool(data=True));n.file.close();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
