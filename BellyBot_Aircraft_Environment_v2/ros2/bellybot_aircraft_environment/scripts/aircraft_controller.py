#!/usr/bin/env python3
"""ROS 2 controller for entry_resolved and transfer_resolved. Start paused.
Only this controller owns actuation in aircraft.launch.py. Gazebo still needs
contact/latency commissioning; controller checks do not certify a real robot.
"""
import json,math,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64,Float64MultiArray,Bool,String,Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from aircraft_guard import Guard,smooth,lift_cap
from transfer_core import Transfer
P=math.pi
class Controller(Node):
 def __init__(self):
  super().__init__('aircraft_controller');self.declare_parameter('mode','transfer');self.mode=self.get_parameter('mode').value;self.declare_parameter('bag_height',.38);self.declare_parameter('bag_mass',32.);self.h=float(self.get_parameter('bag_height').value);self.mass=float(self.get_parameter('bag_mass').value)
  self.guard=Guard(Path(get_package_share_directory('bellybot_aircraft_environment'))/'config/robot_geometry.json');self.q={};self.stamp={};self.pose=None;self.pose_at=-1e9;self.bag=None;self.bag_at=-1e9;self.last_bag=None;self.estop=False;self.queue=[];self.segment=None;self.transfer=Transfer();self.entry='IDLE';self.fault='';self.pub={};self.last_t=None;self.command={};self.command_v={};self.target_arm=[0.,0.,0.,0.]
  for topic in ['/aircraft/robot_joints','/aircraft/corner_joints','/aircraft/buffer_joints']:self.create_subscription(JointState,topic,self.joints,20)
  self.create_subscription(TFMessage,'/aircraft/robot_pose',self.robot_pose,20);self.create_subscription(TFMessage,'/aircraft/world_poses',self.world_poses,20)
  self.create_subscription(Float64MultiArray,'/aircraft/pose_target',self.target,10);self.create_subscription(Bool,'/aircraft/stop',self.stop,10);self.create_subscription(Empty,'/aircraft/start_entry',self.start_entry,10);self.create_subscription(Empty,'/aircraft/start_transfer',self.start_transfer,10)
  self.cmd=self.create_publisher(Twist,'/aircraft/actuation/cmd_vel',10);self.status=self.create_publisher(String,'/aircraft/status',10)
  names=['lift_stage_target','extension_stage_target','yaw_target','pitch_target','tray_belt_speed','tray_roller_speed','left_wheel_speed','right_wheel_speed']
  for n in names:self.pub[n]=self.create_publisher(Float64,'/aircraft/actuation/'+n,10)
  for n in ['corner_stage_1','corner_stage_2','corner_stage_3','corner_angle','buffer_stage_1','buffer_stage_2','bridge_angle','loader_speed','door_speed','corner_speed','bridge_speed','buffer_speed','door_roller','corner_roller','buffer_roller','bridge_roller']:self.pub[n]=self.create_publisher(Float64,'/aircraft/actuation/'+n,10)
  self.create_timer(.02,self.tick);self.create_timer(.5,self.report)
 def now(self):return self.get_clock().now().nanoseconds*1e-9
 def joints(self,m):
  for n,p in zip(m.name,m.position):self.q[n]=p;self.stamp[n]=self.now()
 def angles(self,q):
  yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));pitch=math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x))));roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y));return roll,pitch,yaw
 def robot_pose(self,msg):
  for tr in msg.transforms:
   if tr.child_frame_id.rstrip('/').split('/')[-1]=='bellybot_dock_stack':
    p=tr.transform.translation;roll,pitch,yaw=self.angles(tr.transform.rotation);self.pose=(p.x,p.y,p.z,roll,pitch,yaw);self.pose_at=self.now()
 def world_poses(self,msg):
  for tr in msg.transforms:
   if tr.child_frame_id.rstrip('/').split('/')[-1]!='tracked_bag':continue
   # SceneBroadcaster model poses are world poses; nested-link transforms are excluded.
   if tr.header.frame_id not in ('','world','transfer_resolved','transfer_cutaway'):continue
   p=tr.transform.translation;yaw=self.angles(tr.transform.rotation)[2];t=self.now();speed=0.
   if self.last_bag and t>self.last_bag[0]:speed=math.hypot(p.x-self.last_bag[1],p.y-self.last_bag[2])/(t-self.last_bag[0])
   self.last_bag=(t,p.x,p.y);self.bag={'x':p.x,'y':p.y,'z':p.z,'yaw':yaw,'speed':speed};self.bag_at=t
 def arm(self):return [sum(self.q[f'lift_stage_{i}_joint'] for i in [1,2,3]),self.q['yaw_joint'],self.q['extension_mid_joint']+self.q['extension_tip_joint'],self.q['tray_pitch_joint']]
 def station(self):return {'corner_lift':sum(self.q[f'corner_stage_{i}_joint'] for i in [1,2,3]),'corner_angle':self.q['corner_rotor_joint'],'buffer_lift':self.q['buffer_stage_1_joint']+self.q['buffer_stage_2_joint'],'bridge_angle':self.q['buffer_bridge_joint']}
 def fresh(self,names):return all(n in self.q and self.now()-self.stamp[n]<.35 for n in names)
 def stop(self,m):
  if m.data:self.estop=True;self.queue=[];self.segment=None;self.transfer.fail('Emergency stop');self.entry='STOPPED'
  # A false message never clears a latched stop; restart the node after investigation.
 def target(self,m):
  try:
   if self.mode!='transfer' or self.estop or self.fault:raise ValueError('Arm motion unavailable')
   if self.transfer.state not in ('IDLE','DONE'):raise ValueError('Transfer is not complete')
   s=self.station()
   if s['buffer_lift']>.004 or abs(s['bridge_angle']+P/2)>.02:raise ValueError('Park the buffer before arm motion')
   if self.queue or self.segment:raise ValueError('Wait for current trajectory')
   self.queue=self.guard.plan(self.arm(),list(m.data),self.h,self.mass)
  except (ValueError,KeyError) as e:self.get_logger().warning('Rejected arm target: '+str(e))
 def start_transfer(self,m):
  try:
   if self.mode!='transfer' or self.estop or self.fault or self.queue or self.segment:raise ValueError('Controller not ready')
   if self.now()-self.bag_at>.35:raise ValueError('No fresh tracked_bag world pose')
   self.transfer.start(self.now(),self.arm())
  except (ValueError,KeyError) as e:self.get_logger().warning('Rejected transfer: '+str(e))
 def start_entry(self,m):
  try:
   if self.mode!='entry' or self.estop or self.fault:raise ValueError('Entry mode unavailable')
   if self.now()-self.pose_at>.35 or max(abs(x) for x in self.arm())>.005:raise ValueError('Fresh localization and fully stowed robot required')
   x,y,z,rr,pp,a=self.pose
   if abs(x-1.3)>.008 or abs(y+3.29)>.03 or abs(z-.183)>.02 or abs(a-P/2)>.005:raise ValueError('Robot must start on the docked platform at the supplied pose')
   self.entry='INGRESS'
  except (ValueError,KeyError) as e:self.get_logger().warning('Rejected entry: '+str(e))
 def send(self,k,v):m=Float64();m.data=float(v);self.pub[k].publish(m)
 def base(self,v=0.,w=0.):
  m=Twist();m.linear.x=float(v);m.angular.z=float(w);self.cmd.publish(m);self.send('left_wheel_speed',(float(v)-float(w)*.375)/.068);self.send('right_wheel_speed',(float(v)+float(w)*.375)/.068)
 def belts(self,d=None):
  d=d or {}
  for key in ['loader','door','corner','bridge','buffer']:self.send(key+'_speed',d.get(key,0.))
  self.send('tray_belt_speed',d.get('tray',0.));self.send('tray_roller_speed',d.get('tray',0.)/.01)
  for k in ['door','corner','buffer']:self.send(k+'_roller',d.get(k,0.)/.011)
  self.send('bridge_roller',d.get('bridge',0.)/.005)
 def held(self):
  if all(n in self.q for n in ['lift_stage_1_joint','lift_stage_2_joint','lift_stage_3_joint','yaw_joint','extension_mid_joint','extension_tip_joint','tray_pitch_joint']):self.arm_output(self.target_arm)
  for key,joints in [('corner_stage_1',['corner_stage_1_joint']),('corner_stage_2',['corner_stage_2_joint']),('corner_stage_3',['corner_stage_3_joint']),('corner_angle',['corner_rotor_joint']),('buffer_stage_1',['buffer_stage_1_joint']),('buffer_stage_2',['buffer_stage_2_joint']),('bridge_angle',['buffer_bridge_joint'])]:
   if joints[0] in self.q:self.send(key,self.q[joints[0]])
 def arm_output(self,a):
  for k,v in zip(['lift_stage_target','yaw_target','extension_stage_target','pitch_target'],[a[0]/3,a[1],a[2]/2,a[3]]):self.send(k,v)
 def station_output(self,o,dt):
  target={'corner_stage_1':o['corner_lift']/3,'corner_stage_2':o['corner_lift']/3,'corner_stage_3':o['corner_lift']/3,'corner_angle':o['corner_angle'],'buffer_stage_1':o['buffer_lift']/2,'buffer_stage_2':o['buffer_lift']/2,'bridge_angle':o['bridge_angle']}
  measured={'corner_angle':'corner_rotor_joint','bridge_angle':'buffer_bridge_joint'}
  for k,t in target.items():
   if k not in self.command:self.command[k]=self.q[measured.get(k,k+'_joint')];self.command_v[k]=0.
   vmax=.35 if k=='corner_angle' else (.4 if k=='bridge_angle' else .04);amax=.4 if k in ['corner_angle','bridge_angle'] else .08;err=t-self.command[k]
   des=math.copysign(min(vmax,math.sqrt(2*amax*abs(err))),err) if abs(err)>1e-9 else 0.
   v=self.command_v[k]+max(-amax*dt,min(amax*dt,des-self.command_v[k]));delta=v*dt
   if abs(delta)>abs(err):delta=err;v=0
   self.command[k]+=delta;self.command_v[k]=v;self.send(k,self.command[k])
 def tick(self):
  t=self.now();dt=.02 if self.last_t is None else max(0,min(.05,t-self.last_t));self.last_t=t
  names=[f'lift_stage_{i}_joint' for i in [1,2,3]]+['yaw_joint','extension_mid_joint','extension_tip_joint','tray_pitch_joint']
  if self.mode=='transfer':names += [f'corner_stage_{i}_joint' for i in [1,2,3]]+['corner_rotor_joint','buffer_stage_1_joint','buffer_stage_2_joint','buffer_bridge_joint']
  stale = [n for n in names if n not in self.q or self.now()-self.stamp[n] >= .35]
  if stale and self.mode == 'transfer' and self.transfer.state not in ('IDLE','DONE'):
   if int(t*5)%10 == 0: print(f"[STALE JOINTS] {stale}", flush=True)
  if t-self.pose_at > .35 and self.mode == 'transfer' and self.transfer.state not in ('IDLE','DONE'):
   if int(t*5)%10 == 0: print(f"[STALE POSE] age={t-self.pose_at:.3f}", flush=True)
  if self.estop or self.fault or not self.fresh(names) or t-self.pose_at>.35:self.base();self.belts();self.held();return
  a=self.arm()
  if self.mode=='entry':
   self.belts();self.arm_output([0,0,0,0]);x,y,z,roll,pitch,ang=self.pose
   if max(abs(v) for v in a)>.01 or abs(roll)>math.radians(2) or abs(pitch)>math.radians(8):self.fault='Entry posture outside checked envelope';self.base();return
   v=w=0.
   if self.entry=='INGRESS':
    if abs(x-1.3)>.030 or abs(ang-P/2)>.050:self.fault='Door path tracking error'
    v=min(.36,max(0.18,-y*.9));w=max(-.04,min(.04,(P/2-ang)*1.2))
    if y>=-.002:self.entry='TURN';v=0
   elif self.entry=='TURN':
    if math.hypot(x-1.3,y)>.015:self.fault='Turn centre drift exceeds 15 mm'
    w=-min(.08,max(.015,ang*.8))
    if abs(ang)<.004:self.entry='EXIT';w=0
   elif self.entry=='EXIT':
    if abs(y)>.020 or abs(ang)>.020:self.fault='Ramp path tracking error'
    v=min(.08,max(0,(4.9-x)*.6));w=max(-.02,min(.02,-ang*1.2))
    if x>=4.898:self.entry='DONE';v=0
   if self.fault:v=w=0
   self.base(v,w);return
  # In work mode the complete chassis is locked. No user cmd_vel bypass is bridged.
  self.base();x,y,z,roll,pitch,ang=self.pose
  dx=abs(x-4.1);dy=abs(y);dang=abs(ang);droll=abs(roll);dpitch=abs(pitch)
  if dx>.060 or dy>.035 or dang>.030 or droll>.060 or dpitch>.060:self.fault=f'Robot departed datum: dx={dx:.4f}, dy={dy:.4f}, dang={dang:.4f}, droll={droll:.4f}, dpitch={dpitch:.4f}';self.belts();self.held();return
  s=self.station();bag=self.bag if t-self.bag_at<.35 else None;o=self.transfer.step(t,s,bag)
  if self.transfer.state=='FAULT':self.fault=self.transfer.error;self.belts();self.held();return
  # IDLE folds the bridge before allowing independent robot motion.
  self.station_output(o,dt);self.belts(o)
  if self.transfer.state not in ('IDLE','DONE'):self.arm_output(self.target_arm);return
  try:
   if self.segment is None and self.queue:
    target=self.queue.pop(0);start=self.target_arm.copy();duration=max(.5,*[1.875*abs(b-c)/v for b,c,v in zip(target,start,[.08,.20,.20,.12])]);self.segment=(t,start,target,duration)
    self.get_logger().info(f"[SEGMENT START] target={target}, dur={duration:.2f}s, rem={len(self.queue)}")
   if self.segment:
    st,start,end,dur=self.segment;u=min(1,(t-st)/dur);goal=[x+(y-x)*smooth(u) for x,y in zip(start,end)];self.guard.evaluate(goal,self.h,self.mass);self.target_arm=goal;self.arm_output(goal)
    if (u>=1 and max(abs(x-y) for x,y in zip(a,end))<.030) or (t-st>dur+2.0):
     self.get_logger().info(f"[SEGMENT COMPLETE] target={end}, elapsed={t-st:.2f}s, max_err={max(abs(x-y) for x,y in zip(a,end)):.4f}")
     self.segment=None
   else:self.arm_output(self.target_arm)
  except ValueError as e:self.fault=str(e);self.belts();self.held()
 def report(self):
  m=String();m.data=json.dumps({'mode':self.mode,'entry':self.entry,'transfer':self.transfer.state,'fault':self.fault,'estop':self.estop,'lift_cap_m':lift_cap(self.h),'joint_feedback_fresh':bool(self.q),'robot_pose_fresh':self.now()-self.pose_at<.35,'bag_pose_fresh':self.now()-self.bag_at<.35,'arm_busy':bool(self.queue or self.segment)});self.status.publish(m)

def main():
 rclpy.init();node=Controller()
 try:rclpy.spin(node)
 finally:node.base();node.belts();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
