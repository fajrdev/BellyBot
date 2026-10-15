#!/usr/bin/env python3
from pathlib import Path
import json,xml.etree.ElementTree as E,shutil
ROOT=Path(__file__).resolve().parents[1];PKG=ROOT/'ros2/bellybot_aircraft_environment'
entries=[]
def bridge(ros,gz,rt,gt,direction):entries.append(dict(ros_topic_name=ros,gz_topic_name=gz,ros_type_name=rt,gz_type_name=gt,direction=direction))
bridge('/clock','/clock','rosgraph_msgs/msg/Clock','gz.msgs.Clock','GZ_TO_ROS')
for name,model in [('robot','bellybot_dock_stack'),('corner','corner_transfer'),('buffer','longitudinal_buffer')]:bridge('/aircraft/'+name+'_joints','/model/'+model+'/joint_states','sensor_msgs/msg/JointState','gz.msgs.Model','GZ_TO_ROS')
for model in ['bellybot_dock_stack','corner_transfer','longitudinal_buffer']:bridge('/joint_states','/model/'+model+'/joint_states','sensor_msgs/msg/JointState','gz.msgs.Model','GZ_TO_ROS')
bridge('/aircraft/robot_pose','/model/bellybot_dock_stack/pose','tf2_msgs/msg/TFMessage','gz.msgs.Pose_V','GZ_TO_ROS')
bridge('/aircraft/world_poses','/model/tracked_bag/pose','tf2_msgs/msg/TFMessage','gz.msgs.Pose_V','GZ_TO_ROS')
bridge('/aircraft/actuation/cmd_vel','/model/bellybot_dock_stack/cmd_vel','geometry_msgs/msg/Twist','gz.msgs.Twist','ROS_TO_GZ')
for name in ['lift_stage_target','extension_stage_target','yaw_target','pitch_target','tray_belt_speed','tray_roller_speed','left_wheel_speed','right_wheel_speed']:bridge('/aircraft/actuation/'+name,'/model/bellybot_dock_stack/'+name,'std_msgs/msg/Float64','gz.msgs.Double','ROS_TO_GZ')
for name,path in [('corner_stage_1','corner/stage_1'),('corner_stage_2','corner/stage_2'),('corner_stage_3','corner/stage_3'),('corner_angle','corner/angle'),('buffer_stage_1','buffer/stage_1'),('buffer_stage_2','buffer/stage_2'),('bridge_angle','buffer/bridge_angle'),('loader_speed','loader/speed'),('door_speed','door_belt_contact/speed'),('corner_speed','corner_belt/speed'),('bridge_speed','bridge_belt/speed'),('buffer_speed','buffer_belt/speed'),('door_roller','door_belt_contact/roller_speed'),('corner_roller','corner_belt/roller_speed'),('buffer_roller','buffer_belt/roller_speed'),('bridge_roller','bridge_belt/roller_speed')]:bridge('/aircraft/actuation/'+name,'/environment/'+path,'std_msgs/msg/Float64','gz.msgs.Double','ROS_TO_GZ')
(PKG/'config/aircraft_bridge.yaml').write_text('\n'.join('- '+'\n  '.join(f'{k}: {v}' for k,v in e.items()) for e in entries)+'\n')
# Updated package requirements and install location. Import helpers live next to executable.
p=PKG/'package.xml';r=E.parse(p).getroot();r.find('version').text='2.0.0'
for dep in ['rclpy','python3-numpy','sensor_msgs','geometry_msgs','tf2_msgs','bellybot_dock_stack_description']:
 if dep not in [x.text for x in r.findall('exec_depend')]:E.SubElement(r,'exec_depend').text=dep
E.indent(r);E.ElementTree(r).write(p,encoding='utf-8',xml_declaration=True)
p=PKG/'CMakeLists.txt';s=p.read_text();s=s if 'scripts/aircraft_controller.py' in s else s.replace('ament_package()','install(PROGRAMS scripts/aircraft_controller.py DESTINATION lib/${PROJECT_NAME})\ninstall(FILES scripts/aircraft_guard.py scripts/transfer_core.py DESTINATION lib/${PROJECT_NAME})\nament_package()');p.write_text(s)
print('Wired',len(entries),'ROS/Gazebo bridges')
