#!/usr/bin/env python3
"""Direct SDF export for Gazebo Harmonic; avoids URDF fixed-link lumping.

Track and conveyor contact controllers, wheel joints, paired stage targets,
depth camera and separate infeed. Runtime validation must occur in Gazebo.
"""
import copy,math
import xml.etree.ElementTree as ET
import numpy as np
from model import *
from build import urdf,bag_urdf,xmlwrite,fmt

def tag(p,n,v,**attrs):e=ET.SubElement(p,n,**attrs);e.text=str(v);return e
def plugin(p,lib,name):return ET.SubElement(p,'plugin',filename='gz-sim-'+lib+'-system',name='gz::sim::systems::'+name)

def convert(root,name,home):
    m=ET.Element('model',name=name,canonical_link=root.find('link').get('name'))
    tag(m,'self_collide','true')
    for source in root.findall('link'):
        name=source.get('name');l=ET.SubElement(m,'link',name=name)
        pose=home.get(name,np.eye(4));tag(l,'pose',fmt(pose[:3,3]/1000)+' 0 0 0')
        si=source.find('inertial');i=ET.SubElement(l,'inertial');org=si.find('origin')
        tag(i,'pose',org.get('xyz','0 0 0')+' '+org.get('rpy','0 0 0'))
        tag(i,'mass',si.find('mass').get('value'));it=ET.SubElement(i,'inertia')
        for k,v in si.find('inertia').attrib.items():tag(it,k,v)
        for kind in ['visual','collision']:
            for j,source_shape in enumerate(source.findall(kind)):
                shape=ET.SubElement(l,kind,name=source_shape.get('name',f'{kind}_{j}'))
                origin=source_shape.find('origin')
                if origin is not None:tag(shape,'pose',origin.get('xyz','0 0 0')+' '+origin.get('rpy','0 0 0'))
                g=ET.SubElement(shape,'geometry');sg=source_shape.find('geometry')
                if sg.find('mesh') is not None:
                    sm=sg.find('mesh');me=ET.SubElement(g,'mesh')
                    tag(me,'uri',sm.get('filename').replace('package://','model://'));tag(me,'scale',sm.get('scale'))
                elif sg.find('box') is not None:tag(ET.SubElement(g,'box'),'size',sg.find('box').get('size'))
                else:
                    c=ET.SubElement(g,'cylinder')
                    for k,v in sg.find('cylinder').attrib.items():tag(c,k,v)
                if kind=='visual':
                    mat=ET.SubElement(shape,'material');rgba=source_shape.find('material/color').get('rgba')
                    tag(mat,'ambient',rgba);tag(mat,'diffuse',rgba);tag(mat,'specular','.1 .1 .1 1')
                else:
                    surf=ET.SubElement(shape,'surface');fr=ET.SubElement(surf,'friction');ode=ET.SubElement(fr,'ode')
                    mu=.8 if 'track' in name else (.7 if 'belt' in name else .5)
                    tag(ode,'mu',mu);tag(ode,'mu2',mu)
    for s in root.findall('joint'):
        typ=s.get('type');j=ET.SubElement(m,'joint',name=s.get('name'),type='revolute' if typ=='continuous' else typ)
        child=s.find('child').get('link');tag(j,'parent',s.find('parent').get('link'));tag(j,'child',child)
        tag(j,'pose','0 0 0 0 0 0',relative_to=child)
        if typ=='fixed':continue
        axis=ET.SubElement(j,'axis');tag(axis,'xyz',s.find('axis').get('xyz'))
        limit=ET.SubElement(axis,'limit')
        for k,v in s.find('limit').attrib.items():tag(limit,k,v)
        dyn=ET.SubElement(axis,'dynamics')
        for k,v in s.find('dynamics').attrib.items():tag(dyn,k,v)
    return m

def track(m,link_name,topic,speed):
    c=plugin(m,'track-controller','TrackController');tag(c,'link',link_name)
    tag(c,'velocity_topic',topic);tag(c,'track_orientation','0 3.141592653589793 0' if 'belt' in link_name else '0 0 0')
    tag(c,'max_command_age',.5);tag(c,'min_velocity',-speed);tag(c,'max_velocity',speed)
    acceleration=3 if 'belt' in link_name else 1
    tag(c,'min_acceleration',-acceleration);tag(c,'max_acceleration',acceleration)

def velocity_group(m,joints,topic):
    c=plugin(m,'joint-controller','JointController')
    for n in joints:tag(c,'joint_name',n)
    tag(c,'topic',topic);tag(c,'initial_velocity',0);tag(c,'use_force_commands','false')

def position_group(m,joints,topic,speed,initial=0):
    c=plugin(m,'joint-position-controller','JointPositionController')
    for n in joints:tag(c,'joint_name',n)
    tag(c,'topic',topic);tag(c,'initial_position',initial)
    tag(c,'use_velocity_commands','true');tag(c,'cmd_max',speed)

def add_controllers(robot,feed):
    root='/model/bellybot_dock_stack/'
    for side in ['left','right']:
        track(robot,side+'_track_link',root+side+'_track_velocity',.4)
        velocity_group(robot,[f'{side}_wheel_{i}_joint' for i in range(5)],root+side+'_wheel_speed')
    c=plugin(robot,'tracked-vehicle','TrackedVehicle')
    for side in ['left','right']:
        t=ET.SubElement(c,side+'_track');tag(t,'link',side+'_track_link');tag(t,'velocity_topic',root+side+'_track_velocity')
    tag(c,'tracks_separation',P['track_centres']/1000);tag(c,'track_height',(2*P['track_radius']+4)/1000)
    tag(c,'steering_efficiency',1);tag(c,'topic',root+'cmd_vel')
    for axis,v,a in [('linear_velocity',.2,.3),('angular_velocity',.25,.4)]:
        l=ET.SubElement(c,axis)
        for k,value in [('min_velocity',-v),('max_velocity',v),('min_acceleration',-a),('max_acceleration',a)]:tag(l,k,value)
    tag(c,'frame_id','odom');tag(c,'child_frame_id','base_link')
    position_group(robot,[f'lift_stage_{i}_joint' for i in [1,2,3]],root+'lift_stage_target',P['lift_stage_velocity'])
    position_group(robot,['extension_mid_joint','extension_tip_joint'],root+'extension_stage_target',P['extension_stage_velocity'])
    position_group(robot,['yaw_joint'],root+'yaw_target',P['yaw_velocity'])
    position_group(robot,['tray_pitch_joint'],root+'pitch_target',P['tray_pitch_velocity'])
    position_group(feed,['infeed_lift_joint'],'/model/bellybot_infeed/lift_target',P['infeed_lift_velocity'])
    track(robot,'tray_belt',root+'tray_belt_speed',P['belt_velocity'])
    velocity_group(robot,['tray_rear_roller_joint','tray_front_roller_joint'],root+'tray_roller_speed')
    state=plugin(robot,'joint-state-publisher','JointStatePublisher');tag(state,'topic',root+'joint_states')
    position_group(feed,['infeed_extension_joint'],'/model/bellybot_infeed/extension_target',P['infeed_velocity'],.4)
    track(feed,'infeed_belt','/model/bellybot_infeed/belt_speed',P['belt_velocity'])
    velocity_group(feed,['infeed_front_roller_joint','infeed_rear_roller_joint'],'/model/bellybot_infeed/roller_speed')
    state=plugin(feed,'joint-state-publisher','JointStatePublisher');tag(state,'topic','/model/bellybot_infeed/joint_states')
    base=robot.find("link[@name='base_link']")
    sensor=ET.SubElement(base,'sensor',name='front_depth',type='depth_camera')
    tag(sensor,'pose','.699 0 .231 0 0 0');tag(sensor,'always_on','true');tag(sensor,'update_rate',15)
    tag(sensor,'topic',root+'depth');cam=ET.SubElement(sensor,'camera')
    tag(cam,'horizontal_fov',1.2);im=ET.SubElement(cam,'image');tag(im,'width',320);tag(im,'height',240);tag(im,'format','R_FLOAT32')
    clip=ET.SubElement(cam,'clip');tag(clip,'near',.05);tag(clip,'far',5)

def sdfwrap(m):s=ET.Element('sdf',version='1.9');s.append(m);return s

def primitive_model(world,name,size,xyz,color=(.65,.68,.7,1),rpy=(0,0,0),collision=True):
    m=ET.SubElement(world,'model',name=name);tag(m,'static','true');tag(m,'pose',fmt((*xyz,*rpy)))
    l=ET.SubElement(m,'link',name='body')
    for kind in ['visual','collision'] if collision else ['visual']:
        e=ET.SubElement(l,kind,name=kind);tag(ET.SubElement(ET.SubElement(e,'geometry'),'box'),'size',fmt(size))
        if kind=='visual':mat=ET.SubElement(e,'material');tag(mat,'ambient',fmt(color));tag(mat,'diffuse',fmt(color))
    return m

def world(robot,feed,bag,cargo=False,floor_test=False,batch_test=False):
    s=ET.Element('sdf',version='1.9');w=ET.SubElement(s,'world',name='dock_stack')
    p=ET.SubElement(w,'physics',name='one_ms',type='ignored');tag(p,'max_step_size',.001);tag(p,'real_time_factor',1)
    plugin(w,'physics','Physics');plugin(w,'user-commands','UserCommands');plugin(w,'scene-broadcaster','SceneBroadcaster')
    sensors=plugin(w,'sensors','Sensors');tag(sensors,'render_engine','ogre2')
    scene=ET.SubElement(w,'scene');tag(scene,'ambient','.6 .6 .6 1');tag(scene,'background','.91 .93 .94 1')
    light=ET.SubElement(w,'light',name='sun',type='directional');tag(light,'pose','0 0 5 0 0 0')
    tag(light,'diffuse','.85 .85 .85 1');tag(light,'direction','-.3 .2 -1')
    primitive_model(w,'floor',(10,6,.05),(0,0,-.025),(.68,.71,.72,1))
    r=copy.deepcopy(robot);tag(r,'pose','0 0 .002 0 0 0');w.append(r)
    f=copy.deepcopy(feed);tag(f,'pose','0 0 0 0 0 0')
    # Fixed only to the WORLD, never to the mobile robot.
    joint=ET.SubElement(f,'joint',name='infeed_world_anchor',type='fixed');tag(joint,'parent','world');tag(joint,'child','infeed_base')
    w.append(f)
    if floor_test:
        b=copy.deepcopy(bag);tag(b,'pose','1.620 0 .002 0 0 0');w.append(b)
    if batch_test:
        # Single free bag initially supported by the unextended infeed.
        # Raise/extend infeed and robot to the batch preset before any transfer.
        b=copy.deepcopy(bag);tag(b,'pose','-1.300 0 .501 0 0 0');w.append(b)
        # Front clears longer tracks; top leaves 15 mm below cassette underside.
        # Rigid receiving fixture, not a validated compliant luggage stack.
        primitive_model(w,'batch_receiving_fixture',(1.3,.55,.555),(1.43,0,.2775),(.38,.46,.43,1))
    if cargo:
        # Illustrative aircraft-like test envelope, not an aircraft type drawing.
        for side in [-1,1]:
            primitive_model(w,f'lower_wall_{side}',(6,.025,.5),(0,side*1.4,.25),(.82,.84,.85,1))
        # Elliptical roof panels y=1.4*sin(t), z=.5+.68*cos(t).
        for i in range(16):
            a=-math.pi/2+i*math.pi/16;b=a+math.pi/16
            y1,z1=1.4*math.sin(a),.5+.68*math.cos(a)
            y2,z2=1.4*math.sin(b),.5+.68*math.cos(b)
            length=math.hypot(y2-y1,z2-z1);roll=math.atan2(z2-z1,y2-y1)
            primitive_model(w,f'curved_roof_{i}',(6,length,.012),(0,(y1+y2)/2,(z1+z2)/2),(.85,.87,.88,1),(roll,0,0))
        for x in [-2.5,-1.5,-.5,.5,1.5,2.5]:
            primitive_model(w,f'roof_light_{x}',(.6,.04,.008),(x,0,1.165),(1,.96,.82,1),collision=False)
        # A separate single rigid bag for physics testing; no attachment to tray.
        b=copy.deepcopy(bag);tag(b,'pose','0 0 .501 0 0 0');w.append(b)
        for i in range(2):
            for j in range(3):
                # Elevated receiving stack outside the parked robot footprint.
                primitive_model(w,f'stack_{i}_{j}',(.55,.42,.26),(1.8+(j%2)*.025,-.6+j*.6,.13+i*.26),
                                (.9,.35,.07,1) if j%2==0 else (.4,.44,.46,1))
    return s

def main():
    build();home=fk();r=convert(urdf(),'bellybot_dock_stack',home)
    f=convert(urdf(group='infeed'),'bellybot_infeed',home)
    b=convert(bag_urdf(),'sample_bag_32kg',{'bag':np.eye(4)})
    add_controllers(r,f)
    dest=PKG/'gazebo';dest.mkdir(parents=True,exist_ok=True)
    for name,m in [('bellybot_dock_stack.sdf',r),('infeed.sdf',f),('sample_bag.sdf',b)]:xmlwrite(sdfwrap(m),dest/name)
    xmlwrite(world(r,f,b,floor_test=True),dest/'floor_pickup.sdf');xmlwrite(world(r,f,b),dest/'bench.sdf');xmlwrite(world(r,f,b,True),dest/'cargo_hold.sdf')
    xmlwrite(world(r,f,b,batch_test=True),dest/'batch_transfer.sdf')
    print('Exported Gazebo models and bench, floor, batch-transfer and cargo worlds',flush=True)

if __name__=='__main__':main()
