#!/usr/bin/env python3
"""Spawn one physically free tracked bag on the level door belt. Dry-run default."""
import argparse,json,subprocess,tempfile,xml.etree.ElementTree as E
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];PKG=ROOT/'ros2/bellybot_aircraft_environment'
p=argparse.ArgumentParser();p.add_argument('--world',default='transfer_resolved');p.add_argument('--bag',choices=['bag_hard_20','bag_hard_26','bag_hard_32','bag_duffel_23','bag_fragile_20'],default='bag_hard_32');p.add_argument('--execute',action='store_true');a=p.parse_args()
r=E.parse(PKG/'models'/a.bag/'model.sdf').getroot();m=r.find('model');m.set('name','tracked_bag');size=[float(x) for x in m.findtext('.//collision/geometry/box/size').split()];z=.300+size[2]/2+.002;E.SubElement(m,'pose').text=f'1.3 -1.60 {z} 0 0 1.5707963267948966'
pl=E.SubElement(m,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher')
for k,v in [('topic','/model/tracked_bag/pose'),('publish_model_pose','true'),('publish_link_pose','false'),('publish_visual_pose','false'),('publish_collision_pose','false'),('use_pose_vector_msg','true'),('update_frequency','50')]:E.SubElement(pl,k).text=v
# Resolve meshes for a directly created model; user-defined names do not change resource lookup.
for u in m.findall('.//mesh/uri'):
 if u.text.startswith('model://'):u.text=(PKG/'models'/u.text[8:]).resolve().as_uri()
request='sdf: '+json.dumps(E.tostring(r,encoding='unicode'))+' allow_renaming: false'
cmd=['gz','service','-s',f'/world/{a.world}/create','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','4000','--req',request]
print('One free bag:',a.bag,'dimensions:',size,'mass:',m.findtext('.//inertial/mass'),'kg; model name tracked_bag')
if a.execute:subprocess.run(cmd,check=True)
else:print('Dry run. Pass --execute after starting and unpausing the revised world. Remove the prior tracked_bag before another run.')
