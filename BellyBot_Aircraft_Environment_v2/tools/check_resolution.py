#!/usr/bin/env python3
"""Whole-triangle FCL distances and sampled rigid pose clearances. Not dynamics."""
from pathlib import Path
import sys,math,json
import numpy as np,trimesh as tm
ROOT=Path(__file__).resolve().parents[1];PKG=ROOT/'ros2/bellybot_aircraft_environment';sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(PKG/'scripts'))
from scene_tools import URDF,corner_q,buffer_q
from aircraft_guard import Guard,lift_cap
r=URDF(ROOT/'ros2/bellybot_dock_stack_description/urdf/bellybot_portable.urdf');rm=r.manager();hold=tm.collision.CollisionManager()
for f in (PKG/'models/nb_hold/meshes').glob('collision_*.stl'):
 if any(s in f.name for s in ['load_floor','entry_pocket_floor']):continue
 hold.add_object(f.stem,tm.load_mesh(f))
# Merge robot for efficient placement of the entire mesh without hull cutout shortcuts.
body=tm.util.concatenate([m for _,m in r.geometry()]);bot=tm.collision.CollisionManager();bot.add_object('robot',body)
def check(T):bot.set_transform('robot',T);return bot.min_distance_other(hold,return_names=True)
entry=[]
for y in np.linspace(-3.29,0,1646):
 T=tm.transformations.euler_matrix(0,0,math.pi/2);T[:3,3]=[1.3,y,.183];d,n=check(T);entry.append({'phase':'ingress','x':1.3,'y':float(y),'z':.183,'yaw':math.pi/2,'distance_m':d,'pair':n})
for a in np.linspace(math.pi/2,0,786):
 T=tm.transformations.euler_matrix(0,0,a);T[:3,3]=[1.3,0,.183];d,n=check(T);entry.append({'phase':'turn','x':1.3,'y':0,'z':.183,'yaw':float(a),'distance_m':d,'pair':n})
print('Entry',min(q['distance_m'] for q in entry),flush=True)
# Conservative descend envelope: pitch changes with progression; height is raised
# as needed to avoid all intersections with the continuous flat/ramp ground.
def terrain(x):return np.maximum(0,np.minimum(.18,(4.06-x)*.1))
exit_samples=[]
for x in np.linspace(1.3,4.9,901):
 a=math.atan(.1)*min(1,max(0,(x-1.60)/1.32))*min(1,max(0,(4.72-x)/1.32))
 # Lowest support is the rigid 1.32m shoe. Max support envelope avoids floor penetration.
 u=np.linspace(-.66,.66,265);z=float(np.max(terrain(x+u*math.cos(a))+u*math.sin(a)))+.003
 T=tm.transformations.euler_matrix(0,a,0);T[:3,3]=[x,0,z];d,n=check(T);exit_samples.append({'x':float(x),'pitch':a,'z':z,'distance_m':d})
print('Exit ramp side',min(q['distance_m'] for q in exit_samples),flush=True)
co=URDF(PKG/'urdf/corner_transfer.urdf');bu=URDF(PKG/'urdf/longitudinal_buffer.urdf');door=URDF(PKG/'urdf/door_belt.urdf');fixed=door.manager();corner_checks=[]
for lift,angle,bl,bridge,label in [(.198*i/50,0,.128*i/50,-math.pi/2,'lift') for i in range(51)]+[(.198,math.pi/2*i/120,.128,-math.pi/2,'rotate') for i in range(121)]+[(.198,math.pi/2,.128,-math.pi/2+math.pi/2*i/80,'bridge') for i in range(81)]:
 c=co.manager(corner_q(lift,angle));b=bu.manager(buffer_q(bl,bridge))
 # Root support plates touch floor by design. Check moving pieces only against surroundings.
 moving=tm.collision.CollisionManager()
 for n,m in co.geometry(corner_q(lift,angle)):
  if n.startswith(('corner_rotor','corner_belt')):moving.add_object(n,m)
 d1,n1=moving.min_distance_other(fixed,return_names=True);d2,n2=moving.min_distance_other(b,return_names=True);d3,n3=moving.min_distance_other(hold,return_names=True)
 corner_checks.append({'phase':label,'lift':lift,'angle':angle,'bridge':bridge,'door_gap':d1,'buffer_gap':d2,'wall_gap':d3})
print('Corner gaps',min(q['door_gap'] for q in corner_checks),min(q['buffer_gap'] for q in corner_checks),flush=True)
# Robot vs parked replacement buffer throughout approved yaw/reach examples.
# Conservative AABBs for each individual visual primitive, avoiding large boxes
# around empty space between a whole mechanism's legs. Disjoint boxes guarantee
# disjoint meshes; the reported distance is a lower bound, not an exact distance.
scene=tm.load(PKG/'models/longitudinal_buffer/meshes/visual.glb',force='scene');home=bu.fk();now=bu.fk(buffer_q());buffer_boxes=[]
for node in scene.graph.nodes_geometry:
 T,gn=scene.graph[node];m=scene.geometry[gn].copy();m.apply_transform(T);key=next(k for k in sorted(home,key=len,reverse=True) if node.startswith(k+'_'));m.apply_transform(now[key]@np.linalg.inv(home[key]));buffer_boxes.append(m.bounds)
bs=np.array(buffer_boxes);corners=[]
for link,n,m in r.parts:
 lo,hi=m.bounds;corners.append((link,n,np.array([[x,y,z,1] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])))
rc=[]
for lift in [.08,.13,.23]:
 for yaw in np.linspace(-math.radians(65),math.radians(65),53):
  for reach in [0,.25,.50]:
   q={**{f'lift_stage_{i}_joint':lift/3 for i in [1,2,3]},'yaw_joint':yaw,'extension_mid_joint':reach/2,'extension_tip_joint':reach/2};T=np.eye(4);T[:3,3]=[4.1,0,.003];fk=r.fk(q,T);bounds=[]
   for link,n,p in corners:v=(fk[link]@p.T).T[:,:3];bounds.append([v.min(0),v.max(0)])
   a=np.array(bounds);d=np.maximum(0,np.maximum(a[:,None,0,:]-bs[None,:,1,:],bs[None,:,0,:]-a[:,None,1,:]));dist=float(np.linalg.norm(d,axis=2).min());rc.append({'lift':lift,'yaw':float(yaw),'reach':reach,'gap':dist})
print('Robot/parked buffer conservative bound',min(x['gap'] for x in rc),flush=True)
# Pure controller contracts: reject unsafe commands, accept conservative receiving poses.
g=Guard(PKG/'config/robot_geometry.json');tests={}
for label,q,h,want in [('tall_receive',[.13,0,0,0],.38,True),('small_receive',[.23,0,0,0],.28,True),('roof_reject',[.6,0,0,0],.38,False),('yaw_reach_reject',[.13,1.,1.,0],.32,False),('yaw_low_reject',[0,1.,0,0],.32,False),('safe_yaw',[.13,.8,.4,0],.38,True)]:
 try:report=g.evaluate(q,h);actual=True
 except ValueError as e:report=str(e);actual=False
 tests[label]={'expected':want,'accepted':actual,'result':report};assert actual==want,(label,report)
assert min(v['distance_m'] for v in entry)>.03
assert min(v['distance_m'] for v in exit_samples)>.025
assert min(v['door_gap'] for v in corner_checks)>0
assert min(v['buffer_gap'] for v in corner_checks)>0
assert min(v['gap'] for v in rc)>0
report={'method':'Triangle mesh BVH/FCL distance to finite doorway and liner collision solids; actual v3 visual geometry. Entry translation step <=2mm and turn step <=0.00201rad. Sampled ramp support envelope; not tracked contact stability or a physics run. Transfer sweep is sampled; robot/buffer uses conservative per-part AABB distance bounds across three lift heights; no free-bag dynamics.',
 'runtime_tested':False,'entry_samples':len(entry),'entry_ingress_min_gap_m':min(q['distance_m'] for q in entry if q['phase']=='ingress'),'entry_turn_min_gap_m':min(q['distance_m'] for q in entry if q['phase']=='turn'),
 'entry_continuous_wall_gap_lower_bound_m':min(q['distance_m'] for q in entry)-.002,'entry_deck_height_m':.18,'entry_door_top_clearance_m':.89-(.535+.183),'exit_ramp_samples':len(exit_samples),'exit_ramp_min_wall_gap_m':min(q['distance_m'] for q in exit_samples),
 'transfer_samples':len(corner_checks),'min_corner_door_gap_m':min(q['door_gap'] for q in corner_checks),'min_corner_buffer_gap_m':min(q['buffer_gap'] for q in corner_checks),'robot_buffer_samples':len(rc),'min_robot_buffer_gap_m':min(q['gap'] for q in rc),'robot_buffer_distance_method':'Conservative per-part AABB lower bound, not exact mesh distance','controller_contracts':tests,
 'lift_caps':{str(h):lift_cap(h) for h in [.28,.32,.38]},'required_host_checks':['Gazebo schema and runtime','Track friction and ramp settling','Payload centering, slip, impact, transfer gaps','Throughput >=12 bags/min','Measured load patch forces <=600 kg/m2','All custom fixture structural/load ratings']}
(ROOT/'docs/resolution_checks.json').write_text(json.dumps(report,indent=2));(ROOT/'config/entry_path_samples.json').write_text(json.dumps({'entry':entry[::10],'exit':exit_samples[::5]},indent=2));print(json.dumps(report,indent=2))
