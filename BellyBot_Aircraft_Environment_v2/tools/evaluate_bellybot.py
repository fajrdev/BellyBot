#!/usr/bin/env python3
"""BellyBot v3 mesh/FK enclosure checks. No Gazebo or contact dynamics.
Uses convex hull vertices of each visual mesh: exact for containment by convex
half-spaces, but does not test mesh-to-mesh self-collision or free bag dynamics.
"""
import sys,json,math,argparse,xml.etree.ElementTree as E
from pathlib import Path
import numpy as np
import trimesh as tm
ROOT=Path(__file__).resolve().parents[1]
CFG=json.loads((ROOT/'config/environment.json').read_text())

def vector(e,attr,default):return np.fromstring(e.get(attr,default) if e is not None else default,sep=' ')
def origin(e):
 T=tm.transformations.euler_matrix(*vector(e,'rpy','0 0 0'));T[:3,3]=vector(e,'xyz','0 0 0');return T
class Robot:
 def __init__(self,package,urdf="bellybot_portable.urdf"):
  self.package=Path(package);self.root=E.parse(self.package/'urdf'/urdf).getroot();self.joints=self.root.findall('joint');self.parts=[];self.base=next(iter({l.get('name') for l in self.root.findall('link')}-{j.find('child').get('link') for j in self.joints}))
  for link in self.root.findall('link'):
   for i,v in enumerate(link.findall('visual')):
    me=v.find('geometry/mesh')
    if me is None:continue
    uri=me.get('filename');file=self.package/uri.split('package://bellybot_dock_stack_description/')[1] if uri.startswith('package://') else self.package/'urdf'/uri;mesh=tm.load_mesh(file,process=True);mesh.apply_scale(vector(me,'scale','1 1 1'));mesh.apply_transform(origin(v.find('origin')))
    points=mesh.convex_hull.vertices
    color=v.find('material/color');rgba=vector(color,'rgba','.16 .55 .45 1')
    mesh.visual.face_colors=(rgba*255).astype(np.uint8)
    self.parts.append((link.get('name'),v.get('name',f'{link.get("name")}_{i}'),mesh,points))
 def fk(self,q):
  T={self.base:np.eye(4)};todo=list(self.joints)
  while todo:
   rest=[]
   for j in todo:
    parent=j.find('parent').get('link');child=j.find('child').get('link')
    if parent not in T:rest.append(j);continue
    v=q.get(j.get('name'),0);A=origin(j.find('origin'));axis=vector(j.find('axis'),'xyz','1 0 0');M=np.eye(4)
    if j.get('type')=='prismatic':M[:3,3]=axis*v
    elif j.get('type') in ['revolute','continuous']:M=tm.transformations.rotation_matrix(v,axis)
    T[child]=T[parent]@A@M
   if len(rest)==len(todo):raise ValueError('Joint graph is not connected')
   todo=rest
  return T
 def geometry(self,lift=0,yaw=0,reach=0,pitch=0,base_yaw=0,x=3.5,y=0,base_z=.003,bag_height=0,meshes=False):
  q={**{f'lift_stage_{i}_joint':lift/3 for i in [1,2,3]},'yaw_joint':yaw,'extension_mid_joint':reach/2,'extension_tip_joint':reach/2,'tray_pitch_joint':pitch};fk=self.fk(q)
  W=tm.transformations.euler_matrix(0,0,base_yaw);W[:3,3]=[x,y,base_z]
  points=[];out=[]
  for link,name,m,p in self.parts:
   T=W@fk[link];points.append(tm.transform_points(p,T))
   if meshes:mc=m.copy();mc.apply_transform(T);out.append((name,mc))
  if bag_height:
   bag=tm.creation.box([.78,.50,bag_height]);B=np.eye(4);B[:3,3]=[.43,0,-.00835+bag_height/2+.002];T=W@fk['tray_link']@B if 'tray_link' in fk else W@fk['tray']@B
   bag.apply_transform(T);bag.visual.face_colors=[95,97,119,255];points.append(bag.vertices)
   if meshes:out.append(('screening_bag',bag))
  return (np.concatenate(points),out) if meshes else np.concatenate(points)

def assess(points,aft=False,door=False):
 p=CFG['forward'];x,y,z=points.T
 if aft:
  a=CFG['aft_stress'];H=a['front_height']+(a['rear_height']-a['front_height'])*x/a['length'];upper=a['front_upper_width']+(a['rear_upper_width']-a['front_upper_width'])*x/a['length'];length=a['length']
  normalized=z/H*p['ceiling_height'];half=.61+(np.interp(normalized,[r[1] for r in p['profile_yz']],[r[0] for r in p['profile_yz']])-.61)*(upper/2-.61)/(.915)
  side=half-np.abs(y) # horizontal margin, aft interpolation only
 else:
  H=np.full_like(z,p['ceiling_height']);length=p['length'];margins=[]
  for (y0,z0),(y1,z1) in zip(p['profile_yz'],p['profile_yz'][1:]):
   slope=(y1-y0)/(z1-z0);bound=y0+slope*(z-z0)
   pos=(bound-y)/math.sqrt(1+slope*slope);neg=(bound+y)/math.sqrt(1+slope*slope)
   if door:
    in_portal=(np.abs(x-p['door_center_x'])<p['door_width_x']/2)&(z>=0)&(z<=p['door_clear_height'])&(y>=p['door_outer_y'])
    neg=np.where(in_portal,1e6,neg)
   margins.extend([pos,neg])
  side=np.min(margins,axis=0)
 vals={'roof_m':float(np.min(H-z)),'side_wall_m':float(np.min(side)),'floor_m':float(np.min(z)),
       'bulkhead_m':float(min(np.min(x-.018),np.min(length-.018-x)))}
 vals['enclosure_pass']=min(vals.values())>=-1e-5
 vals['operational_margin_pass']=vals['enclosure_pass'] and vals['roof_m']>=.1 and vals['side_wall_m']>=.025
 vals['max_height_m']=float(z.max());return vals

def run(path):
 robot=Robot(Path(path)/'ros2/bellybot_dock_stack_description')
 print('Loaded',len(robot.parts),'visual parts',flush=True)
 poses={
 'stowed':{},'receive_32kg_bag':{'bag_height':.32},'receive_tall_bag':{'bag_height':.38},
 'loaded_tall_lift_130mm':{'lift':.13,'bag_height':.38},'loaded_280mm_lift_230mm':{'lift':.23,'bag_height':.28},
 'unloaded_mechanical_full_lift':{'lift':.6},
 'v3_yaw_placement':{'lift':.1,'yaw':math.radians(65),'reach':.5,'bag_height':.28},
 'v3_full_reach':{'lift':.2,'yaw':math.radians(95),'reach':.6,'bag_height':.28},
 'floor_pickup':{'reach':1.,'pitch':math.radians(30.92202976)},
 'base_turn_15deg':{'base_yaw':math.radians(15)},'base_turn_30deg':{'base_yaw':math.radians(30)},
 'base_turn_45deg':{'base_yaw':math.radians(45)},'base_turn_90deg':{'base_yaw':math.pi/2}}
 results={n:dict(pose=q,**assess(robot.geometry(**q))) for n,q in poses.items()}
 scan=[]
 # Whole-chassis pivots, outside door recess: 0.5-degree samples.
 for deg in np.arange(0,90.001,.5):scan.append({'degrees':float(deg),**assess(robot.geometry(base_yaw=math.radians(deg)))})
 # A sampled joint workspace at a fixed 130 mm lift and 320 mm bag.
 grid=[]
 for yaw in range(-95,96,5):
  for reach in np.arange(0,1.001,.05):
   r=assess(robot.geometry(lift=.13,yaw=math.radians(yaw),reach=reach,bag_height=.32));grid.append({'yaw_deg':yaw,'reach_m':round(float(reach),3),**r})
 # An unplanned example pivot at door center, to expose why pre-spawn is not proof of access.
 door_sweep=[]
 for deg in np.arange(0,90.001,2):door_sweep.append({'degrees':float(deg),**assess(robot.geometry(base_yaw=math.radians(deg),x=1.3,y=-.18),door=True)})
 aft_results={}
 for x in [2,5,8,9.7]:aft_results[str(x)]=assess(robot.geometry(x=x,bag_height=.32),aft=True)
 feed=Robot(Path(path)/'ros2/bellybot_dock_stack_description','infeed.urdf');feed_results={}
 for label,height,bag_height in [('home',0,0),('tall_bag_matched',.13,.38),('tall_bag_full_stroke',.25,.38)]:
  frames=feed.fk({'infeed_lift_joint':height,'infeed_extension_joint':.4});pts=[]
  for link,name,m,v in feed.parts:pts.append(tm.transform_points(v,frames[link])+[3.5,0,.003])
  if bag_height:
   bag=tm.creation.box([.78,.50,bag_height]);bag.apply_translation([3.5-1.37+.4,0,.495+height+.003+.002+bag_height/2]);pts.append(bag.vertices)
  feed_results[label]=assess(np.concatenate(pts))

 output={'method':'Convex hull vertices of existing visual meshes transformed through URDF. Constant forward cross section exact half-space containment at sampled poses. Bag footprint is a conservative 780 x 500 mm rectangle for all tested heights. No dynamic settling, self-collision, structural or global path proof.',
 'scope':'Base datum 3 mm above floor. Actual floor-strip and doorway detail contacts excluded from working-zone sweep. Aft uses horizontal interpolated margin.',
 'robot_source':'BellyBot DockStack C v3','visual_parts':len(robot.parts),'sampled_pose_count':len(poses)+len(scan)+len(grid)+len(door_sweep)+len(aft_results)+len(feed_results),
 'runtime_tested':False,'poses':results,'base_yaw_sweep':scan,'upper_workspace':grid,'door_pivot_example':door_sweep,'aft_samples':aft_results,'infeed_poses':feed_results,
 'base_first_interference_deg':next((r['degrees'] for r in scan if not r['enclosure_pass']),None),
 'base_first_operational_margin_failure_deg':next((r['degrees'] for r in scan if not r['operational_margin_pass']),None),
 'lift_caps_m':{str(b):round(1.12-.1-.003-.495-b-.005,3) for b in [.26,.28,.30,.32,.38]},
 'floor_pressure_note':'Previous v3 four-patch assigned-mass screening peak 594.31 kg/m2; not revalidated as aircraft-floor compliance. Only 5.69 kg/m2 to hackathon limit; track contact and reactions must be measured.'}
 (ROOT/'docs/bellybot_fit.json').write_text(json.dumps(output,indent=2))
 scene=tm.Scene()
 for n,m in robot.geometry(meshes=True)[1]:scene.add_geometry(m,node_name=n)
 scene.export(ROOT/'exports/bellybot_reference_pose.glb')
 print(json.dumps({n:{k:r[k] for k in ['enclosure_pass','side_wall_m','roof_m','max_height_m']} for n,r in results.items()},indent=2));print('First chassis interference',output['base_first_interference_deg'])
 return robot,output
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--bellybot',required=True);a=p.parse_args();run(a.bellybot)
