#!/usr/bin/env python3
"""Revision 2: original entry fixtures and articulated corner transfer (SI).
Exact construction primitives -> meshes, SDF, URDF and mm STEP. No bag welding.
"""
from pathlib import Path
import sys,copy,json,math,shutil,xml.etree.ElementTree as E
import numpy as np
import trimesh as tm
sys.path.insert(0,str(Path(__file__).resolve().parent))
from build_environment import Asset,box,cyl,beam,matrix,put,nums,write_xml,dae,inertial,friction,inc,world,PKG,ROOT,C,rgba,make_loader
PI=math.pi
class Mechanism:
 def __init__(self,name):self.name=name;self.links={};self.joints=[];self.controllers=[]
 def link(self,n,parent=None,kind='fixed',origin=(0,0,0),rpy=(0,0,0),axis=(0,0,1),limits=None,mass=1,velocity=.2):
  self.links[n]={'parts':[],'parent':parent,'T':matrix(origin,rpy),'mass':mass,'origin':origin,'rpy':rpy}
  if parent:self.joints.append(dict(name=n+'_joint',parent=parent,child=n,type=kind,xyz=axis,limits=limits,velocity=velocity))
  return n
 def add(self,l,n,m,c='teal',collision=True):
  m.visual.face_colors=(np.array(rgba(c))*255).astype(np.uint8);self.links[l]['parts'].append((n,m,c,collision))
 def pos(self,j,topic,vel):self.controllers.append(('pos',j,topic,vel))
 def belt(self,l,topic):self.controllers.append(('belt',l,topic,1.2))
 def export(self):
  d=PKG/'models'/self.name
  if d.exists():shutil.rmtree(d)
  (d/'meshes').mkdir(parents=True,exist_ok=True)
  sd=E.Element('sdf',version='1.9');mo=put(sd,'model',name=self.name);put(mo,'static','false')
  ur=E.Element('robot',name=self.name);put(ur,'link',name='fixture_world');trans={};scene=tm.Scene()
  for n,v in self.links.items():
   p=v['parent'];trans[n]=(trans[p] if p else np.eye(4))@v['T']
   l=put(mo,'link',name=n);put(l,'pose',nums([*v['origin'],*v['rpy']]),relative_to=p or '__model__')
   parts=[(nm,m,c) for nm,m,c,k in v['parts']];merged=tm.util.concatenate([m for _,m,_ in parts]);lo,hi=merged.bounds;com=(lo+hi)/2;inertial(l,v['mass'],np.maximum(hi-lo,.02));put(l.find('inertial'),'pose',nums([*com,0,0,0]))
   dae(parts,d/'meshes'/f'{n}.dae');merged.export(d/'meshes'/f'{n}.stl')
   vi=put(l,'visual',name=n+'_visual');put(put(put(vi,'geometry'),'mesh'),'uri',f'model://{self.name}/meshes/{n}.dae')
   ul=put(ur,'link',name=n);uv=put(ul,'visual');put(put(uv,'geometry'),'mesh',filename=f'package://bellybot_aircraft_environment/models/{self.name}/meshes/{n}.dae')
   ui=put(ul,'inertial');put(ui,'origin',xyz=nums(com),rpy='0 0 0');put(ui,'mass',value=str(v['mass']));it=l.find('inertial/inertia');put(ui,'inertia',**{k:it.findtext(k) for k in ['ixx','iyy','izz','ixy','ixz','iyz']})
   for nm,m,c,k in v['parts']:
    scene.add_geometry(m,node_name=n+'_'+nm,transform=trans[n])
    if not k:continue
    fn=f'collision_{n}_{nm}.stl';m.export(d/'meshes'/fn);co=put(l,'collision',name=nm);put(put(put(co,'geometry'),'mesh'),'uri',f'model://{self.name}/meshes/{fn}');friction(co,.60)
    uc=put(ul,'collision',name=nm);put(put(uc,'geometry'),'mesh',filename=f'package://bellybot_aircraft_environment/models/{self.name}/meshes/{fn}')
   if not p:
    sj=put(mo,'joint',name=n+'_anchor',type='fixed');put(sj,'parent','world');put(sj,'child',n)
    uj=put(ur,'joint',name=n+'_anchor',type='fixed');put(uj,'parent',link='fixture_world');put(uj,'child',link=n);put(uj,'origin',xyz=nums(v['origin']),rpy=nums(v['rpy']))
  for j in self.joints:
   v=self.links[j['child']];sj=put(mo,'joint',name=j['name'],type='revolute' if j['type']=='continuous' else j['type']);put(sj,'parent',j['parent']);put(sj,'child',j['child']);put(sj,'pose','0 0 0 0 0 0',relative_to=j['child'])
   uj=put(ur,'joint',name=j['name'],type=j['type']);put(uj,'parent',link=j['parent']);put(uj,'child',link=j['child']);put(uj,'origin',xyz=nums(v['origin']),rpy=nums(v['rpy']))
   if j['type']!='fixed':
    ax=put(sj,'axis');put(ax,'xyz',nums(j['xyz']));li=put(ax,'limit');bounds=j['limits'] or [-1e16,1e16];put(li,'lower',bounds[0]);put(li,'upper',bounds[1]);put(li,'effort',2500 if j['type']=='prismatic' else 120);put(li,'velocity',j['velocity']);put(put(ax,'dynamics'),'damping',.3)
    put(uj,'axis',xyz=nums(j['xyz']));kwargs=dict(effort='2500' if j['type']=='prismatic' else '120',velocity=str(j['velocity']))
    if j['limits']:kwargs.update(lower=str(bounds[0]),upper=str(bounds[1]))
    put(uj,'limit',**kwargs);put(uj,'dynamics',damping='.3',friction='0')
  for kind,n,topic,vel in self.controllers:
   if kind=='pos':
    pl=put(mo,'plugin',filename='gz-sim-joint-position-controller-system',name='gz::sim::systems::JointPositionController');put(pl,'joint_name',n);put(pl,'topic',topic);put(pl,'use_velocity_commands','true');put(pl,'cmd_max',vel);put(pl,'cmd_min',-vel);put(pl,'p_gain',10)
   elif kind=='roller':
    pl=put(mo,'plugin',filename='gz-sim-joint-controller-system',name='gz::sim::systems::JointController');put(pl,'joint_name',n);put(pl,'topic',topic);put(pl,'use_force_commands','false')
   else:
    pl=put(mo,'plugin',filename='gz-sim-track-controller-system',name='gz::sim::systems::TrackController');put(pl,'link',n);put(pl,'velocity_topic',topic);put(pl,'track_orientation',f'0 {PI} 0');put(pl,'max_command_age',.35);put(pl,'min_velocity',-vel);put(pl,'max_velocity',vel);put(pl,'min_acceleration',-1.2);put(pl,'max_acceleration',1.2)
  pl=put(mo,'plugin',filename='gz-sim-joint-state-publisher-system',name='gz::sim::systems::JointStatePublisher');put(pl,'topic',f'/model/{self.name}/joint_states')
  write_xml(sd,d/'model.sdf');write_xml(ur,PKG/'urdf'/f'{self.name}.urdf');scene.export(d/'meshes/visual.glb');scene.export(ROOT/'exports'/f'{self.name}.glb')
  cfg=E.Element('model');put(cfg,'name',self.name);put(cfg,'version','2.0');put(cfg,'sdf','model.sdf',version='1.9');put(cfg,'description','Custom simulation fixture: source, axes and interlocks supplied; not selected/certified hardware.');write_xml(cfg,d/'model.config')
  return self

def telescopic(m,prefix,base,centers,z0,height0,max_stage,nstages=2):
 current=base;stages=[base]
 for i in range(1,nstages+1):
  current=m.link(f'{prefix}_stage_{i}',current,'prismatic',limits=[0,max_stage],mass=6,velocity=.18);stages.append(current)
  m.pos(current+'_joint',f'/environment/{prefix}/stage_{i}',.18)
 widths=[.092,.072,.052,.032] if nstages==3 else [.074,.054,.034]
 for k,(x,y) in enumerate(centers):
  for li,w in zip(stages,widths):
   for sg in [-1,1]:
    m.add(li,f'guide_{k}_x{sg}',box([.004,w,height0],[x+sg*(w/2-.002),y,z0]),'trim')
    m.add(li,f'guide_{k}_y{sg}',box([w-.008,.004,height0],[x,y+sg*(w/2-.002),z0]),'trim')
  m.add(base,f'drive_{k}',box([.09,.07,.018],[x,y,.026]),'dark',False)
 return current

def belt(m,name,parent,length,width,surface,center=(0,0),roller=True):
 x,y=center;l=m.link(name,parent)
 m.add(l,'contact',box([length,width,.006],[x,y,surface-.003]),'rubber')
 for u in np.arange(-length/2+.05,length/2,.09):m.add(l,f'texture_{u:.3f}',box([.008,width-.015,.0008],[x+u,y,surface+.0004]),'rail',False)
 m.belt(l,f'/environment/{name}/speed')
 if roller:
  for s in [-1,1]:
   r=m.link(name+('_rear' if s<0 else '_front'),parent,'continuous',origin=(x+s*(length/2-.012),y,surface-.014),axis=(0,1,0),mass=.5,velocity=120)
   m.add(r,'drum',cyl(.011,width-.012,rpy=(PI/2,0,0),sections=24),'silver' if 'silver' in C else 'trim',False)
   m.add(r,'shaft',cyl(.004,width+.072,rpy=(PI/2,0,0),sections=12),'trim',False)
   for side in [-1,1]:m.add(parent,f'{name}_bearing_{s}_{side}',box([.026,.035,.036],[x+s*(length/2-.012),y+side*(width/2+.019),surface-.018]),'dark',False)
   m.add(r,'end_marker',box([.018,.002,.003],[0,-width/2,0]),'orange',False)
   m.controllers.append(('roller',r+'_joint',f'/environment/{name}/roller_speed',100))
 return l

def corner():
 m=Mechanism('corner_transfer');b=m.link('corner_base',origin=(1.3,0,0),mass=32)
 m.add(b,'spreader',box([.96,.96,.020],[0,0,.010]),'dark')
 st=telescopic(m,'corner',b,[(-.25,-.25),(-.25,.25),(.25,-.25),(.25,.25)],.105,.170,.34/3,nstages=3)
 m.add(st,'crossplate',box([.60,.60,.014],[0,0,.197]),'teal')
 m.add(st,'bearing_outer',tm.creation.annulus(r_min=.134,r_max=.19,height=.025,sections=40,transform=matrix([0,0,.2165])),'dark',False)
 ro=m.link('corner_rotor',st,'revolute',origin=(0,0,.225),rpy=(0,0,PI/2),axis=(0,0,-1),limits=[0,PI/2],mass=17,velocity=1.0)
 m.add(ro,'bearing_inner',cyl(.13,.027,[0,0,.004],sections=40),'trim')
 m.add(ro,'rotor_support_plate',box([.88,.65,.014],[0,0,.019]),'dark')
 for s in [-1,1]:
  m.add(ro,f'frame{s}',box([1.00,.035,.045],[0,s*.3275,.020]),'teal')
  m.add(ro,f'guard{s}',box([.94,.014,.035],[0,s*.310,.091]),'yellow')
 for x in [-.38,.38]:m.add(ro,f'crossmember{x}',box([.045,.64,.020],[x,0,-.001]),'dark')
 belt(m,'corner_belt',ro,1.0,.60,.075)
 m.pos('corner_rotor_joint','/environment/corner/angle',1.0)
 return m.export()

def buffer():
 m=Mechanism('longitudinal_buffer');b=m.link('buffer_base',origin=(2.70,0,0),mass=28)
 m.add(b,'spreader',box([1.22,.86,.018],[0,0,.009]),'dark')
 st=telescopic(m,'buffer',b,[(-.42,-.23),(-.42,.23),(.36,-.23),(.36,.23)],.140,.240,.129)
 for i,(x,y) in enumerate([(-.42,-.23),(-.42,.23),(.36,-.23),(.36,.23)]):
  m.add(st,f'post_head_{i}',box([.05,.05,.040],[x,y,.280]),'trim')
  m.add(b,f'post_foot_{i}',box([.09,.09,.004],[x,y,.020]),'dark')
 # Belt is from world X 2.044 to 3.644. Center local +0.144.
 for s in [-1,1]:m.add(st,f'rail{s}',box([1.60,.035,.045],[.144,s*.3275,.3325]),'teal');m.add(st,f'guide{s}',box([1.50,.014,.030],[.144,s*.310,.388]),'yellow')
 for x in [-.42,.36]:m.add(st,f'crossbar{x}',box([.04,.62,.020],[x,0,.310]),'dark')
 belt(m,'buffer_belt',st,1.60,.60,.370,(.144,0))
 m.add(st,'tail_deadplate',box([.010,.59,.002],[.951,0,.369]),'trim') # 3.646..3.656, robot belt begins 3.658.
 h=m.link('buffer_bridge',st,'revolute',origin=(-.664,0,.366),axis=(0,1,0),limits=[-PI/2,0],mass=2.8,velocity=.5)
 # Home horizontal in file; controller folds before any rotary/lift transfer.
 m.add(h,'plate',box([.228,.58,.004],[-.114,0,.002]),'dark')
 for i in range(7):
  rr=m.link(f'bridge_roller_{i}',h,'continuous',origin=(-.014-i*.032,0,-.001),axis=(0,1,0),mass=.06,velocity=250)
  m.add(rr,'drum',cyl(.005,.578,rpy=(PI/2,0,0),sections=16),'trim',False)
  m.add(rr,'shaft',cyl(.002,.60,rpy=(PI/2,0,0),sections=12),'dark',False)
  m.controllers.append(('roller',rr+'_joint','/environment/bridge_belt/roller_speed',250))
 m.add(h,'hinge_shaft',cyl(.010,.67,rpy=(PI/2,0,0),sections=24),'trim',False)
 for s in [-1,1]:m.add(st,f'bridge_bearing_{s}',box([.05,.045,.045],[-.652,s*.326,.348]),'dark',False)
 # Belt-contact approximation on this short powered nose. Seven end-supported rollers have their own continuous joints.
 m.belt(h,'/environment/bridge_belt/speed');m.pos('buffer_bridge_joint','/environment/buffer/bridge_angle',.5)
 return m.export()

def door_belt():
 m=Mechanism('door_belt');b=m.link('door_belt_frame',origin=(1.3,-1.55,0),rpy=(0,0,PI/2),mass=38)
 # In local +X (world +Y), from -2.588 to -.512, surface .300.
 for s in [-1,1]:m.add(b,f'frame{s}',box([2.076,.03,.055],[0,s*.3275,.250]),'yellow');m.add(b,f'guide{s}',box([2.02,.014,.030],[0,s*.313,.317]),'yellow')
 # Ground-service supported cantilever, no narrow feet on the cargo floor.
 for s in [-1,1]:m.add(b,f'ground_leg{s}',box([.075,.075,1.67],[-.68,s*.29,-.61]),'trim')
 belt(m,'door_belt_contact',b,2.076,.60,.300)
 return m.export()

def entry():
 a=Asset('entry_deck')
 a.add('floor_spreader',box([1.92,1.16,.012],[1.30,0,.006]),'dark',True)
 for x in [.48,1.30,2.12]:
  for y in [-.48,.48]:a.add(f'pillar_{x}_{y}',box([.07,.07,.148],[x,y,.086]),'trim',True)
 a.add('turn_platform',box([1.92,1.78,.020],[1.30,0,.170]),'teal',True)
 a.add('door_bridge',box([1.18,1.190,.025],[1.30,-1.489,.1675]),'yellow',True)
 a.add('external_platform',box([1.90,2.40,.070],[1.30,-3.29,.145]),'dark',True)
 for x in [.44,2.16]:
  for y in [-4.25,-2.33]:
   a.add(f'ground_leg_{x}_{y}',box([.095,.095,1.56],[x,y,-.635]),'trim',True)
   a.add(f'ground_foot_{x}_{y}',box([.32,.32,.035],[x,y,-1.4325]),'dark',True)
 # 1.8 m / 0.18 m = 5.71 degree straight exit ramp; full 1.18 m support width.
 v=np.array([[x,y,z] for y in [-.59,.59] for x,z in [(2.26,0),(4.06,0),(2.26,.18)]])
 a.add('exit_ramp',tm.convex.convex_hull(v),'trim',True)
 for y in [-.565,.565]:a.add(f'ramp_edge_{y}',beam([2.26,y,.182],[4.06,y,.002],.012,.020),'yellow',False)
 for y in [-.87,.87]:a.add(f'turn_edge_{y}',box([1.86,.015,.001],[1.3,y,.1805]),'yellow',False)
 a.export();return a

def robot_model(pose):
 bp=ROOT/'ros2/bellybot_dock_stack_description';mo=copy.deepcopy(E.parse(bp/'gazebo/bellybot_dock_stack.sdf').getroot().find('model'));p=mo.find('pose')
 if p is None:p=put(mo,'pose')
 p.text=nums(pose)
 # Truth pose telemetry is for simulator commissioning; navigation must supply real localization later.
 pl=put(mo,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher');put(pl,'topic','/model/bellybot_dock_stack/pose');put(pl,'publish_model_pose','true');put(pl,'publish_link_pose','false');put(pl,'publish_sensor_pose','false');put(pl,'publish_collision_pose','false');put(pl,'publish_visual_pose','false');put(pl,'use_pose_vector_msg','true');put(pl,'update_frequency',50)
 return mo

def worlds():
 for name,cut,entrymode in [('entry_resolved',False,True),('transfer_resolved',False,False),('transfer_cutaway',True,False)]:
  world(name,'nb_hold_cutaway' if cut else 'nb_hold',loader=False)
  p=PKG/'worlds'/f'{name}.sdf';s=E.parse(p).getroot();w=s.find('world');w.append(robot_model([1.3,-3.29,.183,0,0,PI/2] if entrymode else [4.1,0,.003,0,0,0]))
  for n in (['entry_deck'] if entrymode else ['belt_loader','door_belt','corner_transfer','longitudinal_buffer']):inc(w,n)
  if entrymode:w.find("frame[@name='bellybot_spawn']/pose").text=nums([1.3,-3.29,.183,0,0,PI/2])
  # transfer starts without a bag: spawn via command, preserving contact dynamics.
  write_xml(s,p)

def step_export(items,stem):
 import cadquery as cq
 ass=cq.Assembly(name='BellyBot_entry_and_transfer_R2');count=0
 for ob in items:
  parts=[]
  if isinstance(ob,Asset):parts=[(n,m,c) for n,m,c in ob.parts]
  else:
   ts={}
   for n,l in ob.links.items():
    motion=np.eye(4)
    if ob.name=='corner_transfer' and n.startswith('corner_stage_'):motion[:3,3]=[0,0,.198/3]
    if ob.name=='corner_transfer' and n=='corner_rotor':motion=tm.transformations.rotation_matrix(-PI/2,[0,0,1])
    if ob.name=='longitudinal_buffer' and n.startswith('buffer_stage_'):motion[:3,3]=[0,0,.128/2]
    ts[n]=(ts[l['parent']] if l['parent'] else np.eye(4))@l['T']@motion
    for nm,m,c,k in l['parts']:
     if 'texture_' not in nm and 'end_marker' not in nm:mc=m.copy();mc.apply_transform(ts[n]);parts.append((n+'_'+nm,mc,c))
  for n,m,c in parts:
   faces=[cq.Face.makeFromWires(cq.Wire.makePolygon([cq.Vector(*v) for v in tri*1000],close=True)) for tri in m.triangles]
   so=cq.Solid.makeSolid(cq.Shell.makeShell(faces));assert so.isValid(),n;ass.add(so,name=ob.name+'_'+n,color=cq.Color(*rgba(c)[:3]));count+=1
 out=ROOT/'exports'/(stem+'_mm.step');ass.save(str(out));chk=cq.importers.importStep(str(out)).val();assert chk.isValid();(ROOT/'docs'/(stem+'_check.json')).write_text(json.dumps({'valid':True,'solids':len(chk.Solids()),'units':'mm','scope':'Custom fixtures and full mechanism solids; cosmetic belt stripes omitted. Separate entry and work setups.'},indent=2))
 print('STEP checked',count,flush=True)

def main():
 c=corner();b=buffer();d=door_belt();e=entry();worlds();step_export([e],'Entry_Dock_R2');step_export([c,b,d,make_loader()],'Corner_Transfer_R2');import cadquery as cq
 overall=cq.Assembly(name='BellyBot_Aircraft_Work_Assembly_R2')
 for name,file,xyz,color in [('hold','NB_Forward_Hold_Layout_mm.step',(0,0,0),(.78,.81,.77)),('BellyBot','BellyBot_v3_robot_mm.step',(4100,0,3),(.1,.6,.47)),('transfer','Corner_Transfer_R2_mm.step',(0,0,0),(.23,.48,.43))]:
  shape=cq.importers.importStep(str(ROOT/'exports'/file)).val();overall.add(shape,name=name,loc=cq.Location(cq.Vector(*xyz)),color=cq.Color(*color))
 out=ROOT/'exports/BellyBot_Aircraft_Work_Assembly_R2_mm.step';overall.save(str(out));check=cq.importers.importStep(str(out)).val();assert check.isValid();(ROOT/'docs/Work_Assembly_R2_check.json').write_text(json.dumps({'valid':True,'solids':len(check.Solids()),'units':'mm','robot_pose_mm':[4100,0,3],'configuration':'Corner raised/turned, buffer receiving, robot stowed; no entry fixtures'},indent=2))
 print('Built entry and transfer fixtures and overall work assembly',flush=True)
if __name__=='__main__':main()
