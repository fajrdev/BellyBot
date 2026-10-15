#!/usr/bin/env python3
"""Build original SI-unit meshes, SDF, URDF and previews from environment.json.
Requires numpy + trimesh. Optional STEP export uses CadQuery. No online assets.
"""
from pathlib import Path
import argparse, json, math, copy, xml.etree.ElementTree as E
import numpy as np
import trimesh as tm
ROOT=Path(__file__).resolve().parents[1]
PKG=ROOT/'ros2/bellybot_aircraft_environment'
CFG=json.loads((ROOT/'config/environment.json').read_text())
C={'liner':'#d5d7d1','panel':'#e5e6df','trim':'#909d9d','rail':'#5d7274','floor':'#899d91',
   'dark':'#293941','teal':'#169a87','yellow':'#e8b84a','cloth':'#b69256','rubber':'#252c32',
   'vest':'#c9dd49','orange':'#e46d38','skin':'#bd927d','blue':'#324655'}
def rgba(c):
 c=C.get(c,c).lstrip('#');return [int(c[i:i+2],16)/255 for i in (0,2,4)]+[1]
def put(p,tag,text=None,**attrs):
 e=E.SubElement(p,tag,**attrs)
 if text is not None:e.text=str(text)
 return e
def nums(v):return ' '.join(f'{float(x):.10g}' for x in v)
def write_xml(root,path):
 E.indent(root,space='  ');E.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
def matrix(pos=(0,0,0),rpy=(0,0,0)):
 m=tm.transformations.euler_matrix(*rpy);m[:3,3]=pos;return m
def box(size,pos=(0,0,0),rpy=(0,0,0)):
 return tm.creation.box(extents=size,transform=matrix(pos,rpy))
def cyl(r,h,pos=(0,0,0),rpy=(0,0,0),sections=16):
 return tm.creation.cylinder(r,h,sections=sections,transform=matrix(pos,rpy))
def beam(a,b,width=.012,depth=None):
 a=np.array(a);b=np.array(b);v=b-a
 m=tm.geometry.align_vectors([0,0,1],v);m[:3,3]=(a+b)/2
 return tm.creation.box(extents=[width,depth or width,np.linalg.norm(v)],transform=m)
def prism(verts):
 faces=[[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
 m=tm.Trimesh(np.array(verts),faces,process=True);m.fix_normals();return m
class Asset:
 def __init__(self,name):self.name=name;self.parts=[];self.collisions=[];self.mass=None;self.inertia_size=None
 def add(self,name,mesh,color,collision=False):
  mesh.visual.face_colors=(np.array(rgba(color))*255).astype(np.uint8)
  self.parts.append((name,mesh,color))
  if collision:self.collisions.append((name,mesh.copy()))
 def export(self):
  d=PKG/'models'/self.name; (d/'meshes').mkdir(parents=True,exist_ok=True)
  dae(self.parts,d/'meshes/visual.dae')
  merged=tm.util.concatenate([p[1] for p in self.parts]);merged.export(d/'meshes/visual.stl')
  sc=tm.Scene()
  for n,m,c in self.parts:sc.add_geometry(m,node_name=n,geom_name=n)
  sc.export(d/'meshes/visual.glb')
  s=E.Element('sdf',version='1.9'); model=put(s,'model',name=self.name)
  put(model,'static','true' if self.mass is None else 'false');link=put(model,'link',name='body')
  if self.mass is not None:inertial(link,self.mass,self.inertia_size)
  vis=put(link,'visual',name='detailed_visual');geo=put(vis,'geometry');me=put(geo,'mesh');put(me,'uri',f'model://{self.name}/meshes/visual.dae')
  for n,m in self.collisions:
   fn=f'collision_{n}.stl';m.export(d/'meshes'/fn)
   co=put(link,'collision',name=n);geo=put(co,'geometry');me=put(geo,'mesh');put(me,'uri',f'model://{self.name}/meshes/{fn}')
   friction(co,.6)
  if self.mass is not None and not self.collisions:
   co=put(link,'collision',name='body_collision');g=put(co,'geometry');put(put(g,'box'),'size',nums(self.inertia_size));friction(co,getattr(self,'mu',.45))
  write_xml(s,d/'model.sdf')
  config=E.Element('model');put(config,'name',self.name);put(config,'version','1.0');put(config,'sdf','model.sdf',version='1.9');put(config,'description','Original parameterized hackathon fixture; see provenance and limits.');write_xml(config,d/'model.config')
  return s

def dae(parts,path):
 # COLLADA 1.4.1 with per-part vertex normals and Lambert ambient+diffuse materials; metres, Z up.
 ns='http://www.collada.org/2005/11/COLLADASchema';E.register_namespace('',ns)
 root=E.Element('COLLADA',xmlns=ns,version='1.4.1');a=put(root,'asset');put(a,'created','2026-10-10T00:00:00Z');put(a,'modified','2026-10-10T00:00:00Z');put(a,'unit',name='meter',meter='1');put(a,'up_axis','Z_UP')
 effects=put(root,'library_effects');materials=put(root,'library_materials');geos=put(root,'library_geometries');vs=put(put(root,'library_visual_scenes'),'visual_scene',id='Scene',name='Scene')
 for i,(name,m,c) in enumerate(parts):
  ident=f'p{i}';effect=put(effects,'effect',id=ident+'effect');tech=put(put(effect,'profile_COMMON'),'technique',sid='common');la=put(tech,'lambert')
  col=rgba(c);put(put(la,'ambient'),'color',nums([col[0]*0.45,col[1]*0.45,col[2]*0.45,1.0]));put(put(la,'diffuse'),'color',nums(col));put(put(la,'specular'),'color','0.15 0.15 0.15 1.0')
  mat=put(materials,'material',id=ident+'material',name=name);put(mat,'instance_effect',url='#'+ident+'effect')
  geo=put(geos,'geometry',id=ident+'geometry',name=name);me=put(geo,'mesh')
  source=put(me,'source',id=ident+'positions');ar=put(source,'float_array',nums(m.vertices.ravel()),id=ident+'array',count=str(len(m.vertices)*3))
  acc=put(put(source,'technique_common'),'accessor',source='#'+ident+'array',count=str(len(m.vertices)),stride='3')
  for ax in 'XYZ':put(acc,'param',name=ax,type='float')
  nsource=put(me,'source',id=ident+'normals');nar=put(nsource,'float_array',nums(m.vertex_normals.ravel()),id=ident+'normals-array',count=str(len(m.vertex_normals)*3))
  nacc=put(put(nsource,'technique_common'),'accessor',source='#'+ident+'normals-array',count=str(len(m.vertex_normals)),stride='3')
  for ax in 'XYZ':put(nacc,'param',name=ax,type='float')
  vert=put(me,'vertices',id=ident+'vertices');put(vert,'input',semantic='POSITION',source='#'+ident+'positions')
  put(vert,'input',semantic='NORMAL',source='#'+ident+'normals')
  tri=put(me,'triangles',count=str(len(m.faces)),material=ident+'symbol');put(tri,'input',semantic='VERTEX',source='#'+ident+'vertices',offset='0');put(tri,'p',' '.join(map(str,m.faces.ravel())))
  node=put(vs,'node',id=ident,name=name);inst=put(node,'instance_geometry',url='#'+ident+'geometry');bind=put(put(inst,'bind_material'),'technique_common');put(bind,'instance_material',symbol=ident+'symbol',target='#'+ident+'material')
 put(put(root,'scene'),'instance_visual_scene',url='#Scene');write_xml(root,path)
def inertial(link,m,size):
 l,w,h=size;i=put(link,'inertial');put(i,'mass',m);j=put(i,'inertia')
 for tag,v in zip(['ixx','iyy','izz','ixy','ixz','iyz'],[m*(w*w+h*h)/12,m*(l*l+h*h)/12,m*(l*l+w*w)/12,0,0,0]):put(j,tag,v)
def friction(c,mu):
 s=put(c,'surface');f=put(put(s,'friction'),'ode');put(f,'mu',mu);put(f,'mu2',mu);b=put(s,'bounce');put(b,'restitution_coefficient',.04);put(b,'threshold',.1)
def width_at(z):
 p=CFG['forward']['profile_yz'];return float(np.interp(z,[q[1] for q in p],[q[0] for q in p]))
def make_hold(aft=False,cutaway=False):
 p=CFG['forward'];a=CFG['aft_stress'];L=a['length'] if aft else p['length'];H=p['ceiling_height'];door=a['door_clear_height'] if aft else p['door_clear_height'];dx=p['door_center_x'];dw=p['door_width_x'];D0,D1=dx-dw/2,dx+dw/2
 name='nb_aft_stress' if aft else ('nb_hold_cutaway' if cutaway else 'nb_hold');ob=Asset(name)
 def hz(x):return a['front_height']+(a['rear_height']-a['front_height'])*x/L if aft else H
 def yz(z,x):
  if not aft:return width_at(z)
  norm=z/hz(x);fraction=(width_at(norm*H)-.61)/(1.525-.61);upper=a['front_upper_width']+(a['rear_upper_width']-a['front_upper_width'])*x/L
  return .61+fraction*(upper/2-.61)
 # Structural floor; apron is a world object, not part of the cargo-floor frame.
 ob.add('load_floor',box([L,1.22,.045],[L/2,0,-.0225]),'floor',True)
 for x in np.arange(.55,L,.55):ob.add(f'floor_seam_{x:.2f}',box([.007,1.22,.002],[x,0,-.001]),'rail')
 for y in [-.50,-.36,.36,.50]:
  ob.add(f'flush_rail_{y}',box([L,.029,.004],[L/2,y,-.0015]),'rail')
  for x in np.arange(.18,L,.24):ob.add(f'rail_rivet_{y}_{x:.2f}',cyl(.0035,.001,[x,y,.0005],sections=8),'trim')
 xs=sorted(set([0.,L,D0,D1]+list(np.arange(.55,L,.55))))
 for i,(x0,x1) in enumerate(zip(xs[:-1],xs[1:])):
  xm=(x0+x1)/2;hc=hz(xm);levels=sorted(set([0,.10/H,.42/H,.80/H,1.,min(1,door/hc)]))
  for side in [-1,1]:
   for j,(t0,t1) in enumerate(zip(levels[:-1],levels[1:])):
    if side==-1 and D0<=xm<=D1 and t1*hc<=door+1e-8:continue
    v=[]
    for x in [x0,x1]:
     z0,z1=t0*hz(x),t1*hz(x);y0,y1=side*yz(z0,x),side*yz(z1,x);th=side*p['liner_thickness']
     v.extend([[x,y0,z0],[x,y1,z1],[x,y1+th,z1],[x,y0+th,z0]])
    m=prism(v)
    # Cutaway removes visuals only: use full-model collisions in the SDF below.
    if not (cutaway and side==-1):ob.add(f'liner_{side}_{i}_{j}',m,'panel' if (i+j)%3 else 'liner',True)
    else:ob.collisions.append((f'liner_{side}_{i}_{j}',m))
   if not(cutaway and side==-1):
    for t0,t1 in zip(levels[:-1],levels[1:]):
     z0,z1=t0*hc,t1*hc
     if side==-1 and D0<=xm<=D1 and z1<=door+1e-8:continue
     ob.add(f'seam_{side}_{i}_{t0}',beam([x0,side*(yz(z0,x0)-.002),z0],[x0,side*(yz(z1,x0)-.002),z1],.009),'trim')
    for t in [.20,.61,.94]:
     z=t*hc
     if side==-1 and D0<=xm<=D1 and z<door:continue
     ob.add(f'fastener_{side}_{i}_{t}',cyl(.0038,.003,[x0+.025,side*(yz(z,x0)-.003),z],[math.pi/2,0,0],8),'rail')
  # Flat forward ceiling / sloping aft stress ceiling.
  v=[]
  for x in [x0,x1]:
   h=hz(x);y=yz(h,x);v.extend([[x,-y,h],[x,y,h],[x,y,h+.035],[x,-y,h+.035]])
  m=prism(v)
  if not cutaway:ob.add(f'ceiling_{i}',m,'liner',True)
  else:ob.collisions.append((f'ceiling_{i}',m))
 # End walls are exact convex section fans (no giant bounding-box blockers).
 for x in [0,L]:
  h=hz(x);points=[[-.61,0],[.61,0]]+[[yz(t*h,x),t*h] for t in [.10/H,.42/H,.80/H,1.]]+[[-yz(t*h,x),t*h] for t in [1.,.80/H,.42/H,.10/H]]
  vv=[[xx,y,z] for xx in [x-.018,x+.018] for y,z in points]
  m=tm.convex.convex_hull(np.array(vv));ob.add(f'bulkhead_{x}',m,'liner',True)
  for y in np.linspace(-.55,.55,5):ob.add(f'end_protect_{x}_{y}',beam([x+(.025 if x==0 else -.025),y,.04],[x+(.025 if x==0 else -.025),y,h-.05],.025),'trim')
 # Floor-level entry pocket is an assumed transition through the chamfer.
 ob.add('entry_pocket_floor',box([dw,.95,.035],[dx,-1.085,-.0175]),'trim',True)
 ob.add('sill_lip',box([dw,.035,p['sill_lip_height']],[dx,-1.53,p['sill_lip_height']/2]),'rail',True)
 for x in [D0-.018,D1+.018]:ob.add(f'door_jamb_{x}',box([.036,.038,door],[x,-1.54,door/2]),'trim',True)
 ob.add('door_header',box([dw+.072,.06,.035],[dx,-1.54,door+.0175]),'trim',True)
 # Stowed open door above aperture, and folded tan liner curtain beside it.
 if not cutaway:
  ob.add('open_door_panel',box([dw,.54,.04],[dx,-1.69,door+.13],[.65,0,0]),'panel')
  ob.add('folded_curtain',box([.085,.66,.016],[D0-.055,-.99,.67],[.64,0,0]),'cloth')
 for x in np.arange(.60,L-.30,1.3):
  if not cutaway:
   ob.add(f'lamp_housing_{x}',cyl(.065,.016,[x,0,hz(x)-.008]),'trim')
   ob.add(f'lamp_lens_{x}',cyl(.050,.004,[x,0,hz(x)-.018]),'#fff1c2')
 # Tan restraint grid at far end: visual only. Structural end wall has collision.
 xx=L-.065;h=hz(xx)
 for y in np.arange(-.8,.81,.20):
  if abs(y)<yz(.20,xx):ob.add(f'net_vertical_{y}',beam([xx,y,.18],[xx,y,h-.08],.017,.007),'cloth')
 for z in np.arange(.20,h-.03,.17):
  y=yz(z,xx)-.07;ob.add(f'net_horizontal_{z}',beam([xx,-y,z],[xx,y,z],.016,.007),'cloth')
 return ob

def make_bag(name,size,mass,color,soft=False,fragile=False):
 a=Asset(name);a.mass=mass;a.inertia_size=size;a.mu=.70 if soft else (.30 if fragile else .45)
 l,w,h=size
 if soft:
  v=[]
  for xx,sy,sz in [(-l/2,.65,.70),(-l*.38,1,1),(l*.32,1,1),(l/2,.60,.65)]:
   for t in np.linspace(0,2*math.pi,12,endpoint=False):v.append([xx,w/2*sy*math.cos(t),h/2*sz*math.sin(t)])
  mesh=tm.convex.convex_hull(v);a.add('soft_rigid_proxy',mesh,color)
 else:
  # Beveled visual case. One conservative cuboid collision, centered inertia.
  corners=np.array([[x,y,z] for x in [-l/2+.022,l/2-.022] for y in [-w/2+.022,w/2-.022] for z in [-h/2+.022,h/2-.022]])
  v=[]
  for corner in corners:
   for vec in np.eye(3):
    for sg in [-1,1]:v.append(corner+vec*.022*sg)
  a.add('case',tm.convex.convex_hull(v),color)
  a.add('zipper_band',box([l-.01,w+.001,.013],[0,0,0]),'dark')
  for x in np.linspace(-l*.33,l*.33,5):a.add(f'rib_{x}',box([.010,w*.82,.007],[x,0,h/2-.001]),'trim')
 # Handles remain inside conservative box to avoid artificial collision snags.
 a.add('handle',box([.17,.023,.014],[0,0,h/2-.007]),'dark')
 if fragile:a.add('fragile_label',box([.18,.15,.001],[l*.16,0,h/2+.0006]),'yellow')
 else:a.add('id_tag',box([.065,.045,.001],[l*.25,w*.23,h/2+.0006]),'#eee6c9')
 return a

def make_workers():
 a=Asset('manual_workers')
 for j,(x,y,yaw) in enumerate([(1.8,-.12,-1.0),(3.2,.0,math.pi)]):
  T=matrix([x,y,0],[0,0,yaw])
  pieces=[]
  def b(n,m,c,co=True):m.apply_transform(T);a.add(f'worker_{j}_{n}',m,c,co)
  b('torso',box([.26,.38,.42],[0,0,.50],[0,.45,0]),'vest')
  b('neck',cyl(.043,.13,[.145,0,.715]),'skin');head=tm.creation.icosphere(subdivisions=2,radius=.10);head.apply_translation([.17,0,.83]);b('head',head,'skin')
  for s in [-1,1]:
   b(f'thigh{s}',beam([0,s*.14,.34],[.23,s*.14,.12],.13),'blue')
   b(f'shin{s}',beam([.23,s*.14,.12],[-.16,s*.14,.095],.11),'blue')
   b(f'boot{s}',box([.20,.115,.09],[-.22,s*.14,.065]),'dark')
   b(f'upperarm{s}',beam([.10,s*.23,.65],[.28,s*.24,.40],.085),'blue')
   b(f'forearm{s}',beam([.28,s*.24,.40],[.47,s*.19,.26],.068),'skin')
  b('vest_stripe',box([.016,.40,.30],[.12,0,.51],[0,.45,0]),'orange',False)
 return a

def make_loader():
 p=CFG['loader'];dx=CFG['forward']['door_center_x'];a=Asset('belt_loader');dy=p['high_y']-p['low_y'];dz=p['high_surface_z']-p['low_surface_z'];angle=math.atan2(dz,dy);length=math.hypot(dy,dz)
 T=matrix([dx,(p['low_y']+p['high_y'])/2,(p['low_surface_z']+p['high_surface_z'])/2],[0,-angle,math.pi/2]);sz=[length,p['width'],.035]
 for n,m,c in [('belt',box(sz,[0,0,-.0175]),'rubber'),('edge_l',box([length,.035,.10],[0,-p['width']/2-.0175,0]),'yellow'),('edge_r',box([length,.035,.10],[0,p['width']/2+.0175,0]),'yellow')]:m.apply_transform(T);a.add(n,m,c)
 for u in np.arange(-length/2+.15,length/2,.15):
  m=box([.009,p['width']-.05,.001],[u,0,.001]);m.apply_transform(T);a.add(f'grip_{u}',m,'rail')
 apron=-CFG['forward']['apron_drop']
 for y in [p['low_y']+.8,p['high_y']-.8]:
  z=p['low_surface_z']+(y-p['low_y'])/dy*dz
  for xx in [dx-.26,dx+.26]:
   a.add(f'leg_{xx}_{y}',beam([xx,y,apron+.12],[xx,y,z-.08],.065),'trim',True)
   a.add(f'wheel_{xx}_{y}',cyl(.12,.06,[xx,y,apron+.12],[0,math.pi/2,0]),'dark',True)
 a.export();d=PKG/'models/belt_loader';s=E.parse(d/'model.sdf').getroot();mo=s.find('model');mo.find('static').text='false';body=mo.find('link');inertial(body,130,[length,.80,.55]);put(body.find('inertial'),'pose',nums([*T[:3,3],0,-angle,math.pi/2]))
 belt=put(mo,'link',name='belt_contact');pose=np.array(T[:3,3])-T[:3,2]*.0175;put(belt,'pose',nums([*pose,0,-angle,math.pi/2]));inertial(belt,10,sz)
 co=put(belt,'collision',name='belt_surface');put(put(put(co,'geometry'),'box'),'size',nums(sz));friction(co,.65)
 bj=put(mo,'joint',name='belt_frame_fixed',type='fixed');put(bj,'parent','body');put(bj,'child','belt_contact')
 for side in [-1,1]:
  center=(T@np.array([0,side*(p['width']/2+.0175),0,1]))[:3];co=put(body,'collision',name=f'guard{side}');put(co,'pose',nums([*center,0,-angle,math.pi/2]));put(put(put(co,'geometry'),'box'),'size',nums([length,.035,.10]));friction(co,.45)
 j=put(mo,'joint',name='anchored_to_apron',type='fixed');put(j,'parent','world');put(j,'child','body')
 plugin=put(mo,'plugin',filename='gz-sim-track-controller-system',name='gz::sim::systems::TrackController');put(plugin,'link','belt_contact');put(plugin,'velocity_topic','/environment/loader/speed');put(plugin,'track_orientation','0 3.141592653589793 0');put(plugin,'max_command_age',p['command_timeout']);put(plugin,'min_velocity',-.5);put(plugin,'max_velocity',.5);put(plugin,'min_acceleration',-.3);put(plugin,'max_acceleration',.3)
 write_xml(s,d/'model.sdf');return a

def inc(world,name,instance=None,pose=None):
 i=put(world,'include');put(i,'uri','model://'+name)
 if instance:put(i,'name',instance)
 if pose is not None:put(i,'pose',nums(pose))
 return i
SCENES={}
def world(name,hold='nb_hold',humans=False,bags=False,loader=True):
 root=E.Element('sdf',version='1.9');w=put(root,'world',name=name);put(w,'gravity','0 0 -9.81');p=put(w,'physics',name='one_ms',type='ignored');put(p,'max_step_size',.001);put(p,'real_time_factor',1)
 for f,n in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('contact','Contact')]:put(w,'plugin',filename=f'gz-sim-{f}-system',name=f'gz::sim::systems::{n}')
 ps=put(w,'plugin',filename='gz-sim-sensors-system',name='gz::sim::systems::Sensors');put(ps,'render_engine','ogre2')
 scene=put(w,'scene');put(scene,'ambient','.66 .66 .63 1');put(scene,'background','.10 .16 .18 1');put(scene,'shadows','true')
 light=put(w,'light',name='apron_light',type='directional');put(light,'pose','0 0 6 0 0 0');put(light,'diffuse','.8 .84 .87 1');put(light,'direction','-.25 .4 -1');put(light,'cast_shadows','true')
 for i,x in enumerate([1.0,3.,5.,7.]):
  li=put(w,'light',name=f'hold_light_{i}',type='point');put(li,'pose',nums([x,0,1.04,0,0,0]));put(li,'diffuse','.85 .82 .70 1');at=put(li,'attenuation');put(at,'range',2.5);put(at,'constant',.55);put(at,'linear',.15);put(at,'quadratic',.07);put(li,'cast_shadows','false')
 apron=put(w,'model',name='apron');put(apron,'static','true');link=put(apron,'link',name='slab')
 for tag in ['visual','collision']:
  v=put(link,tag,name='surface');put(v,'pose',nums([4,-1,-CFG['forward']['apron_drop']-.06,0,0,0]));put(put(put(v,'geometry'),'box'),'size','18 13 .12')
  if tag=='visual':ma=put(v,'material');put(ma,'ambient','.30 .34 .34 1');put(ma,'diffuse','.30 .34 .34 1')
 inc(w,hold);entries=[(hold,np.eye(4))]
 if loader:inc(w,'belt_loader');entries.append(('belt_loader',np.eye(4)))
 if humans:inc(w,'manual_workers');entries.append(('manual_workers',np.eye(4)))
 if bags:
  entries_bags=[('bag_hard_32',[6.85,-.27,.163,0,0,0]),('bag_hard_20',[6.85,.28,.133,0,0,0]),('bag_duffel_23',[6.02,-.22,.162,0,0,.14]),('bag_fragile_20',[6.03,.30,.192,0,0,-.05])]
  for i,(bn,po) in enumerate(entries_bags):inc(w,bn,f'cargo_{i}',po);entries.append((bn,matrix(po[:3],po[3:])))
 if humans:
  po=[1.3,-.38,.153,0,0,math.pi/2];inc(w,'bag_hard_26','manual_case',po);entries.append(('bag_hard_26',matrix(po[:3],po[3:])))
 # Named coordinate frames resolve to cargo floor datum, not aircraft flight frames.
 fr=put(w,'frame',name='cargo_floor',attached_to='world');put(fr,'pose','0 0 0 0 0 0')
 fr=put(w,'frame',name='door_handoff',attached_to='world');put(fr,'pose','1.3 -.512 .300 0 0 1.5707963268')
 fr=put(w,'frame',name='bellybot_spawn',attached_to='world');r=CFG['robot_spawn'];put(fr,'pose',nums([r['x'],r['y'],r['z'],0,0,r['yaw']]))
 gui=put(w,'gui',fullscreen='false');cam=put(gui,'camera',name='inspection');put(cam,'pose','10 -9 6 0 .45 2.3');put(cam,'view_controller','orbit')
 write_xml(root,PKG/'worlds'/f'{name}.sdf');SCENES[name]=entries

def export_urdf():
 r=E.Element('robot',name='aircraft_environment');base=put(r,'link',name='cargo_floor')
 for name in ['nb_hold_cutaway','belt_loader']:
  link=put(r,'link',name=name);vis=put(link,'visual');mesh=put(put(vis,'geometry'),'mesh',filename=f'package://bellybot_aircraft_environment/models/{name}/meshes/visual.dae',scale='1 1 1');joint=put(r,'joint',name=name+'_fixed',type='fixed');put(joint,'parent',link='cargo_floor');put(joint,'child',link=name)
 write_xml(r,PKG/'urdf/environment.urdf')

def build():
 assets={}
 for a in [make_hold(),make_hold(cutaway=True),make_hold(aft=True),make_workers(),
  make_bag('bag_hard_20',[.62,.42,.26],20,'#87664e'),make_bag('bag_hard_26',[.72,.46,.30],26,'#325d74'),
  make_bag('bag_hard_32',[.78,.50,.32],32,'#6e7185'),make_bag('bag_duffel_23',[.70,.40,.32],23,'#657e55',True),make_bag('bag_fragile_20',[.64,.44,.38],20,'#b3754d',fragile=True)]:
  a.export();assets[a.name]=a;print('Built',a.name,flush=True)
 a=make_loader();assets[a.name]=a
 world('manual_reference',humans=True,bags=True)
 world('robot_clearance',loader=False)
 world('loading_task',bags=True)
 world('presentation_cutaway','nb_hold_cutaway',bags=True)
 world('aft_taper_stress','nb_aft_stress',loader=False)
 export_urdf()
 for name in ['manual_reference','presentation_cutaway']:
  scene=tm.Scene()
  for asset,T in SCENES[name]:
   for n,m,c in assets[asset].parts:scene.add_geometry(m.copy(),node_name=f'{asset}_{n}',transform=T)
  scene.export(ROOT/'exports'/f'{name}.glb')
 manifest={k:{'visual_parts':len(a.parts),'collision_parts':len(a.collisions),'mass':a.mass,'bounds':tm.util.concatenate([m for _,m,_ in a.parts]).bounds.tolist()} for k,a in assets.items()}
 (ROOT/'docs/assets.json').write_text(json.dumps(manifest,indent=2));(PKG/'config/environment.json').write_text(json.dumps(CFG,indent=2))
 return assets

if __name__=='__main__':build()
