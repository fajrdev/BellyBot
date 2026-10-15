"""URDF FK for the delivered packages; exact triangulated visual/primitive assets."""
from pathlib import Path
import math,xml.etree.ElementTree as E
import numpy as np,trimesh as tm
ROOT=Path(__file__).resolve().parents[1]
def vec(e,k,default):return np.fromstring(e.get(k,default) if e is not None else default,sep=' ')
def origin(e):
 t=tm.transformations.euler_matrix(*vec(e,'rpy','0 0 0'));t[:3,3]=vec(e,'xyz','0 0 0');return t
class URDF:
 def __init__(self,path):
  self.path=Path(path);self.root=E.parse(path).getroot();self.joints=self.root.findall('joint');self.parts=[]
  for l in self.root.findall('link'):
   for vi,v in enumerate(l.findall('visual')):
    me=v.find('geometry/mesh')
    if me is None:continue
    uri=me.get('filename')
    p=ROOT/'ros2'/uri[len('package://'):] if uri.startswith('package://') else self.path.parent/uri
    # Load DAE via corresponding STL + material color; GLB has the exact same dimensions.
    if p.suffix=='.dae':p=p.with_suffix('.stl')
    m=tm.load_mesh(p);m.apply_scale(vec(me,'scale','1 1 1'));m.apply_transform(origin(v.find('origin')))
    col=v.find('material/color');m.visual.face_colors=(vec(col,'rgba','.08 .58 .48 1')*255).astype(np.uint8)
    self.parts.append((l.get('name'),v.get('name',str(vi)),m))
  self.base=next(iter({x.get('name') for x in self.root.findall('link')}-{j.find('child').get('link') for j in self.joints}))
 def fk(self,q=None,world=None):
  q=q or {};ts={self.base:np.eye(4) if world is None else world};todo=self.joints.copy()
  while todo:
   remaining=[]
   for j in todo:
    p=j.find('parent').get('link');c=j.find('child').get('link')
    if p not in ts:remaining.append(j);continue
    a=origin(j.find('origin'));axis=vec(j.find('axis'),'xyz','0 0 1');val=q.get(j.get('name'),0);b=np.eye(4)
    if j.get('type')=='prismatic':b[:3,3]=axis*val
    if j.get('type') in ('revolute','continuous'):b=tm.transformations.rotation_matrix(val,axis)
    ts[c]=ts[p]@a@b
   if len(remaining)==len(todo):raise ValueError('disconnected URDF')
   todo=remaining
  return ts
 def geometry(self,q=None,world=None):
  ts=self.fk(q,world);out=[]
  for l,n,m in self.parts:mc=m.copy();mc.apply_transform(ts[l]);out.append((l+'_'+n,mc))
  return out
 def manager(self,q=None,world=None):
  cm=tm.collision.CollisionManager()
  cm.add_object('assembly',tm.util.concatenate([m for n,m in self.geometry(q,world)]))
  return cm

def corner_q(lift=0,angle=0):return {**{f'corner_stage_{i}_joint':lift/3 for i in [1,2,3]},'corner_rotor_joint':angle}
def buffer_q(lift=0,bridge=-math.pi/2):return {'buffer_stage_1_joint':lift/2,'buffer_stage_2_joint':lift/2,'buffer_bridge_joint':bridge}
