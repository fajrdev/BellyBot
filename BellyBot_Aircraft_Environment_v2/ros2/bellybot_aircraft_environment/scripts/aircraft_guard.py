"""Enforced forward-hold pose envelope. Pure Python + numpy, used by the ROS node.
Pressure is a four-patch assigned-mass screen, not measured aircraft floor loading.
"""
import math,json
from pathlib import Path
import numpy as np

def lift_cap(height):
 if not math.isfinite(height) or not 0<=height<=.38:raise ValueError('Measured payload height must be 0..0.38 m')
 return max(0,min(.60,math.floor((1.12-.100-.003-.495-max(.040,height+.005)+1e-9)*100)/100))
def smooth(u):return 10*u**3-15*u**4+6*u**5
def rotation(a,t):
 a=np.asarray(a);a=a/np.linalg.norm(a);x,y,z=a;c=math.cos(t);s=math.sin(t);d=1-c
 return np.array([[c+x*x*d,x*y*d-z*s,x*z*d+y*s],[y*x*d+z*s,c+y*y*d,y*z*d-x*s],[z*x*d-y*s,z*y*d+x*s,c+z*z*d]])
class Guard:
 def __init__(self,file):
  self.data=json.loads(Path(file).read_text());self.links=self.data['links'];self.joints=self.data['joints'];self.verts={l['name']:np.c_[np.array(l['vertices']),np.ones(len(l['vertices']))] for l in self.links}
 def evaluate(self,q,height=.38,mass=32):
  if len(q)!=4 or not all(math.isfinite(v) for v in q):raise ValueError('Expected lift m, yaw rad, reach m, pitch rad')
  lift,yaw,reach,pitch=q;cap=lift_cap(height);tol=1e-5
  if not 0<=mass<=32:raise ValueError('Payload mass outside 0..32 kg')
  if not (-tol<=lift<=cap+tol and abs(yaw)<=math.radians(65)+tol and -tol<=reach<=1+tol and -tol<=pitch<=math.radians(32)+tol):raise ValueError('Aircraft joint operating limit exceeded')
  lift=max(0.,min(cap,lift));reach=max(0.,min(1.,reach));pitch=max(0.,min(math.radians(32),pitch))
  if abs(yaw)>.01 and (lift<.08 or reach>.50):raise ValueError('Yaw requires >=80 mm lift and <=500 mm reach')
  if pitch>.001 and (abs(yaw)>.01 or lift>.005 or reach<.72):raise ValueError(f'Floor pickup requires yaw=0, lift=0, reach>=720 mm: got yaw={yaw:.4f}, lift={lift:.4f}, reach={reach:.4f}, pitch={pitch:.4f}')
  vals={**{f'lift_stage_{i}_joint':lift/3 for i in [1,2,3]},'yaw_joint':yaw,'extension_mid_joint':reach/2,'extension_tip_joint':reach/2,'tray_pitch_joint':pitch}
  frames={self.data['root']:np.eye(4)};todo=self.joints.copy()
  while todo:
   rem=[]
   for j in todo:
    if j['parent'] not in frames:rem.append(j);continue
    b=np.eye(4);value=vals.get(j['name'],0)
    if j['type']=='prismatic':b[:3,3]=np.array(j['axis'])*value
    elif j['type'] in ('revolute','continuous'):b[:3,:3]=rotation(j['axis'],value)
    frames[j['child']]=frames[j['parent']]@np.array(j['origin'])@b
   if len(rem)==len(todo):raise ValueError('Bad kinematic graph')
   todo=rem
  points=[];total=0.;moment=np.zeros(3)
  for l in self.links:
   t=frames[l['name']];points.append((t@self.verts[l['name']].T).T[:,:3]);total+=l['mass'];moment+=l['mass']*(t@np.r_[l['com'],1])[:3]
  tray=frames['tray'];com=tray@np.array([.43,0,-.00835+.002+height/2,1]);moment+=mass*com[:3];total+=mass
  if mass>0:
   v=np.array([[.43+x,y,-.00835+.002+z,1] for x in [-.39,.39] for y in [-.25,.25] for z in [0,height]]);points.append((tray@v.T).T[:,:3])
  xyz=np.concatenate(points)+[4.1,0,.003];x,y,z=xyz.T;side=1e9
  for (y0,z0),(y1,z1) in zip([(.61,0),(.82,.10),(1.24,.42),(1.43,.80)],[(.82,.10),(1.24,.42),(1.43,.80),(1.525,1.12)]):
   s=(y1-y0)/(z1-z0);side=min(side,float(((y0+s*(z-z0)-abs(y))/math.sqrt(1+s*s)).min()))
  roof=float((1.12-z).min());floor=float(z.min());end=float(min(x.min()-.018,7.682-x.max()))
  if roof<.100-1e-7 or side<.025-1e-7 or floor<-.00001 or end<.025:raise ValueError(f'Enclosure clearance rejected: roof={roof:.3f}, wall={side:.3f}, floor={floor:.3f} m')
  com=moment/total;reactions=[total/4*(1+sx*com[0]/.33+sy*com[1]/.375) for sx in [-1,1] for sy in [-1,1]];pressure=max(reactions)/(.66*.35)*1.5
  if min(reactions)<0 or pressure>600:raise ValueError(f'Assigned-mass pressure screen rejected: {pressure:.1f} kg/m2')
  return {'roof_m':roof,'wall_m':side,'lift_cap_m':cap,'pressure_screen_kg_m2':pressure}
 def plan(self,start,target,height=.38,mass=32):
  # Conservative staged travel: level, retract, raise, yaw, then extend/lift.
  self.evaluate(start,height,mass);self.evaluate(target,height,mass);q=list(start);out=[]
  def add(p):
   nonlocal q
   if max(abs(a-b) for a,b in zip(q,p))<1e-8:return
   for i in range(81):self.evaluate([a+(b-a)*smooth(i/80) for a,b in zip(q,p)],height,mass)
   out.append(p.copy());q=p.copy()
  if q[3]>.001 and (target[2]<.72 or abs(target[1]-q[1])>.001 or target[0]>.005):p=q.copy();p[3]=0;add(p)
  if abs(target[1]-q[1])>.001:
   p=q.copy();p[3]=0;add(p);p[2]=0;add(p);p[0]=max(.08,q[0],target[0]);add(p);p[1]=target[1];add(p)
  if target[3]>.001:
   p=q.copy();p[3]=0;p[0]=0;p[1]=0;add(p);p[2]=target[2];add(p)
  add(list(target));return out
