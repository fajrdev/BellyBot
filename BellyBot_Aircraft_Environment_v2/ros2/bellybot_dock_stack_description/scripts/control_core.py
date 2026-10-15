"""V3 SI kinematics and sampled swept-clearance guards. No ROS dependency."""
import math

def clamp(x,a,b):return min(b,max(a,x))
def smoothstep(u):
 u=clamp(u,0,1);return 10*u**3-15*u**4+6*u**5

def duration_for(delta,vmax,amax):
 return max(.05,1.875*abs(delta)/vmax,math.sqrt((10/math.sqrt(3))*abs(delta)/amax))

def wheel_speeds(v,w):return ((v-w*.750/2)/.068,(v+w*.750/2)/.068)

def tip_height(lift,pitch):return .5035+lift-.920*math.sin(pitch)-.0335*math.cos(pitch)
def floor_pitch(lift=0):return math.asin((.5035+lift-.002)/math.hypot(.920,.0335))-math.atan2(.0335,.920)
def lift_limit(bag_height=.28,local_roof=1.18,margin=.10):return clamp(local_roof-margin-.495-max(.040,bag_height+.005),0,.6)

def validate_pose(q,bag_height=.28,local_roof=1.18):
 if len(q)!=6 or not all(math.isfinite(v) for v in q):raise ValueError('Six finite values required: lift,yaw,reach,infeed,pitch,feed_height')
 if not (math.isfinite(bag_height) and bag_height>=0 and math.isfinite(local_roof)):raise ValueError('Invalid payload or roof')
 lift,yaw,reach,feed,pitch,fh=q
 bounds=[(0,.6),(-math.radians(95),math.radians(95)),(0,1.),(0,.4),(0,math.radians(32)),(0,.25)]
 if any(x<a-1e-9 or x>b+1e-9 for x,(a,b) in zip(q,bounds)):raise ValueError('Mechanical limit exceeded')
 # Conservative bag-plus-guide upper bound across pitch; bag assumed centered.
 zmax=.5035+lift+.005+max(.0315*math.cos(pitch),(-.00835+bag_height)*math.cos(pitch)-.05*math.sin(pitch))
 if zmax>local_roof-.10:raise ValueError('Payload / guide roof envelope exceeded')
 if tip_height(lift,pitch)<.0019:raise ValueError('Nose would enter floor')
 if pitch>.001:
  if reach<.84 or abs(yaw)>.01 or lift>.005:raise ValueError('Floor mode requires lift zero, yaw zero, reach >= 0.84 m')
 if abs(yaw)>.01 and lift<.08:raise ValueError('Raise at least 80 mm before yaw near infeed')
 if abs(yaw)>.01 and feed>.15+1e-9:raise ValueError('Retract infeed to 150 mm or less before yaw')
 if pitch<.001 and lift<.08 and abs(yaw)>.01:raise ValueError('Stow yaw before lowering')

def plan(start,target,bag_height=.28,local_roof=1.18):
 """[total lift,yaw,total reach,infeed stroke,pitch,infeed height].
 Roof/pitch checks cover sampled poses, not arbitrary obstacles or a free bag.
 """
 validate_pose(start,bag_height,local_roof);validate_pose(target,bag_height,local_roof)
 q=list(start);out=[]
 def add(v):
  nonlocal q
  if any(abs(a-b)>1e-8 for a,b in zip(v,q)):
   for i in range(41):
    u=smoothstep(i/40);validate_pose([a+(b-a)*u for a,b in zip(q,v)],bag_height,local_roof)
   out.append(list(v));q=list(v)
 # Preserve the floor angle during the short insertion/withdrawal stroke.
 if q[4]>.001 and target[4]>.001 and abs(target[0]-q[0])<1e-8 and abs(target[1]-q[1])<1e-8:
  add(target);return out
 # Level a pitched tray before changing other axes; insertion/retraction can stay level with the floor.
 if q[4]>.001 and (abs(target[1]-q[1])>.001 or abs(target[0]-q[0])>.001 or target[2]<.84):
  v=q.copy();v[4]=0;add(v)
 if abs(q[1]-target[1])>.001:
  v=q.copy();v[3]=min(q[3],.15);add(v)
  v=q.copy();v[0]=max(q[0],target[0],.08);v[4]=0;add(v)
  v=q.copy();v[2]=0;add(v)
  v=q.copy();v[1]=target[1];add(v)
 if target[4]>.001:
  v=q.copy();v[4]=0;v[0]=0;v[1]=0;add(v)
  v=q.copy();v[2]=target[2];add(v)
  add(target)
 else:
  if q[4]>.001:
   v=q.copy();v[4]=0;add(v)
  # Lift and extension may overlap only for yaw zero/raised, level tray paths checked above.
  add(target)
 return out

VMAX=[.36,1.,1.4,.12,.6,.12]
AMAX=[.50,1.5,2.5,.12,.8,.20]

def segment_duration(a,b):return max(duration_for(y-x,v,z) for x,y,v,z in zip(a,b,VMAX,AMAX))
