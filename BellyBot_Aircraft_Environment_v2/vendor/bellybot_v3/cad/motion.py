"""Unloaded CAD commissioning preview, including floor reach; not throughput footage."""
import math,csv,json
import numpy as np
from model import ROOT,P,configuration,build,LINKS,floor_pitch

def key(lift=0,yaw=0,reach=0,feed=.4,wheel=0,roller=0,pitch=0,fh=0):
 q=configuration(lift,yaw,reach,feed,wheel,pitch,fh)
 q.update(tray_front_roller_joint=roller,tray_rear_roller_joint=roller,infeed_front_roller_joint=roller,infeed_rear_roller_joint=roller)
 return q
STEPS=[('Home / five wheels per side',1,key()),('Wheel rotation test',3,key(wheel=4.)),
 ('Level tray / extend clear of chassis',4,key(reach=.84)),('Pitch nose down to floor',3,key(reach=.84,pitch=floor_pitch())),
 ('Advance nose along floor',2,key(reach=1.,pitch=floor_pitch())),('Reverse insertion motion',2,key(reach=.84,pitch=floor_pitch())),
 ('Level before retracting',3,key(reach=.84)),('Retract floor-access cassette',4,key()),
 ('Match tray and infeed to next layer',3,key(lift=.1,fh=.1)),('Extend at fixed layer height',3,key(lift=.1,fh=.1,reach=.8)),
 ('Retract / belt compensation direction',3,key(lift=.1,fh=.1,roller=80.)),
 ('Retract infeed clear of yaw sweep',4,key(lift=.1,fh=.1,feed=.15)),
 ('Illustration yaw pose',4,key(lift=.1,yaw=math.radians(65),fh=.1,feed=.15)),
 ('Illustration extension',3,key(lift=.1,yaw=math.radians(65),reach=.5,fh=.1,feed=.15)),('Hold / kinematics only',2,key(lift=.1,yaw=math.radians(65),reach=.5,fh=.1,feed=.15))]
DURATION=sum(t for _,t,_ in STEPS)
def sample(t):
 prev=key();t=max(0,t)
 for label,T,target in STEPS:
  if t<=T:
   u=t/T;s=10*u**3-15*u**4+6*u**5;ds=(30*u*u-60*u**3+30*u**4)/T
   return ({n:prev[n]+(v-prev[n])*s for n,v in target.items()},{n:(v-prev[n])*ds for n,v in target.items()},label)
  t-=T;prev=target
 return dict(prev),{n:0. for n in prev},'Stopped'
def main():
 build();out=ROOT/'ros2/bellybot_dock_stack_description/config';names=list(key())
 with (out/'joint_demo.csv').open('w',newline='') as f:
  w=csv.writer(f);w.writerow(['time_s',*names])
  for t in np.linspace(0,DURATION,round(DURATION*60)+1):
   q,dq,_=sample(float(t))
   for n,v in q.items():
    d=next(d for d in LINKS.values() if d['joint']==n)
    assert d['kind']=='continuous' or d['lower']-1e-9<=v<=d['upper']+1e-9,(n,v)
    assert abs(dq[n])<=d['velocity']+1e-8,(n,dq[n],d['velocity'])
   w.writerow([t,*[q[n] for n in names]])
 (ROOT/'docs/motion_profile.json').write_text(json.dumps({'duration_s':DURATION,'sample_hz':60,'sequence':[{'phase':n,'duration_s':d} for n,d,_ in STEPS],'kinematic_only':True,'joint_limits_checked':True},indent=2))
if __name__=='__main__':main()
