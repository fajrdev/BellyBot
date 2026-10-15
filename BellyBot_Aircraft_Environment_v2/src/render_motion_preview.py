#!/usr/bin/env python3
"""Render a labelled, time-compressed CAD animation. NOT a physics result.
The illustrated bag is positioned kinematically for explanation only. The SDF
worlds/controller never attach or teleport bags. Frames go outside the release.
"""
from pathlib import Path
import sys,math,subprocess,tempfile
import numpy as np,trimesh as tm
from PIL import ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];P=ROOT/'ros2/bellybot_aircraft_environment';sys.path.insert(0,str(ROOT/'tools'))
from scene_tools import URDF,corner_q,buffer_q
from render_environment import render,collect
PIt=math.pi
class Colored:
 def __init__(self,name):
  self.r=URDF(P/'urdf'/f'{name}.urdf');self.home=self.r.fk();self.parts=[]
  for n,m in collect(P/'models'/name/'meshes/visual.glb'):
   k=next(k for k in sorted(self.home,key=len,reverse=True) if n.startswith(k+'_'));self.parts.append((k,n,m))
 def geometry(self,q):
  f=self.r.fk(q);out=[]
  for k,n,m in self.parts:v=m.copy();v.apply_transform(f[k]@np.linalg.inv(self.home[k]));out.append((n,v))
  return out

def main():
 scratch=Path(sys.argv[1]) if len(sys.argv)>1 else Path(tempfile.mkdtemp(prefix='bellybot_motion_',dir=str(ROOT.parent/'plane_work/entry_v2')));start=int(sys.argv[2]) if len(sys.argv)>2 else 0;bot=URDF(ROOT/'ros2/bellybot_dock_stack_description/urdf/bellybot.urdf');corner=Colored('corner_transfer');buffer=Colored('longitudinal_buffer');door=Colored('door_belt');case=collect(P/'models/bag_hard_32/meshes/visual.glb')
 shell=collect(P/'models/nb_hold_cutaway/meshes/visual.glb',skip=['bulkhead_0','end_protect_0','net_']);deck=collect(P/'models/entry_deck/meshes/visual.glb');loader=collect(P/'models/belt_loader/meshes/visual.glb');font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',17);fps=6;nframes=180
 def S(x):x=max(0,min(1,x));return x*x*(3-2*x)
 for i in range(start,nframes):
  t=i/fps;q={};B=np.eye(4)
  if t<14:
   if t<5:x=1.3;y=-3.29*(1-S(t/5));z=.183;a=PIt/2;pitch=0;dist=y+3.29;title='DOOR ENTRY / APPROACH'
   elif t<9:x=1.3;y=0;z=.183;a=PIt/2*(1-S((t-5)/4));pitch=0;dist=3.29;title='DOOR ENTRY / TURN ON RAISED DECK'
   else:
    x=1.3+3.6*S((t-9)/5);y=0;a=0;pitch=math.atan(.1)*min(1,max(0,(x-1.6)/1.32))*min(1,max(0,(4.72-x)/1.32));u=np.linspace(-.66,.66,265);z=float(np.max(np.clip((4.06-(x+u*math.cos(pitch)))*.1,0,.18)+u*math.sin(pitch)))+.003;dist=3.29+x-1.3;title='DOOR ENTRY / EXIT RAMP'
   B=tm.transformations.euler_matrix(0,pitch,a);B[:3,3]=[x,y,z]
   turn=PIt/2-a
   for side,sg in [('left',1),('right',-1)]:
    for k in range(5):q[f'{side}_wheel_{k}_joint']=(dist+sg*turn*.375)/.068
   parts=shell+deck+bot.geometry(q,B);bounds=[[-.05,-4.5,-1.45],[5.2,1,1.2]]
  else:
   u=t-14;lift=.198;angle=PIt/2;bridge=0;bl=.128;bx=1.3;by=0;ba=0;bs=.498;phase=0
   if u<4:lift=0;angle=0;bridge=-PIt/2;bl=0;by=-1.15*(1-S(u/4));ba=PIt/2;bs=.3;phase=u*.25;title='CORNER TRANSFER / RECEIVE LOW'
   elif u<7:lift=.198*S((u-4)/3);angle=0;bridge=-PIt/2;bl=.128*S((u-4)/3);ba=PIt/2;bs=.3+lift;title='CORNER TRANSFER / LIFT CLEAR'
   elif u<10:angle=PIt/2*S((u-7)/3);bridge=-PIt/2;ba=PIt/2-angle;bs=.498;title='CORNER TRANSFER / ROTATE 90 DEGREES'
   elif u<12:bridge=-PIt/2*(1-S((u-10)/2));title='CORNER TRANSFER / DEPLOY BRIDGE'
   else:bx=1.3+2.8*S((u-12)/4);phase=u*.25;title='CORNER TRANSFER / FEED BELLYBOT'
   cq=corner_q(lift,angle);bq=buffer_q(bl,bridge);dq={}
   for prefix,d in [('corner_belt',cq),('buffer_belt',bq),('door_belt_contact',dq)]:
    for end in ['rear','front']:d[prefix+'_'+end+'_joint']=phase/.011
   for k in range(7):bq[f'bridge_roller_{k}_joint']=phase/.005
   q={'tray_rear_roller_joint':phase/.01,'tray_front_roller_joint':phase/.01};B[:3,3]=[4.1,0,.003];bag=[];M=tm.transformations.euler_matrix(0,0,ba);M[:3,3]=[bx,by,bs+.162]
   for n,m in case:v=m.copy();v.apply_transform(M);bag.append((n,v))
   parts=shell+loader+door.geometry(dq)+corner.geometry(cq)+buffer.geometry(bq)+bot.geometry(q,B)+bag;bounds=[[.0,-2.8,-.05],[5.7,1,1.2]]
  f=scratch/f'{i:04}.png';im=render(parts,f,title,'Geometric motion illustration; time compressed',el=34,az=-65,size=(1280,800),bounds=bounds);draw=ImageDraw.Draw(im);draw.rectangle((0,740,1280,800),fill='#163f39');draw.text((30,760),'CAD KINEMATICS ONLY  |  Not a Gazebo physics run or a measured cycle time',font=font,fill='white');im.save(f)
  if i%30==0:print('Preview frame',i,flush=True)
 subprocess.run(['ffmpeg','-y','-loglevel','error','-framerate',str(fps),'-i',str(scratch/'%04d.png'),'-c:v','libx264','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(ROOT/'exports/Entry_and_Transfer_Kinematic_Preview.mp4')],check=True)
 print('Created 30-second labelled geometric preview',flush=True)
if __name__=='__main__':main()
