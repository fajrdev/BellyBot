#!/usr/bin/env python3
"""Render the delivered R2 geometry. Illustrations are not physics recordings."""
from pathlib import Path
import sys,math,json
import numpy as np,trimesh as tm
ROOT=Path(__file__).resolve().parents[1];P=ROOT/'ros2/bellybot_aircraft_environment';sys.path.insert(0,str(ROOT/'tools'))
from scene_tools import URDF,corner_q,buffer_q
from render_environment import collect,render

def posed(name,q):
 r=URDF(P/'urdf'/f'{name}.urdf');home=r.fk();now=r.fk(q);out=[]
 for n,m in collect(P/'models'/name/'meshes/visual.glb'):
  key=next(k for k in sorted(home,key=len,reverse=True) if n.startswith(k+'_'));m.apply_transform(now[key]@np.linalg.inv(home[key]));out.append((n,m))
 return out

def robot(x=4.1,y=0,z=.003,yaw=0,pitch=0,lift=0):
 r=URDF(ROOT/'ros2/bellybot_dock_stack_description/urdf/bellybot.urdf');T=tm.transformations.euler_matrix(0,pitch,yaw);T[:3,3]=[x,y,z];return r.geometry({f'lift_stage_{i}_joint':lift/3 for i in [1,2,3]},T)

def main():
 cut=collect(P/'models/nb_hold_cutaway/meshes/visual.glb',skip=['bulkhead_0','end_protect_0','net_']);deck=collect(P/'models/entry_deck/meshes/visual.glb');door=collect(P/'models/door_belt/meshes/visual.glb');loader=collect(P/'models/belt_loader/meshes/visual.glb')
 co=posed('corner_transfer',corner_q(.198,math.pi/2));buf=posed('longitudinal_buffer',buffer_q(.128,0));bag=collect(P/'models/bag_hard_32/meshes/visual.glb')
 for n,m in bag:m.apply_translation([1.3,0,.660])
 work=cut+door+loader+co+buf+robot()+bag
 render(work,ROOT/'docs/transfer_design.png','BELLYBOT / CONNECTED CORNER TRANSFER','Low door belt  /  lift + 90-degree rotation  /  folding bridge  /  matched-height buffer',el=35,az=-67,bounds=[[.0,-2.8,-.05],[5.7,1.0,1.2]])
 entryparts=cut+deck+robot(1.3,0,.183,math.pi/4)
 render(entryparts,ROOT/'docs/entry_design.png','BELLYBOT / RESOLVED DOOR ENTRY','180 mm temporary turn deck  /  robot unchanged  /  minimum checked turning gap 32.8 mm',el=32,az=-63,bounds=[[-.05,-4.5,-1.45],[5.0,1.0,1.25]])
 detail=door+co+buf+robot()+bag
 render(detail,ROOT/'docs/transfer_detail.png','TRANSFER / MECHANICAL LAYOUT','0.300 m door belt -> 0.498 m receiving tray  /  rigid bodies remain physically separate',el=27,az=-75,bounds=[[.55,-2.65,.0],[4.95,.6,.95]])
 # Stowed work view with entry fixtures removed.
 sc=tm.Scene()
 for n,m in work:sc.add_geometry(m,node_name=n)
 sc.export(ROOT/'exports/Resolved_Transfer_Assembly.glb')
 sc=tm.Scene()
 for n,m in entryparts:sc.add_geometry(m,node_name=n)
 sc.export(ROOT/'exports/Resolved_Entry_Assembly.glb')
 from PIL import Image,ImageDraw,ImageFont
 composite=Image.new('RGB',(1800,1300),'white')
 cases=[('01 / APPROACH',1.3,-2.6,.183,math.pi/2),('02 / CLEAR THE DOOR',1.3,0,.183,math.pi/2),('03 / TURN ON THE DECK',1.3,0,.183,math.pi/4),('04 / LEAVE ALONG THE HOLD',2.0,0,.183,0)]
 for i,(title,x,y,z,a) in enumerate(cases):
  img=render(cut+deck+robot(x,y,z,a),ROOT/'docs'/f'entry_stage_{i+1}.png',title,'Temporary entry fixture; no conveyor station in this mode',el=50,az=-62,size=(900,650),bounds=[[.2,-3.5,-.4],[4.5,1,1.2]])
  composite.paste(img,((i%2)*900,(i//2)*650))
 composite.save(ROOT/'docs/entry_sequence.png')
 print('Rendered R2 actual-mesh previews',flush=True)
if __name__=='__main__':main()
