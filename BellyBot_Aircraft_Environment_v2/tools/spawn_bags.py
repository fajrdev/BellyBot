#!/usr/bin/env python3
"""Seeded FREE rigid bags on the external loader. Run after Gazebo starts.
Interval is WALL CLOCK, for commissioning only; benchmark scoring uses sim time.
Dry-run is the default; --execute is required to spawn entities.
"""
import argparse,json,math,random,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];PKG=ROOT/'ros2/bellybot_aircraft_environment';cfg=json.loads((ROOT/'config/environment.json').read_text())
p=argparse.ArgumentParser();p.add_argument('--world',default='loading_task');p.add_argument('--count',type=int,default=1);p.add_argument('--interval',type=float,default=5);p.add_argument('--seed',type=int,default=42);p.add_argument('--prefix',default='incoming');p.add_argument('--execute',action='store_true');a=p.parse_args()
if not 1<=a.count<=120 or a.interval<1:p.error('count 1..120, interval >=1 s')
if not all(c.isalnum() or c=='_' for c in a.prefix+a.world):p.error('Names must be alphanumeric/underscore')
rng=random.Random(a.seed);bags=[('bag_hard_20',.26),('bag_hard_26',.30),('bag_hard_32',.32),('bag_duffel_23',.32),('bag_fragile_20',.38)]
l=cfg['loader'];dy=l['high_y']-l['low_y'];dz=l['high_surface_z']-l['low_surface_z'];ang=math.atan2(dz,dy)
# quaternion for Rz(pi/2) Ry(-ang)
cy=math.cos(math.pi/4);sy=math.sin(math.pi/4);cp=math.cos(-ang/2);sp=math.sin(-ang/2);quat=(-sy*sp,cy*sp,sy*cp,cy*cp)
for i in range(a.count):
 name,h=rng.choice(bags);clear=h/2+.008;y=l['low_y']+.60-clear*math.sin(ang);z=l['low_surface_z']+.60*dz/dy+clear*math.cos(ang);x=cfg['forward']['door_center_x'];q=quat
 req=f'sdf_filename: {json.dumps(str((PKG/"models"/name/"model.sdf").resolve()))} name: "{a.prefix}_{i:03d}" allow_renaming: false pose: {{position: {{x: {x} y: {y} z: {z}}} orientation: {{x: {q[0]} y: {q[1]} z: {q[2]} w: {q[3]}}}}}'
 if a.execute:
  result=subprocess.run(['gz','service','-s',f'/world/{a.world}/create','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','5000','--req',req],capture_output=True,text=True,check=True)
  if 'data: true' not in result.stdout:raise RuntimeError('Spawn was not confirmed: '+result.stdout+result.stderr)
  print('Spawned',a.prefix+f'_{i:03d}',name,flush=True)
  if i+1<a.count:time.sleep(a.interval)
 else:print(req)
