#!/usr/bin/env python3
from pathlib import Path
import sys,json,xml.etree.ElementTree as E
import numpy as np,trimesh as tm
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'))
from scene_tools import URDF,vec,origin
r=URDF(ROOT/'ros2/bellybot_dock_stack_description/urdf/bellybot_portable.urdf');links=[]
for l in r.root.findall('link'):
 n=l.get('name');m=[p for ln,_,p in r.parts if ln==n];points=tm.convex.convex_hull(np.concatenate([p.vertices for p in m])).vertices if m else np.zeros((1,3));inert=l.find('inertial');mass=float(inert.find('mass').get('value')) if inert is not None else 0
 links.append({'name':n,'vertices':points.round(9).tolist(),'mass':mass,'com':vec(inert.find('origin') if inert is not None else None,'xyz','0 0 0').tolist()})
joints=[]
for j in r.joints:joints.append({'name':j.get('name'),'parent':j.find('parent').get('link'),'child':j.find('child').get('link'),'type':j.get('type'),'origin':origin(j.find('origin')).tolist(),'axis':vec(j.find('axis'),'xyz','1 0 0').tolist()})
out={'root':r.base,'links':links,'joints':joints,'scope':'Per-link convex hulls of original v3 visual meshes plus assigned inertial COMs. SI; no enclosing aircraft geometry was enlarged.'}
(ROOT/'ros2/bellybot_aircraft_environment/config/robot_geometry.json').write_text(json.dumps(out,separators=(',',':')))
print('Guard cache',sum(len(l['vertices']) for l in links),'vertices',sum(l['mass'] for l in links),'kg')
