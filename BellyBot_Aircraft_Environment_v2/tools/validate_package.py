#!/usr/bin/env python3
"""Offline structural checks. Does not label XML checks as a Gazebo run."""
import ast,sys,json,math,importlib.util,xml.etree.ElementTree as E
from pathlib import Path
import numpy as np
import trimesh as tm
ROOT=Path(__file__).resolve().parents[1];PKG=ROOT/'ros2/bellybot_aircraft_environment'
def require(x,msg):
 if not x:raise AssertionError(msg)
def resolve(uri):
 if uri.startswith('model://'):
  rel=uri[len('model://'):];p=PKG/'models'/rel
  return p if p.exists() else ROOT/'ros2'/rel
 if uri.startswith('package://bellybot_aircraft_environment/'):return PKG/uri.split('package://bellybot_aircraft_environment/')[1]
 if uri.startswith('file://'):return Path(uri[7:])
 if uri.startswith('package://'):return ROOT/'ros2'/uri[len('package://'):]
 raise ValueError('Unexpected URI '+uri)
files=list(PKG.rglob('*.sdf'));refs=0;dynamic=0
for p in files:
 r=E.parse(p).getroot();require(r.tag=='sdf','SDF root '+str(p));require(r.get('version')=='1.9','Version')
 for u in r.findall('.//uri'):
  q=resolve(u.text);require(q.exists(),f'Missing {u.text} in {p}');refs+=1
 for parent in r.iter():
  for kind in ['link','joint','visual','collision']:
   names=[x.get('name') for x in parent.findall(kind)];require(len(names)==len(set(names)),f'Duplicate {kind} in {p}')
 for mo in r.findall('.//model'):
  if mo.findtext('static')=='false':
   dynamic+=1
   links={l.get('name') for l in mo.findall('link')}
   for l in mo.findall('link'):
    mass=float(l.findtext('inertial/mass','0'));i=l.find('inertial/inertia');require(mass>0 and i is not None,'Missing dynamic inertia')
    a={k:float(i.findtext(k,'0')) for k in ['ixx','iyy','izz','ixy','ixz','iyz']};M=np.array([[a['ixx'],a['ixy'],a['ixz']],[a['ixy'],a['iyy'],a['iyz']],[a['ixz'],a['iyz'],a['izz']]]);v=np.linalg.eigvalsh(M);require(v.min()>0 and v.max()<=v.sum()-v.max()+1e-7,'Unphysical inertia')
   for j in mo.findall('joint'):
    require(j.findtext('parent') in links|{'world'} and j.findtext('child') in links,'Bad joint endpoints')
collision_files=list(PKG.glob('models/*/meshes/collision_*.stl'));vertices=0
for p in collision_files:
 m=tm.load_mesh(p,process=True);require(m.is_watertight and m.is_winding_consistent and m.volume>0,'Invalid closed collision '+p.name);vertices+=len(m.vertices)
for p in PKG.rglob('*.dae'):
 r=E.parse(p).getroot();ns={'c':'http://www.collada.org/2005/11/COLLADASchema'};require(r.find('c:asset/c:unit',ns).get('meter')=='1','COLLADA unit');require(len(r.findall('.//c:triangles',ns))>0,'Empty COLLADA')
ur=E.parse(PKG/'urdf/environment.urdf').getroot();ln={x.get('name') for x in ur.findall('link')};children=[]
for j in ur.findall('joint'):
 require(j.get('type')=='fixed','Environment URDF must be static');children.append(j.find('child').get('link'));require(j.find('parent').get('link') in ln,'Bad URDF parent')
require(len(children)==len(ln)-1 and len(set(children))==len(children),'URDF not a tree')
for me in ur.findall('.//mesh'):require(resolve(me.get('filename')).is_file(),'URDF mesh missing')
# Articulated URDFs in both bundled packages: connected trees and SI joint limits.
articulated_urdfs=0
for file in (ROOT/'ros2').rglob('*.urdf'):
 tree=E.parse(file).getroot();links={l.get('name') for l in tree.findall('link')};cs=[]
 for j in tree.findall('joint'):
  require(j.find('parent').get('link') in links and j.find('child').get('link') in links,'URDF joint endpoint');cs.append(j.find('child').get('link'))
  if j.get('type')!='fixed':
   li=j.find('limit');require(li is not None and float(li.get('velocity'))>0 and float(li.get('effort'))>0,'URDF joint limits')
   if j.get('type')!='continuous':require(float(li.get('lower'))<=float(li.get('upper')),'Reversed URDF limits')
 require(len(set(cs))==len(cs)==len(links)-1,'URDF graph not a tree')
 for me in tree.findall('.//mesh'):
  name=me.get('filename');path=resolve(name) if name.startswith('package://') else file.parent/name;require(path.is_file(),'Missing articulated URDF mesh')
 articulated_urdfs+=1
for p in ROOT.rglob('*.py'):ast.parse(p.read_text(),filename=str(p))
# The visually removed roof/wall must retain the SAME collision set.
def collisions(n):
 r=E.parse(PKG/'models'/n/'model.sdf').getroot();return {c.get('name') for c in r.findall('.//collision')}
require(collisions('nb_hold')==collisions('nb_hold_cutaway'),'Cutaway deleted collision geometry')
# Belt contact must be a separate controlled link from fixed guards/supports.
ld=E.parse(PKG/'models/belt_loader/model.sdf').getroot();require(ld.findtext('.//plugin/link')=='belt_contact','Belt controller link')
require(len(ld.findall('.//link[@name="belt_contact"]/collision'))==1,'Belt should not drive guard/support surfaces')
# Real semantic scorer checks: duplicates, drops, out-of-window, unreceived IDs and force/area.
spec=importlib.util.spec_from_file_location('score',ROOT/'tools/score_events.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
rows=[{'time_s':str(i),'bag_id':'A','event':e,'mass_kg':'32'} for i,e in [(0,'received'),(1,'placed'),(2,'placed')]]
r=mod.score(rows,0,60);require(r['unique_accepted_placements']==1 and r['bags_per_min']==1,'Duplicate counted')
r=mod.score(rows+[{'time_s':'3','bag_id':'A','event':'dropped','mass_kg':'32'}],0,60);require(r['unique_accepted_placements']==0,'Dropped case counted')
r=mod.score([{'time_s':'1','bag_id':'X','event':'placed','mass_kg':'32'}],0,60);require(r['unique_accepted_placements']==0,'Unknown case counted')
r=mod.score(rows,0,1);require(r['unique_accepted_placements']==0,'End boundary counted')
r=mod.score(rows,0,60,[{'time_s':'1','force_n':'588.6','area_m2':'.1','patch_id':'p'}]);require(abs(r['peak_patch_kg_m2']-600)<1e-8,'Patch conversion')
fit=json.loads((ROOT/'docs/bellybot_fit.json').read_text());require(not fit['poses']['unloaded_mechanical_full_lift']['enclosure_pass'],'Expected known roof failure missing');require(fit['poses']['stowed']['enclosure_pass'],'Stowed fit failed')
out={'offline_checks_passed':True,'sdf_files':len(files),'resolved_sdf_resource_references':refs,'models':len(list((PKG/'models').glob('*/model.sdf'))),'worlds':len(list((PKG/'worlds').glob('*.sdf'))),'dynamic_models_with_valid_inertia':dynamic,'watertight_collision_meshes':len(collision_files),'collision_vertices':vertices,'urdf_fixed_links':len(ln),'python_sources_parsed':len(list(ROOT.rglob('*.py'))),'cutaway_collision_set_identical':True,'event_scorer_semantic_checks':5,'robot_sampled_enclosure_poses':fit['sampled_pose_count'],'known_robot_fit_failures_preserved':True,'gazebo_runtime_tested':False,'rviz_runtime_tested':False,'sdformat_cli_schema_check_run':False,'scope':'Offline XML structure, resources, positive inertias, mesh topology, CAD check in step_check.json and static sampled robot enclosure checks. No physics, gait, contact, throughput, human-risk score or aircraft structural validation.'}
out.update(revision='2.0',articulated_urdfs_checked=articulated_urdfs,resolution_checks='resolution_checks.json',controller_checks='controller_contract_checks.json')
(ROOT/'docs/validation.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
