#!/usr/bin/env python3
"""Compose a NEW inspection world with existing BellyBot SDF; never edits inputs.
Meshes are resolved to absolute file URIs. Re-run after moving the robot package.
This places the robot inside the hold; it is NOT an entry-path demonstration.
"""
import argparse,copy,json,xml.etree.ElementTree as E
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];PKG=ROOT/'ros2/bellybot_aircraft_environment'
p=argparse.ArgumentParser();p.add_argument('--bellybot',type=Path,required=True,help='Extracted v3 package root');p.add_argument('--output',type=Path,required=True);p.add_argument('--world',default='robot_clearance');p.add_argument('--include-infeed',action='store_true');p.add_argument('--force',action='store_true');a=p.parse_args()
if a.output.exists() and not a.force:p.error('Output exists. Choose another output or pass --force.')
bp=a.bellybot.resolve()/'ros2/bellybot_dock_stack_description'
if not bp.is_dir():p.error('Expected v3 root containing ros2/bellybot_dock_stack_description.')
source=PKG/'worlds'/(a.world+'.sdf')
root=E.parse(source).getroot();world=root.find('world')
if a.world=='manual_reference':p.error('Use a robot-only world. Manual workers occupy the work zone.')
if world.find("model[@name='bellybot_dock_stack']") is not None:p.error('This revised world already includes BellyBot; use aircraft.launch.py.')
spawn=json.loads((ROOT/'config/environment.json').read_text())['robot_spawn']
for fn in ['bellybot_dock_stack.sdf']+(['infeed.sdf'] if a.include_infeed else []):
 model=copy.deepcopy(E.parse(bp/'gazebo'/fn).getroot().find('model'))
 pose=model.find('pose')
 if pose is None:pose=E.SubElement(model,'pose')
 pose.text=f"{spawn['x']} {spawn['y']} {spawn['z']} 0 0 {spawn['yaw']}"
 for u in model.findall('.//mesh/uri'):
  prefix='model://bellybot_dock_stack_description/'
  if u.text.startswith(prefix):
   mesh=bp/u.text[len(prefix):]
   if not mesh.is_file():raise FileNotFoundError(mesh)
   u.text=mesh.as_uri()
 world.append(model)
a.output.parent.mkdir(parents=True,exist_ok=True);E.indent(root,space='  ');E.ElementTree(root).write(a.output,encoding='utf-8',xml_declaration=True)
print('Created',a.output.resolve());print('Resource path:',PKG/'models');print('Start PAUSED; use operational envelope config before commanding joints. Robot SDF and controls are unchanged.')
