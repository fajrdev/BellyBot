#!/usr/bin/env python3
"""One connected URDF for live work-mode RViz; all operational joints retained."""
import copy,xml.etree.ElementTree as E
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];P=ROOT/'ros2/bellybot_aircraft_environment'
r=E.parse(P/'urdf/environment.urdf').getroot();r.set('name','BellyBot_aircraft_R2')
robot=E.parse(ROOT/'ros2/bellybot_dock_stack_description/urdf/bellybot.urdf').getroot()
for x in robot:
 if x.tag in ('link','joint'):r.append(copy.deepcopy(x))
j=E.SubElement(r,'joint',name='robot_work_datum',type='fixed');E.SubElement(j,'parent',link='cargo_floor');E.SubElement(j,'child',link='base_link');E.SubElement(j,'origin',xyz='4.1 0 .003',rpy='0 0 0')
for name in ['door_belt','corner_transfer','longitudinal_buffer']:
 for x in E.parse(P/'urdf'/f'{name}.urdf').getroot():
  if x.tag=='link' and x.get('name')=='fixture_world':continue
  x=copy.deepcopy(x)
  if x.tag=='joint' and x.find('parent').get('link')=='fixture_world':x.find('parent').set('link','cargo_floor')
  r.append(x)
E.indent(r);E.ElementTree(r).write(P/'urdf/aircraft_system.urdf',encoding='utf-8',xml_declaration=True)
