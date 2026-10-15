#!/usr/bin/env python3
"""Export CAD assemblies, STEP, STL, GLB and explicit serial-joint URDF variants."""
import copy,csv,json,math
import xml.etree.ElementTree as ET
import cadquery as cq
import numpy as np
import trimesh
from model import *

def fmt(v):return ' '.join(f'{float(x):.10g}' for x in v)
def xmlwrite(root,path):
    path.parent.mkdir(parents=True,exist_ok=True);ET.indent(root)
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)

def inertia(name):
    shapes=[p['shape'] for p in PARTS if p['link']==name];s=compound(shapes)
    m=LINKS[name]['mass'];V=s.Volume();center=cq.Shape.centerOfMass(s).toTuple()
    I=np.array(cq.Shape.matrixOfInertia(s))*m/V*1e-6
    if np.min(np.linalg.eigvalsh(I))<=0:raise ValueError('Inertia '+name)
    return m,np.array(center)/1000,I

def urdf(group='robot',portable=False,display=False):
    robot=ET.Element('robot',name='bellybot_dock_stack_C')
    robot.append(ET.Comment('Explicit SI joints by default; independent nested stages. CAD-derived inertia uses assigned link mass, not hardware measurement.'))
    selected=[n for n,d in LINKS.items() if display or d['group']==group]
    for name in selected:
        d=LINKS[name];el=ET.SubElement(robot,'link',name=name);mass,com,I=inertia(name)
        ine=ET.SubElement(el,'inertial');ET.SubElement(ine,'origin',xyz=fmt(com),rpy='0 0 0')
        ET.SubElement(ine,'mass',value=str(mass))
        ET.SubElement(ine,'inertia',ixx=str(I[0,0]),iyy=str(I[1,1]),izz=str(I[2,2]),ixy=str(I[0,1]),ixz=str(I[0,2]),iyz=str(I[1,2]))
        for p in [p for p in PARTS if p['link']==name]:
            vis=ET.SubElement(el,'visual',name=p['name']);g=ET.SubElement(vis,'geometry')
            prefix='../' if portable else f'package://{PACKAGE}/'
            ET.SubElement(g,'mesh',filename=prefix+'meshes/'+p['name']+'.stl',scale='0.001 0.001 0.001')
            mat=ET.SubElement(vis,'material',name=p['color']);ET.SubElement(mat,'color',rgba=fmt((*COLORS[p['color']],1)))
        for i,c in enumerate(d['collisions']):
            col=ET.SubElement(el,'collision',name=f'{name}_collision_{i}')
            ET.SubElement(col,'origin',xyz=fmt(np.array(c['xyz'])/1000),rpy=fmt(c['rpy']))
            g=ET.SubElement(col,'geometry')
            if c['kind']=='box':ET.SubElement(g,'box',size=fmt(np.array(c['size'])/1000))
            else:ET.SubElement(g,'cylinder',radius=str(c['radius']/1000),length=str(c['length']/1000))
        if d['parent']:
            joint=ET.SubElement(robot,'joint',name=d['joint'],type=d['kind'])
            ET.SubElement(joint,'parent',link=d['parent']);ET.SubElement(joint,'child',link=name)
            ET.SubElement(joint,'origin',xyz=fmt(np.array(d['origin'])/1000),rpy='0 0 0')
            if d['kind']!='fixed':
                ET.SubElement(joint,'axis',xyz=fmt(d['axis']))
                limit={'velocity':str(d['velocity']),'effort':str(d['effort'])}
                if d['kind']!='continuous':limit.update(lower=str(d['lower']),upper=str(d['upper']))
                ET.SubElement(joint,'limit',**limit)
                ET.SubElement(joint,'dynamics',damping='.02' if d['kind']=='continuous' else '4',friction='0')
    if display:
        j=ET.SubElement(robot,'joint',name='infeed_display_mount',type='fixed')
        ET.SubElement(j,'parent',link='base_link');ET.SubElement(j,'child',link='infeed_base')
        ET.SubElement(j,'origin',xyz='0 0 0',rpy='0 0 0')
    return robot

def bag_urdf():
    r=ET.Element('robot',name='test_bag_32kg');l=ET.SubElement(r,'link',name='bag')
    dims=np.array([P['bag_length'],P['bag_width'],P['bag_height']])/1000;m=P['bag_mass'];a,b,c=dims
    ine=ET.SubElement(l,'inertial');ET.SubElement(ine,'origin',xyz=fmt((0,0,c/2)),rpy='0 0 0')
    ET.SubElement(ine,'mass',value=str(m))
    ET.SubElement(ine,'inertia',ixx=str(m*(b*b+c*c)/12),iyy=str(m*(a*a+c*c)/12),izz=str(m*(a*a+b*b)/12),ixy='0',ixz='0',iyz='0')
    for p in [p for p in PARTS if p['link']=='sample_bag']:
        v=ET.SubElement(l,'visual',name=p['name']);g=ET.SubElement(v,'geometry')
        ET.SubElement(g,'mesh',filename=f'package://{PACKAGE}/meshes/'+p['name']+'.stl',scale='0.001 0.001 0.001')
        mat=ET.SubElement(v,'material',name=p['color']);ET.SubElement(mat,'color',rgba=fmt((*COLORS[p['color']],1)))
    co=ET.SubElement(l,'collision');ET.SubElement(co,'origin',xyz=fmt((0,0,c/2)),rpy='0 0 0')
    g=ET.SubElement(co,'geometry');ET.SubElement(g,'box',size=fmt(dims))
    return r

def export_glb(q,path,bag=False):
    scene=trimesh.Scene();ts=fk(q)
    for p in PARTS:
        if p['link']=='sample_bag' and not bag:continue
        vertices,faces=p['shape'].tessellate(.6,.25)
        v=np.array([vv.toTuple() for vv in vertices]);t=ts[p['link']]
        v=(v@t[:3,:3].T+t[:3,3])/1000
        mesh=trimesh.Trimesh(v,np.array(faces),process=False)
        mesh.visual.face_colors=np.array([*COLORS[p['color']],1])*255
        scene.add_geometry(mesh,node_name=p['name'])
    path.write_bytes(scene.export(file_type='glb'))

def main():
    build()
    for d in [ROOT/'exports',ROOT/'docs',PKG/'meshes',PKG/'urdf']:d.mkdir(parents=True,exist_ok=True)
    print('Exporting colored STEP assemblies',flush=True)
    assembly(POSES['home']).export(str(ROOT/'exports/BellyBot_DockStack_home.step'))
    assembly(POSES['placement'],bag=True).export(str(ROOT/'exports/BellyBot_DockStack_placement.step'))
    assembly(POSES['home'],feed=False).export(str(ROOT/'exports/BellyBot_DockStack_robot.step'))
    assembly(POSES['floor_insert'],feed=False).export(str(ROOT/'exports/BellyBot_DockStack_floor_access.step'))
    detail=cq.Assembly(name='Corrected_yaw_mount_and_retained_slides');ts=fk(POSES['home'])
    for p in PARTS:
        if p['link'] in ['lift_stage_3','yaw_carrier','extension_mid','tray_carriage']:
            detail.add(located(p['shape'],ts[p['link']]),name=p['name'],color=cq.Color(*COLORS[p['color']]))
    detail.export(str(ROOT/'exports/BellyBot_DockStack_yaw_mechanism.step'))
    for name in LINKS:
        cq.exporters.export(compound([p['shape'] for p in PARTS if p['link']==name]),str(ROOT/'exports'/(name+'.step')))
    print('Exporting articulation meshes',flush=True)
    for p in PARTS:p['shape'].exportStl(str(PKG/'meshes'/(p['name']+'.stl')),tolerance=.25,angularTolerance=.16,relative=False)
    for name,kwargs in [('bellybot.urdf',{}),('bellybot_portable.urdf',{'portable':True}),
      ('display.urdf',{'display':True}),
      ('display_portable.urdf',{'display':True,'portable':True}),('infeed.urdf',{'group':'infeed'})]:
        xmlwrite(urdf(**kwargs),PKG/'urdf'/name)
    xmlwrite(bag_urdf(),PKG/'urdf/sample_bag.urdf')
    export_glb(POSES['placement'],ROOT/'exports/BellyBot_DockStack_placement.glb',bag=True)
    export_glb(POSES['home'],ROOT/'exports/BellyBot_DockStack_home.glb')
    with (ROOT/'docs/parts.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['part','link','solid_count','volume_mm3','color'])
        for p in PARTS:w.writerow([p['name'],p['link'],len(p['shape'].Solids()),round(p['shape'].Volume(),3),p['color']])
    (ROOT/'docs/link_definitions.json').write_text(json.dumps(LINKS,indent=2))
    (ROOT/'docs/poses.json').write_text(json.dumps(POSES,indent=2))
    print(f'Exported {len(PARTS)} groups / {len(LINKS)} links.',flush=True)

if __name__=='__main__':main()
