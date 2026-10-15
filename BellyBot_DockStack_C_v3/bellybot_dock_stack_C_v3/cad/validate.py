#!/usr/bin/env python3
"""Geometry and independent URDF checks. Does not run ROS/Gazebo."""
import itertools,json,math,struct,sys
import xml.etree.ElementTree as ET
import numpy as np
import cadquery as cq
from yourdfpy import URDF
from model import *
COLLISION_CACHE={}

def overlap(a,b):
    return all(min(getattr(a,c+'max'),getattr(b,c+'max'))-max(getattr(a,c+'min'),getattr(b,c+'min'))>.02 for c in 'xyz')

def intersections(q):
    ts=fk(q);parts=[]
    for p in PARTS:
        if p['link']=='sample_bag':continue
        shape=located(p['shape'],ts[p['link']]);parts.append((p,shape,shape.BoundingBox()))
    hits=[]
    for i,(p,a,ba) in enumerate(parts):
        for other,b,bb in parts[i+1:]:
            if p['link']==other['link'] or not overlap(ba,bb):continue
            relative=np.linalg.inv(ts[p['link']])@ts[other['link']]
            key=(p['name'],other['name'],tuple(np.round(relative.ravel(),8)))
            if key not in COLLISION_CACHE:
                COLLISION_CACHE[key]=p['shape'].intersect(located(other['shape'],relative)).Volume()
            volume=COLLISION_CACHE[key]
            if volume>.25:hits.append([p['name'],other['name'],round(volume,3)])
    return hits

def extent(q,bag=False,feed=False):
    ts=fk(q);mins=[];maxs=[]
    for p in PARTS:
        if p['link']=='sample_bag' and not bag:continue
        if p['link'] in LINKS and LINKS[p['link']]['group']=='infeed' and not feed:continue
        b=located(p['shape'],ts[p['link']]).BoundingBox()
        mins.append([b.xmin,b.ymin,b.zmin]);maxs.append([b.xmax,b.ymax,b.zmax])
    return [np.min(mins,axis=0).tolist(),np.max(maxs,axis=0).tolist()]

def check_urdfs():
    report={};rng=np.random.default_rng(9618)
    paths=['bellybot.urdf','bellybot_portable.urdf','display.urdf','display_portable.urdf','infeed.urdf']
    for filename in paths:
        path=PKG/'urdf'/filename;root=ET.parse(path).getroot()
        names={l.get('name') for l in root.findall('link')};joints=root.findall('joint')
        children=[j.find('child').get('link') for j in joints]
        assert len(joints)==len(names)-1 and len(children)==len(set(children))
        for m in root.findall('.//mesh'):
            f=m.get('filename').replace(f'package://{PACKAGE}/','').removeprefix('../')
            assert (PKG/f).is_file();assert m.get('scale')=='0.001 0.001 0.001'
            data=(PKG/f).read_bytes();n=struct.unpack_from('<I',data,80)[0];assert len(data)==84+50*n
        for l in root.findall('link'):
            i=l.find('inertial/inertia').attrib
            mat=np.array([[float(i['ixx']),float(i['ixy']),float(i['ixz'])],
                          [float(i['ixy']),float(i['iyy']),float(i['iyz'])],
                          [float(i['ixz']),float(i['iyz']),float(i['izz'])]])
            vals=np.linalg.eigvalsh(mat);assert vals.min()>0 and vals.max()<=vals.sum()-vals.max()+1e-8
        robot=URDF.load(str(path),load_meshes=False);error=0
        for _ in range(40):
            q={d['joint']:rng.uniform(d['lower'],d['upper']) if d['kind']!='continuous' else rng.uniform(-7,7)
               for d in LINKS.values() if d['kind']!='fixed'}
            if 'mimic' in filename:
                q['lift_stage_2_joint']=q['lift_stage_3_joint']=q['lift_stage_1_joint']
                q['extension_tip_joint']=q['extension_mid_joint']
            robot.update_cfg({k:v for k,v in q.items() if k in robot.actuated_joint_names})
            ts=fk(q)
            for n in names:
                target=ts[n].copy();target[:3,3]/=1000
                # Infeed-only root and mobile root both use the same initial axes.
                error=max(error,float(np.max(np.abs(robot.get_transform(n)-target))))
        assert error<1e-9,(filename,error)
        report[filename]={'links':len(names),'joints':len(joints),'continuous_joints':sum(j.get('type')=='continuous' for j in joints),
                         'independent_fk_poses':40,'max_matrix_error':error}
    return report

def main():
    from build import inertia
    from motion import sample,DURATION
    build();report={'revision':P['revision'],'cad_valid':all(p['shape'].isValid() for p in PARTS),
       'part_groups':len(PARTS),'solid_count':sum(len(p['shape'].Solids()) for p in PARTS),
       'gazebo_runtime_tested':False,'rviz_runtime_tested':False}
    step=cq.importers.importStep(str(ROOT/'exports/BellyBot_DockStack_home.step'))
    report['step_reimport']={'valid':step.val().isValid(),'solids':len(step.solids().vals())}
    report['urdf']=check_urdfs()
    poses=dict(POSES)
    poses.update({f'yaw_{a}':configuration(.1,math.radians(a),0,infeed=.15) for a in [-95,-45,45,95]})
    for i in [1,2,3]:poses[f'independent_lift_{i}']=dict(POSES['home'],**{f'lift_stage_{i}_joint':.2})
    for n in ['extension_mid_joint','extension_tip_joint']:poses['independent_'+n]=dict(POSES['home'],**{n:.5})
    poses['slide_mid']=configuration(.1,0,.46)
    poses['floor_pitch_half']=configuration(extension=.84,pitch=floor_pitch()/2)
    poses['infeed_lift_max']=configuration(.25,0,0,feed_height=.25)
    for a in [-95,-65,-30,30,65,95]:
        poses[f'low_yaw_{a}']=configuration(.08,math.radians(a),.8,infeed=.15,feed_height=.25)
    for a,b in [(0,.25),(.25,0),(.125,.375),(.375,.125)]:
        poses[f'independent_slide_{a}_{b}']=dict(POSES['home'],extension_mid_joint=a,extension_tip_joint=b)
    report['solid_intersections']={}
    for n,q in poses.items():
        print('Solid pose',n,flush=True);hits=intersections(q);report['solid_intersections'][n]=hits
        if hits:print(json.dumps(hits),flush=True)
    report['envelopes_mm']={n:extent(q,bag=(n in ['placement','floor_insert','batch_reach'])) for n,q in POSES.items()}
    # Exact tip transform, using the underside of the 3 mm replaceable nose.
    report['floor_access']={}
    for n in ['floor_approach','floor_insert']:
        t=fk(POSES[n])['tray'];pos=t@np.array([920,0,-33.5,1.]);report['floor_access'][n]={'tip_mm':pos[:3].tolist(),'pitch_deg':math.degrees(POSES[n]['tray_pitch_joint'])}
        assert abs(pos[2]-2)<1e-6
    # Independent endpoint pose and analytic collision primitive checks along demonstration.
    # Exact solid tests are named above; path sampling checks floor/height/joint envelopes.
    min_tip=1e9;max_height=0;maxspeed={}
    corners={}
    for n in LINKS:
        b=compound([p['shape'] for p in PARTS if p['link']==n]).BoundingBox()
        corners[n]=np.array(list(itertools.product([b.xmin,b.xmax],[b.ymin,b.ymax],[b.zmin,b.zmax])))
    for t in np.linspace(0,DURATION,int(DURATION*60)+1):
        q,dq,_=sample(t);ts=fk(q);tip=ts['tray']@np.array([920,0,-33.5,1]);min_tip=min(min_tip,tip[2])
        for n,d in LINKS.items():
            if d['kind']=='fixed':continue
            v=q.get(d['joint'],0);assert d['kind']=='continuous' or d['lower']-1e-8<=v<=d['upper']+1e-8
            assert abs(dq.get(d['joint'],0))<=d['velocity']+1e-7
            maxspeed[d['joint']]=max(maxspeed.get(d['joint'],0),abs(dq.get(d['joint'],0)))
        for n,vertices in corners.items():
            if LINKS[n]['group']=='robot':max_height=max(max_height,(vertices@ts[n][:3,:3].T+ts[n][:3,3])[:,2].max())
    assert min_tip>=1.999 and max_height<1200
    report['demo_envelope_sampling']={'samples':int(DURATION*60)+1,'min_nose_mm':min_tip,'max_robot_height_mm':float(max_height),'max_joint_speeds':maxspeed,'scope':'unloaded joint/height/floor envelopes; not a continuous solid-collision proof'}
    # Load screen now uses every assigned link COM and an explicit four-support approximation.
    robot=[n for n,d in LINKS.items() if d['group']=='robot'];props={n:inertia(n) for n in robot}
    loads=[];A=P['track_contact_length']*P['track_width']/1e6/2;ax=P['track_contact_length']/4000;ay=P['track_centres']/2000
    for n in ['home','placement','floor_insert','batch_reach','full_reach']:
        q=POSES[n];ts=fk(q);M=P['bag_mass'];moment=M*(ts['sample_bag']@np.array([0,0,P['bag_height']/2,1]))[:3]/1000
        for linkn,(m,c,I) in props.items():
            pos=ts[linkn]@np.r_[c*1000,1];M+=m;moment+=m*pos[:3]/1000
        com=moment/M;reactions=[M/4*(1+sx*com[0]/ax+sy*com[1]/ay) for sx,sy in itertools.product([-1,1],repeat=2)]
        loads.append({'pose':n,'loaded_mass_kg':M,'com_m':com.tolist(),'four_patch_reaction_kg_equivalent':reactions,
          'factored_max_patch_kg_m2':max(reactions)/A*1.5,'minimum_reaction_kg_equivalent':min(reactions),
          'longitudinal_CoM_margin_m':P['track_contact_length']/2000-abs(com[0]),'lateral_CoM_margin_m':P['base_width']/2000-abs(com[1])})
    for case in loads:
        case['within_screen_limit']=bool(case['factored_max_patch_kg_m2']<=P['floor_limit_kg_m2'] and case['minimum_reaction_kg_equivalent']>=0)
    report['floor_pressure_screen']={'cases':loads,'limit_kg_m2':P['floor_limit_kg_m2'],
      'all_named_cases_within_screen_limit':all(c['within_screen_limit'] for c in loads),
      'assumptions':'four equal-compliance half-track patches, uniform pressure per patch, assigned masses, centered rigid bag. Negative calculated reaction invalidates the assumed four-contact load sharing. Not aircraft floor compliance.', 'compliance_proven':False}
    report['mechanism']={'lift_stage_overlap_mm':P['column_length']-10-P['lift_stage_stroke'],
      'extension_fixed_mid_overlap_mm':(870+830)/2-500,'extension_mid_carriage_overlap_mm':(830+880)/2-500,
      'cassette_pitch_deg':[0,32],'extension_total_mm':1000,'infeed_height_travel_mm':250}
    sdf=ET.parse(PKG/'gazebo/bellybot_dock_stack.sdf').getroot().find('model')
    links={l.get('name') for l in sdf.findall('link')};joints={j.get('name') for j in sdf.findall('joint')}
    for p in sdf.findall('plugin'):
        for j in p.findall('joint_name'):assert j.text in joints
        for l in p.findall('.//link'):assert l.text in links
    for u in sdf.findall('.//mesh/uri'):assert (PKG/u.text.replace(f'model://{PACKAGE}/','')).is_file()
    report['gazebo_xml']={'references_valid':True,'sdformat_schema_tested':False}
    (ROOT/'docs/validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({'floor_access':report['floor_access'],'loads':loads},indent=2),flush=True)
    assert not any(report['solid_intersections'].values()),'Resolve cross-link solid interference'
    print('OFFLINE VALIDATION PASS',flush=True)
if __name__=='__main__':main()
