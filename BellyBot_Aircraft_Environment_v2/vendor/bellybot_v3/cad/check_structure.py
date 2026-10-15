#!/usr/bin/env python3
"""Solid-level connectivity of corrected rigid groups, plus guide engagement.
This verifies geometric connections, not joint strength, welds, preload or FEA.
"""
import json,itertools,math
from model import *

def components(parts):
    nodes=[(p['name']+f'#{i}',s) for p in parts for i,s in enumerate(p['shape'].Solids())]
    adj=[set() for _ in nodes]
    for i,(_,a) in enumerate(nodes):
        ab=a.BoundingBox()
        for j in range(i+1,len(nodes)):
            b=nodes[j][1];bb=b.BoundingBox()
            if any(getattr(ab,c+'min')>getattr(bb,c+'max')+.005 or getattr(bb,c+'min')>getattr(ab,c+'max')+.005 for c in 'xyz'):continue
            if a.distance(b)<.005:adj[i].add(j);adj[j].add(i)
    groups=[];seen=set()
    for i in range(len(nodes)):
        if i in seen:continue
        todo=[i];seen.add(i);group=[]
        while todo:
            a=todo.pop();group.append(nodes[a][0])
            for b in adj[a]:
                if b not in seen:seen.add(b);todo.append(b)
        groups.append(group)
    return {'solid_count':len(nodes),'component_count':len(groups),'components':groups}

def main():
    build();report={'revision':P['revision'],'scope':'solid-level geometric connectivity and guide engagement, not strength validation','rigid_groups':{}}
    for n in ['yaw_carrier','extension_mid','tray_carriage','tray']:
        # Tray decorative stripe bodies are paint/index features, not load-bearing members.
        parts=[p for p in PARTS if p['link']==n and not p['name'].startswith('hazard_stripes')]
        r=components(parts);report['rigid_groups'][n]=r
        print(n,r['solid_count'],r['component_count'],flush=True)
        if r['component_count']!=1:print(r['components'],flush=True)
    # Both 54 mm shoes stay completely on the 830 mm dedicated guide.
    report['guide_engagement']={}
    for stroke in [0,125,250,375,500]:
        intervals=[(x+stroke-27,x+stroke+27) for x in [-280,-120]]
        assert all(-415<=a<=b<=415 for a,b in intervals)
        report['guide_engagement'][str(stroke)]={'shoe_intervals_mm':intervals,'min_end_margin_mm':min(min(a+415,415-b) for a,b in intervals)}
    report['bearing_mount']={'reference':'THK RU178X envelope','bore_mm':115,'outer_diameter_mm':240,'width_mm':28,'inner_bolt_circle_mm':139,'outer_bolt_circle_mm':217,'bolts_per_ring':12,'race_internals_simplified':True}
    (ROOT/'docs/structure_checks.json').write_text(json.dumps(report,indent=2)+'\n')
    assert all(r['component_count']==1 for r in report['rigid_groups'].values()),'Disconnected rigid members remain'
    print('STRUCTURAL CONNECTIVITY PASS',flush=True)
if __name__=='__main__':main()
