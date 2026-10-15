#!/usr/bin/env python3
"""Exact-solid clearance checks against the proposed rigid batch receiver."""
import json
from model import *
from validate import overlap

def main():
    build();fixture=box(1300,550,555,1430,0,277.5);fb=fixture.BoundingBox();cases={}
    for reach in [0,.4,.8]:
        ts=fk(configuration(.1,0,reach,feed_height=.1));hits=[]
        for p in PARTS:
            if p['link']=='sample_bag':continue
            shape=located(p['shape'],ts[p['link']])
            if overlap(shape.BoundingBox(),fb):
                volume=shape.intersect(fixture).Volume()
                if volume>.25:hits.append([p['name'],volume])
        cases[str(reach)]=hits
    report={'fixture_front_m':.78,'fixture_top_m':.555,'fixture_width_m':.55,
       'nominal_belt_surface_m':.595,'cassette_underside_clearance_mm':15,
       'outboard_runner_lateral_clearance_mm':42,'exact_solid_intersections_by_reach_m':cases,
       'scope':'three fixed-height zero-yaw poses; no physical contact or bag-transfer result'}
    (ROOT/'docs/batch_fixture_checks.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));assert not any(cases.values())
if __name__=='__main__':main()
