#!/usr/bin/env python3
"""Pure geometric helper, not a certified controller or complete collision checker.
Feed WORLD-frame mesh vertices / envelope samples at every trajectory sample.
This helper is for the constant-section work zone, excluding the doorway.
"""
import math,json
from pathlib import Path
CFG=json.loads((Path(__file__).resolve().parents[1]/'config/environment.json').read_text())
def half_width(z):
    p=CFG['forward']['profile_yz']
    for (y0,z0),(y1,z1) in zip(p,p[1:]):
        if z<=z1:return y0+(y1-y0)*max(0,z-z0)/(z1-z0)
    return p[-1][0]
def loaded_lift_cap(bag_height,local_roof=1.12,base_z=.003,margin=.1):
    if not all(math.isfinite(v) for v in [bag_height,local_roof,base_z,margin]) or bag_height<0:raise ValueError('Finite dimensions and nonnegative bag height required')
    return max(0,min(.6,local_roof-margin-base_z-.495-bag_height-.005))
def assess_points(points,wall_margin=.025,roof_margin=.1):
    h=CFG['forward']['ceiling_height'];length=CFG['forward']['length'];n=0;fail=[]
    for x,y,z in points:
        n+=1
        if not all(math.isfinite(v) for v in [x,y,z]):raise ValueError('Nonfinite point')
        if x<.04 or x>length-.04:fail.append('bulkhead')
        if z<-.0005:fail.append('floor')
        if z>h-roof_margin:fail.append('roof_margin')
        if abs(y)>half_width(z)-wall_margin:fail.append('wall_margin')
    if not n:raise ValueError('No geometry supplied')
    return {'ok':not fail,'violations':sorted(set(fail)),'sampled_points':n}
