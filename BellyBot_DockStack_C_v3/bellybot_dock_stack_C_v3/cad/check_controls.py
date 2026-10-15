#!/usr/bin/env python3
"""ROS-independent motion guards and supervisor sequencing regression checks."""
import ast,json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
SCRIPTS=ROOT/'ros2/bellybot_dock_stack_description/scripts'
sys.path.insert(0,str(SCRIPTS))
from control_core import plan,validate_pose,floor_pitch,tip_height,wheel_speeds,segment_duration

def main():
    checks=[]
    def passed(name):checks.append(name)
    home=[0.,0.,0.,.4,0.,0.]
    approach=[0.,0.,.84,.4,floor_pitch(),0.]
    inserted=[0.,0.,1.,.4,floor_pitch(),0.]
    assert abs(tip_height(0,floor_pitch())-.002)<1e-12;passed('2 mm floor nose')
    route=plan(home,approach)
    assert route[0][2]>=.84 and route[0][4]==0 and route[-1]==approach;passed('extend before pitch')
    assert plan(approach,inserted)==[inserted];passed('floor insertion preserves pitch')
    assert plan(inserted,approach)==[approach];passed('floor withdrawal preserves pitch')
    route=plan(inserted,home)
    assert route[0][4]==0 and route[0][2]==1.;passed('level before retraction')
    route=plan(inserted,[.1,math.radians(65),.8,.15,0.,.1])
    prev=inserted
    for q in route:
        if q[1]!=prev[1]:assert q[2]==0 and q[0]>=.08 and q[4]==0 and q[3]<=.15
        prev=q
    passed('retract and raise before yaw')
    bad={
      'pitch over chassis':[0,0,.5,.4,floor_pitch(),0],
      'nose below floor':[0,0,1,.4,math.radians(32),0],
      'loaded roof exceeded':[.6,0,0,.4,0,0],
      'yaw limit exceeded':[.1,2,0,.4,0,0],
      'extension limit exceeded':[0,0,1.1,.4,0,0],
      'NaN command':[0,0,float('nan'),.4,0,0],
      'yaw with mast down':[0,.5,0,.15,0,0],
      'yaw with infeed extended':[.1,.5,0,.4,0,0]}
    for name,q in bad.items():
        try:validate_pose(q)
        except ValueError:passed('reject '+name)
        else:raise AssertionError(name)
    l,r=wheel_speeds(.136,0);assert abs(l-2)<1e-12 and l==r
    l,r=wheel_speeds(0,.2);assert l<0<r and abs(l+r)<1e-12;passed('wheel rolling and differential turning')
    assert segment_duration(home,[0,0,.8,.4,0,0])>1.3;passed('acceleration included in extension time')
    for path in SCRIPTS.glob('*.py'):ast.parse(path.read_text(),filename=str(path))
    passed('all ROS scripts parse')
    report={'passed':len(checks),'checks':checks,'ros_runtime_tested':False}
    (ROOT/'docs/control_checks.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
