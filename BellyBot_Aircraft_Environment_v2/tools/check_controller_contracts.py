#!/usr/bin/env python3
"""Semantic rejection/interlock checks. Synthetic feedback is NOT a physics test."""
from pathlib import Path
import sys,math,json
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'ros2/bellybot_aircraft_environment/scripts'))
from transfer_core import Transfer
from aircraft_guard import Guard,lift_cap
P=math.pi;checks=[]
def record(label,condition):assert condition,label;checks.append(label)
t=Transfer()
try:t.start(0,[.20,0,0,0]);ok=False
except ValueError:ok=True
record('Reject receiving lift above matched conveyor stroke',ok)
t.start(0,[0,0,0,0]);q={'corner_lift':0,'corner_angle':0,'buffer_lift':0,'bridge_angle':0};o=t.step(.1,q,None)
record('Initial horizontal bridge folds before moving other joints',o['bridge_angle']==-P/2 and o['corner_angle']==0 and t.state=='PARK')
t.state='ROTATE';t.started=0;o=t.step(.1,{'corner_lift':.198,'corner_angle':.2,'buffer_lift':.128,'bridge_angle':0},{'x':1.3,'y':0,'yaw':P/2,'speed':0,'z':.658})
record('Reject rotation with unfolded bridge',t.state=='FAULT' and all(o[k]==0 for k in ['corner','buffer','tray']))
t=Transfer();t.state='ROTATE';t.step(.1,{'corner_lift':.01,'corner_angle':0,'buffer_lift':0,'bridge_angle':-P/2},{'x':1.3,'y':0,'yaw':P/2,'speed':0,'z':.470});record('Reject rotation below inlet clearance height',t.state=='FAULT' and 'too low' in t.error)
t=Transfer();t.state='TRANSFER';t.step(.1,{'corner_lift':.198,'corner_angle':P/2,'buffer_lift':.128,'bridge_angle':0},None);record('Lost tracked bag pose stops sequence',t.state=='FAULT')
t=Transfer();t.state='TRANSFER';t.step(.1,{'corner_lift':.198,'corner_angle':P/2,'buffer_lift':.10,'bridge_angle':0},{'x':2,'y':0,'yaw':0,'speed':0,'z':.658});record('Reject mismatched belt heights',t.state=='FAULT')
t=Transfer();t.state='RECEIVE';t.step(50,{'corner_lift':0,'corner_angle':0,'buffer_lift':0,'bridge_angle':-P/2},None);record('Timed out phase latches fault',t.state=='FAULT')
t=Transfer();t.state='TRANSFER';t.step(.1,{'corner_lift':.198,'corner_angle':P/2,'buffer_lift':.128,'bridge_angle':0},{'x':3.,'y':0,'z':.16,'yaw':0,'speed':0});record('Reject bag fallen below the conveyor',t.state=='FAULT')
g=Guard(R/'ros2/bellybot_aircraft_environment/config/robot_geometry.json');path=g.plan([0,0,0,0],[.13,.8,.4,0]);record('Yaw plan raises and retracts before yaw',all(p[0]>=.08 and p[2]<=.5 for p in path if abs(p[1])>.01))
record('Bag-height caps are conservative and reproducible',[lift_cap(h) for h in [.28,.32,.38]]==[.23,.19,.13])
out={'passed':True,'checks':checks,'scope':'Pure logic only. No ROS, Gazebo or free-payload dynamics executed.'};(R/'docs/controller_contract_checks.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
