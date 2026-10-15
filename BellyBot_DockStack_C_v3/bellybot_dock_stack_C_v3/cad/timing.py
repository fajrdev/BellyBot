#!/usr/bin/env python3
"""Analytical planning benchmark. Never labelled as observed throughput."""
import sys,json,math,csv
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2/bellybot_dock_stack_description/scripts'))
from control_core import duration_for,segment_duration,plan,floor_pitch

def main():
 # Nominal 760 mm bag. Receiving front is x=.78 m, clear of the longer crawler.
 receive=duration_for(.90,1.5,3.)
 extend=duration_for(.80,1.4,2.5)
 # Retraction .8 m while a modeled bag advances .625 m: belt travel 1.425 m.
 retract=max(duration_for(.8,1.4,2.5),duration_for(1.425,1.5,3.))
 verify=.20
 phases=[('Receive from staged infeed',receive),('Extend at layer height',extend),('Retract and advance belt',retract),('Verify and inter-command allowance',verify)]
 T=sum(t for _,t in phases)
 # Rectangular contact kinematics until the trailing bag edge passes the tray nose.
 # This is an assumption, not contact dynamics: bag position follows belt before separation.
 fraction=(.8+.49-(.8-.760/2))/(.8+.625)
 clear_center=.8+.625*fraction
 old_lower=2*(.1/.24+math.radians(65)/.55+.5/.4)
 # New oblique path includes lift/yaw/reach and belt/receive time, real velocity/acceleration profiles.
 a=[0,0,0,.4,0,0];b=[.1,math.radians(65),.5,.15,0,0]
 forward=plan(a,b);ret=plan(b,a)
 def path_time(start,poses):
  t=0
  for p in poses:t+=segment_duration(start,p);start=p
  return t
 generic=path_time(a,forward)+path_time(b,ret)+receive+verify
 floor=[0,0,.84,.4,floor_pitch(),0]
 floor_route=path_time(a,plan(a,floor))+path_time(floor,plan(floor,a))
 result={'scope':'analytical kinematic estimate, no physics contact/actuator test',
   'v1_speed_only_lower_bound_s':old_lower,
   'layer_batch':{'phases':[{'phase':n,'time_s':t} for n,t in phases],'nominal_cycle_s':T,'ideal_bags_min':60/T,
     'minimum_productive_fraction_for_12':12*T/60,'nonproductive_allowance_s_per_bag':5-T,
     'at_90_percent_bags_min':.90*60/T,
     'assumptions':['layer and infeed pre-set to same height','yaw zero during batch','one bag staged at receiving point',
       '0.80 m extension and simultaneous 1.425 m belt travel during retraction','receiver front x=0.780 m; top z=0.555 m below tray surface z=0.595 m',
       'belt tracking and friction sufficient to produce intended bag motion',
       '0.20 s verification/command allowance, not measured','all setup, repositioning, faults and retries consume the remaining time budget'],
     'contact_separation_assumption':{'quintic_blend_at_separation':fraction,'bag_center_x_m':clear_center,'bag_rear_x_m':clear_center-.38,'stack_front_x_m':.78,
       'full_support_margin_m':clear_center-.38-.78,'surface_drop_m':.040,'note':'ignores slip, stack friction, 40 mm settling and compliant deformation; validate in Gazebo and hardware'}},
   'generic_65_degree_route_estimate_s_before_separate_deposit_allowance':generic,
   'floor_round_trip_mechanism_only_s_before_bag_transfer':floor_route,
   'load_sizing_estimates':{'assumed_payload_kg':32,'belt_acceleration_m_s2':3.,'payload_horizontal_belt_force_N':32*3,
     'min_flat_belt_friction_coefficient_no_slip':3/9.80665,'floor_ramp_deg':math.degrees(floor_pitch()),
     'min_ramp_mu_constant_speed':math.tan(floor_pitch()),'ramp_gravity_component_N':32*9.80665*math.sin(floor_pitch()),
     'pitch_static_payload_moment_Nm':32*9.80665*.43,'notes':'excludes tray weight, insertion loads, efficiency, structure and fragility. No motor selected.'},
   'full_hackathon_throughput_verified':False}
 (ROOT/'docs/timing.json').write_text(json.dumps(result,indent=2))
 with (ROOT/'docs/cycle_budget.csv').open('w',newline='') as f:
  w=csv.writer(f);w.writerow(['phase','analytical_seconds']);w.writerows(phases)
 print(json.dumps(result,indent=2))
if __name__=='__main__':main()
