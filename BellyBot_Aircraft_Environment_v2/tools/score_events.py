#!/usr/bin/env python3
"""Score an explicit simulation-time window. Never infers throughput from supply.
CSV: time_s,bag_id,event,mass_kg. Events: received,placed,dropped,intervention.
Only first successful placement per known received bag counts. Any later drop
invalidates placement in the window. Patch CSV: time_s,patch_id,force_n,area_m2.
"""
import argparse,csv,json,math
from pathlib import Path

def score(rows,start,duration,patches=None):
 if not math.isfinite(start) or not math.isfinite(duration) or duration<=0:raise ValueError('Finite positive duration required')
 events=sorted(rows,key=lambda r:float(r['time_s']));received={};placed={};drops=set();interventions=0;invalid=[]
 for r in events:
  t=float(r['time_s']);mass=float(r['mass_kg']) if r.get('mass_kg') else None
  if not math.isfinite(t) or (mass is not None and not math.isfinite(mass)):raise ValueError('Nonfinite event')
  if t>=start+duration:continue
  bid=r['bag_id'];ev=r['event']
  if ev=='received':
   if mass is None or not 20<=mass<=32:invalid.append(bid)
   else:received[bid]=mass
  if start<=t<start+duration:
   if ev=='placed':
    if bid not in received:invalid.append(bid)
    else:placed.setdefault(bid,t)
   elif ev=='dropped':drops.add(bid)
   elif ev=='intervention':interventions+=1
 valid=set(placed)-drops-set(invalid);rate=60*len(valid)/duration
 pressure=[]
 if patches is not None:
  # Each row must already contain the simultaneous SUM of normal force over one non-overlapping patch.
  for r in patches:
   if not start<=float(r['time_s'])<start+duration:continue
   f=float(r['force_n']);area=float(r['area_m2'])
   if not math.isfinite(f) or not math.isfinite(area) or f<0 or area<=0:raise ValueError('Invalid force/area')
   pressure.append(f/9.81/area)
 return {'window_start_sim_s':start,'window_duration_sim_s':duration,'unique_accepted_placements':len(valid),'bags_per_min':rate,'throughput_target_met':rate>=12,
 'dropped_bags':sorted(drops),'manual_interventions':interventions,'invalid_bag_ids':sorted(set(invalid)),
 'peak_patch_kg_m2':max(pressure) if pressure else None,'floor_limit_met':max(pressure)<=600 if pressure else None,
 'overall_hackathon_compliance':'NOT DETERMINED: geometry, fragility, autonomy and physical floor support need separate evidence'}
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--events',type=Path,required=True);p.add_argument('--start',type=float,default=0);p.add_argument('--duration',type=float,required=True);p.add_argument('--patches',type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
 with a.events.open() as f:rows=list(csv.DictReader(f))
 patches=None
 if a.patches:
  with a.patches.open() as f:patches=list(csv.DictReader(f))
 report=score(rows,a.start,a.duration,patches);s=json.dumps(report,indent=2);print(s)
 if a.output:a.output.write_text(s+'\n')
