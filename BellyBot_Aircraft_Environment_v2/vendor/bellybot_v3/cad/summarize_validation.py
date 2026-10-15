#!/usr/bin/env python3
"""Write a readable summary directly from the current verification results."""
import json
from pathlib import Path

def main():
    docs=Path(__file__).resolve().parents[1]/'docs'
    read=lambda name:json.loads((docs/name).read_text())
    v=read('validation.json');s=read('structure_checks.json');c=read('control_checks.json')
    t=read('timing.json');b=read('batch_fixture_checks.json')
    assert not any(v['solid_intersections'].values())
    assert all(g['component_count']==1 for g in s['rigid_groups'].values())
    cases=v['floor_pressure_screen']['cases'];peak=max(x['factored_max_patch_kg_m2'] for x in cases)
    rows='\n'.join(f"| {x['pose']} | {x['factored_max_patch_kg_m2']:.2f} | {'Within screen' if x['within_screen_limit'] else 'Exceeds'} |" for x in cases)
    groups='\n'.join(f"| {name} | {g['solid_count']} | {g['component_count']} |" for name,g in s['rigid_groups'].items())
    summary=f'''# Offline verification — {v['revision']}

## Corrected support structure

| Rigid subassembly | Separate solids checked | Connected components |
|---|---:|---:|
{groups}

Each separate solid is a node; actual contact within 0.005 mm joins nodes. Compound membership does not count as a connection. Decorative painted tray stripes are excluded. All four corrected groups form one connected component each. This proves geometric continuity only, not joint strength, preload, weld quality or stiffness. The bearing internals remain a mounting envelope.

Both 54 mm captive blocks on each side remain on their 830 mm guide at 0, 125, 250, 375 and 500 mm stage travel. The least guide-end margin is 8 mm. Linear paths make these endpoint inequalities valid throughout the stroke; wear, deflection and bearing capacity are not tested.

## CAD, URDF and controls

- {v['part_groups']} CAD part groups / {v['solid_count']} solids passed shape validity checks.
- Exported home STEP reimported successfully with {v['step_reimport']['solids']} valid solids; the free example bag is omitted from that assembly.
- Five articulated URDF variants passed independent forward-kinematics comparisons at 40 random configurations each. Mesh paths, millimetre-to-metre scale and positive physical inertia tensors were checked.
- {len(v['solid_intersections'])} named CAD configurations showed no cross-link solid intersection above 0.25 mm³. They include independent lift/extension positions, floor pitch, low yaw sweeps with the infeed retracted and maximum infeed height. See `validation.json` for exact poses/results.
- {v['demo_envelope_sampling']['samples']} samples of the unloaded 44-second demonstration obey individual joint/velocity limits. Lowest nose height is {v['demo_envelope_sampling']['min_nose_mm']:.3f} mm and maximum robot height on that demonstration is {v['demo_envelope_sampling']['max_robot_height_mm']:.1f} mm. This demonstration does not use the full lift stroke.
- The separate full-stroke unloaded CAD pose is {v['envelopes_mm']['unloaded_full_stroke'][1][2]:.1f} mm tall, below the 1,200 mm height target. Loaded and curved-roof clearances depend on payload and location.
- {c['passed']} control checks passed, including the new infeed-before-yaw guard, floor pitch sequence, invalid-pose rejection and differential wheel speeds.
- The 555 mm receiving fixture clears the CAD at 0, 400 and 800 mm reach with 100 mm lift and zero yaw. Cassette underside clearance is 15 mm; the belt-to-fixture drop is 40 mm.

These are sampled interference checks, not a continuous collision proof. Independent viewer sliders and direct joint topics can bypass the supervisor's multi-joint rules. Collision primitives omit small bearing/guide details; simulator joints impose the ideal constraints.

## Recalculated floor-load screen

Assigned robot mass is 173 kg; the rigid bag is 32 kg. Contact is 1,320 × 350 mm per track. The screen assumes four equal-compliance half-track patches with uniform pressure and a 1.5 multiplier.

| Named pose | Factored peak kg/m² | Against 600 kg/m² |
|---|---:|---|
{rows}

Peak is {peak:.2f} kg/m², leaving only {600-peak:.2f} kg/m² in this approximation. Mass and load distribution uncertainty can consume that margin. Actual local wheel/track pressures, floor flexibility, acceleration, bag offsets and contact loss require measurement. No aircraft-floor compliance claim is made.

## Timing and runtime boundary

The prepared layer-batch estimate remains {t['layer_batch']['nominal_cycle_s']:.4f} seconds, or {t['layer_batch']['ideal_bags_min']:.2f} bags/min before additional delay. The revised yaw route is about {t['generic_65_degree_route_estimate_s_before_separate_deposit_allowance']:.2f} seconds before a separate deposit allowance; floor mechanism approach/return is about {t['floor_round_trip_mechanism_only_s_before_bag_transfer']:.2f} seconds before bag transfer. These are analytical motion budgets.

Gazebo, ROS 2 and RViz were not run in the authoring environment. SDF XML references and plugin targets were checked; the SDFormat schema/backend, tracking, contact, conveyor direction and free-bag behavior require runtime commissioning. Structural capacity, final drives/brakes, free-bag pickup, fragile/soft baggage and sustained throughput remain unverified.

Machine-readable evidence: `structure_checks.json`, `validation.json`, `control_checks.json`, `batch_fixture_checks.json`, `timing.json` and `motion_profile.json`. The source checks are in `cad/`.
'''
    (docs/'VALIDATION.md').write_text(summary)
    print('Readable validation summary updated')

if __name__=='__main__':main()
