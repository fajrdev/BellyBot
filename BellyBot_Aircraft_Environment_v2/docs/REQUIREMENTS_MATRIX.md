# Hackathon requirement mapping — revision 2

| Supplied requirement | Revision 2 evidence | Status |
|---|---|---|
| Robot under 1.2 m | Stowed 0.538 m; height-aware guard uses stricter 1.12 m hold roof and 100 mm margin | Geometric checks pass in approved work profile |
| Handle 20–32 kg bags | Five mass/inertia bag models and assigned-mass robot screen; 32 kg default | Modeled, contact and hardware capacity unverified |
| At least 12 bags/min | Event scorer retained; connected transfer permits a real timed trial | **Not demonstrated; supplied serial commissioning sequence is slower** |
| No more than 600 kg/m² | Four-patch assigned-mass pose rejection, scorer for measured patch force/area | Aircraft-floor compliance not proven; fixture anchoring bypasses support reactions |
| Narrow-body door entry | 180 mm temporary raised deck and tracked route; 60 mm jamb, 32.77 mm turning clearance | Geometry resolved for this fixture; ramp dynamics/setup pending |
| Side-to-longitudinal transfer | Lift-and-rotate conveyor, folding nose, lowering buffer, controller interlocks | Geometry connected; free-bag transfer not runtime tested |
| Autonomy and safety | Feedback-driven commissioning sequence, pose/height rejection and latched stop | Partial; general perception, people detection and pile placement not delivered |
| STEP model | Aircraft fixture, original robot, separate entry and corner-transfer STEP assemblies | Included; no structural or manufacturing approval |
| MP4, maximum 2 minutes | Included 30-second MP4 is explicitly labeled geometric, not a physics demonstration | Final simulation recording still needed |
| 5–8 page report with team details | Six-page revision review | Team fields and final measured results still needed |

Reference: supplied `reference/Hackathon_Guidelines_Oct_2.pdf`. These status labels distinguish model/code delivery from demonstrated hackathon performance.
