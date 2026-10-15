# Sources, observations and assumptions

1. **User-supplied hackathon guideline**, `reference/Hackathon_Guidelines_Oct_2.pdf`: confined height below 1.2 m; baggage 20–32 kg; target at least 12 bags/min; floor load at most 600 kg/m². STEP, a video no longer than two minutes and a 5–8 page report are submission deliverables. This environment ZIP complements the existing robot deliverables; it does not claim final submission readiness.
2. **User-supplied three photos**, preserved as `reference/supplied_hold_1.png` through `3.png`. Visible: low flat ceiling, multiple sloping liner sections, longitudinal metal floor strips, side doorway, tan suspended material, lamps, kneeling/forward-leaning workers and a suitcase. Dimensions, exact aircraft model and forces cannot be measured reliably from these perspective images.
3. **Boeing, 737NG passenger overview, PDF page 17, “Cargo compartment dimensions.”** https://www.boeing.com/content/dam/boeing/v2/company/startupboeing/historical/737NG_passenger.pdf . Used for the forward-hold dimensional baseline and aft endpoint stress dimensions. The 7.7 m forward length is the published 737-800 example. This is not evidence that the user's photos show that aircraft. Intermediate chamfers, station locations, trim and sill details are original assumptions.
4. **Boeing airport-planning door reference**, lower-deck clear-opening table, as a cross-check: https://www.boeing.com/content/dam/boeing/boeingdotcom/commercial/airports/acaps/737NG_REV-B.pdf . Use clear opening, not outside door-panel size. No aircraft service or structural authorization is implied.
5. **User's video:** https://www.youtube.com/watch?v=6gra6c2N8n4 . Public metadata identifies Tim Falagan's *Ramp Agent.. “A” Team!* (139 seconds). The video-analysis service lacked credits; public page metadata was accessible, but playback/frame retrieval did not succeed. **The video was not watched, and no timeline or measured-cycle claim is made.** The manual scene is grounded in the supplied stills. A direct video upload would allow a further motion-based revision.
6. **Gazebo Harmonic ROS pairing:** https://gazebosim.org/docs/harmonic/ros_installation/ . Target Jazzy/Harmonic on Ubuntu 24.04. This is a specific supported target, not a promise for every ROS/Gazebo version.
7. **Gazebo Sim 8 TrackController API:** https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1TrackController.html . Supports conveyor contact-surface motion and command timeout. It does not simulate a detailed belt drivetrain or payload deformation.

Accessed 10 October 2026. Online source PDFs and video are linked, not redistributed. Supplied images/guidelines retain their original ownership. Other geometry is original procedural work.

## What is assumed

The forward cross-section passes through (half-width, height) points (0.61,0), (0.82,0.10), (1.24,0.42), (1.43,0.80), (1.525,1.12) metres. Only the floor width, top width and maximum height are sourced dimensions. The intermediate curve is faceted to resemble the references and is adjustable. This uncertainty directly affects the reported turning limit.

The door is a rectangular local tunnel. Its header underside is at Z=0.89 m; an assumed 8 mm sill lip reduces the modeled clearance at that edge to 0.882 m (a conservative detail relative to the published 0.89 m opening); rounded corners, real hinges, seals and its curved shell projection are not reconstructed. The flat entry pocket is an assumed transition. Apron elevation, loader shape/speed, lighting and human proportions are assumed. Worker models are fixed collision proxies, not biomechanical actors. Floor strips are flush visual details; curtain and netting are visual only. Roof lamps and trim are simplified visuals, so a 100 mm roof margin is used in the proposed operating screen.

The aft stress model linearly interpolates its endpoint widths/heights. This deliberately exposes local roof constraints but does not reproduce a particular aft-hold station profile. Do not use it to compute actual aircraft cargo volume or route approval.

## Revision 2 additions

- Dorner, Rotating Conveyors (primary manufacturer description): https://www.dornerconveyors.com/solutions/rotating-conveyors . Supports lift-and-rotate as an established conveyor mechanism class; the delivered dimensions, lift columns and 32 kg design payload are our custom concept, not a selected or manufacturer-rated Dorner product.
- Gazebo Sim 8 PosePublisher: https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1PosePublisher.html . Used for measured simulator model-pose feedback.
- ROS/Gazebo bridge official package: https://index.ros.org/p/ros_gz_bridge/ . Used Pose_V → TFMessage, Model → JointState and scalar/Twist bridges.
- The 180 mm deck, ramp, 300 mm inlet belt, corner-table dimensions and all intermediate transfer interfaces are newly designed fixtures, not dimensions measured from the user video. Source video playback remains unavailable.
- Aircraft dimensional source rechecked: https://www.boeing.com/content/dam/boeing/v2/company/startupboeing/historical/737NG_passenger.pdf , PDF page 17. The baseline dimensions are unchanged from revision 1.
