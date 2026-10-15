# Scene 06 — Stop Recovery

## Screen time and objective
01:33–01:48. Demonstrate one real implemented stop and a controlled supervised recovery. Recommended first test is loss of required bag feedback while docked, not an unimplemented human-detection story.

## Technical motion sequence
Begin stationary with bag safely supported and belts at low speed. Interrupt the observer/required bag feedback using a logged test injection, not a simulated worker model unless actual collision/person sensing exists. Controller detects stale evidence, stops upstream/door/tray belts and base, holds joints at safe measured configuration and latches STOP_LATCHED. Restore data, inspect cause and bag state, explicitly request recovery; reset only if fresh valid evidence and safe support. Resume the interrupted transaction without double-counting. If original controller is used in a separate legacy shot, recovery requires investigated node restart; label it, do not send false and claim reset.

## Gazebo implementation
Store injection start/stop timestamps and actual stopped-belt/base evidence. No deleting the obstructing/misaligned bag to hide it. An obstruction trial is optional only after a real sensor/observer detects it before contact and prevents contact. Keep the /clock running for simulated latency measurement. Also test wall-clock watchdog separately for paused clock/lost transport; a freeze must not make the stop test vacuously pass. Restart must preserve bag IDs and completed-placement ledger.

## ROS 2 communications and orchestration
New /mission/stop Bool true always latches. /mission/reset Trigger can accept only after cause cleared and explicit recovery request. Record fault reason, required feedback age, stop command time, measured velocity response and reset decision. Stop false is ignored as reset. Original /aircraft/stop does not support reset; do not advertise /mission/reset before implementing it.

## Evidence, camera and acceptance gate
Exit: implemented detection, output stop, latched rejection of new motion, supervised reset and controlled continuation. Separate stop-command latency from time to measured motion stop; ROS Bool publication alone is not a stop measurement. If recovery takes longer than 15 screen seconds, show labelled cuts outside the throughput interval. Narration: “Missing transfer feedback causes a software stop; restart requires validation.” This is not a safety-rated emergency stop or proof of human-safe operation.

## Failure handling and recording rule
A failed prerequisite blocks the related claim, not just the camera shot. Save failure evidence and report the smallest fix; do not force readiness, bypass guards or substitute a kinematic animation without a “concept animation” label. Camera time is not a deadline for physics. New /mission interfaces in this brief require implementation; see technical/03_ROS2_GAZEBO_CONTRACT.md. Existing mechanism facts come from supplied archives [file:1][file:2]; contest targets come from [file:5].
