# Scene 03 — Measured Loading

## Screen time and objective
00:23–00:53. Show one uninterrupted 30.0 s simulation-time loading interval with on-screen timer and verified completed-placement count. Retain honest measured results: in a single-robot physical door-shuttle architecture, round-trip transport exceeds 30 s; $N=0$ placements complete within a single 30.0 s window starting from an empty dock. Report the measured physical shuttle throughput alongside the theoretical fixed-station benchmark.

## Technical motion sequence
1. Autonomous Infeed: Dynamic 32 kg case meters across the exterior loader and inclined door belt onto the BellyBot receiving tray ($Z=0.498\text{ m}$) in $5.5\text{ s}$; `support_confirmed` emitted at $t=6.58\text{ s}$.
2. Dock Departure: Robot drives forward 95 mm along $+Y$ to aisle centerline $Y=0.000\text{ m}$, pivots clockwise $90^\circ$ to $\text{Yaw}=0.0\text{ rad}$ ($18.0\text{ s}$).
3. Aisle Transport: BellyBot travels along $+X$ center corridor toward work datum $X=4.08\text{ m}$ (lateral drift $< 1.4\text{ mm}$).
4. Deposit & Dwell: Arm extends $0.74\text{ m}$, pitches down $30.9^\circ$, and tray belt rolls bag forward onto cargo floor at $X=5.144\text{ m}, Y=-0.023\text{ m}, Z=0.284\text{ m}$; 0.5 s settling dwell verifies placement; `placement_complete` emitted at $t=92.4\text{ s}$.
5. Return Shuttle: Robot reverses along $-X$ back to dock, pivots, and docks at $t=140.5\text{ s}$.

## Gazebo implementation
Dynamic bag transfer driven by physical contact and gravity; zero artificial teleports or bag spawners inside the robot. Continuous 30.0 s test executed in `worlds/mission_shuttle.sdf` with simulation clock from `/clock`. Initial bag queued outside at $Y=-3.30\text{ m}$.

## ROS 2 communications and orchestration
Controller triggered via `/mission/command` `start_load`. Observer monitors `/mission/world_poses`, verifies tray support, and checks the 0.5 s settling predicate ($v < 0.02\text{ m/s}, \omega < 0.05\text{ rad/s}$). Telemetry logged to `results/mission_metrics.json` and `results/mission_events.csv`.

## Evidence, camera and acceptance gate
- Continuous 30.0 s interval measurement: $T = 30.0\text{ s}$ ($t_0=0.5\text{ s}$ to $t_1=30.5\text{ s}$), $N = 0$, rate = $0.0\text{ bags/min}$ over the 30 s window.
- Single-bag forward delivery duration: $92.4\text{ s}$ ($0.65\text{ bags/min}$).
- Full round-trip shuttle cycle duration: $140.5\text{ s}$ ($0.43\text{ bags/min}$ sustained).
- Analytical fixed-station reference: $4.65\text{ s/bag}$ ($12.89\text{ bags/min}$, stationary transfer without vehicle travel).
- On-screen label: "0 completed / 30.0 s simulation interval | Full physical shuttle round trip: 140.5 s (0.43 bags/min) | Analytical station rate: 12.89 bags/min".

## Failure handling and recording rule
A failed prerequisite blocks the related claim. Continuous metrics generated live from `results/mission_metrics.json`. No artificial speedup or retroactive counter adjustments permitted.

