# Scene 07 — Results Limitations

## Screen time and objective
01:48–02:00. Summarize measured operation, structural compatibility, and physical limitations directly from verified simulation artifacts (`results/mission_metrics.json` and `results/mission_events.csv`), without substituting analytical calculations for runtime evidence.

## Technical motion sequence
End with BellyBot stationary at the door dock, all belts stopped, deposited cargo stable on the floor, and the return corridor clear.

## Measured Results Summary Table
| Metric / Parameter | Value | Scope / Evidence Source | Target / Guideline |
|---|---|---|---|
| Single-bag forward delivery | **92.4 s** (0.65 bags/min) | Measured sim time from dock to verified rest | N/A |
| Full shuttle cycle round trip | **140.5 s** (0.43 bags/min) | Measured round trip (infeed, travel, deposit, return) | N/A |
| 30.0 s continuous interval | **0 completed** (0.0 bags/min) | Measured 30.0 s sim-time interval (`results/mission_metrics.json`) | Contest: $\ge 12$ bags/min |
| Analytical fixed station | **12.89 bags/min** (4.65 s/bag) | Stationary continuous infeed without transit | Ideal benchmark |
| Reverse discharge cycle | **138.8 s** sim time | Tray retrieval return down $5.48^\circ$ door belt to $Y=-5.101\text{ m}$ | Reversible logistics |
| Minimum roof clearance | **345 mm** margin | Measured SE(3) sweep in curved hold ($Z_{\text{max}} = 0.805\text{ m}$) | Safe clearance $> 100\text{ mm}$ |
| Minimum floor clearance | **3 mm** | Level chassis datum above aircraft floor | Ground clearance |
| Lateral corridor tracking drift | **< 1.4 mm** | Straight-line center aisle tracking at $v=0.12\text{ m/s}$ | Aisle clearance |
| Peak floor pressure (screen) | **512.4 kg/m²** | CAD mass-equivalent factored patch reaction | Limit: $\le 600\text{ kg/m}^2$ |
| Emergency stop response | **< 20 ms** | Live E-Stop latch on `/mission/stop` | Immediate shutdown |
| Supervised recovery | **Verified** | Precondition-checked `/mission/reset` Trigger service | Verified recovery |

## Unresolved Limitations & Boundary Disclosures
1. **Physical Shuttle Cycle Throughput vs Stationary Rate:** A single physical BellyBot acting as a dock shuttle requires $\approx 140.5\text{ s}$ per round trip ($0.43\text{ bags/min}$). Achieving the contest target of $\ge 12\text{ bags/min}$ requires either a continuous infeed line or a multi-robot fleet (28–30 synchronized shuttles). Analytical claims of $12.89\text{ bags/min}$ apply strictly to stationary fixed-deck infeed, not mobile shuttle cycles.
2. **Floor Scoop on Rigid Luggage:** In rigid-body physics, rigid box luggage sitting on a flat floor lacks normal vertical force components at sharp 90° corners; the tray roller pushes rather than wedges underneath. Real-world floor pickup requires compliant, deformable luggage shells or a beveled floor ramp.
3. **Upper-Layer Stacking Dynamics:** Lifting to second layer ($Z \approx 0.55\text{ m}$) is kinematically verified in CAD, but dynamic free-box stacking without lateral cargo nets or compliant settling remains uncommissioned in rigid physics simulation.
4. **Manual Setup vs Autonomous Operations:** The changeover between the temporary vehicle entry ramp and the $5.48^\circ$ inclined doorway belt is declared as a manual ground-crew procedure; it is not an automated mechanism transformation.
5. **Personnel Safety:** The software stop is a latched controller safety state based on feedback loss, not a certified safety-rated emergency stop (ISO 13849/IEC 62061) or an autonomous human-detection vision system.

## Gazebo implementation & ROS 2 Orchestration
Overlays driven dynamically from `results/mission_metrics.json`. All seven scene briefs committed to version control with runtime verification logs preserved in `results/mission_events.csv`.

