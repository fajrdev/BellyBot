# Boeing 737-800 Cargo Hold Simulation — Baggage Handling & Stacking

## 1. Narrow-Body Aircraft Context & Hold Geometry

To evaluate BellyBot A against the SUTD Hackathon Brief under authentic operational constraints, a simulation environment was constructed modeling the **Boeing 737-800 Aft Cargo Compartment**.

### 1.1 Dimensional Confinement & Door Clearance
| Parameter | Aircraft Specification | BellyBot A Dimension | Clearance / Status |
|---|---|---|---|
| **Hold Usable Height** | 1.15 m (< 1.2 m ceiling) | Stowed: 0.468 m / Max Lift: 0.708 m | **PASS** — Clears ceiling with > 440 mm margin |
| **Cargo Door Width** | 1.22 m (48 inches) | Robot Width: 1.100 m | **PASS** — ~60 mm lateral margin per side |
| **Cargo Door Height** | 0.89 m (35 inches) | Robot Stowed Height: 0.468 m | **PASS** — > 420 mm vertical margin |
| **Hold Floor Length** | 7.00 m bulk compartment | Robot Length: 1.250 m | **PASS** — Sits in longitudinal aisle |
| **Flat Floor Width** | 1.80 m | Track Width: 1.100 m | **PASS** — 350 mm lateral margin per side |

---

## 2. Baggage Stacking Strategy (2-Layer Arrangement)

The simulation evaluates 6 baggage items representing the specified **20–32 kg** range with irregular dimensions:
- **Bag 1:** Orange Hard-shell Case ($32\text{ kg}$, $0.76 \times 0.48 \times 0.36\text{ m}$)
- **Bag 2:** Blue Duffel Bag ($24\text{ kg}$, $0.72 \times 0.44 \times 0.32\text{ m}$)
- **Bag 3:** Crimson Roller Case ($30\text{ kg}$, $0.75 \times 0.46 \times 0.35\text{ m}$)
- **Bag 4:** Green Soft-shell Suitcase ($22\text{ kg}$, $0.70 \times 0.42 \times 0.30\text{ m}$)
- **Bag 5:** Purple Trunk Case ($26\text{ kg}$, $0.68 \times 0.45 \times 0.34\text{ m}$)
- **Bag 6:** Charcoal Heavy Case ($32\text{ kg}$, $0.76 \times 0.48 \times 0.36\text{ m}$)

### Stacking Configuration
- **Layer 1 (Floor Layer):** Bags 1, 2, and 3 placed directly on the cargo hold floor panels ($X = 2.2\text{ m}$, $1.4\text{ m}$, $0.6\text{ m}$).
- **Layer 2 (Upper Stack):** Bags 4, 5, and 6 stacked on top of Layer 1 with the telescopic lift raised $+200\text{ mm}$ ($Z = 0.54\text{ m}$, $0.48\text{ m}$, $0.52\text{ m}$).
- Total stack height with a $360\text{ mm}$ top bag is $\sim 900\text{ mm}$, leaving $\mathbf{250\text{ mm}}$ clearance under the $1.15\text{ m}$ hold ceiling.

---

## 3. Critical Engineering Gap: The 410 mm Tray Minimum Height

### Impact on First-Layer Placement
- **Observed Behavior:** The current BellyBot A vertical telescope mechanism has a minimum tray deck height of **$\sim 410\text{ mm}$**.
- **Floor Drop Assessment:**
  - When placing Layer 1 bags, the bag must drop from the tray deck to the hold floor.
  - Drop height $\Delta z \approx 230\text{ mm}$.
  - Impact velocity: $v = \sqrt{2 g \Delta z} \approx \mathbf{2.12\text{ m/s}}$.
  - Kinetic impact energy for a $32\text{ kg}$ suitcase: $E = m g \Delta z \approx \mathbf{72.2\text{ J}}$.
- **Conclusion:** A $72\text{ J}$ drop impact violates baggage handling standards and poses a severe risk of damage to fragile contents.
- **Recommended Architectural Solution for Concept A v0.3:**
  - Add an articulated tilting nose or auxiliary lowering spatula to the front of the tray that lowers bag placement to within $\le 50\text{ mm}$ of the floor.

### Upper Layer Placement
- When placing Layer 2 onto the already stacked lower bags, the telescopic lift holds the tray at $610\text{ mm}$.
- Drop height is reduced to $\Delta z \approx 40\text{ mm}$, with impact energy $\le 12.6\text{ J}$, confirming gentle stack placement.

---

## 4. Throughput & Cycle Time Analysis vs Hackathon Target

- **Hackathon Target:** $\ge \mathbf{12\text{ bags/minute}}$ ($\le \mathbf{5.0\text{ s/bag}}$ average cycle time).
- **Physical Trajectory Breakdown per Bag:**
  1. Infeed docking & transfer dwell: $\sim 1.2\text{ s}$
  2. Acceleration & transit down hold aisle ($1.5\text{–}3.5\text{ m}$ at safe $0.20\text{ m/s}$): $\sim 7.5\text{–}12.0\text{ s}$
  3. Lift, yaw ($45^\circ$), and telescopic extension ($240\text{ mm}$): $\sim 3.0\text{–}4.5\text{ s}$
  4. Detachment & settling dwell: $\sim 0.8\text{ s}$
  5. Retraction, zero-return & reverse transit: $\sim 8.0\text{–}12.0\text{ s}$
  - **Total Single-Robot Reciprocating Cycle Time:** $\mathbf{20.5\text{–}30.5\text{ s/bag}}$ (Rate: $\mathbf{2.0\text{–}2.9\text{ bags/minute}}$).

### Key Insight: The Throughput Discrepancy
A single autonomous mobile robot reciprocating back and forth along a 7-meter aircraft hold cannot physically achieve 12 bags/minute with realistic acceleration limits ($a \le 0.3\text{ m/s}^2$ to avoid tipping/sliding) and single-bag carriage. To reach 12 bags/minute, the robotic system architecture must incorporate:
1. **Multi-Bag Buffer / Carousel:** Transporting 3–4 bags per transit cycle.
2. **Powered Telescoping Indexing Floor:** Moving the stack line towards the door, reducing travel distance to $< 1.0\text{ m}$.
3. **High-Speed Gantry / Linear Carriage:** Shuttle speeds $> 1.5\text{ m/s}$ with synchronized fly-by deposit.

---

## 5. Aircraft Floor Loading Compliance

- Robot Mass Budget: $152\text{ kg}$
- Max Single Baggage Payload: $32\text{ kg}$ (Total: $184\text{ kg}$)
- Track Ground Contact Area: $0.836\text{ m}^2$ ($1.10\text{ m} \times 0.38\text{ m} \times 2$)
- **Uniform Static Loading:** $184 / 0.836 = \mathbf{220.1\text{ kg/m}^2} \ll 600\text{ kg/m}^2$ (**PASS**)
- **Worst-Case Factored Dynamic Patch Loading (Yaw $37.4^\circ$, Full Reach):** $\mathbf{574.0\text{ kg/m}^2}$
  - Aircraft limit: $600\text{ kg/m}^2$.
  - Safety margin: **$4.3\%$** ($26\text{ kg/m}^2$).
  - Strict acceleration and payload center-of-mass controls are required to avoid exceeding hold panel ratings during dynamic maneuvers.
