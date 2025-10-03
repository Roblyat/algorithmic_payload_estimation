## Category Q7 — Minimal-Interaction / Task-Driven ID & Control

**Scope:** Estimating or adapting dynamics with **minimal interaction** (e.g., single push) or **during the task** via **online ID/control** and **learned force control** on rigid, position-controlled robots. [Q7_1–Q7_4]
**Common sensors:** Pushing: pusher **F/T** + planar velocities. Task-driven ID/control: state/action histories or ((q,\dot q,\ddot q,\tau)). Learned force control: **wrist F/T** + EE pose/velocity. [Q7_1–Q7_4]
**Assumptions:** Planar/quasi-static for pushing; known morphology for model-based RL; **goal pose known** for RL force control; no dedicated PE. [Q7_1–Q7_4]
**Typical outputs:** **2D object inertial params** from one push; **task-sufficient models** for real-time control; or **safe, learned force behavior** on position-controlled robots. [Q7_1–Q7_4]

**Representative evidence (growing list):**

* Q7_1 — *Robotic Pushing* (physics-guided features + multi-output RF; **single push** estimates (M, r_{x,y}, I_{zz})). [Q7_1]
* Q7_2 — *Parametrized Models + Optimistic MPC* (real-time online ID + control with LS features). [Q7_2]
* Q7_3 — *UP-OSI* (universal policy + online SI; adapts to **varying friction/unknown masses**). [Q7_3]
* Q7_4 — *RL Force Control on Rigid Position-Controlled Robots*** (SAC + parallel/admittance; **fail-safe**; **UR3e** ring/peg insertion). [Q7_4]
* Q7_5 — *RL-Based Variable Impedance for HRC*** (**PPO** outputs (M,B,K); **0.1 mm** clearance simulation; teach-less, position-control friendly). [Q7_5]

**Strengths (from cards):**

* Minimal-interaction object ID (Q7_1). [Q7_1]
* Task-driven online ID/control with strong sample-efficiency (Q7_2–Q7_3). [Q7_2–Q7_3]
* **Safe, real-robot learning of low-level force control** on **position-controlled** arms (Q7_4). [Q7_4]
* **Adaptive impedance** on **position-controlled** arms; no demonstrations; robust to pose deviations (Q7_5). [Q7_5]

**Weaknesses (from cards):**

* Pushing is planar/2D; sim→real gap (Q7_1). [Q7_1]
* UP-OSI largely simulation; assumes morphology (Q7_3). [Q7_3]
* **Requires wrist F/T**; gain-range hyperparameters matter; assumes goal pose (Q7_4). [Q7_4]
* **Requires wrist F/T + vision;** results mainly in **simulation**; relies on reference trajectory/force profile (Q7_5). [Q7_5]

**Best-fit contexts:**

* Pre-grasp tabletop object assessment (Q7_1). [Q7_1]
* Rapid adaptation to changing payloads/surfaces during tasks (Q7_2–Q7_3). [Q7_2–Q7_3]
* **Contact-rich assembly on industrial, position-controlled robots** needing learned force behavior with safety (Q7_4). [Q7_4]
* **High-precision HRC assembly** with tiny clearances on **position-controlled** robots (Q7_5). [Q7_5]

**Failure modes:**

* Non-quasi-static pushes; poor contact tracking (Q7_1). [Q7_1]
* Real-robot noise/latency; lack of safety constraints (Q7_3). [Q7_3]
* F/T noise; inaccurate pose/goal; limited sim→real validation (Q7_5). [Q7_5]

**Synthesis (Q7):**
Q7 now includes **single-push ID**, **online ID+control**, and **RL-learned variable impedance** for HRC. **Q7_5** shows a **teach-less PPO** approach that adjusts (M,B,K) on the fly, enabling compliant, precise collaboration in **position control** settings. [Q7_1–Q7_5]
