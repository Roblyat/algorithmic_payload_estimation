## Category Q5 — Physics-Informed (PINNs / Physically-Constrained Learning / Safe-ID)
**Scope:** Physics-informed ID with **explicit physical constraints** (e.g., log-Cholesky/LMI) and/or **safe trajectory design** for online identification; includes residual PINNs and **provably-safe** online PDPI with bounded-noise guarantees. [Q5_1, Q5_2]  
**Common sensors:** Joint states \(q,\dot q\); applied torques \( \tau \) (bounded noise); often **no torque/EE F/T** beyond controller signals. [Q5_1, Q5_2]  
**Assumptions:** Feasible model structure/priors; sufficient excitation; friction representable or absorbed by residuals; bounded sensor noise for interval bounds. [Q5_1, Q5_2]  
**Typical outputs:** **Inverse-dynamics torques**, **mass/CoM/inertia** (with **physical consistency/interval bounds**), feedforward models, and **safe** ID trajectories. [Q5_1, Q5_2]

**Representative evidence (growing list):**
- Q5_1 — *Residual-Driven Decomposed PINNs* (LS→PINNs; adaptive physics weight; smooth friction; **currents-only**; large torque-RMSE drops on high-friction joints). [Q5_1]
- Q5_2 — *Provably-Safe, Online System Identification*** (**ARMOUR** safe excitation + **momentum/log-Cholesky** ID; **interval bounds**; **Kinova Gen3 hardware**; only method to succeed in all tasks). [Q5_2]

**Strengths (from cards):**
- Maintains **physical consistency** while learning nonlinear friction; works **without torque/F/T sensors** (currents-only). (Q5_1) [Q5_1]
- **Provable safety** (limits & obstacles) during online PDPI; **rigorous interval bounds** under bounded noise; **hardware-validated**. (Q5_2) [Q5_2]

**Weaknesses (from cards):**
- **Offline** PINN tuning; joint-dependent gains; relies on current→torque map. (Q5_1) [Q5_1]
- Assumes bounded torque-sensor noise & known obstacles; **locally** exciting trajectories; discretization choice. (Q5_2) [Q5_2]

**Best-fit contexts:**
- Plants without torque/F/T seeking better inverse dynamics under nonlinear friction. (Q5_1) [Q5_1]
- **Online PDPI** for unknown payloads in clutter where **safety is non-negotiable**. (Q5_2) [Q5_2]

**Failure modes:**
- Poor current calibration/insufficient excitation; mis-weighted physics term. (Q5_1) [Q5_1]
- Over-conservative bounds (slow progress) or invalid noise bounds; planner fails to find feasible excitation. (Q5_2) [Q5_2]

**Synthesis (Q5):**
Q5 now spans **physics-informed residual learning** (Q5_1) and **provably-safe online PDPI** with **interval guarantees** (Q5_2). Together, they show how **physical priors + safe excitation** enable trustworthy identification **without extra F/T sensors**, even near torque limits and obstacles. [Q5_1, Q5_2]

**Gap notes (Q5):**
- Toward **streaming PINNs/interval ID** with uncertainty quantification; multi-robot generalization; tighter but **still safe** excitation; integrate **payload perception** to shrink initial bounds. [Q5_1, Q5_2]
