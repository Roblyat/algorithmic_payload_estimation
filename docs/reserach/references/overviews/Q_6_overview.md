## Category Q6 — Domain / Sim-to-Real, Continual Adaptation & Contact/PDPI (Sensor-Lean)

**Scope:** Sensor-lean methods for **contact** or **PDPI** during normal operation, including **UV-image contact** (no torques), **incremental/ensemble PDPI** without dedicated excitation, **optimizer-driven RDPI→force**, and now **parameter-difference PDPI** using **commanded signals** and **nonlinear friction**. [Q6_1–Q6_6]
**Common sensors:** Proprioception (q,\dot q) and often controller **(\tau)** via currents/gains; typically **no wrist F/T**; sometimes commanded (q_d,\dot q_d,\ddot q_d) from the controller. [Q6_1–Q6_6]
**Assumptions:** Access to controller signals (or good sim coverage), adequate excitation or adaptation, rigid contacts/joints, and reliable kinematics/Jacobian when needed. [Q6_1–Q6_6]
**Typical outputs:** **Contact maps/flags**, **payload inertial parameters**, or **sensorless EE forces** for safety/admittance. [Q6_1–Q6_6]

**Representative evidence (growing list):**

* Q6_1 — *Contact Localization … without Torque Sensing* (UV/CDF + domain randomization; **ACC ~91.5%**, **~3 cm**). [Q6_1]
* Q6_2 — *Incremental Ensemble Learning* (online PDPI on arbitrary paths; notes **forgetting**). [Q6_2]
* Q6_3 — *Ensemble PDPI (Bagging NN/DT)* ((q,\dot q,\tau) only; **no accelerations/filters**; fast). [Q6_3]
* Q6_4 — *Addressing Catastrophic Forgetting …* (grow ensemble + expert selection; **no forgetting**). [Q6_4]
* Q6_5 — *PSO-based Sensorless Force Estimation* (optimizer-driven RDPI + excitation; **RMS ≈ 0.7 N**). [Q6_5]
* Q6_6 — *Parameter-Difference Online PDPI (UR10)*** (**nonlinear friction + commanded signals + RLS**; **~6.59 s**; **mass error ~0.04%**; manual-guidance forces ↓). [Q6_6]

**Strengths (from cards):**

* Motion-ready **contact** without torque sensors (Q6_1). [Q6_1]
* **Arbitrary-path PDPI** via incremental/ensemble learning (Q6_2–Q6_4). [Q6_2–Q6_4]
* **Sensorless force** via optimizer-driven RDPI (Q6_5). [Q6_5]
* **Highly accurate, fast mass ID** using **commanded trajectories** + **nonlinear friction**; **no external F/T** (Q6_6). [Q6_6]

**Weaknesses (from cards):**

* Sim/meshing pipeline; rigid-contact assumption (Q6_1). [Q6_1]
* Threshold tuning, torque quality; single-robot studies (Q6_2–Q6_4). [Q6_2–Q6_4]
* For Q6_5: needs built-in torque sensing and filtered (\ddot q) (offline ID). [Q6_5]
* For **Q6_6**: needs **offline base-ID** & drive gains; **excitation run**; **inertia tensor** less accurate. [Q6_6]

**Best-fit contexts:**

* **Online mapping/pHRI** without torque sensors/skins (Q6_1). [Q6_1]
* **Frequent reconfiguration** demanding fast PDPI (Q6_2–Q6_4). [Q6_2–Q6_4]
* Plants with **controller access to commanded trajectories** needing **quick mass/CoM** for compensation/guidance comfort (Q6_6). [Q6_6]

**Failure modes:**

* OOD dynamics beyond randomization (Q6_1). [Q6_1]
* Poor torque quality / mis-set thresholds / expert growth (Q6_2–Q6_4). [Q6_2–Q6_4]
* For Q6_6: lack of commanded signals or **inaccurate drive gains K**; weak excitation; flexible joints. [Q6_6]

**Synthesis (Q6):**
Q6 now spans **sensor-lean contact**, **continual PDPI**, **optimizer-driven RDPI→force**, and **parameter-difference PDPI** using **commanded signals** with **nonlinear friction**. **Q6_6** demonstrates **very fast (~6.6 s)** and **high-accuracy mass** identification **without external sensors**, delivering measurable **ergonomic gains** in manual guidance. [Q6_1–Q6_6]
