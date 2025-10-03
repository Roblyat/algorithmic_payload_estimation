## Category Q4 — Hybrid Observers (Model + Learning Residuals)

**Scope:** Pipelines that start from **analytic dynamics** (RBD + friction) and add **learned residuals** (NN/GP) inside observers/filters or as **residual compensators** to improve **sensorless force** or **feedforward inverse dynamics** on industrial platforms. [Q4_1–Q4_3]
**Common sensors:** Joint states and controller torques/currents; typically **no wrist F/T** at runtime; model terms (M,C,G,J). [Q4_1–Q4_3]
**Assumptions:** Residuals are **learnable**; noise either **stochastic-filtered** (KF/DKF) or absorbed by residuals; model baseline available. [Q4_1–Q4_3]
**Typical outputs:** **External torques/EE forces** (sensorless) and/or **feedforward torques** with reduced residuals. [Q4_1–Q4_3]

**Representative evidence (growing list):**

* Q4_1 — *Sensorless force estimation … NN friction + DKF* (KUKA heavy-duty; best MSE vs momentum/model-based). [Q4_1]
* Q4_2 — *Adaptive Neural Trajectory Tracking … State Constraints* (CTC + RBFNN + NDO + BLF; n-DoF; UUB; sensitivity; 7-DoF sims). [Q4_2]
* Q4_3 — *Hybrid Inverse Dynamics Compensation via BLL*** (**Bagging–LSTM–Linear residual** on Panda; **avg residual ↓ 0.5651→0.1096 Nm**; **beats LSTM/GP**). [Q4_3]

**Strengths (from cards):**

* Robust **sensorless force** with **NN friction** and **noise-calibrated DKF** (Q4_1). [Q4_1]
* **Constraint-aware** tracking with **sensorless disturbance** (NDO) and UUB (Q4_2). [Q4_2]
* **Feedforward torque accuracy** boost via **ensemble residual learning (BLL)**; strong per-joint metrics (Q4_3). [Q4_3]

**Weaknesses (from cards):**

* Needs ID/NN training; Gaussian noise assumption; model/Jacobian reliance; single platform (Q4_1). [Q4_1]
* Simulation-only; many gains/hyper-params; bounded-disturbance assumption (Q4_2). [Q4_2]
* **Offline**, single-robot dataset; hyper-param tuning for bagging/LSTM/linear; no explicit PDPI (Q4_3). [Q4_3]

**Best-fit contexts:**

* Industrial HRC/manufacturing needing robust **contact awareness** without wrist F/T (Q4_1). [Q4_1]
* **State-constrained tracking** with sensorless disturbance estimation (Q4_2). [Q4_2]
* **Feedforward control** on cobots/industrial arms needing **lower torque residuals** without extra sensors (Q4_3). [Q4_3]

**Failure modes:**

* Poor NN tuning/excitation; inaccurate (J,M,C,G); non-Gaussian noise (Q4_1). [Q4_1]
* Mis-tuned BLF/observer gains; unmeasured states (Q4_2). [Q4_2]
* Data shift beyond training; bagging ensemble not diverse; residuals with unmodeled disturbances (Q4_3). [Q4_3]

**Synthesis (Q4):**
Q4 spans **hybrid observers** for **sensorless interaction** (Q4_1), **constraint-secure tracking** (Q4_2), and **residual-learned feedforward** (Q4_3). Adding learned residuals (NN ensembles) **on top of RBD** materially reduces torque errors, complementing DKF/NDO pipelines for safer, more precise manipulation. [Q4_1–Q4_3]
