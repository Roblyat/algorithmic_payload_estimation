## 🔎 Q5 - Physics-Informed / Differentiable

### Q5_1 — *A Residual-Driven Decomposed PINNs Method for Dynamics Identification of Robot Manipulators* (RCAR, 2025)

- **Task:** **RDPI** with **physics-informed residual learning**: two-stage **LS → PINNs** that refines dynamics and friction using an **adaptive dual-constraint** loss and **smooth (tanh) friction**. 
- **Setting:** **6-DoF EYOU-ARM**; **15 training trajectories × 10 s** @ **200 Hz**; separate unseen test trajectory. 
- **Sensors/Data:** (q,\dot q,\ddot q) and **motor currents**; **no joint torque sensor / no wrist F/T**. **Currents→torques** via known constants (K_t=\mathrm{diag}(216,216,216,39.6,39.6,39.6),\mathrm{Nm/A}). 

- **Method core:**

  * **Stage 1 (LS):** RBD linearization ( \tau=Y_b(q,\dot q,\ddot q),p_b ); **Fourier-optimized excitation**; solve (p_b=(W^\top W)^{-1}W^\top\tau). 
  * **Stage 2 (PINNs):** Learn **torque residual** ( \tau_{\text{res}}=\tau_{\text{obs}}-Y_b p_b ) with input (x=[q,\dot q,\ddot q]). **Dual loss:** (L=(1-\alpha)L_{\text{data}}+\alpha L_{\text{physics}}) with **learnable** (\alpha\in(0,1)); **physics loss** enforces (M(q)\ddot q+C(q,\dot q)\dot q+G(q)+F_f(\dot q)). **Friction:** (F_f=\mu_c\tanh(K_f\dot q)+\mu_v\dot q) (smooth). 

- **Key result:**

  * **Torque RMSE reductions on high-friction joints:** J2 **↓65.3%** (6.636→2.306 Nm), J3 **↓42.2%** (3.205→1.854), J1 **↓18.1%** (5.047→4.134). **Limited gains** on viscous-dominant joints (J4–J6). 

- **Strengths:** Keeps **physical consistency** via PINNs while learning complex friction; **adaptive physics weighting** avoids over/under-constraining; works with **currents (no torque sensor/F/T)**; **faster convergence** from LS init. [Q5_1] 
**Weaknesses / assumptions:** **Offline** training; needs **excitation logs** and valid **current→torque** map; benefits are **joint-dependent** (less on viscous-only joints); hyperparameters (PINN, (\alpha), friction slope) require tuning. [Q5_1] 
- **Notes:** Derives (M,C,G) reconstruction from (Y_b p_b); discusses **loss-landscape issues** in PINNs and mitigations (smooth friction, adaptive weighting). Figures show per-joint RMSE bars and time traces. [Q5_1] 

- **Problem statement (paper’s own):** **LS/CLS/EKF** struggle with **nonlinear friction**; pure DL can be **unphysical** and data-hungry. Need a **hybrid, physics-informed** ID that **retains priors** yet **captures friction** smoothly. [Q5_1] 
- **Context / Use case:** Improve **feedforward inverse dynamics** and monitoring on arms **without torque sensors** using only **encoders + currents**, especially where **Coulomb/Stribeck-like** effects dominate. [Q5_1] 
- **SoA / Contribution:** Proposes **residual-driven decomposed PINNs** (LS→PINNs with **adaptive physics** and **tanh friction**) that **substantially lowers torque RMSE** on high-friction joints vs LS baseline. [Q5_1] 

---

### Q5_2 — *Provably-Safe, Online System Identification* (arXiv, 2025; UMich ROAHM Lab)

- **Task:** **Online PDPI** (end-effector/payload inertial parameters) with **provable safety**: generates **locally exciting** yet **constraint-satisfying** trajectories and computes **rigorous interval bounds** on mass/CoM/inertia under bounded sensor noise. 
- **Setting:** **Kinova Gen3 (7-DoF)** hardware; manipulation with **unknown dumbbell payloads** near design limits; **obstacle avoidance**, **joint/velocity/torque limits** enforced during ID and task execution. 
- **Sensors/Data:** Joint **q, q̇**, applied **τ** (bounded noise); **no acceleration estimates** (uses momentum-based regressors/integration); manufacturer/nominal link params with bounded intervals. 

- **Method core:**

  * **Safe planner/controller (ARMOUR):** receding-horizon trajectory optimization that guarantees **collision-free** tracking within limits for all θ in interval **[θ]**; cost promotes **excitation** via **cond(Wₑ) minimization** (end-effector columns of standard regressor). 
  * **Robust ID:** momentum-based linear regressors + **log-Cholesky** parameterization to **enforce physical consistency** and avoid accelerations; **perturbation analysis** yields **provably over-approximative interval bounds** on θₑ given bounded torque noise. 
  * **Closed-loop loop:** alternate safe excitation ↔ ID updates (≈ **4** trajectories; ~**7.5 s** ID phase) and tighten bounds iteratively. 

- **Key result:**

  * **Only the proposed method** finishes **all 3 hardware tasks** (two pick-and-place/stack with obstacles, one tight obstacle-avoidance path) across **five trials each**; baselines fail via torque-limit violation, collisions, or misplacement. 
  * Excitation design **reduces regressor condition number** vs random trajectories (e.g., **274 vs 480** for 4 lb payload), yielding **tighter interval bounds** on inertial parameters. 

- **Strengths:** **Provable safety** during ID (joint/velocity/torque/obstacle); **no acceleration estimation**; **physically consistent** parameters via log-Cholesky; **tight interval bounds** with bounded-noise guarantees; **hardware-validated**. [Q5_2] 
- **Weaknesses / assumptions:** Assumes **bounded torque-sensor noise** and known obstacle set; planner currently finds **locally** exciting trajectories (Bezier/RH); interval method relies on **forward-Euler** discretization and nominal model intervals. [Q5_2] 
- **Notes:** Open-source code; comparisons vs adaptive control, gravity-PID, random excitation; details **Algorithm 1/2**, theorems for momentum regressor, sensitivity/perturbation, and **log-Cholesky diffeomorphism**. [Q5_2] 

- **Problem statement (paper’s own):** Online PDPI typically ignores **safety** during data collection (collisions/torque limits) and lacks **rigorous uncertainty** handling; need **safe exciting trajectories** + **robust ID** giving **guaranteed bounds**. [Q5_2] 
- **Context / Use case:** **Cobot/industrial** manipulation with **unknown payloads** in clutter, where you must **identify mass/CoM/inertia online** without violating limits. [Q5_2] 
- **SoA / Contribution:** First **integrated framework** to couple **provably-safe** excitation/trajectory planning with **physically consistent, interval-bounded** online PDPI; **outperforms** strong baselines on **real hardware**. [Q5_2] 

---