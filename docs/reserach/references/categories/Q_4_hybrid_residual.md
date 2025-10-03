## 🔎 Q4 -  Hybrid / Residual

### Q4_1 — *Sensorless force estimation for industrial robots using disturbance observer and neural learning of friction approximation* (RCIM, 2021)

- **Task:** **Sensorless external force estimation** on **heavy-duty industrial robots** by combining (i) **model-based ID** (incl. Stribeck friction), (ii) **NN friction approximation**, and (iii) a **Disturbance Kalman Filter (DKF)** that fuses a **momentum observer** with a **disturbance observer**. 
- **Setting:** **KUKA KR 6 R700 sixx (6-DoF)**; controller cycle ≈ **12 ms**; tests in **free motion**, **with external load**, and **sensor-based validation** using an **MCS10** multi-axis sensor (for ground truth only). 
- **Sensors/Data:** Controller joint **position** and **motor torque**; no wrist F/T in runtime (only for a validation experiment). Uses **(M,C,G,J)** from the robot model; velocities/accelerations from differentiated/filtered positions. 

- **Method core:**

  * **Model ID:** per-joint **Stribeck** friction parameters via SQP; dynamic parameters via LS on periodic trajectories (Butterworth-filtered ( \ddot q )). 
  * **NN friction learning:** 3-layer FFNN trained on **momentum-observer residuals** in free motion to approximate friction better than Stribeck (e.g., Joint-3 friction MSE **4.97 vs 14.67 Nm²**; max abs. error **10.5 vs 25.6 Nm** near direction changes). 
  * **DKF pipeline:** linear disturbance model merged with **generalized momentum** dynamics; **covariances calibrated** from measured **velocity noise** (Gaussian check) and NN residual MSE; map joint torques to EE wrench via **(J^{\dagger})**. 

- **Key result:**

  * **Free motion (no external force):** DKF lowest error (e.g., Joint-3 torque MSE **0.0176 Nm²** vs **7.11** MO and **8.26** model-based). 
  * **With external load:** DKF best on Joint-3 (**1.1705** vs **4.76/4.86 Nm²**) and Joint-2 (**8.86** vs **11.67/11.65 Nm²**). Peak errors much lower with DKF. 
  * **Sensor-based validation (MCS10):** DKF **3.24 Nm²** vs momentum **5.72** and model-based **11.83 Nm²**; quick steady-state recovery. 

- **Strengths:** Designed for **heavy-duty, no-F/T** robots; **NN friction** strongly reduces uncertainty; **covariance calibration** (meas./process) gives **robust DKF**; consistent wins across scenarios. [Q4_1] 
- **Weaknesses / assumptions:** Needs **initial parameter ID** and **NN training data**; assumes **Gaussian** noise; relies on accurate **(J)** and model terms; NN architecture/gains chosen by **trial-and-error**; single robot case. [Q4_1] 
- **Notes:** Provides full Stribeck ID, NN training splits, noise PDF checks, DKF discretization via matrix exponentials, and comprehensive MSE/peak-error comparisons. [Q4_1] 

- **Problem statement (paper’s own):** Classical model-based/momentum observers are **sensitive to friction and model uncertainty**; wrist F/Ts are impractical on heavy robots. Need **sensorless** estimation robust to **nonlinear friction** and **measurement noise**. [Q4_1] 
- **Context / Use case:** **Industrial HRC** and force-controlled manufacturing on heavy robots lacking wrist F/T; desire accurate **contact awareness** for safety and control. [Q4_1] 
- **SoA / Contribution:** A **hybrid observer**: model ID → **NN friction** → **DKF (momentum+disturbance)** with **noise calibration**, achieving **state-of-practice** sensorless force estimation on a **6-DoF KUKA**. [Q4_1] 

---

### Q4_2 — *Adaptive Neural Trajectory Tracking Control for n-DOF Robotic Manipulators With State Constraints* (IEEE T-II, 2023)

- **Task:** **Trajectory tracking under uncertainties + disturbances with state constraints**, via **Computed-Torque Control (CTC)** + **RBFNN uncertainty approximation** + **Nonlinear Disturbance Observer (NDO)** + **Barrier Lyapunov Functions (BLF)**; **stability (UUB) proven**. 
- **Setting:** **n-DoF framework**, validated in **simulation on a 7-DoF arm**; compares vs three ablations (CTC+RBFNN+NDO, CTC+RBFNN+BLF, RBFNN+NDO+BLF). 
- **Sensors/Data:** Joint **(q,\dot q,\ddot q)**; controller torques **(\tau)**; nominal model terms **(M,C,G,J)**; **no wrist F/T** (disturbances estimated sensorlessly). 

- **Method core:**

  * **CTC** reduces nonlinearity using **nominal** dynamics. **RBFNNs** approximate lumped uncertainties ( \Delta f + \delta ).
  * **NDO** estimates **RBFNN approximation error + external disturbances** (D); design uses auxiliary state (S) and gains (k_s).
  * **BLF + backstepping** enforce **state constraints** on (z_1, z_2), with adaptive weight update law; **UUB** stability derived.
  * **Sensitivity analysis** ranks controller parameters’ influence (e.g., (k_1,k_2,k_s)). 

- **Key result:** Faster **convergence** and **lower IAE/ITAE** than ablations; all trajectories **respect constraints**; analysis highlights **(k_1,k_2)** strongly shaping performance, **(k_s)** impactful for later joints in 7-DoF case. 

- **Strengths:** **Explicit constraint handling** (BLF) with **provable UUB**; **sensorless disturbance estimation** via NDO; practical guidance via **parameter sensitivity**. [Q4_2] 
- **Weaknesses / assumptions:** **Simulation-only** validation; requires **nominal model**, **many gains/hyper-params**; assumes **bounded, continuous disturbances** and measurable states. [Q4_2] 
- **Notes:** Details RBFNN structure/centers from constraint bounds, adaptive laws, BLF forms, and S-curve trajectory planning used in tests. [Q4_2] 

- **Problem statement (paper’s own):** High-DoF manipulators face **parameter variations, unknown nonlinearities, and time-varying disturbances**; controllers must **track without violating constraints** and stay **robust** without precise models. [Q4_2] 
- **Context / Use case:** Safety-critical **trajectory tracking with state limits** (workspace/collision limits) where **wrist F/T is unavailable** and disturbances must be **estimated sensorlessly**. [Q4_2] 
- **SoA / Contribution:** Integrates **CTC + RBFNN + NDO + BLF** into a single scheme with **stability proof** and **parameter-sensitivity guidance** for n-DoF robots. [Q4_2] 

---

### Q4_3 — *Robot Hybrid Inverse Dynamics Model Compensation Method Based on the BLL Residual Prediction Algorithm* (Robotica, 2025)

- **Task:** **Hybrid inverse dynamics residual compensation** for feedforward control using a **Bagging–LSTM–Linear (BLL)** residual predictor added to **RBD + friction** (Coulomb/viscous) on the **Franka Panda** public dataset. 
- **Setting:** **Franka Emika Panda (7-DoF)**; residuals built from **Newton–Euler** torques vs measured torques; training with **bootstrap ensembles** of LSTMs; **linear layer** trained post-ensemble for residual correction; extensive per-joint metrics and ablations vs **LSTM** and **GP**. 
- **Sensors/Data:** Joint (q,\dot q,\ddot q) (acceleration from differentiated velocity), controller torques; **no wrist F/T at runtime**; nominal (M,C,G) from RBD; friction ( \xi=f_v\dot q + f_c\mathrm{sign}(\dot q) + \epsilon ). 

- **Method core:**

  * **Hybrid torque model:** ( \tau = f_{\text{RBD}}(q,\dot q,\ddot q) + f_F(\dot q) + f_{\text{BLL}}(q,\dot q,\ddot q;\theta) ). 
  * **BLL residual prediction:** (1) **Bagging**: train (n) **independent LSTMs** on bootstrap samples of residuals; (2) **Average** their predictions; (3) **Linear layer** (fully connected) trained afterward on the **validation set average** to **optimize remaining residuals**. 

- **Key result:**

  * Average joint-torque **residual reduced from 0.5651 Nm to 0.1096 Nm** after compensation (≈**80%** reduction). Per-joint **R² ≈ 0.84–0.99**; joint-wise MSE/RMSE/MAE close to zero. **BLL beats LSTM and GP** across MSE, RMSE, MAE, R²; e.g., vs LSTM: mean improvements **MSE +19.23%**, **RMSE +12.68%**, **MAE +21.69%**, **R² +5.84%**; vs GP much larger. 

- **Strengths:** Simple **residual add-on** to RBD; **ensemble LSTM** curbs overfitting; **post-hoc linear** layer tightens fit; **substantial residual reduction** and better metrics than **LSTM/GP** baselines. [Q4_3] 
- **Weaknesses / assumptions:** **Offline training**; **single robot/dataset**; assumes decent (M,C,G) and friction baseline; bagging + LSTM + linear adds tuning complexity; torque only (no explicit PDPI). [Q4_3] 
- **Notes:** Provides equations for BLL pipeline, training hyper-params, and comprehensive tables (per-joint MSE/RMSE/MAE/R²) and comparisons vs **LSTM/GP**; uses Gaz et al. Panda ID dataset. [Q4_3] 

- **Problem statement (paper’s own):** Pure RBD with simple friction leaves **non-modeled residuals** (temperature-dependent friction, flexibility, clearances), hurting feedforward control. Need a **data-efficient residual learner** that improves inverse dynamics without wrist F/T. [Q4_3] 
- **Context / Use case:** **Industrial feedforward control** on collaborative arms where **accurate torque prediction** is needed but sensor costs and modeling gaps preclude perfect RBD. [Q4_3] 
- **SoA / Contribution:** Introduces **BLL** (bagging LSTM + linear) as a **residual compensation** block that **significantly shrinks torque residuals** and **outperforms LSTM/GP** baselines on Panda data. [Q4_3] 

---

### XX. **Programmatic Imitation Learning from Unlabeled and Noisy Demonstrations (PLUNDER)** (RA-L, 2024)

* **find in 'Peripheral/Noise #2'**

---
