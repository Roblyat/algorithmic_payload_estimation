## 🔎 Q3 -  Deep Sequence Models (MLP / GRU / TCN / Transformer)

### Q3_1 — *Extended Deep Lagrangian Network for Robotic Arm Dynamics Considering Motor Couplings* (IEEE, 2025)

- **Task:** **Physics-informed inverse dynamics learning** with **motor coupling & friction**, using **motor currents** (no torque sensors) — an enhanced **DeLaN-Motor** model. 
- **Setting:** **UR10e**, simulation (URSim) **and** real robot; **300 trajectories**, **85k samples** (80k train / 5k test); compares **FFNN**, **DeLaN**, and **DeLaN-Motor**. 
- **Sensors/Data:** (q,\dot q,\ddot q) + **motor current** (i_m) (no wrist F/T); predicts **motor current** (surrogate for torque) with **Mahalanobis-weighted** loss (per-joint covariance). Uses **sin/cos** angle features. 

- **Method core:**

  * **Actuator model:** (\tau \approx k_t r, i_m - \tau_F) (motor inertia negligible under high ratio/low accel). 
  * **Friction model (harmonic drive):** asymmetric smooth Coulomb with power-flow direction (dir/rev) + **tanh** smoothing near zero velocity; mapped through gear ratio. 
  * **DeLaN-Motor:** embeds Lagrangian structure (H(q), C(q,\dot q), G(q)) and augments with actuator/friction; network outputs (\hat i_m); trains with covariance-normalized loss. 

- **Key result:**

  * **Simulation (ideal, no friction):** DeLaN best (R^2 \approx 0.9997); DeLaN-Motor close; FFNN worse. 
  * **Real robot (with friction):** **DeLaN-Motor (R^2=0.9727)**; **DeLaN (R^2=0.2652)**; **FFNN (R^2=0.8683)**. Per-joint MSEs show large DeLaN errors on joints 0–1; DeLaN-Motor sharply reduces them. 

- **Strengths:** Adds **motor/friction physics** to DeLaN; **works without torque sensors** (uses currents); **big real-world gain** vs DeLaN/FFNN; maintains **physical structure** (PD inertia). [Q3_1] 
- **Weaknesses / assumptions:** Needs **accurate current sensing** and valid **current→torque region**; **friction form** fixed (may mis-model complex effects); predicts **currents (not torques)**; single platform (UR10e). [Q3_1] 
- **Notes:** Provides full architectures/equations (actuator, friction, DeLaN-Motor), dataset details, and tables with joint-wise MSE & overall (R^2). [Q3_1] 

- **Problem statement (paper’s own):** Classical PINNs/DeLaN often **assume torque data** and **neglect motor friction**, harming realism on high-ratio arms; need a **friction-inclusive, actuator-aware** physics-informed model using **currents**. [Q3_1] 
- **Context / Use case:** **Model-based control** and monitoring on cobots **without torque sensors**, where **harmonic-drive friction** is prominent. [Q3_1] 
- **SoA / Contribution:** Extends **DeLaN** with **actuator + friction** modeling (**DeLaN-Motor**), achieving **state-of-practice accuracy** on real UR10e ( (R^2\approx0.973) ) while preserving physical priors and using only **currents + kinematics**. [Q3_1] 

---

### Q3_2 — *Parameter Identification of a Robot Arm Manipulator Based on a Convolutional Neural Network* (IEEE Access, 2022)

- **Task:** **RDPI** for a **2-DoF** arm using a **CNN** that estimates **parameter residuals** from an image built out of robot signals; aims to **avoid trajectory optimization** and compare vs LS. 
- **Setting:** Real **2-DoF** lab arm with **FPGA** PD control; sampling **2.5 ms**; four validation trajectories (filtered random, circle, profile, sinusoid). Synthetic numeric tests + real experiments. 
- **Sensors/Data:** **Joint positions** (encoders), torque inferred from **PWM duty** (no wrist F/T; no torque sensor); velocities/accelerations via **DCT-based** differentiation and filtering. 

- **Method core:**

  * Convert ({\tau, q,\dot q,\ddot q}) + **initial params** (\beta_n) into a **100×100×2 image** (Z) using **DCT-II subsampling** and a pairwise construction that mixes measured torque and model-reconstructed torque; logarithmic scaling emphasizes peaks. 
  * **CNN** (3 conv + 2 pool + 3 FC) outputs **parameter residuals**; inverse-sigmoid recovers residuals; iterate to get (\beta), then **reconstruct torque** with the rigid-body+friction model; accept if **metr2ev** (time+frequency similarity) exceeds a threshold. 

- **Key result:**

  * **Numeric tests:** torque reconstruction **≥ 97.38%** similarity. **Experimental:** reconstruction **≥ 93.55%** and **outperforms LS on validation** trajectories (LS fits ID run better on one joint but **generalizes worse**). 

- **Strengths:** Eliminates **trajectory optimization**, uses **only encoders + PWM torque proxy**, **DCT** pipeline handles differentiation/noise, and shows **better validation** than LS; amenable to **embedded** implementation. [Q3_2] 
- **Weaknesses / assumptions:** Demonstrated on **2-DoF** only; torque proxy from PWM assumes stable mapping; CNN & thresholds need tuning; frequency-domain metric adds complexity; offline training. [Q3_2] 
**Notes:** Details full **Algorithm 1** (ID) and **Algorithm 2** (metr2ev), CNN architecture, DCT derivation/filtering, and FPGA I/O pipeline; provides extensive numeric/experimental tables and spectra. [Q3_2] 

- **Problem statement (paper’s own):** Traditional ID needs **optimal excitation** and **time-consuming** trajectory trials; LS may struggle under friction/noise. Desire **faster, data-driven** parameter ID without trajectory optimization. [Q3_2] 
- **Context / Use case:** Labs/industry wanting **parameter ID without torque/F/T sensors** and minimal experiment design overhead; future **embedded** ID. [Q3_2] 
- **SoA / Contribution:** Introduces a **signal-to-image CNN** that predicts **parameter residuals** from DCT-processed logs, achieving **high reconstruction** and **better validation than LS** on multiple trajectories. [Q3_2] 

---

### Q3_3 — *A Learning-Based Approach for Estimating Inertial Properties of Unknown Objects From Encoder Discrepancies* (RA-L, 2023)

- **Task:** **PDPI** (mass & CoM) **without F/T sensors**, using **encoder discrepancies** + learned joint-torque model and **attention-weighted WLS** closed-form solver. 
- **Setting:** **4-DoF OpenMANIPULATOR-X**; steady-state samples at multiple discrete poses; **camera AprilTag** gives object pose; extensive train/test with known/unknown objects; plus a **continuous trajectory with switching loads** demo. 
- **Sensors/Data:** **Encoders** (q) and desired angles (q_d) (→ **position error**); sign of rotation; **no wrist F/T**; **camera** for tag pose; robot model for (J,,M,C,G). 

- **Method core:**

  * **Torque NN:** predicts joint torques from ([q,; q_d!-!q,; \mathrm{sgn}(\omega)]), aiming to reconstruct external-related torque while mitigating friction. 
  * **Closed-form PDPI:** derive ( \tau-\tau^g = A x ) with (x=[m,; m,p_{\text{COM}}^\top]^\top); stack multiple steady-state samples; solve by **weighted least squares**. 
  * **Attention weights:** per-sample, per-joint **attention model** generates WLS weights to emphasize informative joints (e.g., near EE) and de-emphasize noisy/redundant ones. 

- **Key result:**

  * On **4 novel objects**, method **accurately estimates mass & CoM**, outperforming **sensor-based** and **position-error** baselines; attention further **reduces COM error**. Demonstrates **switching-force** tracking along a trajectory using sliding windows (128 samples). 

- **Strengths:** **No F/T sensors**; analytic **closed-form mapping** from torques to ((m,\mathrm{CoM})); **attention-weighted** WLS improves robustness; works with **steady-state snapshots** and **continuous** runs. [Q3_3] 
- **Weaknesses / assumptions:** Needs **camera/AprilTag** for object frame; assumes **known robot dynamics** and **PD control**; requires **many steady-state samples**; shown on **4-DoF** platform; training for torque & attention models. [Q3_3] 
- **Notes:** Provides full derivation (F=B_i x), ( \tau-\tau^g = A x), training datasets (≈**82k** steady-state samples), and evaluation metrics; repeats identification to average randomness. [Q3_3] 

- **Problem statement (paper’s own):** F/T sensors are **heavy/expensive** for small robots; estimating torques from **encoder discrepancies** is promising but joints contribute **unequally**—need **learned torque** plus **adaptive weighting** to recover object mass/CoM. [Q3_3] 
- **Context / Use case:** **Small/light robots** without F/T; off-line or **quasi-online** PDPI using only built-in encoders and a **cheap camera tag**. [Q3_3] 
- **SoA / Contribution:** First to use **attention** to weight joints for PDPI from encoder discrepancies; delivers **mass & CoM** without F/T by combining a **torque NN** with an **analytic WLS** estimator. [Q3_3] 

---

### Q3_4 — *End-Effector Force and Joint Torque Estimation of a 7-DoF Robotic Manipulator Using Deep Learning* (Electronics, 2021)

- **Task:** **End-effector force estimation** and **joint-torque estimation** using deep nets (**MLP/Conv1D/LSTM**) — **no analytic dynamics required**; uses a **base-mounted F/T sensor** (no wrist payload). 
- **Setting:** **Franka Emika Panda (7-DoF)**; large-scale **simulation** (≈3.92 M samples) + **real-robot** experiments; hyper-param search; comparisons to **DeLaN**. 
- **Sensors/Data:** **Base F/T sensor** (under the robot base) + joint **q, q̇, q̈**; some variants include **joint torques/currents**; **no wrist F/T** for estimation. 

- **Method core:**

  * Train deep nets (MLP/Conv1D/**LSTM**) for (i) **EE force** from base F/T + kinematics and (ii) **inverse dynamics** (torques) from kinematics. **Sequence inputs** (5–10 steps) and optimized LSTM give best accuracy. Benchmarks include **DeLaN** for inverse dynamics. 

- **Key result:**

  * **EE force (sim):** optimized **LSTM RMSE ≈ 0.1533 N**.
  * **Joint torques:** optimized **LSTM RMSE ≈ 0.5115 Nm** (sim); **≈ 0.7778 Nm** (real). LSTM outperforms MLP/Conv and **beats DeLaN when external forces act**; DeLaN is better only in “no-external-force” simulation. 

- **Strengths:** **No wrist F/T** and **no explicit dynamics model**; **real-time feasible** once trained; supports **hybrid sim→real** training; LSTM provides **lowest RMSE** among tested nets. [Q3_4] 
- **Weaknesses / assumptions:** Requires **base F/T hardware** and large, diverse training data; generalization depends on data coverage; some variants rely on **torque/current availability**; simulation-to-real gap; results shown on a **single platform**. [Q3_4] 
- **Notes:** Details dataset sizes/splits, architectures, and RMSE tables; discusses safety and adversarial considerations; includes **joint-torque** comparison vs **DeLaN**. [Q3_4] 

- **Problem statement (paper’s own):** Observer-based force/torque estimation needs accurate dynamics; wrist F/T adds payload/cost. Need **data-driven** estimators that **avoid dynamics modeling** and **wrist sensors**. [Q3_4] 
- **Context / Use case:** **Mobile/compact robots** where wrist F/T is impractical; **force-aware control** and **haptics** with base sensing; **inverse-dynamics feedforward** without analytic ID. [Q3_4] 
- **SoA / Contribution:** Demonstrates **base-sensor + deep learning** pipeline for **EE force** and **torque** estimation, achieving **0.153 N** (sim) and **≤0.78 Nm** (real) LSTM RMSE and surpassing DeLaN under external forces. [Q3_4] 

---

### Q3_5 — *Neural-Learning-Based Force Sensorless Admittance Control for Robots With Input Deadzone* (IEEE T-IE, 2021)

- **Task:** **Sensorless external force/torque estimation** and **admittance control** under **actuator deadzone**, using a **flat RBFNN/RVFLNN** inverse-dynamics observer + **adaptive RBFNN controller** with stability guarantees. 
- **Setting:** **Baxter (7-DoF)** experiments; free-motion data for training; contact interaction with environment modeled as damping–stiffness; tests with and without NN compensation; **no wrist F/T** (used only for ground truth/validation). 
- **Sensors/Data:** Joint (q,\dot q,\ddot q); actuator input (\tau); model terms (M,C,G,J); **no external F/T** in the estimator. 

- **Method core:**

  * **Inverse-dynamics observer:** learns (\tau_{\text{free}}) with **flat RBFNN** (incremental/broad-learning updates) and estimates (\tau_{\text{ext}} = \Psi(q,\dot q,\ddot q)\hat\Pi - \tau); maps to (f_{\text{ext}}=J^{-T}\tau_{\text{ext}}). **Incremental node addition** avoids full retraining; **node discard** for far centers reduces compute. 
  * **Admittance control** (task-space spring-damper) with **joint-space saturation** to enforce motion bounds. 
  * **Adaptive RBFNN controller** compensates uncertainties & **deadzone** nonlinearity; **UUB** stability via Lyapunov proof. 

- **Key result:**

  * **End-effector tracking error < 0.04** with NN compensation; NN-estimated external torque tracks ground truth well on validation/test trajectories. **Training time ~5.088 s; ~1000 NN nodes**, with node-discard halving compute while retaining approximation quality. 

- **Strengths:** **No wrist F/T**; handles **deadzone** and model errors; **incremental learning** (fast updates, no full retrain); **stability guarantees (UUB)**; joint-space **saturation** keeps motions safe. [Q3_5] 
- **Weaknesses / assumptions:** Requires **free-motion training logs** and model terms (M,C,G); performance depends on **NN tuning** and **PE-like excitation**; demonstrated on **single platform**; deadzone form is learned but still platform specific. [Q3_5] 
- **Notes:** Details RVFLNN update, dynamic node management, controller gains, and full Lyapunov analysis; provides plots of tracking, torque estimation, and node effects. [Q3_5] 

- **Problem statement (paper’s own):** Model-based observers degrade with **inaccurate dynamics** and **actuator deadzone**; force sensors are costly and alter dynamics. Need **sensorless** admittance + observer that **learns uncertainties** incrementally and remains **stable**. [Q3_5] 
- **Context / Use case:** **pHRI/compliant manipulation** on platforms without wrist F/T, with actuator nonlinearities; need **online force awareness** and **safe, bounded** motion. [Q3_5] 
- **SoA / Contribution:** Introduces a **flat RBFNN** force observer + **adaptive NN controller** handling **deadzone**, with **incremental learning** and **UUB** guarantees; shows **<0.04** EE tracking error and effective force estimation on **Baxter** without wrist F/T. [Q3_5] 

---

### Q3_6 — *An Adaptive Sparse GRNN-Based Force Observer for Teleoperation System* (Eng. Appl. of AI, 2023)

- **Task:** **Sensorless interaction force estimation** for teleoperation using an **Adaptive Sparse GRNN (ASGRNN)** observer—no accurate dynamic parameters required. 
- **Setting:** **Master–slave teleoperation** (Omega_7 master, **UR5e** slave); **250 Hz** master–slave mapping; **20 Hz** data logging; two environments (**soft foam**, **rigid plastic**). Wrist F/T is used **only for ground truth** during training/validation, not for runtime estimation. 
- **Sensors/Data:** Joint (q,\dot q,\ddot q), **motor currents** (i_a); no wrist F/T at runtime; nominal (D,C,G,J) mentioned for context but **model parameters not required** by ASGRNN. 

- **Method core:**

  * **ASGRNN force observer:** GRNN with (i) **wrapper + L1** feature selection (choose informative components from a 24-dim input; **threshold = 18** found best), (ii) **support-vector pruning** via **fixed pruning rate** or **distance threshold** (≈**98–99%** pruning; lower compute), and (iii) **Improved Ant Lion Optimization (IALO)** to tune **bandwidth** (h) (faster convergence than ALO/AOA/HHO/SO; optimum (h\approx 0.00545)). 

- **Key result:**

  * **Soft env.:** ASGRNN **MSE 0.006293**, **MAE 0.033388**, **(R^2=0.9995)**—MSE **↓82.9% vs GPR**, **↓73.8% vs RF**, **↓43.5% vs MINN**. 
  * **Stiff env.:** ASGRNN **MSE 0.006992**, **MAE 0.036485**, **(R^2=0.9998)**—MSE **↓81.6% vs GPR**, **↓80.5% vs RF**, **↓35.8% vs MINN**. 
  * **Feature selection impact:** from 24→**18** dims: **MSE 0.06854 → 0.00616** (≈**91%** reduction). **Pruning**: ~**99%** SV removal with **no loss** in MSE and lower time. 

- **Strengths:** **No wrist F/T at runtime**, **no accurate dynamics needed**; **automatic feature selection** and **sparse SV set** cut compute; **IALO** speeds/robustifies hyperparameter search; validated in **soft & stiff** contacts. [Q3_6] 
- **Weaknesses / assumptions:** Requires **offline data** with ground-truth forces for training; performance tied to **feature threshold** and **bandwidth tuning**; trained on **single platform**; coverage of workspace and contact modes must be sufficient. [Q3_6] 
- **Notes:** Discusses sensor removal & **compensating mass** to keep dynamics consistent post-training; compares against **GPR**, **MINN**, **RF**; details **IALO** (new random walk, weighted update, opposition-based learning). [Q3_6] 

- **Problem statement (paper’s own):** Force sensors are **bulky/costly/unusable** in many teleop settings; **model-based** observers need accurate dynamics and suffer near singularities. Need a **model-light, sensorless** estimator with **feature sparsity** and **fast hyperparameter search**. [Q3_6] 
- **Context / Use case:** **Teleoperation** (surgery, nuclear cleanup) where wrist F/T is impractical; desire **accurate, low-cost** force feedback from joint signals/currents. [Q3_6] 
- **SoA / Contribution:** Proposes **ASGRNN** with **sparse features + SV pruning + IALO**; achieves **state-of-practice accuracy** without runtime F/T and **beats GPR/MINN/RF** across soft/stiff contacts. [Q3_6] 

---