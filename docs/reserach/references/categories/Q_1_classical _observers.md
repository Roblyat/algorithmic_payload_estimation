## 🔎 Q1 - Classical / Observers

### Q1_1 — *Fast Object Inertial Parameter Identification for Collaborative Robots* (ICRA, 2022)

- **Task:** PDPI (object inertial parameters: mass, CoM, inertia). 
- **Setting:** Short-batch ID for cobots under safe, low-SNR motions; typical trajectory ≈1–3 s (~150 samples). Real robot: uFactory xArm 7 + Robotiq FT-300; requires object **shape/pose** (via vision or prior). 
- **Sensors/Data:** End-effector force-torque; joint-derived (ω, α) / accelerations; optional vision for shape; velocities/accelerations may be noisy; gravity dominates at cobot speeds. 

- **Method core:**

  * Approximate rigid-body dynamics to **boost SNR** in safe (slow) regimes;
  * **Point-Mass Discretization (PMD)** of object shape;
  * **Weighted mixing** of reduced (gravity-dominant) vs full model using motion “dynamism” (ν) via a tanh weighting;
  * Convex program with non-negative masses + L2 regularization; yields **physically consistent** parameters; single scalar hyperparameter (c_1). 

- **Key result:**

  * **Simulations (moderate noise, ~150 obs):** COM error ~10–16%; inertia error ~44% (better than OLS/RTLS; GEO competitive but needs good init). 
  * **Real xArm 7 (1.5 s):** PMD-25K avg errors: **mass 4.45%**, **COM 4.50%**, **inertia 29.11%**; OLS/RTLS/GEO often yield implausible or much worse inertia. 

- **Strengths:** Robust at **low SNR** and **short time budgets**; preserves physical consistency; fast convex solve; complements classical methods in cobot regime. 

- **Weaknesses / assumptions:** Needs **object shape/pose**; F/T bias/drift still relevant; reduced model alone loses full inertia identifiability (hence weighting); not true streaming “online” (short-batch). 

- **Notes:** Provides code/supplementary; defines scale-invariant error metrics; ISO TCP speed bound context; ADL stats underlying slow-motion assumption. 

- **Problem statement (paper’s own):** Cobots operate with kinematic/safety limits → **low SNR** (vel, acc, F/T), making existing ID **slow/inaccurate**; need **fast** inertial parameter ID suited to cobots. 

- **Context / Use case (paper’s scope):** Collaborative manipulation in human-centric settings; quick object model before/while task execution; typical durations ≲ 2–3 s; gravity-dominant regime. 

- **SoA / Contribution (paper’s delta):** Introduces **PMD + SNR-aware weighting** to accelerate/robustify ID in cobot regime; maintains **physical consistency**; outperforms OLS/RTLS and rivals GEO without strong init, especially at low SNR and short horizons. 

---

### Q1_2 — *The Sum of Its Parts: Visual Part Segmentation for Inertial Parameter Identification of Manipulated Objects* (ICRA, 2023)

- **Task:** PDPI (mass, COM, inertia) for manipulated objects; cobot-safe regime. 
- **Setting:** RGB-D + F/T; **stop-and-go** or very slow motions for safety; demo on uFactory xArm 7 with RealSense D435 + Robotiq FT-300; 20-tool dataset. 
- **Sensors/Data:** Wrist F/T; RGB-D point clouds → mesh → **volumetric** tetrahedralization; reduced dependence on precise kinematics during stalls. 

- **Method core:**

  * **HPS (Homogeneous Part Segmentation)**: assume object = few homogeneous-density parts; estimate part masses → recover full inertial params. 
  * **Two-stage segmentation**: (1) fast surface clustering (boundary-preserving) + (2) **Hierarchical Tetrahedra Clustering (HTC)** for volumetric parts; much faster than HTC alone. 
  * **Convex LS** with non-negativity on part masses; uses **stall (stop-and-go) timesteps** to cut noise; near real-time pipeline (ID ≈ 0.5 s in demo). 

- **Key result:**

  * **Segmentation speed-up:** HTC w/ initial clustering ~**3.48 s** vs **9.73 s**; similar USE/GCE quality. 
  * **Noisy identification:** HPS keeps **100% physical consistency**; lower COM/inertia error than OLS (often inconsistent) and GEO (needs strong prior). Example means (Low noise): HPS ēm **0.40%**, ēC **0.32%**, ēJ **11.12%**; OLS inconsistent; GEO inertia ~**48–53%**. 
  * **Real demo:** autonomous **hammer balancing** using estimated COM after short acquisition; full pipeline runs fast on cobot hardware. 

- **Strengths:** Works with **safe slow/stop-and-go** motions; robust to noise; **physically consistent**; near real-time; open-source code + dataset. [Q1_2] 
- **Weaknesses / assumptions:** Requires RGB-D shape reconstruction + **part segmentation**; relies on **homogeneous-part** assumption; may fail if parts with different densities are merged; still **short-batch** (not continuous streaming). [Q1_2] 
- **Notes:** Clear analysis of when over/under-segmentation does/doesn’t hurt ID; discusses coplanar centroids causing mass “zeroing.” [Q1_2] 

- **Problem statement (paper’s own):** Estimate **full** inertial parameters **safely** for cobots where low SNR makes fast-motion methods unsafe/unsuitable. [Q1_2] 
- **Context / Use case (paper’s scope):** Human-centric cobot settings; tool manipulation; quick object understanding before/while tasks. [Q1_2] 
- **SoA / Contribution (paper’s delta):** Combines **vision + F/T** with **volumetric** part segmentation to enable accurate PDPI under slow motions; provides **20-object dataset**; outperforms OLS and GEO at realistic noise levels; runs near real-time on cobots. [Q1_2] 

---

### Q1_3 — *Online Payload Identification for Tactile Robots Using the Momentum Observer* (ICRA, 2022)

- **Task:** PDPI (mass, CoM, inertia) — **online during task execution**. 
- **Setting:** Franka Emika Panda (tactile robot with joint torque sensing); proprioceptive-only (no wrist F/T, no RGB-D); trajectories with superposed sinusoids; evaluated under temporal/spatial motion changes. 
- **Sensors/Data:** Joint torques/positions/velocities; momentum observer (MO) estimates external joint torques; **filtered** kinematics to align with MO phase lag; joint accelerations derived from filtered velocities. 

- **Method core:**

  * Integrates **momentum observer** torque estimates into **RLS** PDPI. 
  * **Observer-matched filter** to time-align and denoise kinematics (matches MO’s first-order dynamics; gain (k_O) tuned offline). 
  * **Calibration without identical motions:** introduce a **virtual calibration object** (mean parameters from a prior calibration run) to subtract unmodeled dynamics; robust to **temporal/spatial misalignment**. 

- **Key result:**

  * Filtering yields **lower errors** (esp. CoM); **RTLS** offers little/no gain over **RLS** once signals are properly filtered. 
  * **Virtual calibration** remains accurate under phase shifts and time-stretching; timestepwise subtraction (needs identical motions) degrades notably for inertia. 

- **Strengths:** **Online**, proprioceptive-only; robust to modest trajectory deviations; avoids extra sensors; lower complexity than RTLS after filtering. 
- **Weaknesses / assumptions:** Requires joint-torque sensing + reasonably accurate robot dynamics for MO; performance depends on (k_O) tuning and a calibration step; still needs some excitation; very low motion reduces identifiability. 
- **Notes:** Uses Panda + Franka hand (ground-truth inertial params provided); trajectories with 8 superposed sinusoids; discusses bias/drift and friction/backlash as unmodeled torques lumped in (\tau_{\text{ext}}). 

- **Problem statement (paper’s own):** Lack of an **intuitive online** inertial payload ID method usable **while executing a task**; current approaches often need identical calibration motions or extra sensors. 
- **Context / Use case (paper’s scope):** **pHRI/cobot** settings where safety limits velocity/acceleration; need **on-the-fly** PDPI for compensation, accuracy, and collision safety. 
- **SoA / Contribution (paper’s delta):** MO+RLS with **observer-matched filtering** + **virtual calibration** enables **online** PDPI from **proprioception only**, robust to motion misalignments; **surpasses common LS baselines**; obviates RTLS under filtered signals. 
---

### Q1_4 — *External Torque Estimation Using Higher Order Sliding-Mode Observer for Robot Manipulators* (IEEE/ASME T-Mech, 2022)

**Task:** External torque/force estimation (sensorless contact awareness; supports safety/compensation, not direct PDPI). 
**Setting:** 7-DOF Rethink **Sawyer**; simulations + experiments with injected torques; no external F/T needed (built-in F/T used only for validation/force comparison). 
**Sensors/Data:** Joint position/velocity/torque; robot model (M,C,G); Jacobian for wrench mapping. 

**Method core:**

* Rewrite dynamics in **generalized momentum** form; design **higher-order sliding-mode** (modified super-twisting) observer to reconstruct unknown external torque under **bounded nonlinear friction**.
* Add **Luenberger observer**; synthesize gain via **SOS/H∞** conditions to stabilize momenta error; combine with SM injection; **finite-time** convergence analysis.
* Map estimated joint torques to **end-effector wrench** via (F̂_{ext}=(JJ^T)^{-1}J,τ̂_{ext}). 

**Key result:**

* Accurate reconstruction of applied torques (e.g., 10 N·m @ J2, 5 N·m @ J5) in sim/exp; **robust** under 5–10% model error and noise; with **L-gain** the oscillations drop markedly.
* **Beats ESO** baseline (tuning-sensitive) in experiments; end-effector force estimates **match force sensor** trends. 

**Strengths:** **Sensorless** contact awareness; robust to bounded friction/model uncertainties; **finite-time** guarantees; validated on real 7-DOF arm; improves over ESO; straightforward PD feed-forward compensation. [Q1_4] 

**Weaknesses / assumptions:** Requires accurate/available **M,C,G** (gravity assumed accurate), **sector-bounded friction** (joint-wise), **joint-torque sensing**, and **gain tuning** (K₁,K₂,K₃, L via SOS); mapping to wrench depends on **J** quality; not PDPI. [Q1_4] 

**Notes:** Includes detailed Lyapunov/SOS derivations; shows **with/without L** comparison; discusses noise/model-error tests; provides end-effector force validation. [Q1_4] 

**Problem statement (paper’s own):** Achieve **robust, fast, sensorless** estimation of **external joint torques** for manipulators interacting with environments, despite **nonlinear friction** and **model uncertainties**. [Q1_4] 

**Context / Use case (paper’s scope):** pHRI/collaborative tasks (assembly, grasping fragile objects, collision handling) where safe force control and **online contact awareness** are needed without external F/T. [Q1_4] 

**SoA / Contribution (paper’s delta):** HOSM+LO observer with SOS/H∞ synthesis that **outperforms ESO**, shows **finite-time** properties, and works **sensorlessly** (joint-torque based), complementing classical observers used in ID/force control pipelines. [Q1_4] 

---

### Q1_5 — *Contact force and torque sensing for serial manipulator based on an adaptive Kalman filter with variable time period* (RCIM, 2021)

- **Task:** External wrench estimation (sensorless contact **force/torque**); supports safety/impedance, not PDPI. 
- **Setting:** UR5; real experiments with payloads 100–1000 g and speeds 0.04/0.06/0.08 m·s⁻¹; sampling 125 Hz. 
- **Sensors/Data:** **Motor current**, joint position/velocity; no F/T; Jacobian for wrench mapping; identifies motor torque constant and **friction** offline. 

- **Method core:**

  * **Adaptive Kalman Filter (AKF)** = **mode-switching moving average** (WMA/HMA) with **variable time period** → denoise current + set process noise, then **classical KF** on generalized-momentum dynamics (De Luca form). 
  * Discrete state-space with (x=[p^\top\ f^\top]^\top); no acceleration inversion; online update of noise covariances. 

- **Key result:**

  * vs **CKF** baseline: **lower variance/RMSE** for force & torque; **faster response**, especially for payloads ≥ 400 g. 
  * **Variable-t** + WMA/HMA switching improves response while preserving trends; consistent gains across speeds. 

- **Strengths:** Sensorless (no F/T); robust denoising + faster response; avoids acceleration; computationally light generalized-momentum model. [Q1_5] 
- **Weaknesses / assumptions:** Needs accurate (M,C,G) and **J**; **friction model & motor constants** calibrated; assumes contact events spaced > (t_s!\cdot!t) (≈200–400 ms); not PDPI. [Q1_5] 
- **Notes:** Detailed variance/response-time tables; discusses Gaussianity vs averaging span; highlights limitations in high-frequency contact (machining). [Q1_5] 

- **Problem statement (paper’s own):** Improve **sensorless** contact force/torque estimation accuracy **and response time** under noisy motor currents, without F/T sensors. [Q1_5] 
- **Context / Use case (paper’s scope):** Cooperative manufacturing, collision handling, impedance control where **low-cost** sensing and quick response matter. [Q1_5] 
- **SoA / Contribution (paper’s delta):** Introduces **AKF with mode-switching variable-span moving average**, calibrated covariances, and generalized-momentum dynamics → **faster**, **more accurate** sensorless CFT than CKF. [Q1_5] 

---

### Q1_6 — *Accurate Payload Dynamics Estimation and Compensation of a Robotic Manipulator without External Motion Measuring Sensors* (EECR, 2025)

- **Task:** PDPI **+ contact-force compensation** (mass, CoM, inertia → compute non-contact F/T and subtract). 
- **Setting:** xMate ER7 pro + wrist 6D F/T; **no external motion sensors** (no IMU, no vision); online estimation with designed excitation; sampling up to **1 kHz**. 
- **Sensors/Data:** Wrist **6D F/T**; joint positions/velocities; **kinematics (J)**; linear/ang. accel via **2nd-order filter** from kinematics. 

- **Method core:**

  * **Step-by-step decoupling** + **LS**: (i) estimate F/T **offsets** via symmetric static poses (±axes, 60 s holds/axis), (ii) **mass & CoM** from gravity-only statics, (iii) **inertia** from dynamics. 
  * **Velocity/acceleration** from kinematics + **2nd-order filter** (F(s)=\frac{\omega_m^2}{s^2+2\zeta\omega_m s+\omega_m^2}). 
  * **Excitation trajectory design** via finite Fourier series, **fmincon** optimization to **minimize cond**((SV)) under safety/limits constraints. 

- **Key result:**

  * Example payload: ~**1.97 kg**, mass error **9.44%** vs scale; compensation pipeline (offset→gravity→full dynamics) improves contact-F/T estimates; best performance when **both static & dynamic** components are compensated (C3 > C2 > C1). 
  * Acceleration estimates show **delay/oscillation trade-off** at motion reversals (filter tuning effect). 

- **Strengths:** **6D F/T–only**, **no external motion sensors**; online PDPI + contact-F/T compensation; explicit **offset calibration**; **decoupled LS** improves conditioning; **optimized excitation**. [Q1_6] 
- **Weaknesses / assumptions:** Needs good **kinematics/J**; filter-derived accelerations can lag; assumes **quasi-constant F/T offsets** and **contact-free** during estimation; requires informative excitation; **6D F/T mandatory**. [Q1_6] 
- **Notes:** Full Newton–Euler linearization shown; static/dynamic regressors given; detailed offset procedure (three axes, 60 s each). [Q1_6] 

- **Problem statement (paper’s own):** Contact F/T is confounded by **non-contact** dynamics from unknown payload; need **online** payload parameter estimation and **compensation** **without** external motion sensors. [Q1_6] 
- **Context / Use case (paper’s scope):** Industrial/cobot contact tasks with arbitrary payloads where **only wrist 6D F/T** and robot kinematics are available. [Q1_6] 
- **SoA / Contribution (paper’s delta):** A **6D F/T–only** pipeline (offset→mass/CoM→inertia) using **kinematics + 2nd-order filtering** and **optimized excitation**, delivering **online PDPI** and better contact-F/T by subtracting modeled non-contact forces—**no IMU/vision required**. [Q1_6] 

---

### Q1_7 — *On the Fully Decoupled Rigid-Body Dynamics Identification of Serial Industrial Robots* (IEEE T-RO, 2025)

- **Task:** RDPI **and** PDPI (mass/CoM/inertia) — **offline**, via fully-decoupled identification. 
- **Setting:** Industrial 6-DoF **COMAU RACER3** (no joint-torque sensors) and 7-DoF **FLEXIV RIZON4** (with joint-torque sensors); multiple **reciprocating S-curve (RSC)** trajectories; decoupled experiments for joints/links/payload; validation with torque prediction error and payload ground truth. 
- **Sensors/Data:** Actuation/joint torques (sensor torques or motor currents×ratio), joint q, q̇, q̈ (q̈ from filtered q̇), kinematics/J; no external vision/IMU required. 

- **Method core:**

  * Proposes **Fully-Decoupled Rigid-Body Dynamics Identification (FDRDI)** using **RSC** segments: **CV** (velocity), **CA/CD** (±acceleration) pairs to isolate **friction**, **gravity**, and **inertia** effects. 
  * Stepwise pipeline: (1) friction from CV symmetry; (2) link gravity from CV averaging; (3) diagonal inertia (ZZ_i(+I_{Ai})) from CA/CD slopes; (4) off-diagonal inertia from a 2-joint coupling regressor with trajectory optimized for **cond(Φ)** and **‖Φ‖**; (5) payload gravity/inertia from **loaded–unloaded differences** with friction variation canceled by CV symmetry. 
  * Uses **OLS/CLS** once decoupled; argues decoupling > fancy estimators when excitation is sufficient. 

- **Key result:**

  * **Lower torque prediction RMS errors** across velocities vs **CRDI/PDRDI/CLS/IHLS**; **improved payload parameter accuracy** (table & plots). 
  * First report of **full decoupling** enabling independent ID of friction, link gravity/inertia, and payload gravity/inertia; **robust to friction variations** between loaded/unloaded runs via RSC symmetry. 

- **Strengths:** Full **decoupling** → better SNR and conditioning; **trajectory design** ensures per-parameter excitation; **payload ID robust** to friction changes; **no extra sensors** beyond torques/kinematics; open-source trajectory code. [Q1_7] 
- **Weaknesses / assumptions:** **Offline** (not streaming); requires executing multiple **designed RSC** motions with good tracking; depends on accurate kinematics/J and filtering; some steps time-consuming; assumes bounded friction behavior and good symmetry. [Q1_7] 
- **Notes:** Recommends S-curves (finite jerk) over trapezoids; gives isolation formulas (e.g., CV canceling gravity for friction; CA/CD pairing for inertia); includes COMAU/FLEXIV BPS tables and validation methodology. [Q1_7] 

- **Problem statement (paper’s own):** Coupled ID with condition-number-only trajectory optimization can leave **poorly excited** parameters (esp. inertias) and is **sensitive to friction variation** for payload ID. [Q1_7] 
- **Context / Use case:** **Industrial calibration** (pre-task, offline) for accurate dynamics and payload parameters to improve force control and pHRI safety. [Q1_7] 
- **SoA / Contribution:** Introduces **FDRDI + RSC symmetry** to **independently** identify friction, gravity, inertia (links & payload), adding a **norm term** to trajectory design; shows consistent accuracy gains vs CRDI/PDRDI/CLS/IHLS. [Q1_7] 

---

### Q1_8 — *A Two-Stage Payload Dynamic Parameter Identification Method for Interactive Industrial Robots With Large Components* (IEEE T-ASE, 2025)

- **Task:** PDPI (mass, CoM, inertia) for **large, high-payload components**; improves external force sensing for **pHRI assembly**. 
- **Setting:** Industrial platform (KUKA KR60HA + ATI Omega160 6-DoF F/T) on mobile base; **feasible workspace** constraints; safety-aware **static postures** + **dynamic excitation**; payloads up to ~35 kg curved components with suction grippers. 
- **Sensors/Data:** Wrist **6-DoF F/T**, joint states; no external vision/IMU; considers **base inclination errors** and **sensor zero-drift**; online states via KF. 

- **Method core:**

  * **Two-stage ID in sensor frame {S}:**

    1. **Static stage:** estimate base inclination (U,V) and **time-varying F/T zero-drift** + CoM using multiple safe postures (Algorithm 1).
    2. **Dynamic stage:** Newton–Euler **dynamic regressor**; **RRTLS** (Recursive Restricted TLS) for **online, low-cost** updates.
  * **Safety-aware planning:** define **posture coordinate system {P}**, feasible workspace; design **Fourier-series** excitation (fmincon) for joints 4–6 under limits and workspace constraints. 

- **Key result:**

  * **Runtime & setup:** Ntarget = 15 static postures (~2 s each); dynamic ID at **250 Hz** for **10 s**; total ≈ **40 s**. Excitation: N=5 terms, **f₀ = 0.1 Hz**. 
  * **Force-sensing improvement:** vs static-TLS (ignores dynamics) and dynamic-RTLS (ignores base tilt/zero-drift), two-stage method yields **lower external force/torque** after compensation in both high- and low-speed tests; with **secondary zero-drift**, max |F| ≈ **3.03 N**, max |τ| ≈ **1.67 N·m** on low-speed pHRI trajectory. 

- **Strengths:** Tailored to **large, heavy** payloads; **comprehensive error treatment** (base tilt + zero-drift + dynamics); **RRTLS online**; safety-aware posture/trajectory design; demonstrably **better external force sensing**. [Q1_8] 
- **Weaknesses / assumptions:** Needs **6-DoF F/T**, accurate kinematics; careful posture feasibility & tracking; extra time for multi-posture/static stage; assumes quasi-stationary drift within a run; still **not fully streaming** PDPI during arbitrary motion. [Q1_8] 
- **Notes:** Provides detailed static/dynamic formulations in {S}; KF for ω,α; explicit formulas for base tilt recovery; excitation via condition-number objective; compares TLS/RTLS/two-stage (tables/plots). [Q1_8] 

- **Problem statement (paper’s own):** For **high-payload, large components**, prior methods under-model **base tilt**/**zero-drift** or dynamics → poor PDPI → degraded external force sensing; need **comprehensive**, **online**, **safe** identification. [Q1_8] 
- **Context / Use case:** **Human-robot collaborative assembly** (aircraft wing parts, curved large components) with suction gripping, requiring accurate force sensing under safety constraints. [Q1_8] 
- **SoA / Contribution:** Combines **static (tilt & drift)** + **dynamic (inertia) ID** in sensor frame and introduces **RRTLS** for online updates; adds **feasible-workspace** posture planning + **optimal excitation**; improves compensated forces/torques over TLS/RTLS baselines. [Q1_8] 

---

### Q1_9 — *Identifying Current Dynamics of Robot Payload Based on Iterative Weighting Estimation* (IEEE TIM, 2025)

- **Task:** **Payload identification on the current level** (not classic torque-level PDPI): estimate payload **current dynamics** and friction-variation directly from **motor currents**, avoiding torque-constant uncertainty; supports safety features (e.g., collision detection). 
- **Setting:** **UR10**; **125 Hz** RTDE; **15 optimized Fourier** trajectories (10 for ID, 5 for CV); runs twice (unloaded/loaded with rigidly attached payloads of ~3.2–4.1 kg). 
- **Sensors/Data:** **Motor currents + joint positions** (vel/accel via filtering); **no torque sensors, no F/T, no vision**; improved **Stribeck** friction (arctan at zero-vel, nonlinear viscous exponent). 

- **Method core:**

  * Derive **current-level linear regressor** (i = H\chi), then **payload-only** stacked model from **loaded–unloaded current differences** (i_e = H_e\chi_e) with payload current dynamics and **friction-variation** terms. 
  * **Iterative weighting WLS**: (i) covariance-based normalization → WLS; (ii) **outlier masking** via residual threshold (e.g., (\delta=2.5)) to reduce heavy tails; iterate until mask converges. 
  * **Excitation design:** 5-term Fourier per joint; base (f_0=0.1) Hz; vertical pose offsets; low-pass (4th-order Butterworth, 10 Hz) for currents. 

- **Key result:**

  * Across 3 payloads, proposed **current-level** method achieves the **lowest RMSE** for payload current reconstruction vs four torque-based baselines (incl. Khalil-style and double-weighting torque methods). 
  * **Collision detection** case: compensating identified payload current dynamics **reduces residual thresholds** and improves detection reliability. 

- **Strengths:** **No torque constants needed**; robust via covariance normalization + outlier rejection; explicit **nonlinear friction** (continuous at zero vel.); validated across multiple payloads; supports **safety** (collision detection). [Q1_9] 
- **Weaknesses / assumptions:** Requires **matched loaded/unloaded trajectories**; **designed excitation** and decent kinematics/filters; **56 current-level parameters** (less physically interpretable than 10 inertial params); mainly **offline**; assumes linear torque–current relation. [Q1_9] 
- **Notes:** Discusses threshold choice for outlier mask (normal probability plots); warns of **rank deficiency** if too many samples masked. [Q1_9] 

- **Problem statement (paper’s own):** Torque-based PDPI pipelines **inherit errors** from uncertain torque constants; switch to **current-level identification** to reduce cumulative errors. [Q1_9] 
- **Context / Use case:** Industrial robots **without joint torque sensors** needing payload awareness and safer contact/collision monitoring. [Q1_9] 
- **SoA / Contribution:** First **current-dynamics** payload ID with **iterative weighting WLS** + improved friction model; outperforms four torque-level baselines; demonstrates **collision-detection** improvement. [Q1_9] 

---

### Q1_10 — *An accurate identification method based on double weighting for inertial parameters of robot payloads* (Robotica, 2022)

- **Task:** PDPI (mass, CoM, inertia) — **offline**; robustifies payload ID via **double weighting** (covariance normalization + data weighting). 
- **Setting:** **UR10**; motor currents (→ torques via identified joint drive gains (K)), joint positions (vel/acc via filtering); **paired runs** w/ and w/o payload; **Fourier** trajectories (period 10 s), designed for regressor conditioning. 
- **Sensors/Data:** **Motor currents**, joint state; **no wrist F/T**, **no vision**; nonlinear friction model (Stribeck with (|\dot q|^{\alpha}) term). 

- **Method core:**

  * Step 1 (**Dynamic ID, two-loop**): WLS with **covariance update** and **data weighting** (outlier masking (\delta!=!2.5)) in **one loop** + outer loop for **nonlinear friction**; yields **converged** covariance ( \Sigma ) + data weights (P). Faster than 3-loop variants (−70% iterations, −60% time). 
  * Step 2 (**Payload ID, double weighting**): Build **stacked model** for no-payload vs payload runs; **first weight** by ( \Sigma^{-1/2}) (WLS), **second weight** by (P) (outlier-downweighting); include **linear friction variation** due to payload. Solve for the **10 payload inertial parameters**. 

- **Key result:**

  * **Best accuracy** vs four baselines (Swevers-style prior; Khalil diff-traj; Khalil joint WLS; Gaz static-coeff): **mass/CoM errors smallest**; inertia elements closer than most baselines (still hard in lightweight cases). 
  * Two-loop dynamic ID achieves **similar torque RMSE** to prior robust ID but with far **fewer iterations**. 

- **Strengths:** Works **without F/T**; **robust to outliers** (data weighting) and heteroskedastic noise (covariance normalization); models **nonlinear friction**; demonstrates **trajectory design** and practical filtering; improves **mass/CoM** markedly. [Q1_10] 
- **Weaknesses / assumptions:** **Offline**, requires **matched** with/without payload trajectories; depends on accurate **(K)** (current→torque) & kinematics; assumes **linear current–torque** map; still limited for **full inertia** when payload is light; thresholding needs tuning. [Q1_10] 
- **Notes:** Gives UR10 (K) values, DH, filter settings; outlier-mask convergence plots; variance reductions across joints; comparative tables (mass/CoM). [Q1_10] 

- **Problem statement (paper’s own):** Prior PDPI pipelines lack **data weighting**, suffer from outliers and friction variation; online methods lag in accuracy; need a **more accurate** payload ID procedure. [Q1_10] 
- **Context / Use case:** Industrial robots **without torque or F/T sensors** needing reliable **mass/CoM** for planning/safety/collision detection offline. [Q1_10] 
- **SoA / Contribution:** Introduces **double weighting** with a faster **two-loop** dynamic ID and a stacked WLS for payload + **friction variation** → **state-of-the-art mass/CoM** accuracy vs four baselines. [Q1_10] 

---

### Q1_11 — *Payload Identification and Gravity/Inertial Compensation for Six-Dimensional Force/Torque Sensor with a Fast and Robust Trajectory Design Approach* (Sensors, 2022)

- **Task:** PDPI (mass, CoM) + **gravity & inertial compensation** to recover **true contact forces** at the wrist; **fast** identification via designed excitation. 
- **Setting:** **UR10** with a **self-developed 6D F/T sensor** that includes **acceleration & angular-rate sensing**; 10 s identification trajectory; ~100 Hz data; synchronized control/DAQ (8 ms servoj). 
- **Sensors/Data:** 6D **F/T**, integrated **acc/ang-acc** from the sensor, joint states/poses for transforms; no vision/IMU external to the F/T, no torque sensors. 

- **Method core:**

  * Closed-form **least squares** for (i) **F/T zero offsets**, (ii) **payload mass G & CoM (x,y,z)**, (iii) **base mounting inclination** (U,V), using stacked linear equations in sensor frame and multiple poses (**N≥12**). 
  * **Inertial compensation**: use measured linear/angular acceleration to compute (F_{\text{inert}}=m a), (T_{\text{inert}}=J \alpha); subtract from measured F/T (plus gravity) to obtain external contact wrench. 
  * **Excitation trajectory design** (joints 4–6): **finite Fourier series** with constraints; **minimize** (\mathrm{cond}(a)) to reduce parameter sensitivity; start/stop and limits enforced. 

- **Key result:**

  * **End-to-end time ≈ 10 s** for ID; after gravity+inertia compensation, residuals within **≈ 0.5 N** (forces) and **≈ 0.2 N·m** (torques) on varied test paths—better than traditional (≥60 s) gravity-only methods. 

- **Strengths:** Very **fast** PDPI + compensation; **direct inertial sensing** simplifies modeling; explicit recovery of **base tilt** and **sensor zeros**; practical trajectory design with proven improvement over gravity-only compensation. [Q1_11] 
- **Weaknesses / assumptions:** Requires **special 6D F/T with integrated accelerometers**; **offline short run** (not continuous streaming); assumes **rigidly attached** payload and accurate kinematics/transforms; needs **synchronized** DAQ/control; inertia tensor handled via theory/assumption. [Q1_11] 
- **Notes:** Provides full linear systems, pose requirements, details of Fourier trajectory (N=5 terms, (f_0=0.1) Hz), and platform specs; shows compensation results across varied trajectories. [Q1_11] 

- **Problem statement (paper’s own):** Force control needs **accurate contact wrench**; gravity-only methods are slow and insufficient under **high speed/large loads**; need **fast** PDPI + **inertial** compensation. [Q1_11] 
- **Context / Use case:** Industrial **force-controlled** tasks with sizeable payloads where quick calibration is needed between jobs. [Q1_11] 
- **SoA / Contribution:** Introduces a **10 s** excitation-based identification that **jointly** estimates **G, CoM, base tilt, sensor zeros** and applies **inertial compensation** using **on-sensor** acceleration—achieving tighter residuals than longer gravity-only pipelines. [Q1_11] 

---

### Q1_12 — *Composite Disturbance Filtering for Interaction Force Estimation With Online Environmental Stiffness Exploration* (IEEE/ASME T-MECH, 2025)

- **Task:** **Sensorless interaction force estimation** with **online environmental stiffness identification** (no wrist F/T used in estimation). Introduces **EEFO** (EM-Bayesian Environmental Exploration Force Observer) within a **Composite Disturbance Filtering (CDF)** framework. 
- **Setting:** **Surgical robot** platform (Toumai) interacting with **soft tissue–like silicone**; validated via Monte-Carlo simulations (constant & time-varying stiffness) and physical experiments (ground truth from a force sensor used **only for evaluation**). 
- **Sensors/Data:** **Robot dynamics only**: joint states, model terms (M,C,G,J), motor torque input (τ_m); **no direct end-effector force** and **no additional position sensor** at the tool. Force is treated as a **state-coupled disturbance**; stiffness is **unknown & time-varying** and identified **online**. 

- **Method core:**

  * Build a **robot–environment coupled model** combining manipulator momentum dynamics with a **force-generation (constitutive) model** (F = K_θ , δ) (linear, isotropic stiffness), and derive **separability** conditions (observability + FIM identifiability). 
  * **EEFO**: EM iterations around a **KF-like** state estimator; E-step filters the augmented state ([p, ω]), M-step updates stiffness (θ); outputs interaction force (\hat F). Complexity (O(m^3 \times i)). 

- **Key result:**

  * **Best accuracy** among DO, NDO, GMO, KF, and DKF baselines; **ARMSE improves ≥ 28% vs DKF** (next-best). **Faster transients** (e.g., **63.4%** reduction in stabilization time in an experiment phase). 

- **Strengths:** Uses **both** robot dynamics **and** environment model; **no force sensor**; handles **time-varying stiffness**; gives **identifiability** conditions; superior accuracy and noise attenuation in sims/experiments. [Q1_12] 
- **Weaknesses / assumptions:** Assumes **linear, isotropic** stiffness (model bias if violated); depends on **model quality** ((M,C,G,J)); **EM iterations** add compute; needs good synchronization/filters; validated in **1–3D force** settings with note on extending to full wrench. [Q1_12] 
- **Notes:** Provides full augmented system, separability analysis, and EM gradients; discusses robustness to parameter uncertainty and GP-based residual compensation. [Q1_12] 

- **Problem statement (paper’s own):** Observer-only (robot-dynamics-only) approaches neglect **environmental characteristics**; stiffness-ID methods often need **extra force/position sensing**. Need **simultaneous** force estimation and stiffness identification **without additional sensors**. [Q1_12] 
- **Context / Use case:** **Minimally invasive surgery / soft-tissue interaction** needing reliable force awareness without wrist F/T; potential for contact-aware control. [Q1_12] 
- **SoA / Contribution:** Proposes **EEFO (CDF + EM)** for **online** stiffness exploration + **sensorless force estimation**, proves separability conditions, and **outperforms DO/NDO/GMO/KF/DKF** in accuracy & noise attenuation. [Q1_12] 

---

### Q1_13 — *Toward Sensorless Interaction Force Estimation for Industrial Robots Using High-Order Finite-Time Observers* (IEEE T-IE, 2022)

- **Task:** **Sensorless interaction force estimation** (no wrist F/T), targeting **fast time-varying forces**; integrates into collision detection, impedance/drag control. 
- **Setting:** Real 6-axis **industrial robot (ER3A)**; dynamics **identified first** (base parameters) then used in observer; evaluations include tracking, **collision events**, and **sensorless drag** demos. 
- **Sensors/Data:** Joint position/velocity, commanded **motor torque**; derived (M,C,G,f) from identified model; **no end-effector force sensor** in estimation (force sensor only for ground truth). 

- **Method core:**

  * Build inverse dynamics ( \tau = H,\mu ) (base params) via LS on designed excitation; extract (M(q), C(q,\dot q), g(q)), friction (f). 
  * Design **High-Order Finite-Time Observer (HOFTO)** with nonlinear **sig(^m)** terms and order (n) to match force variation class; prove **finite-time convergence** (constant/poly forces) and **bounded-region** convergence for **time-varying** forces; includes stability/identifiability analysis. 
  * Contrast vs **ESO/DOB**: HOFTO removes steady-state bias for fast variations; practical **gain selection** guided by higher-order sliding-mode literature. 

- **Key result:**

  * **Higher precision** than **ESO** and **DOB** in experiments (offset/MA/RMS indices improved); robust even when a joint moves **ultra-slow** (hard case). Successful **collision detection** and **sensorless drag/teaching** demos. 

- **Strengths:** **Finite-time** estimation; targets **fast time-varying** interaction forces; rigorous stability; deployable on standard industrial arm; enables **collision detection** & **impedance/drag** without F/T. [Q1_13] 
- **Weaknesses / assumptions:** Relies on **accurate model ID** ((M,C,G,f)); **gain tuning** vs noise/chattering trade-off; **bounded-region** convergence for general time-varying forces; computation grows with observer order. [Q1_13] 
- **Notes:** Provides proofs, tuning remarks, and comparative plots/tables vs ESO/DOB; discusses chattering sources and mitigation via fractional powers (m\in(0,1)). [Q1_13] 

- **Problem statement (paper’s own):** Direct torque-balance and ESO-type observers struggle with **noise amplification** (double differentiation) and **steady-state errors** for fast-varying forces; need an **accurate, finite-time** sensorless observer. [Q1_13] 
- **Context / Use case:** **Industrial force control** (collision detection, impedance/drag) where **F/T sensors are costly/undesired**. [Q1_13] 
- **SoA / Contribution:** Introduces **HOFTO** for sensorless force estimation with **finite-time** guarantees and **better accuracy** than ESO/DOB; validated on a real 6-axis robot, including **collision** and **drag** tasks. [Q1_13] 

---


### Q1_14 — *Dynamic Model Identification for Industrial Robots* (IEEE Control Systems Magazine, 2007)

- **Task:** **RDPI** (full rigid-body base parameters incl. friction, springs) and **PDPI** (payload mass/CoM/inertia) — **offline**, periodic band-limited identification. 
- **Setting:** **KUKA IR361** (RDPI, 3 links) and **KUKA KR15** (PDPI plug-in); **Orocos** controller for periodic trajectories; typical runs **10–30 s periods**, **150 Hz** sampling, **≥10–16 periods** (≈160 s) for averaging; validation with **pick-and-place/spot-weld-like** trajectories. 
- **Sensors/Data:** **Joint encoders** (q), **motor currents → τ** (via identified/known constants), no wrist F/T, no vision; friction modeled **Coulomb + viscous**; optional gravity-comp springs & rotor inertia terms. 

- **Method core:**

  * **Integrated pipeline**: trajectory design → data acquisition → frequency-domain signal processing → **WLS/ML** estimation → validation. **Periodic band-limited excitation** with **finite Fourier series**, **d-optimal** objective under safety/workspace limits. 
  * **Exact differentiation**: DFT → rectangular window at commanded harmonics → multiply by (j\omega), (-\omega^2) → **noise-robust** (\dot q), (\ddot q). **Period averaging** reduces variance (\propto 1/\sqrt{M}). 
  * **WLS (ML simplified)** since regressor (\Phi(q,\dot q,\ddot q)) is noise-free after processing; covariance carries actuator-torque noise. **Validation** via torque-prediction RMS and confidence bounds. 
  * **PDPI plug-in**: use prior **link model**; form payload regressor (\Phi_L) and re-estimate **wrist-joint friction**; **wrist-dominant excitation** for payload; reference payload CAD for ground truth. 

- **Key result:**

  * Torque-prediction RMS ≈ **noise level** on currents for excitation & validation paths; peaks mainly at **velocity reversals** (friction limits). **Payload accuracy**: mass **≤5%**, CoM **≤1 cm**, inertias within 2σ of CAD reference over 10 trials. 

- **Strengths:** Foundational **periodic ID** with **exact diff.** and **WLS/ML**; clear **experiment design** (d-optimal) and **validation** recipe; **industrial feasibility** (KR15 plug-in); strong torque-prediction accuracy; **PDPI** feasible **without F/T**. [Q1_14] 
- **Weaknesses / assumptions:** **Offline**, requires periodic runs & controller access; **rigid-body** + simple **Coulomb/viscous friction** (errors at reversals); needs accurate **current→torque** map; high harmonics risk **flex mode excitation**; PDPI relies on **prior link ID** and designed wrist excitation. [Q1_14] 
- **Notes:** Presents **barycentric** parameters, payload mapping to classical inertial params, and **confidence-interval** model validation; includes **Fourier** trajectory examples (0.1 Hz, five harmonics; 30 s with 20th/25th harmonic). [Q1_14] 

- **Problem statement (paper’s own):** Accurate **dynamic models** are required for **offline programming**, **task optimization**, and **model-based control**; manufacturers don’t provide full inertial/friction data → need **experimental ID** with strong SNR and tractable estimation. [Q1_14] 
- **Context / Use case:** **Industrial calibration** (shop-floor PDPI before jobs; RDPI in labs) to improve torque feedforward and safety margins under constraints. [Q1_14] 
- **SoA / Contribution:** Canonical **periodic, band-limited** RDPI/PDPI with **DFT-based exact differentiation**, **WLS/ML**, and **d-optimal** excitation; demonstrates **KR15 payload plug-in** accuracy and robust torque prediction on realistic paths. [Q1_14] 

---

### Q1_15 — *A Novel Sliding Mode Momentum Observer for Collaborative Robot Collision Detection* (Machines, 2022)

- **Task:** **Sensorless collision detection** (dynamic **and** quasi-static) via **NSOMO** (Novel Sliding-Mode Momentum Observer) + **TVDT** (Time-Varying Dynamic Threshold). 
- **Setting:** **7-DoF Franka Emika** testbed; simulations + hardware experiments (human–robot collisions on multiple body parts; dynamic impacts; sinusoidal and quasi-static squeezes). 
- **Sensors/Data:** **Proprioceptive only** (joint states, actuation torque/model terms (M,C,G)); **no wrist F/T for estimation** (force/torque used only as ground truth where noted). Uses **offline data** to identify TVDT. 

- **Method core:**

  * Design a **new reaching law (NRL)** and embed it in a sliding-mode momentum observer to form **NSOMO**; proves **finite-time stability** and derives **disturbance stability bounds** on the momentum error. 
  * Build **TVDT** by identifying joint-wise disturbance bounds from offline data; at run-time, declare collision when residual (|r_i|>\delta_i), and infer **contacted joint(s)** from indicator vector. 

- **Key result:**

  * **Faster & smoother** than GM and SOMO: for “sudden impact,” **delay ~0.01 s (NSOMO) vs 0.03 s (SOMO) vs 0.10 s (GM)**; lower RMS error on impact and sine tests; reduced jitter near steady state. Supports **collision localization** and safe stop. 

- **Strengths:** **Finite-time** convergence; **higher bandwidth** with **less chattering**; handles **dynamic & quasi-static** contacts; **time-varying thresholds** reduce false alarms; **localization** of collision joint. [Q1_15] 
- **Weaknesses / assumptions:** Relies on **accurate dynamics** and good **gain tuning**; **TVDT** needs **offline identification** and may be task/robot dependent; convergence bounds assume **bounded** external torque; careful filtering/synchronization required. [Q1_15] 
- **Notes:** Gives reaching-time analysis, Lyapunov proofs, parameter-tuning guidance; compares vs **GM/SOMO/ESO/DOB**; pHRI tests under **ISO/TS 15066** safety lens (discussion). [Q1_15] 

- **Problem statement (paper’s own):** Existing momentum/observer methods trade off **sensitivity, smoothness, noise immunity**; fixed thresholds misclassify; need **fast, robust**, low-jitter sensorless detection with **adaptive thresholds**. [Q1_15] 
- **Context / Use case:** **Cobot safety (pHRI)** where **no external F/T** is preferred; rapid detection for collision response/impedance/drag modes. [Q1_15] 
- **SoA / Contribution:** Proposes **NSOMO+NRL** with **provable** finite-time behavior and **TVDT**; demonstrates **lower delay** and **lower RMS** than GM/SOMO across **impact/sine/quasi-static** cases with **collision localization**. [Q1_15] 

---

### Q1_16 — *Dynamic Parameter Identification of Collaborative Robot Based on WLS-RWPSO Algorithm* (Machines, 2023)

- **Task:** **RDPI** (full rigid-body base parameters + joint friction) — **offline**; two-stage: **Weighted Least Squares (WLS)** initialization → **Random-Weight PSO (RWPSO)** refinement. 
- **Setting:** **6-DoF cobot (ROCR6)**; **5th-order Fourier** excitation (ω₀=0.05 Hz, BW 0.25 Hz); **1 kHz** sampling; ~20 s run (20 000 samples → 2 000 used). 
- **Sensors/Data:** **Joint positions + motor currents** (→ torques via torque constants); velocities/accelerations via differentiation + **Butterworth LP**; **Kalman filter** for torque denoising; **no wrist F/T**, no vision. 

- **Method core:**

  * Lagrange dynamics → **linear regressor** ( \tau = Y(q,\dot q,\ddot q),\theta ) with **Coulomb + viscous** friction (sign approximated by **tanh** near zero) and **minimal base parameters** (rank reduction). 
  * **WLS** solves heteroskedastic torque noise using joint-wise variance weights; provides **initial θ** and covariance. 
  * **RWPSO**: PSO with **random inertia weight** (W=\mu+\sigma \mathcal N(0,1)) to **escape local optima**; convergence/stability shown via Lyapunov/state-transition analysis. 

- **Key result:**

  * **Faster convergence** and **lower RMS residuals** vs **PSO** and **WLS-PSO** on ROCR6; e.g., population converges in ~**20 generations** (RWPSO) vs **83** (PSO); validation RMS lower across all 6 joints. 

- **Strengths:** Practical **offline RDPI** without F/T; **heteroskedastic noise handling** (WLS); **global search** with RWPSO (avoids local minima); **Kalman + LP** preprocessing; documented excitation design and constraints. [Q1_16] 
- **Weaknesses / assumptions:** Needs **accurate current→torque constants**; **designed periodic excitation**; friction limited to **Coulomb/viscous** (no static/Stribeck); **offline** calibration; multiple tuning knobs (PSO hyper-params, filters). [Q1_16] 
- **Notes:** Provides DH, excitation parameterization, weighting formulas, and full algorithm flow; reports joint-wise RMS tables and iteration curves. [Q1_16] 

- **Problem statement (paper’s own):** Classical ID suffers **insufficient prediction accuracy**, **torque fluctuations**, and **error peaks** near reversals due to simplistic friction and noise; need improved accuracy and robustness. [Q1_16] 
- **Context / Use case:** **Cobot calibration** for better model-based control/monitoring **without F/T**; shop-floor periodic runs feasible. [Q1_16] 
- **SoA / Contribution:** Combines **WLS** (heteroskedastic) with **RWPSO** (random-weight global search) + **KF** preprocessing, yielding **lower torque RMS** and **faster convergence** than LS-PSO and WLS-PSO baselines. [Q1_16] 

---

### Q1_17 — *An online payload identification method based on parameter difference for industrial robots* (Robotica, 2024)

- **Task:** **Online PDPI** via a **parameter-difference** formulation: identify robot base parameters **with** payload online (RLS) and subtract **offline** base parameters **without** payload to **algebraically recover** payload mass, CoM, and inertia terms. 
- **Setting:** **UR10** industrial robot; **real hardware** experiments on optimized **Fourier** excitation; application to **manual guidance** with measured user forces. 
- **Sensors/Data:** **Motor currents → joint torques** (via identified drive gains **K**), **commanded** (q_d,\dot q_d,\ddot q_d) from the controller (used to **avoid noisy acceleration derivatives**); **no wrist F/T** or external IMU at runtime. 

- **Method core:**

  * Dynamics with **nonlinear friction** ( \tau_{f,j}=(F_{c,j}+F_{v,j}|{\dot q}*j|^{\alpha_j})\operatorname{sgn}(\dot q_j)+B_j ) to improve identification. **Offline:** identify robot base params ( \pi_a ) and friction ( \alpha_a ). **Online:** RLS on **with-payload** regressor (Y_b(q_d,\dot q_d,\ddot q_d,\alpha_a)) → ( \pi_b ). **Then:** ( \varepsilon=\pi_b-\pi_a ) and a **symbolic linear map** from selected components of ( \varepsilon ) to payload ( \phi_L={m, mr_x, mr_y, mr_z, I*{xx},…,I_{zz}} ). 
  * Trajectory: **Fourier-series** excitation optimized to **reduce regressor condition numbers** under joint limits. 

- **Key result:**

  * **Convergence ~6.59 s** to online solution; **payload mass error ≈ 0.04%**, outperforming classical online/ static baselines (e.g., 4.11% and 0.92%). **Inertia tensor** accuracy remains limited. 
  * **Application:** compensating identified payload reduces **manual-guidance mean forces** by **≈13% (X), 26% (Y), 44% (Z)** and **variance** by **≈52–65%**. 

- **Strengths:** **No external F/T**, uses **commanded signals** to avoid noisy (\ddot q); **nonlinear friction** improves model fit; **fast** online solve; clear algebraic recovery of payload terms; demonstrated **force-reduction** in guidance. [Q6_6] 
- **Weaknesses / assumptions:** Requires **offline robot ID** (base params & friction) and **drive gains K**; needs **optimized excitation** (not arbitrary-task streaming); **inertia tensor** estimates remain **inaccurate**; assumes **rigid joints** and reliable commanded signals access. [Q6_6] 
- **Notes:** Details full RLS loop, excitation design, and **symbolic equations** mapping parameter differences to ( \phi_L ). Compares linear vs **nonlinear friction** (lower RMSE). 

- **Problem statement (paper’s own):** Online payload ID often uses **linear friction** and **actual** (noisy) trajectories → degraded accuracy; need a method that is **accurate online** using **proprioception only**. [Q6_6] 
- **Context / Use case:** Industrial cells picking **frequent, unknown payloads** where controller access to **commanded trajectories** exists and **safe, quick** mass/CoM estimates are needed for **feedforward compensation / guidance comfort**. [Q6_6] 
- **SoA / Contribution:** Introduces **parameter-difference + nonlinear friction + commanded-signal** pipeline yielding **state-of-the-art mass accuracy** in **~6.6 s** without external sensors; shows tangible **ergonomic benefits** in manual guidance. [Q6_6] 

---