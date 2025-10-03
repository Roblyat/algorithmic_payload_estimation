## 🔎 Q₆ Research (Domain Adaptation / Latent Context)

---

### Q6_1 — *Contact Localization for Robot Arms in Motion **without Torque Sensing*** (ICRA, 2021)

- **Task:** **Contact detection & localization** on a **moving** arm **without joint torque sensing**, trained in sim with **Domain Randomization** and using a **cylindrical projection** to turn link surfaces into images for CNNs/T-Convs. 
- **Setting:** **Franka Emika Panda (7-DoF)**; predicts contacts on the last 7 links; trained on **2,800** simulated trajectories; validated in **simulation and real-world** via an **obstacle mapping** task. 
- **Sensors/Data:** **Proprioception only**—joint velocities ( \dot q ), per-link linear & angular velocities, deltas to targets, and **“collision-free”** deltas from a parallel sim; **no torques, no wrist F/T**. Inputs encoded as **32×32** per-link feature images over **5-step windows**. 

- **Method core:**

  * **Cylindrical projection** maps each link mesh to a **UV image**; features rasterized per vertex; a **transposed-conv head** predicts a **contact distance field (CDF)** per link; a parallel head predicts **contact/no-contact** per link. 
  * **Domain Randomization** over dynamics & impedance gains to bridge sim→real; training set is heavily imbalanced (≈**2%** positive), handled via weighted BCE. 

- **Key result:**

  * Overall **contact detection accuracy ~91.5%**; **mean localization error ~3.0 cm** (AMCD), with CDF head yielding **FNR ~10.5%** and balanced AMCD-GT/AMCD-P. Real-world **voxel obstacle mapping** shows comparable performance to sim. 

- **Strengths:** Works **while moving**; **no torque** or wrist F/T; **dense, multi-contact** localization via CDF; **sim→real** transfer from domain randomization; efficient per-link shared weights. [Q6_1] 
- **Weaknesses / assumptions:** Needs **sim pipeline** and link meshes; relies on **feature engineering** (UV projection) and dataset coverage; contact surfaces assumed **rigid/stationary**; resolution limited by 32×32 UV grid. [Q6_1] 
- **Notes:** Provides full architecture (encoder → link-specific processors → heads), mesh prep (TetWild/libigl/MeshLab), metrics (ACC/FNR/FPR, AMCD), and voxel-grid mapping with Bayesian updates. [Q6_1] 

- **Problem statement (paper’s own):** Prior methods often assume **static arm**, **single contact**, accurate **torques/models**. Needed: **multi-contact**, **in-motion** localization from **proprioception alone** that transfers to real robots. [Q6_1] 
- **Context / Use case:** **pHRI & exploration** where **torque sensors** or **robot skins** are unavailable/unreliable; mapping unknown obstacles during motion. [Q6_1] 
- **SoA / Contribution:** Introduces **UV-image CDF prediction** + **domain randomization** to achieve **~3 cm** localization **without torque sensing**, demonstrated in sim and real mapping. [Q6_1] 

---

### Q6_2 — *Payload Parameters Identification Using Incremental Ensemble Learning* (ICCCR, 2024)

- **Task:** **Online PDPI during arbitrary task trajectories (no dedicated excitation)** using an **incremental ensemble** of **single-layer NNs** (weak learners) that adapt on-the-fly when motion leaves the training distribution. 
- **Setting:** **Franka Emika Panda (7-DoF)**; initial training on excitation-path data; deployment on a **new linear joint-space path**; simultaneous **simulator + real cobot** to supply data for incremental updates. 
- **Sensors/Data:** Joint **q, q̇, τ** (controller torques); **no wrist F/T**, **no acceleration**; outputs **10 payload inertial parameters** (m,; m r_x,m r_y,m r_z,; I_{xx},I_{xy},I_{xz},I_{yy},I_{yz},I_{zz}). 

- **Method core:**

  * **Ensemble (5 weak learners):** each a **single hidden-layer (100-neuron) ReLU NN** trained with **SGD**; final estimate = **mean** of learners. 
  * **Distribution check:** compute **Euclidean distance** from new point (q_{\text{new}}) to **subset means**; if **min distance > δ** (max intra-subset radius), trigger **incremental learning** before estimation. 
  * **Three-stage loop:** **(i) classify** in/out of training space → **(ii) incremental update** on the fly (using sim payload set) → **(iii) online estimation** on real cobot sample. 

- **Key result:**

  * On the **new linear path**, incremental ensemble achieves **MAE ≈ 0.01 kg (m)**, **0.0068 kg·m (COM)**, **0.0008 kg·m² (inertia)**, while a **batch ensemble fails** (very large MAEs) on the same path. **However**, after updating for the new path, performance on the **old excitation path** degrades (e.g., mass MAE **0.037 kg** → **catastrophic forgetting**). 

- **Strengths:** Eliminates **dedicated excitation**; **no F/T and no acceleration**; adapts to **new task paths** online; full **10-parameter PDPI**. [Q6_2] 
- **Weaknesses / assumptions:** **Forgetting** of prior tasks after updates; relies on **torque signal quality** and initial excited set; **hand-tuned δ threshold**; single-platform demonstration. [Q6_2] 
- **Notes:** Details δ computation, learner structure, loss/SGD, and comparisons against a **batch ensemble**; shows table/figures with MAE per parameter on new vs prior paths. [Q6_2] 

- **Problem statement (paper’s own):** Classical PDPI and batch ML need **specific excitation** and assume **clean kinematics/accelerations**; changing tasks causes **delays** or **errors**. Need **incremental learning** that **adapts during arbitrary tasks**. [Q6_2] 
- **Context / Use case:** **SME/Industry 4.0** settings with **frequent task/path changes** requiring **rapid PDPI** without stop-and-excite procedures. [Q6_2] 
- **SoA / Contribution:** First **incremental-ensemble PDPI** for cobots that performs **on-path identification** with **low MAE** on new tasks, highlighting a **forgetting trade-off** versus batch methods. [Q6_2] 

---

### Q6_3 — *Online Identification of Payload Inertial Parameters Using Ensemble Learning for Collaborative Robots* (IEEE RA-L, 2024)

- **Task:** **Online PDPI** (mass, CoM, full inertia) using a **bagging ensemble** of simple learners (**NN** or **DT**) from **(q,\dot q,\tau)** only—**no accelerations**, **no wrist F/T**, **no special filtering/calibration**. 
- **Setting:** **Franka Emika Panda (7-DoF)**; large **simulation dataset (≈2.50×10^5 samples; 77 payloads)** + **real-robot validation** with a configurable payload; training on **sinusoidal excitation**; tested vs **RLS** and **RLS+filter**. 
- **Sensors/Data:** Joint **positions, velocities, torques**; **no acceleration**; outputs **10 inertial parameters** ((m,; m r_x,m r_y,m r_z,; I_{xx},I_{xy},I_{xz},I_{yy},I_{yz},I_{zz})). 

- **Method core:**

  * **Bagging ensemble** of **5 weak learners** (either **single-hidden-layer NN** or **Decision Trees**). Train on bootstrapped subsets; output is per-parameter **mean** across learners. Two regimes: **single-stage** (all 10 params) vs **two-stage** (mass/CoM first, then inertias). 
  * Key design choice: **exclude joint accelerations** to avoid noisy differentiation and **eliminate filtering/calibration** steps common in LS/RLS pipelines. 

- **Key result:**

  * **Outperforms RLS** on real Panda: **mass error ↓ 75–78%**, **CoM error ↓ 49.5–60%**; also beats **RLS+filter** on several parameters (e.g., **Ixx/Ixy/Iyy**). **Estimates converge within the first time step** (e.g., **Izz**), vs ~**3 s** for RLS. 
  * **DT vs NN ensembles:** DT better on many components (m, several inertias) in single-stage; two-stage flips advantages for some inertia terms. 

- **Strengths:** **No extra sensors**, **no acceleration**, **no filtering/calibration**; **fast** (first-sample) convergence; strong **sim→real** agreement; flexible (NN or DT weak learners). [Q6_3] 
- **Weaknesses / assumptions:** Needs **excitation trajectory** and high-quality **torque signals**; **hyper-parameter** choices (ensemble size, DT/NN settings); evaluated on **single platform**. [Q6_3] 
- **Notes:** Provides dataset design (payload with independently variable (m, r_x,r_y,r_z)), full MAE tables (single vs two stage; DT vs NN), and head-to-head plots vs RLS/RLS+filter. [Q6_3] 

- **Problem statement (paper’s own):** LS/RLS approaches suffer from **noisy accelerations** in the regressor and need **filters/calibration**, slowing deployment. Need **sensor-lean, acceleration-free** PDPI that is **fast and accurate**. [Q6_3] 
- **Context / Use case:** **Cobot reconfiguration** during teaching/commissioning—rapid payload ID for safe tracking, collision detection, and grasp stability **without extra hardware**. [Q6_3] 
- **SoA / Contribution:** First **bagging-ensemble PDPI** on a Panda with **q, (\dot q), (\tau)** only, showing **large error reductions vs RLS/RLS+filter** and **instantaneous** convergence behavior. [Q6_3] 

---

### Q6_4 — *Addressing Catastrophic Forgetting in Payload Parameter Identification Using Incremental Ensemble Learning* (Frontiers in Robotics & AI, 2024)

- **Task:** **Online PDPI during arbitrary task paths** with an **incremental ensemble** that **prevents catastrophic forgetting** by (i) **adding a new weak learner per new “bag”** and (ii) using a **classifier** to pick the most accurate learner for each incoming sample. 
- **Setting:** **Franka Emika Panda (7-DoF)**; initial excitation-path training; deployment on a **novel linear path**; **sim + real cobot run concurrently** (sim supplies training data for new bags while real executes the task). 
- **Sensors/Data:** Joint **q, q̇, τ** only; **no accelerations**, **no wrist F/T**. Output **10 payload parameters**: (m, m r_x, m r_y, m r_z, I_{xx}, I_{xy}, I_{xz}, I_{yy}, I_{yz}, I_{zz}). 

- **Method core:**

  * **Classification into “bags”** (spherical regions) using **joint-space** threshold ( \delta ) and **Cartesian** threshold ( \rho ); store only each bag’s **mean**. If new point (X_{\text{new}}) lies outside all bags → **form a new bag**. 
  * **Incremental ensemble (IEM):** when a new bag forms, **copy the nearest bag’s weak learner** and **incrementally train** it via **SGD** on sim data for the current payload set; **add** this learner to the ensemble. 
  * **Selection (online estimation):** a **classifier** selects the **nearest-bag learner**; the ensemble output equals that learner’s prediction (selection factor (n_s) = 1 for selected, else 0). **In-bag** points are estimated **instantaneously**. 

- **Key result:**

  * **Novel path:** Proposed IEM **succeeds** with **MAE ≈ 0.007 kg (m)**, **0.008 kg·m (COM)**, **0.0007 kg·m² (inertia)**; **BEM fails** (very large MAEs). **Old IEM** also adapts but **forgets** old path. 
  * **Forgetting test (old path after update):** **Old IEM** degrades to **m MAE 0.037 kg**, **COM 0.025 kg·m**, **inertia 0.0022 kg·m²**; **Proposed IEM maintains** its pre-update accuracy—**no catastrophic forgetting**. 
  * **Timings:** classification per point **~0.001 s**; **estimation** (in-bag) **~0.003 s**; **with update** (new bag) **~0.012 s**; convergence on new path **~2 s** (m, COM), **~1 s** (inertia). 

- **Strengths:** **Arbitrary-path PDPI** without excitation; **no accelerations/F/T**; **eliminates catastrophic forgetting** by **growing** the ensemble and **bag-specific selection**; fast per-point latency. [Q6_4] 
- **Weaknesses / assumptions:** **Ensemble size grows** with workspace coverage (compute/storage); requires **threshold tuning** ((\delta,\rho)); depends on **torque quality**; evaluated on **one platform**. [Q6_4] 
- **Notes:** Provides full **algorithms** for classification and incremental learning, **payload design**, **tables** comparing BEM/old IEM/proposed IEM (excitation vs novel path), and **convergence/time** plots. [Q6_4] 

- **Problem statement (paper’s own):** Prior **incremental ensembles** adapt to new paths but **forget** old ones; batch ensembles need **excitation paths**. Need **continual PDPI** that **adds knowledge** without erasing prior skills. [Q6_4] 
- **Context / Use case:** **SME / Industry 4.0** with frequent **task/path changes** and **unknown payloads**; require **fast, safe** reconfiguration **without stop-and-excite** and **without forgetting** prior tasks. [Q6_4] 
- **SoA / Contribution:** Introduces **bagged-classified incremental ensembles** that **add a weak learner per new region** and **select the right expert** online—**adapting** to new paths **while preserving** old performance. [Q6_4] 

---

### Q6_5 — *External force estimation for robotic manipulator based on particle swarm optimization* (IJARS, 2021)

- **Task:** **Sensorless external force estimation** by first doing **RDPI** with an **improved PSO** (plus optimized **Fourier** excitation), then mapping residual torques to **EE forces** via the **Jacobian**. 
- **Setting:** **Kinova Jaco2 (6-DoF)**; identification on optimized periodic trajectories; validation via **elastic impact** with a spring and **Robotiq FT-300** (for ground truth only). 
- **Sensors/Data:** Joint **q, q̇, q̈** (from filters), **τ** (built-in torque sensors); **no wrist F/T at runtime**. 

- **Method core:**

  * Linear-in-parameters **RBD + friction** model; base-parameter reduction (QR/SVD). 
  * **PSO** identifies dynamics with **mutation** and **decaying inertia**; **fitness:** torque MSE; compares vs **BBO** and **Cuckoo Search**. 
  * **Excitation design:** finite **Fourier series** (0.1 Hz fundamental) optimized to minimize **cond(F)** under joint limits. 
  * **Force estimation:** ( \tau_{\text{ext}}=\tau - (M\ddot q+C+G) - \tau_f ), then (F_{\text{ext}}=(J^\top)^{-1}\tau_{\text{ext}}). 

- **Key result:**

  * **PSO** beats BBO/CS in convergence and identification error; **RMS EE force error ≈ 0.7 N** on validation (Z-axis), lower than baselines (1.10 N, 1.69 N). 

- **Strengths:** **No wrist F/T** at runtime; **optimized excitation** improves conditioning; full pipeline from **ID → sensorless force**; quantitative comparison vs two global optimizers. [Q6_5] 
- **Weaknesses / assumptions:** Requires **built-in joint torque** sensing; **offline** ID (not streaming); relies on **filtered accelerations** and **Jacobian accuracy**; validation focused on **one platform** and mainly **Z-force**. [Q6_5] 
- **Notes:** Gives DH table, PSO/BBO/CS hyper-params, RMS tables, and full excitation coefficients; highlights larger errors near **velocity reversals** (friction/unmodeled dynamics). [Q6_5] 

- **Problem statement (paper’s own):** Wrist **F/T sensors** are costly and fragile (e.g., nuclear environments); **model errors** degrade observer accuracy—need a **cost-effective**, **sensorless** method with robust **parameter ID**. [Q6_5] 
- **Context / Use case:** **Hazardous/remote** manipulation (e.g., nuclear waste handling) where **force control** is needed but F/T sensors are impractical. [Q6_5] 
- **SoA / Contribution:** Shows **PSO-based RDPI + optimized excitation** can deliver **≈0.7 N** RMS sensorless force on **Jaco2**, outperforming **BBO/CS** ID baselines. [Q6_5] 

---

### Q6_6 — *An online payload identification method based on parameter difference for industrial robots* (Robotica, 2024)

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

### XX. Q_Peripheral - Optimization of Adaptive Algorithm for Precise Motion Control of Multi-Degree-of-Freedom Robotic Arms

* **Problem/Issue**: Precise control of multi-DoF arms suffers under uncertainty, noise, and nonlinearities.

* **SoA**: Fuzzy control, heuristic optimization, DRL for real-time adaptation.

* **Math Background**: Adaptive control, fuzzy logic, online optimization, DRL policy learning.

* **Methods**:

  * Combined fuzzy control + deep reinforcement learning.
  * Optimized adaptive algorithm for trajectory tracking.

* **Results**: Improved accuracy + robustness in real-time control.

* **Contribution**: Domain adaptation method for **high-precision trajectory control**.

* **Outlook**: Extend to contact-rich manipulation.

* **Focus**: **Robot control adaptation**, not parameter ID.

---