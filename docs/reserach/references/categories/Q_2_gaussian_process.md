## 🔎 Q2 -  Gaussian Process (GP) / Hybrid GP

### Q2_1 — *Decoupling Observer for Contact Force Estimation of Robot Manipulators Based on Enhanced Gaussian Process Model* (CCIS, 2022)

- **Task:** **Sensorless contact force estimation** with **incomplete model** via hybrid **Enhanced GP (EGP)** model and **Gaussian-Process Disturbance Kalman Filter (GPDKF)** that decouples contact force from residual dynamics/noise. 
- **Setting:** **3-DoF OpenManipulator** (DYNAMIXEL XM-430 actuators); **1 kHz** data; currents → torques via N–T curves; **force sensor only as ground truth**; training set 38,078 samples / test 9,067 samples. 
- **Sensors/Data:** Joint **q, q̇, q̈** (q̈ from poly-fit diff. of q̇), **motor current → τm**; **no wrist F/T in estimation**; nominal (M,C,G) from EL model (MATLAB RST). 

- **Method core:**

  * Split dynamics: ( \tau_m = \tau_{EL}(q,\dot q,\ddot q) + \tau_\Delta + \tau_{ext} ). Learn **residual** ( \tau_\Delta ) with **GPR** (SE/ARD kernel + sparse inducing points); get **mean** ( \mu_{GP} ) **and covariance** ( \Sigma_{GP} ). 
  * Form **EGP momentum model** ( \dot p = C^T \dot q - G + \tau_m - \tau_{ext} - \tau_\Delta ), treat **contact force** as output of an **exogenous process** ((S,H)). 
  * **GPDKF**: Augment state (x=[p^T,\omega^T]^T); use **( \mu_{GP} )** in the input and **( \Sigma_{GP} )** in **process noise** to **attenuate uncertainty**; standard KF predict–update yields ( \hat{\tau}*{ext} ) and ( \hat{F}*{ext}=(J^T)^+ \hat{\tau}_{ext} ). 

- **Key result:**

  * Constant-force test (**Z-axis**): **GPDKF RMSE 1.15 N; conv. time 3.30 s**, vs **EDKF 3.32 N / 4.41 s** and **DKF 5.66 N / 9.61 s**. Outperforms LWPR/MLP/SVR alternatives for residual modeling; GP, LWPR, MLP similar and all ≪ SVR in residual RMSE. 

- **Strengths:** **Decouples** contact force from residual/model errors; uses **probabilistic GP** → **cautious estimation** (uncertainty enters KF); **no force sensor** needed in estimation; **faster** convergence and **lower RMSE** than DKF/EDKF. [Q2_1] 
- **Weaknesses / assumptions:** GP assumes **Gaussian residuals** and **SE (smooth) kernel**; requires **training data** and **hyper-parameter** tuning; torque from currents needs a **linear region**; experiments on **3-DoF** and **partial wrench** demo. [Q2_1] 
- **Notes:** Provides algorithm (TRAIN/ESTIMATION), identifiability/separability discussion, and implementation details (KF covariances using **(\Sigma_{GP})**; time-varying process noise). [Q2_1] 

- **Problem statement (paper’s own):** Force observers (GMO/DOB/KF) **bias under poor models**; prior semi-parametric (NN+DKF) **ignores uncertainty** → error propagation. Need **semi-parametric** force observer that **accounts for learned uncertainty**. [Q2_1] 
- **Context / Use case:** Low-cost platforms needing **force awareness without F/T**; contact-aware control under modeling gaps. [Q2_1] 
- **SoA / Contribution:** Introduces **EGP + GPDKF** to **decouple** residual dynamics and **optimally weigh** GP uncertainty inside KF, achieving **lower error and faster transients** than DKF/EDKF. [Q2_1] 

---

### Q2_2 — *Contact Force Estimation of Robot Manipulators With Imperfect Dynamic Model: On Gaussian Process Adaptive Disturbance Kalman Filter* (IEEE T-ASE, 2024)

- **Task:** **Sensorless contact force estimation** under **imperfect manipulator & force models** via **GPADKF**: GP-learned residual dynamics + **variational Bayes (VB)** adaptation of force-dynamics noise inside a disturbance KF. 
- **Setting:** **3-DoF OpenManipulator**; **1 kHz** data; currents→torques via motor N–T curves (kept in linear region); offline GP training (≈47k samples; 38k train/9k test); experiments with **dynamic** and **constant** contact scenarios; force sensor only for ground truth. 
- **Sensors/Data:** Encoders (q,\dot q) ( (\ddot q) optional), **motor current → τ_m**, nominal (M,C,G,J); **no wrist F/T used in estimation**. 

- **Method core:**

  * Hybrid **Enhanced GP (EGP)** model: learn residual torque (τ_1) via **GPR** (SE/ARD kernel; sparse pseudo-points). Use **GP mean** (µ) for compensation and **GP covariance** (Σ) to **inject uncertainty** into the filter. 
  * **Virtual measurement equation** for contact torque using generalized momentum; treats contact as an **exogenous process** (ω) with (d_1=Hω). 
  * **GPADKF:** KF on (ω) with **VB** to **identify unknown process noise covariance** online; decouples contact force from residual dynamics and noise. 

- **Key result:**

  * **Outperforms DKF/EDKF/GMO/NDO/RFO**: lower RMSE/MAE; **faster convergence**. In a constant-force test, GPADKF reduces convergence and stabilization times by **≈32–34%** vs DKF/EDKF. Dynamic contact demo shows best RMSE vs all baselines. 

- **Strengths:** Handles **multi-source uncertainties** (model errors + learning errors + force-model noise); **uncertainty-aware** (GP (Σ) + VB); **no F/T** needed; integrates as a drop-in observer in control stacks. [Q2_2] 
- **Weaknesses / assumptions:** Needs **offline GP training** (distribution-shift risk); **SE kernel** smoothness; relies on **current→torque** linear region; experiments on **3-DoF** and limited wrench; assumes **low-speed motion/small inertia error** for ignoring (\ddot q). [Q2_2] 
- **Notes:** Gives full GPADKF (TRAIN/ESTIMATION) algorithm; discusses sample efficiency vs GP online cost; provides identifiability/observability conditions and integration scenarios (KF-MPC, DO-based control). [Q2_2] 

- **Problem statement (paper’s own):** Traditional inverse-dynamics/DKF observers **bias/diverge** with **imperfect manipulator & force models** and **unknown noise covariances**; need a **semi-parametric, uncertainty-aware** observer that **decouples contact** from residual dynamics. [Q2_2] 
- **Context / Use case:** Low-cost arms where **F/T sensors are undesirable**, operating in **uncertain environments** with varying residual dynamics and force statistics. [Q2_2] 
- **SoA / Contribution:** Extends **CDF** concept with **EGP + VB-adaptive DKF** to **decouple** disturbances and **auto-tune** covariances online; achieves **best accuracy + fastest transients** among compared methods. [Q2_2] 

---

### Q2_3 — *Human-Robot Contact Detection in Assembly Tasks* (ICMERR, 2022)

- **Task:** **Contact detection** (intentional vs incidental) in **pHRI assembly** via a **modular hybrid** pipeline: **GPR torque regressor** + **CNN contact classifier**; real-time on a cobot. 
- **Setting:** **Franka Emika Panda**; real-time deployment; datasets for GPR (non-contact motion) and CNN (contact/non-contact); **force sensor used only for ground truth**. Demonstrates **generalization** to **new speeds (27–45% vs 25% training)** and **new motions** by retraining **GPR only**. 
- **Sensors/Data:** Encoders (q,\dot q); motor currents/torques from controller; **no wrist F/T in estimation**. GPR maps ([q,\dot q]\rightarrow \tau_{\text{motor}}); external torque estimate (e_\tau=\tau_{\text{motor}}-\tau_{\text{GPR}}). CNN takes time-windowed ([e_q, e_{\dot q}, e_\tau]). 

- **Method core:**

  * **Torque regressor:** **Gaussian Process** with hybrid kernel (RBF + White + Exp-Sine-Squared + Matérn) trained on **non-contact** motion; real-time inference. 
  * **Contact classifier:** lightweight **2D-CNN** over temporal windows (stacked errors) for **binary contact**; trained **once**; **only GPR is retrained** to absorb distribution shifts (speed/motion). 

- **Key result:**

  * **Balanced accuracy ≥ 99%** for online speeds **27–45%** although the CNN was trained at **25%**; **100%** on a **new motion** by retraining **only the GPR**. 

- **Strengths:** **Data-efficient** modularity (retrain GPR only); **no external sensors** at runtime; real-time; robust to **speed/motion distribution shift**; addresses threshold-tuning issues of model-only observers. [Q2_3] 
- **Weaknesses / assumptions:** Requires **per-motion GPR retraining**; relies on controller torque quality and synchronization; contact labels needed once for CNN; reported on **one platform/task domain**. [Q2_3] 
- **Notes:** Compares modular hybrid vs pure data-driven and model-based approaches; positions the design under **Industry 5.0** pHRI needs. [Q2_3] 

- **Problem statement (paper’s own):** Thresholded model-based detectors are sensitive to dynamics/uncertainty; pure data-driven methods **don’t handle distribution shift**. Need a **generalizable** detector that **separates regression (GPR)** from **classification (CNN)**. [Q2_3] 
- **Context / Use case:** **Assembly** pHRI with intentional vs incidental contact; desire to avoid F/T hardware and **reduce re-labeling** when tasks change. [Q2_3] 
- **SoA / Contribution:** Introduces a **GPR→CNN modular** contact detector that **generalizes across speeds/motions** by **retraining only the GPR**, achieving **>99%** balanced accuracy online without wrist F/T. [Q2_3] 

---