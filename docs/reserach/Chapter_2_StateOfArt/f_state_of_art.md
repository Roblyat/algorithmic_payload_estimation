## 1) Research Strategy

<!-- GitHub render -->
![Query Logic Diagram](/docs/reserach/illustrations/query_logic.drawio.png)

### Categories
- $C_1  =$ Classical / Observers
- $C_2  =$ Gaussian Process (GP)
- $C_3  =$ Deep Sequence Models (MLP / GRU / TCN / Transformer / LSTM)
- $C_4  =$ Physics-Informed / Differentiable  
- $C_5  =$ Surveys 
- $C_T  =$ Goal & Domain Terms
  - $Cmt =$ Estimation & Modeling Terms
  - $Cct =$ Robotics Context Terms

### Query Logic (Generalized Set Intersection)

### Combined Representation

$$
C = \{ C_1, \dots, C_4 \}
$$

$$
Q = \bigcup_{i=1}^{5} Q_i
$$

$$
Q_i = \left( \bigvee_{c \in C_i} c \right) 
\;\; \land \;\; 
\left( \bigvee_{e \in Cmt} e \right) 
\;\; \land \;\; 
\left( \bigvee_{r \in Cct} r \right),
\quad i = 1, 2, \dots, 5
$$


# Research Results Summary

## Results in Numbers / Research Trend

[See detailed results](research_trend.md)

---

| Query                                 | Relev. SoA | Rigid-body | Payload | Both  |
| ---------------------------------     | ---------- | ---------- | ------- | ----- |
| **$Q_1  =$ Classical / Observers**    | 17         | 7          | 7       | 3     |
| **$Q_2  =$ Gaussian Process (GP)**    | 4          | 4          | 0       | 0     |
| **$Q_3  =$ Deep Sequence Models**     | 8          | 4          | 4       | 0     |
| **$Q_4  =$ Physics-Informed / Diff.** | 5          | 5          | 0       | 0     |
| **$Q_5  =$ Surveys**                  | 2          | –          | –       | –     |
| **Total**                             | **36**     | **20**     | **11**  | **3** |

### Q1 - SoA Statements

## A. General picture of Q1 (“classical / observers”)

* Q1 methods are almost all **model-based**: they rely on a nominal rigid-body dynamics (RBD) model and/or a **Newton–Euler regressor** plus classical estimators (LS/WLS/RLS/TLS, KF, observers, sliding mode, etc.).

* There is a clear split between:

  * works that aim at **robot dynamic parameter identification (RDPI)**,
  * works that aim at **payload dynamic parameter identification (PDPI)**,
  * works that target **sensorless force / torque estimation** at the end-effector,
  * and only a few that combine these aspects.

* Across Q1, **mass is usually identified accurately**, **CoM moderately well**, and **inertia is hardest** and often poorly validated or weakly excited.

---

## B. Payload identification with FT sensor, mostly without nominal robot model

1. **Gravity- vs full-dynamics excitation (Q1_1, “Fast Object Inertial Parameter Identification…”)**

   * Proposes **PMD** (point mass discretisation) that fits point masses to a *known geometry* in order to infer inertial parameters without an explicit RBD model.
   * Uses **gravity-only WLS–NE** regressors for slow motions and **full WLS–NE** for dynamic motions; targets cobot regimes with low SNR in accelerations and forces.
   * Strengths: very good **mass** estimates, reasonable **CoM**; explicitly analyses **low-SNR cobot constraints**.
   * Limitations: **inertia estimates are poor** for short sequences; **identifiability of the full inertia tensor is lost** in the gravity-dominated regime; relies on **known payload shape** → not applicable to arbitrary unknown objects.

2. **Static+dynamic FT-based PDPI (Q1_5, Q1_7, Q1_10)**

   * Common pattern:

     * **Static poses** → identify payload **mass + CoM**.
     * **Dynamic trajectories** (Fourier / RRT / excitation) → identify **inertia** via LS-type regressor in the sensor frame.
   * Q1_7 shows strong PDPI for heavy (~40 kg) payloads with a two-stage LS/RRTLS scheme but needs many long experiments (15 static poses, 5 dynamic 10 s trajectories) and explicit FT hardware.
   * Q1_5 reports ~10 % mass error but has **no ground truth** for CoM and inertia → limited validation.
   * Q1_10 is mainly a **trajectory-design** paper; their PDPI results are offline, with **no quantitative ground truth**, so it’s only weak evidence.
   * Overall critique: good demonstration that **payload dynamics can be identified in the FT frame**, but:

     * high experimental overhead,
     * reliance on FT hardware (cost, integration),
     * inertia remains the weakest and sometimes unvalidated part.

---

## C. Joint-space / motor-space RDPI + PDPI without FT sensor

3. **Fully decoupled RDPI + PDPI (Q1_6, “FDRDI”)**

   * Designs special **S-curve symmetric trajectories** (RSC / FDRDI) to **decouple friction, gravity, and inertia** and to identify robot and payload parameters without FT sensors.
   * Shows that traditional **coupled identification** (CRDI) gives poor friction / inertia prediction at high velocities due to **extrapolation**.
   * Strengths:

     * Very strong **joint torque prediction** accuracy across the whole velocity range.
     * Clear experimental comparison showing improved **payload parameter accuracy**, especially inertia.
   * Limitations:

     * Requires **multiple carefully designed trajectories**, time-consuming offline procedure.
     * Assumes payload is rigidly mounted and constant; not suited for frequent load changes “on the fly”.

4. **Double-weighted WLS in motor-current space (Q1_8/9)**

   * Works directly in **motor-current space**, using the **same Fourier trajectories with and without payload**; double-weighted WLS improves robustness vs noise.
   * Strengths: good **joint torque prediction** and good PDPI vs CAD for simple shapes.
   * Limitations:

     * Still fully **offline**, trajectory-dependent, and requires repeated “with/without payload” runs.
     * Focuses mainly on torque prediction; less discussion of robustness to friction/backlash or non-ideal transmissions.

5. **Classical joint-space RDPI + PDPI (Q1_13)**

   * Sequential LS–NE regressors: first RDPI, then PDPI using residual torques between “with” and “without” payload.
   * Strong offline results for both RDPI & PDPI; typical of “gold-standard” LS-NE workflows.
   * Limitation: no treatment of **online adaptation** or changing loads; assumes stable friction and gear behaviour.

6. **WLS with PSO & trajectory pre-processing (Q1_15)**

   * Uses WLS with **particle swarm optimisation** and **KLT denoising** to fit base parameters for accurate joint torque prediction.
   * Strength: demonstrates that even classical LS can be significantly improved with better preprocessing and optimisation.
   * Limitation: again, purely RDPI; **no explicit payload identification**, and computational complexity of PSO is not discussed in real-time context.

7. **Online PDPI via parameter difference (Q1_16)**

   * Offline: identify robot base parameters. Online: use **residual joint torques** during operation and an RLS–NE regressor to identify payload parameters.
   * Strong results for **mass**, **CoM**, and even **inertia** against CAD ground truth; identified mass error ~0.43 % in case study.
   * Critique / subtlety:

     * Still needs **exciting motions** for good online convergence.
     * Paper argues that payload inertia contributes little to total dynamics; in practice many controllers therefore **ignore payload inertia** → reveals a wider limitation of current industrial interfaces.
     * Nonlinear friction is handled with a parametric model; table of RMSE shows **noticeable sensitivity to friction modelling**, especially with payload (errors increase on some joints).

---

## D. Observer-based sensorless force / torque estimation

8. **Sliding-mode / momentum-based observers (Q1_3, Q1_14)**

   * Use nominal RBD plus controller torques; external torque = measured − predicted.
   * Q1_14 focuses on **collision detection**: binary “contact or not” and collided joint index via thresholding.
   * Strengths:

     * Conceptually simple; good **binary collision detection** and joint index localisation.
     * Can run in real time on torque-sensor robots.
   * Limitations:

     * Require reasonably accurate RBD and friction model; sensitive to threshold tuning.
     * Do **not** estimate payload parameters; payload is either treated as part of the model or ignored.
     * Sliding-mode observers can be chattering-prone and sensitive to noise.

9. **Adaptive Kalman filter / “Adaptive Moment Filter” (Q1_4)**

   * Uses an **adaptive KF** / AMF to estimate end-effector wrench (and covariance) from motor torques and a nominal RBD.
   * Noise covariances (R, Q), torque constants, gear ratios, and friction parameters are **calibrated offline**.
   * Strengths:

     * Gives **probabilistic (covariance) information** about the wrench.
     * Outperforms a classical KF baseline in specific cup-lifting experiments.
   * Limitations:

     * Strong dependence on **offline calibration and noise tuning**.
     * Results are shown for a narrow manipulation task; unclear robustness to changing payloads or unmodelled dynamics.

10. **High-order finite-time observers (HOFTO, Q1_12)**

    * Builds an identified RBD model via LS–NE, then uses a **high-order finite-time observer** to estimate interaction forces **without FT sensors**.
    * Strengths:

      * Rigorous stability analysis; demonstrably good sensorless force estimation on a real 6-axis robot.
      * Explicitly compares to extended-state observers and highlights reduced steady-state error for time-varying forces.
    * Limitations:

      * Still depends on **accurate offline identification** and a fairly simple friction model (Coulomb + viscous).
      * Does not adapt payload parameters online; payload changes would require re-identification.

11. **DKF + NN friction (Q1_17)**

    * For heavy-duty robots without FT sensor: identify dynamics and Stribeck friction, then refine friction using an NN; use **Disturbance Kalman Filter (DKF)** + momentum observer to estimate contact forces.
    * Strengths:

      * Explicitly addresses **nonlinear friction** and model uncertainty using NN learning.
      * DKF significantly reduces MSE vs pure model-based or momentum observers (e.g. MSE 1.17 vs ~4.8 Nm² in one experiment; large reduction in maximum absolute error at velocity sign-changes).
    * Limitations / critical points:

      * Even with NN friction and DKF, **absolute errors remain several Nm**, especially during dynamic phases and velocity reversals (max errors around 10–36 Nm in some scenarios).
      * Training and tuning of the NN and DKF is non-trivial and robot-specific; generalisation to different robots or payloads is not studied.
      * Again, this is **force estimation only**; no payload parameter identification.

---

## E. Online payload identification using observers and proprioception

12. **Momentum-observer-based online PDPI (Q1_2)**

    * Uses a **momentum observer** as a virtual sensor for external joint torques, then runs an identification procedure using **only proprioceptive sensors** (joint torques, positions, velocities).
    * Introduces a clever **filter design** that matches the momentum observer’s phase lag, allowing time-aligned accelerations from filtered velocities without extra sensors.
    * Proposes new calibration scheme with a **virtual average calibration object** that no longer requires identical motions for calibration and identification.
    * Strengths:

      * Truly **online PDPI** during other tasks; no FT sensor; robust to spatial/temporal misalignment between calibration and identification trajectories.
      * Mass is recovered accurately; CoM and inertia improve significantly with the proposed filtering and calibration.
    * Limitations:

      * Requires reasonably accurate base robot model and momentum observer tuning.
      * Still depends on having **some dedicated calibration motion** and good excitation; performance for very short or low-excitation production motions is less clear.
      * Inertia estimates are still more sensitive and less robust than mass.

---

## F. Q1 Cross-cutting limitations

13. **Strong dependence on accurate base models and friction compensation**

    * Many observer-based methods (momentum, sliding-mode, EKF/AKF, HOFTO, DKF) assume a **well-identified RBD and reasonably modelled friction**; their performance degrades with unmodelled friction, backlash, or temperature-dependent effects, which several papers acknowledge explicitly.
    * Only Q1_17 really pushes NN friction modelling; even there, residual errors remain significant.

14. **Payload inertia is systematically the weakest link**

    * Multiple works either **do not validate inertia** (or lack ground truth) or show that full inertia tensors are poorly identifiable with short, safe cobot trajectories (Q1_1, Q1_5, Q1_7, Q1_10).
    * Q1_16 even notes that manufacturers often **don’t provide interfaces to compensate payload inertia**, because its contribution is small and hard to estimate robustly.

15. **Heavy reliance on dedicated excitation and offline steps**

    * Most strong results require **carefully designed, often long excitation trajectories** (RSC/Fourier/S-curves) and multiple runs with and without payload.
    * That’s fine for commissioning or calibration, but it’s far from the “continuous, task-agnostic dynamic awareness” you’re aiming at.

16. **Limited integration of PDPI and interaction force estimation**

    * Works either:

      * focus on **PDPI** (Q1_1, 2, 5–8, 10, 13, 16), assuming contact forces are negligible or measured by FT sensor, or
      * focus on **contact force estimation** (Q1_3, 4, 12, 14, 17), treating payload as fixed.
    * Only a few (e.g. Q1_7 with FT sensor) really show both **payload identification and contact force estimation**, and even there the contact estimation is evaluated after converged PDPI and with non-negligible computation/measurement times.

17. **Fragmented treatment of robot vs tool vs payload “self awareness”**

    * RDPI papers focus on **robot base parameters**; PDPI papers treat the payload as an add-on; observer papers treat everything as lumped disturbance or external wrench.
    * There is **no unified framework** in Q1 that gives the robot a clean, online estimate of **its own dynamics + tool + payload** and uses that consistently for both **torque prediction** and **force estimation**.

---
---
## Q2 – Gaussian Process (GP) SoA Statements

### A. General picture of Q2 (“Gaussian Process”)

* All Q2 papers use **Gaussian Process Regression (GPR)** as a *non-parametric model* of robot dynamics or residual dynamics.
* Two main roles of GP:

  1. **Residual model on top of a nominal RBD model** for **sensorless contact force estimation** (Q2_1, Q2_2).
  2. **Black-box inverse dynamics / external torque model** for **contact detection** and **model comparison vs deep nets** (Q2_3, Q2_4).
* Advantages highlighted:

  * GP can **capture complex residual dynamics** (friction, unmodelled effects) and provide **uncertainty (variance) estimates**.
  * Physics-inspired kernels (GIP) can **embed structure** and improve **generalisation and data efficiency** compared to generic kernels or unconstrained NNs.
* Limitations across Q2:

  * All methods are **regression on static state vectors** ((q,\dot q,\ddot q)) (no sequence models).
  * Models are **trained offline** on curated datasets; **online adaptation** is limited to re-training or updating the GP, not continuous learning during arbitrary tasks.
  * None of the Q2 papers perform **payload dynamic parameter identification (PDPI)**; payload is either fixed, part of the training data, or left for future work.

---

### B. GP residual models for sensorless contact force estimation (Q2_1, Q2_2)

**Q2_1 – EGP + GPDKF (“Decoupling Observer…”)**

* Builds a **hybrid semi-parametric dynamic model**:

  * Nominal Euler–Lagrange model (M(q),C(q,\dot q),G(q)) from RBD.
  * **Residual dynamics** (\tau_\Delta(q,\dot q,\ddot q)) learned with **GPR** (Enhanced GP, EGP) from **collision-free data**. 
* The learned GP mean and covariance are plugged into a **Gaussian Process Disturbance Kalman Filter (GPDKF)** that reconstructs **external joint torques / contact forces** while **decoupling residual dynamics from contact**. 
* Strengths:

  * **More accurate contact force estimates** than classic DKF and NN-based enhanced DKF (EDKF), especially when residual dynamics are significant.
  * Uses GP **uncertainty** inside the Kalman filter for “cautious” estimation.
* Limitations / caveats:

  * Requires a **nominal rigid-body model** and **Jacobian**; performance still depends on basic model quality.
  * GP is trained **offline** on a large contact-free dataset; **no GP update during contact**.
  * Demonstrations mostly for **simple contact scenarios (constant or slowly varying forces)**; not evaluated for complex, multi-axis interaction or changing payloads.

**Q2_2 – GPADKF (“Contact Force Estimation… With Imperfect Dynamic Model”)**

* Extends Q2_1 to **imperfect dynamic models**:

  * Same hybrid nominal + GP residual model as EGP.
  * Introduces **Gaussian Process Adaptive Disturbance Kalman Filter (GPADKF)** using **variational Bayes** to adapt **noise covariance matrices** online.
* Strengths:

  * More robust when both the **robot model and force model are inaccurate**; **reduces dependency on a “perfect” dynamic model**.
  * Improves **tracking and convergence speed** of contact force estimates vs previous composite disturbance filtering (CDF), DKF and GPDKF baselines.
* Limitations:

  * Still needs **offline GP training** on contact-free data and a **nominal RBD model**.
  * Adaptation focuses on **noise statistics**, not on continuous update of the GP dynamics model itself.
  * Evaluations are limited to **sensorless contact force estimation**; **no PDPI, no explicit treatment of changing tools/payloads**.

---

### C. GP + CNN for human–robot contact detection (Q2_3)

**Q2_3 – “Human-Robot Contact Detection in Assembly Tasks”**

* Proposes a **two-layer architecture**:

  1. **Torque regressor**: a GPR model predicts **required motor torque** for **contact-free motion** from joint positions and velocities (no explicit RBD).
  2. **Contact classifier**: a **CNN** takes the estimated external torque (measured minus GP prediction) and sensor readings, and outputs **binary contact vs non-contact**.
* Key idea: handle **data distribution shifts** (new motions / speeds) by **retraining only the GP regressor** on new contact-free trajectories; the CNN classifier can remain fixed, making the system more **data-efficient** and reducing robot downtime.
* Experiments on a Franka Emika Panda:

  * Show **high contact detection accuracy (~99 %)** under different motions and speeds, with ground truth from joint torque sensors.
* Limitations:

  * Outputs **binary contact labels**, not continuous force estimates or payload parameters.
  * Still requires **separate datasets**: one for GP (no-contact) and one for CNN (collision scenarios).
  * Generalisation to **changing payloads** or different robot types is mentioned as **future work**.

---

### D. Physics-embedding GP for inverse dynamics (Q2_4)

**Q2_4 – “Embedding the Physics in Black-box Inverse Dynamics Identification: a Comparison Between Gaussian Processes and Neural Networks”**

* Compares two **physics-aware black-box inverse dynamics models**:

  * **DeLaN** (Deep Lagrangian Network): NN that **enforces Lagrangian structure** (learned inertia matrix and potential).
  * **GIP-kernel GP**: GP with **Geometrically Inspired Polynomial kernel**, which constrains the regression basis functions to a **finite-dimensional, physically inspired space** but does *not* enforce exact structural properties.
* Extensive experiments on simulated and real manipulators with increasing DOF show:

  * **GIP-GP maintains accuracy** as DOF grows, whereas **DeLaN accuracy degrades quickly**.
  * GIP-GP **better estimates inertial, Coriolis and gravitational torque components** separately, despite not hard-coding the equations of motion.
* Takeaways:

  * Embedding physics via **kernel design** can give strong **regularisation and data efficiency** for GP inverse dynamics.
  * Suggests that combining **GIP kernels with structured NNs** (e.g. DeLaN) is a promising future direction.
* Limitations relative to your goal:

  * Focuses on **inverse dynamics regression quality**, not on **online contact or payload estimation**.
  * Uses **batch, offline training**; no explicit mechanism for continuous online adaptation to new payloads/tools or contact conditions.
  * Evaluates inverse dynamics for tracking/control, not for **sensorless force estimation**.

---

### E. Cross-cutting limitations of Q2

* **No structured robot–tool–payload decomposition**

  * None of the Q2 methods explicitly separate **robot base dynamics**, **tool/gripper dynamics**, and **payload dynamics**.
  * GP residuals always model a **lumped “everything that is missing” term** (friction, flex, tool, payload, etc.), so they do not provide a clean notion of *self-awareness* of robot + tool that could later be reused as a basis for PDPI.

* **GP used for contact or generic inverse dynamics, not for tool/gripper compensation**

  * In Q2_1–Q2_3, GPs are used to improve **sensorless contact force estimation** or **contact detection**; in Q2_4 they target **inverse dynamics quality**.
  * None of them aim at **precise compensation of a known tool/gripper in the measurement frame** as a separate object, which is the core building block you care about for later PDPI.

* **Static-state regression; no sequence-aware handling of backlash / rate effects**

  * All GPs operate on **instantaneous state features** ((q,\dot q,\ddot q)) (or similar stacked vectors).
  * There is **no explicit temporal model** (TCN/LSTM/GRU) to capture **history-dependent effects** such as backlash, stick–slip friction, or actuator hysteresis, which you plan to handle via sequence models on top of an offline-trained dynamics model.

* **Adaptation only via dataset updates, not fast online correction**

  * Q2_1 and Q2_2 improve robustness with **disturbance Kalman filters** and adaptive noise covariances, and Q2_3 proposes to **retrain the GP** when the motion distribution changes.
  * This still means that adaptation is realised through **new offline GP fits on updated datasets**, not through a **lightweight online error-correction model** (e.g. TCN/LSTM on residuals) that can track slow drifts between the offline-trained model and the real robot.

* **Task-specific evaluation**

  * The GP-based observers are tested on **simple, scripted contact tasks**; Q2_3 on **specific assembly motions**; Q2_4 on **tracking torque error**.
  * None of the works demonstrate a **task-agnostic dynamic awareness module** that can be reused across different manipulation tasks while keeping a separate, well-identified representation of robot + tool dynamics.

---
---
## Q3 – SoA statements (Deep sequence / deep learning methods)

### A. General picture

* Q3 collects **deep-learning approaches for inverse dynamics, force estimation and payload identification**:

  * LSTM / GRU-type **sequence models** for residual torque and force estimation.
  * Feed-forward NNs, MLPs, CNNs and ensemble methods for **PDPI** and **contact detection**.
* Most works either:

  * learn **residuals on top of a nominal RBD model**, or
  * learn **direct maps from joint histories to payload parameters / contact labels**.
* Training is always **offline batch** (possibly with incremental updates later); online is **inference only** or incremental fine-tuning.

---

### B. Deep residual inverse dynamics on top of RBD

1. **Residual LSTM on Franka Panda dataset (Q3_1)**

   * Uses public Franka Panda dataset + Gaz et al. model; computes **torque residuals** between data and RBD model.
   * Trains a **bootstrapped LSTM ensemble (BLL-LSTM)** to predict residual joint torques from sequences ((q,\dot q,\ddot q)).
   * Shows **clear improvement** over GP and single models on dataset test splits.
   * Limitations:

     * Purely **offline / dataset-only**; no real-time controller integration or PDPI.
     * Requires an accurate **full robot model** (including friction).

2. **LSTM joint-torque and EE-force estimation (Q3_3)**

   * Franka Panda, FT sensor in base; LSTMs map:

     * base FT wrench + joint states → **EE tip force**,
     * joint states → **joint torques**.
   * LSTMs outperform MLP, 1D-Conv and DeLaN in both sim and real-robot tests.
   * Still **task-specific**, with FT hardware and simulation-generated ground-truth forces.

---

### C. Deep learning for end-effector wrench / contact from proprioception

3. **ASGRNN wrench observer (Q3_4)**

   * UR5 teleoperation; adaptive sparse GRNN maps ((q,\dot q,\ddot q,i)) → 6D wrench, using FT data for supervision.
   * Strong performance for **soft/stiff collision force estimation**, better than MLP and GP baselines.
   * Good candidate for model-free wrench estimation but:

     * Needs **dense labelled FT data**,
     * Evaluated mainly in **teleoperation & collision scenarios**, not PDPI.

4. **CNN contact localisation with domain randomisation (Q3_5)**

   * 7-DoF Panda, **no torque sensors, no FT**; input is link velocities and pose errors, transformed to 2D “contact images”.
   * CNN trained in **IsaacGym with domain randomisation** to classify contact / no-contact and localise the contacting link.
   * Achieves **~98% sim-to-real accuracy** for binary contact and link localisation.
   * Does **not estimate wrench magnitude or payload dynamics**.

---

### D. Learning-based PDPI (payload parameters) with NNs / ensembles

5. **MLP-based PDPI from encoder discrepancies (Q3_2)**

   * OpenMANIPULATOR-X, **no FT**; requires an RBD model and camera pose to express payload parameters.
   * MLP processes joint states and sign of velocity, then LS post-processing recovers **mass & CoM** of known objects.
   * Average errors: ~9% mass, ~18% CoM; shows feasibility but limited accuracy, few payloads and a small robot.

6. **Ensemble learning line (Q3_7 → Q3_6 → Q3_8)**

   * **Batch ensemble (Q3_7)**:

     * Multiple weak learners (NN / decision tree) map ((q,\dot q,\tau)) directly to payload parameters (\phi) for 77 synthetic payloads along a **fixed excitation path**.
     * Good **sim-to-real transfer** on Franka Panda, reducing mass/CoM errors vs RLS.
     * Still needs a **separate excitation trajectory** for each new payload.
   * **Incremental ensemble (IEM) without catastrophic-forgetting handling (Q3_6)**:

     * Extends the ensemble to **incremental learning along arbitrary task paths**, removing the need for a dedicated excitation path.
     * Uses Euclidean distance in feature space to decide when to update / create weak learners.
     * Works, but **suffers catastrophic forgetting**: performance on old paths degrades after adapting to new ones.
   * **Incremental ensemble with classifier (Q3_8)** 

     * Adds a **bag-based classifier** that routes each new path segment to the most relevant weak learner; new bags spawn new learners.
     * Demonstrated on 77 payloads with a Franka Emika cobot; maintains **good accuracy on old paths** while adapting to new ones and **eliminates the explicit excitation-path requirement**.
     * Trade-off: **ensemble size grows** with the number of distinct path “bags”; all models are relatively small feed-forward NNs (no sequence structure).

---

## Q3 – Cross-cutting limitations

1. **Limited use of true sequence models for PDPI**

   * LSTMs / GRNNs are used mainly for **force / torque estimation** (Q3_1, Q3_3, Q3_4), not for **direct payload parameter identification**.
   * PDPI itself is handled mostly by **static MLPs / small NNs / trees**, not by temporal models that exploit long-horizon joint histories.

2. **Either strong model dependence or fully black-box**

   * Some methods need a reasonably accurate **nominal RBD model + camera / pose calibration** (Q3_1, Q3_2, partly Q3_3), so errors in the base model leak into the learned component.
   * The ensemble-based PDPI line (Q3_7, Q3_6, Q3_8) is almost fully **black-box**: it ignores known RBD structure and only sees joint signals and torques.

3. **Narrow evaluation regimes and payload sets**

   * Most works use **one robot** (often Panda) and a **small library of payloads / tasks** (e.g. 77 synthetic payloads with similar mounting).
   * Results are strong on the chosen benchmark paths but give little evidence about **generalisation to unseen payloads, very different motions, or broader operating envelopes**.

4. **No unified treatment of torque prediction, tool/gripper compensation and interaction forces**

   * Contact / EE-wrench estimators (Q3_3, Q3_4, Q3_5) do **not estimate payload parameters**.
   * PDPI methods (Q3_2, Q3_7–Q3_8) **do not estimate contact forces** and are not evaluated as full inverse-dynamics models.
   * There is **no single deep model** that simultaneously delivers good **joint-torque prediction**, **tool/payload compensation** and **contact awareness**.

5. **Incremental ensemble PDPI still has structural issues**

   * The latest incremental ensemble (Q3_8) removes catastrophic forgetting and excitation-path dependence, but:

     * relies on a **growing ensemble** of weak learners and bag classifier,
     * does not exploit temporal structure (no TCN/LSTM in the payload map),
     * is developed for **payloads only**, not including tool/gripper mass and inertia as part of a unified effective rigid body.

---
---

## A. Overall picture of Q4 (Physics-informed / differentiable models)

* Q4 papers use **physics-informed deep nets** (DeLaN / PINNs / hybrids) for **inverse dynamics / joint torque prediction**.
* They exploit **Lagrangian / state-space structure** to enforce energy consistency and physical constraints, typically achieving **better torque prediction and generalisation** than plain MLPs.
* Focus is almost entirely on **RDPI (robot joint dynamics)**:

  * **No explicit payload dynamics identification**.
  * **No interaction force estimation** (force shows up only implicitly via torque labels).
* Training is **offline** on carefully designed excitation trajectories; the models are then **deployed online** for torque prediction / control.
* Most works either ignore contacts or treat them as unmodelled disturbances; friction is handled either via **simple analytic models** or via **learned residuals**.

---

## B. DeLaN and physics-inspired baselines (Q4_2)

* Q4_2 (“Combining Physics and Deep Learning to Continuous-Time Dynamics Models” – DeLaN/HNN survey & benchmark):

  * Introduces / reviews **Deep Lagrangian Networks (DeLaN)** and **Hamiltonian Neural Networks (HNN)** as structured deep models for robot dynamics.
  * Shows that **structured** DeLaN/HNN give **lower NMSE and longer valid prediction times** than black-box baselines on 2-DoF systems and a 4-DoF WAM arm.
  * Highlights limitations:

    * Assumes **no contacts**, conservative dynamics (friction often neglected or added separately).
    * Requires access to **generalised coordinates and forces** (q, q̇, τ).
    * Black-box variants may yield **nearly singular mass matrices** and blow-up errors.

(So: Q4_2 is the **conceptual baseline** showing why structured physics-informed nets matter.)

---

## C. DeLaN with motor couplings / currents (Q4_1)

* Q4_1 (“Extended DeLaN for robotic arm dynamics considering motor couplings”):

  * Extends DeLaN to include **motor actuator dynamics and friction**, using **motor currents/voltages plus robot motion** to model a UR10e arm with gearboxes.
  * Learns:

    * Lagrangian terms (mass matrix, potential),
    * Electrical / friction parameters (torque constant, viscous / Coulomb levels),
    * Mapped so that **motor current** is the supervised output.
  * Demonstrates **good current / torque prediction** on simulated and real data and improved accuracy vs the original DeLaN and a feed-forward NN.
  * Still:

    * Trained **offline** on ~300 random trajectories (no contacts / payload changes).
    * Identifies **combined robot-plus-tool dynamics**, but **no explicit payload model**.
    * Non-conservative effects beyond motor friction (contacts, backlash) are still lumped into residuals.

---

## D. PINN + LS for dynamics identification (Q4_3)

* Q4_3 (“Residual-Driven Decomposed PINNs for dynamics identification of robot manipulators”):

  * Stage 1: classic **LS Newton–Euler base-parameter identification** on a 6-DoF arm using torque labels from motor currents.
  * Stage 2: **PINN refinement** that minimises a **hybrid loss**:

    * data loss on the torque residuals,
    * physics loss enforcing RBD equations (M(q)q̈ + C(q,q̇)q̇ + g(q) = τ).
  * Shows **lower joint-torque RMSE** vs the LS baseline, especially where nonlinear friction is significant.
  * But:

    * Uses ~15 long excitation trajectories, all **offline**.
    * No payload variation, no contacts; **only RDPI**.
    * PINN acts as a refined friction / unmodelled-dynamics learner, not as a full contact/payload estimator.

---

## E. Friction-inclusive PINN + residual sequence model (Q4_4)

* Q4_4 (“PINN-based friction-inclusive dynamics modelling for industrial robots”):

  * Target: **multi-joint industrial robots without joint torque sensors** – uses **joint currents** and motion data only.
  * Builds a **structured PINN** combining:

    * Lagrangian dynamics,
    * an explicit **Stribeck friction model** per joint.
  * Introduces a **dual-loop hybrid learning strategy**:

    * one loop focused on dynamics parameters,
    * one loop on friction parameters,
    * plus a **history-based residual network** (your card: TCN) that learns remaining errors over a time window.
  * Achieves **very strong torque prediction** across lower/upper joints and outperforms DeLaN-type and LS baselines (joint currents as proxy).
  * Limitations:

    * Still **pure RDPI**: no explicit payload, no contact forces.
    * Requires extensive **instrumented trajectories** and careful hyper-parameter tuning.
    * Residual learner is black-box; interpretability of friction vs other errors is limited.

---

## F. H-PINN for joint-level dynamics & parameter ID (Q4_5)

* Q4_5 (“Physics-Informed Neural Network for Model Prediction and Dynamics Parameter Identification of Collaborative Robot Joints”):

  * Proposes **H-PINN**: a **hybrid PINN on an RNN** with customised **RK4 cells** that embed the joint’s state-space dynamics.
  * Uses labelled data (sim or experiment) to jointly learn:

    * Unknown **physical parameters** (inertia, friction, etc.),
    * The joint’s **state-transition model**.
  * Shows accurate **single-joint dynamics prediction** and parameter estimates for a collaborative robot joint.
  * Limitations:

    * Demonstrated only on **one joint**; scaling to a full 6–7-DoF arm is not addressed.
    * Architecture and training are comparatively **complex and expensive**.
    * Again, **no payload / contact modelling** – purely joint dynamics.

---

## G. Cross-cutting Q4 limitations


* **Scope**: All Q4 works target **robot joint dynamics (RDPI)**; **payload dynamics and interaction forces are not estimated explicitly.**
* **Training regime**: Models are **trained offline** on long excitation datasets; runtime is purely feed-forward prediction (no online learning in the strong sense).
* **Sensing assumptions**:

  * Require **accurate joint states and torque or current signals**;
  * No use of FT sensors, but also **no explicit EE wrench estimate**.
* **Contact & payload**:

  * Contacts are excluded from training or treated as disturbances.
  * Payload changes are not addressed; models implicitly assume a fixed tool/payload.
* **Complexity & scalability**:

  * PINN/H-PINN architectures can be **computationally heavy and tricky to tune**, especially if extended beyond low-DoF setups or single joints.

* **Relevance**:

  * Q4 gives **very strong structured baselines** for learning inverse dynamics from encoder + motor data, especially with friction.
  * But they **don’t yet give you**: online awareness of tool/payload, nor force estimation; they’re more like the **“best you can do” for a fixed robot+tool model** that your work can build on.
