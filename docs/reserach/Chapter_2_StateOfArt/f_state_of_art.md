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

## F. Cross-cutting limitations vs your goal

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
