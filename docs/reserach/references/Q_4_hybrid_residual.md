## 🔎 Q4 -  Hybrid / Residual

### 1 **Sensorless Force Estimation for Industrial Robots Using Disturbance Observer and Neural Learning of Friction Approximation** (RCIM 2021)

* **Problem/Issue**:
  Heavy-duty industrial robots often lack built-in F/T sensors. Classical inverse dynamics approaches for **sensorless force estimation** suffer from noise, unmodeled nonlinearities, and especially **friction uncertainty**.

* **SoA**:

  * **Model-based**: inverse dynamics, least squares, disturbance/momentum observers.
  * **Momentum observer + KF**: improves accuracy but sensitive to friction uncertainty.
  * **NNs**: applied for nonlinear friction compensation (LuGre, Stribeck, LSTM for rolling friction).

* **Math Background**:

  * **Newton–Euler dynamics**.
  * **Stribeck friction model**.
  * Disturbance observer formulation, linear state-space.
  * Disturbance Kalman filter (DKF).
  * NN approximation for nonlinear friction residuals.

* **Methods**:

  * Identify Stribeck friction parameters.
  * Use NN (3-layer feedforward, sigmoid activation) to approximate residual friction errors.
  * Combine disturbance observer + generalized momentum observer → DKF formulation.
  * NN-enhanced disturbance Kalman filter compensates model & measurement uncertainty.

* **Results**:

  * Experiments on KUKA KR 6 R700 sixx (6-DoF heavy robot).
  * NN friction model reduced MSE from 14.67 → 4.97 Nm².
  * DKF + NN yielded smallest force estimation errors, robust against noise and dynamic uncertainties.
  * Validated both **free-motion** and **contact force** experiments, with external sensor ground truth.

* **Contribution**:

  * First **hybrid observer–learning framework** for sensorless force estimation in heavy industrial robots.
  * Overcomes friction uncertainty limitations by blending disturbance observers with NN learning.

* **Outlook**:

  * Improve estimation of **nonlinear disturbances**.
  * Apply to **force-controlled HRC manufacturing** without F/T sensors.

* **Focus**:
  **Hybrid observer + neural learning for sensorless force estimation**.

---

### 2. **Adaptive Neural Trajectory Tracking Control for n-DOF Robotic Manipulators With State Constraints** (IEEE TII, 2023)

* **Problem/Issue**:
  Trajectory tracking of n-DOF manipulators is challenging under **uncertain dynamics**, **external disturbances**, and **state constraints**. Classical computed torque control (CTC) cannot handle uncertainties well, and many adaptive methods do not guarantee constraint satisfaction.

* **SoA**:

  * **CTC**: cancels nonlinear dynamics but depends on accurate models.
  * **Adaptive control**: can estimate uncertainties but often ignores constraints.
  * **Disturbance observers**: improve robustness but limited under nonlinearities.
  * **Neural learning (RBFNNs)**: approximate unknown nonlinear functions.
  * **Barrier Lyapunov functions (BLF)**: enforce state constraints.

* **Math Background**:

  * Robot dynamics: τ = M(q)q̈ + C(q, q̇)q̇ + F(q, q̇) + G(q) + τd.
  * Radial Basis Function Neural Networks (RBFNNs) approximate Δf + δ(q, q̇).
  * Nonlinear Disturbance Observer (NDO) compensates approximation errors.
  * BLF ensures constraint satisfaction while maintaining Lyapunov stability.

* **Methods**:

  * Designed **observer-based adaptive neural controller**:

    1. **CTC** reduces nonlinearity.
    2. **RBFNNs** approximate uncertainties.
    3. **NDO** estimates errors + disturbances.
    4. **BLF** ensures state constraints are not violated.
  * Conducted **sensitivity analysis** to rank control parameter importance.

* **Results**:

  * Simulations on a **7-DOF robotic manipulator**.
  * Proposed method achieved **smaller tracking errors** than baseline controllers.
  * **Improved convergence speed and steady-state performance**.
  * Outperformed CTC+RBFNN, CTC+BLF, and RBFNN+NDO+BLF in IAE and ITAE metrics.

* **Contribution**:

  * First **integrated framework** combining CTC, RBFNNs, NDO, and BLF.
  * Guarantees **trajectory tracking under uncertainties and state constraints**.
  * Sensitivity analysis provides guidance for tuning control gains.

* **Outlook**:

  * Extend to **real-world robotic experiments**.
  * Optimize controller gains via automated design (e.g., meta-learning).
  * Combine with probabilistic observers for uncertainty quantification.

* **Focus**:
  **Adaptive neural + disturbance observer control** for **trajectory tracking with state constraints**.

---

### 3. **Robot Hybrid Inverse Dynamics Model Compensation Method Based on the BLL Residual Prediction Algorithm** (Tao et al., *Robotica*, 2025)

* **Problem/Issue**:
  Pure physics-based inverse dynamics models suffer from inaccuracies due to **model simplifications, noise, and friction effects**. There’s a need for **hybrid models** that combine nominal physics with learned residuals.

* **SoA**:

  * Classical: Newton–Euler inverse dynamics (NE).
  * Residual/Hybrid: Neural correction models, ResNN, ADNN.

* **Math Background**:

  * NE equations: τ = f(q, q̇, q̈, parameters).
  * Residual learning with **Bounded Linear Least-Squares (BLL)**.

* **Methods**:

  * Developed a **hybrid inverse dynamics model** = NE nominal + BLL residual prediction.
  * Residual term compensates for unmodeled dynamics & friction.
  * Compared vs. nominal NE and NN residual baselines.

* **Results**:

  * Significantly reduced prediction error under disturbances.
  * Stable, interpretable compensation vs. purely black-box NN residuals.
  * Achieved robust performance across multiple trajectories.

* **Contribution**:

  * New **BLL-based residual prediction framework** for hybrid ID.
  * Bridges interpretable physics-based models with robust ML residuals.

* **Outlook**:

  * Extend to adaptive online learning.
  * Test under broader robotic tasks (industrial arms, manipulation).

* **Focus**:
  **Hybrid inverse dynamics compensation** (residual correction).

### XX. **Programmatic Imitation Learning from Unlabeled and Noisy Demonstrations (PLUNDER)** (RA-L, 2024)

* **find in 'Peripheral/Noise #2'**

---
---

## 🔎 **Conclusion of Q₄ Research (Hybrid / Residual)**

### **Scope Covered**

Q₄ papers address **GAN-based sim-to-real tactile modeling** and **probabilistic hybrid program synthesis** for IL. Both are **residual/hybrid paradigms** combining physics or symbolic priors with generative/adversarial modeling.

### **What They Estimate**

* **Tactile domain**:

  * SightGAN estimates **contact positions** & preserves **force traces** in tactile images.
* **Learning from demonstrations**:

  * PLUNDER estimates **latent action labels** + synthesizes **probabilistic interpretable policies**.

### **Mathematical Backbone**

* **GAN + Residual Losses** (SightGAN): domain adaptation with contact-aware residual terms.
* **Expectation-Maximization + Probabilistic DSL** (PLUNDER): residual hybridization of symbolic program induction with probabilistic reasoning.

### **State of the Art Trend**

* Strong move toward **GAN hybrids** (physics- or task-specific losses).
* Move from opaque neural IL → **interpretable probabilistic policies** robust to noise.
* Both emphasize **bridging sim-to-real gaps** (SightGAN in sensing; PLUNDER in learning).

### **Overall Coverage**

* **Tactile sim-to-real & contact estimation**: covered (GAN + residual physics).
* **Imitation learning under noise & missing labels**: covered (PIL + probabilistic hybrid).
* **Residual correction of models**: both papers highlight **residual loss or probabilistic correction** on top of nominal models.

---

✅ **Conclusion**:
Q₄ demonstrates that **hybrid residual learning frameworks** — GANs with physics-informed residuals (SightGAN) and probabilistic residual program synthesis (PLUNDER) — are pushing robotics research toward **robust sim-to-real transfer, interpretable learning, and noise-resilient estimation**.

---
---