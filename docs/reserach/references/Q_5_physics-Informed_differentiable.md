## 🔎 Q5 - Physics-Informed / Differentiable

---
### 1 **A Residual-Driven Decomposed PINNs Method for Dynamics Identification of Robot Manipulators** (RCAR 2025)

* **Problem/Issue**:
  Classical least squares (LS) identification struggles with **nonlinear friction modeling** (e.g., Coulomb, Stribeck effects). Purely data-driven neural nets lack **physical consistency** and often require vast training data. PINNs help, but existing approaches face ill-conditioning, slow convergence, and poor adaptability to dynamic changes.

* **SoA**:

  * LS, IRLS, EKF widely used but oversimplify friction.
  * PINNs embed physics, but suffer from gradient singularities, oscillations, and local minima.
  * Hybrid physics–data methods emerging for improved consistency.

* **Math Background**:

  * Newton–Euler dynamics with inertia (M), Coriolis (C), gravity (G), and friction (F).
  * Linear parameterization with **base parameter set (BPS)**.
  * LS regression for initialization.
  * Hybrid loss = **data loss + physics loss**, dynamically weighted.
  * Smooth differentiable friction model with **tanh transitions**.

* **Methods**:

  * **Two-stage identification pipeline**:

    1. LS → initial estimates of inertia, Coriolis, and gravitational terms.
    2. PINNs → refine dynamics with adaptive constraint weighting and smooth friction model.
  * **Residual-driven refinement**: PINNs minimize torque residuals from LS.
  * **Adaptive physics layer**: adjusts physics-vs-data weighting in training.
  * Tested on **6-DoF EYOU ARM manipulator** with 15 training trajectories.

* **Results**:

  * Significant RMSE reduction in torque prediction, especially for high-friction joints:

    * Joint 2: 65.3% reduction (6.636 → 2.306 Nm).
    * Joint 3: 42.2% reduction.
    * Joint 1: 18.1% reduction.
  * Limited improvement (and slight overfitting) in low-friction joints.
  * Outperformed LS in capturing **nonlinear frictional effects**.

* **Contribution**:

  * Proposes **residual-driven decomposed PINNs**: cascaded LS + adaptive PINNs.
  * Novel **adaptive constraint weighting** and smooth friction model for stability.
  * Demonstrates strong improvement in **torque prediction under nonlinear friction**.

* **Outlook**:

  * Explore **joint-specific regularization** and richer datasets.
  * Extend to **payload variation and high-speed tasks**.
  * Move toward real-time deployment in collaborative robots.

* **Focus**:
  **Dynamics identification** (parameters + torque) with **PINNs refinement**, especially under nonlinear friction.

---

### 2. **Provably-Safe, Online System Identification** (arXiv 2025)

* **Problem/Issue**:
  Online payload/system identification is needed for safe manipulation with unknown objects. Existing identification methods often ignore **safety constraints** (obstacle avoidance, torque limits) during data collection, risking unsafe trajectories.

* **SoA**:

  * Classical: Exciting trajectory design with Fourier series; LS / adaptive control for inertial parameter ID.
  * Limitations: Unsafe under payload uncertainty, do not guarantee collision avoidance or torque compliance.

* **Math Background**:

  * Momentum-based robot dynamics (integral form avoids acceleration measurement).
  * Interval arithmetic + perturbation analysis → interval bounds on inertial parameters.
  * Optimization of regressor matrix condition number → exciting trajectories.
  * ARMOUR (optimization-based reachability framework) for provably safe planning.

* **Methods**:

  * Framework integrates **trajectory planning + interval-bounded system ID**.
  * Use ARMOUR to generate **provably safe, locally exciting trajectories**.
  * Online loop: execute trajectory → collect data → update interval bounds → recompute trajectory.
  * Interval estimates guaranteed to contain true payload inertial parameters.

* **Results**:

  * Robot: 7-DoF Kinova Gen3 with dumbbell payloads (4–8 lb).
  * Achieved tighter inertial parameter bounds than random or adaptive baselines.
  * Outperformed others in tasks requiring stacking dumbbells while avoiding obstacles.
  * Safe execution under torque limits, unlike baselines which failed or collided.

* **Contribution**:

  * First **provably-safe online system identification** with theoretical guarantees.
  * Integrates safe motion planning and identification in real-time.
  * Open-sourced implementation and validated on hardware.

* **Outlook**:

  * Extend to more complex manipulation tasks and collaborative robots.
  * Improve excitation design (Bezier vs Fourier tradeoff).
  * Consider richer noise models beyond torque dominance.

* **Focus**:
  **Payload inertial parameter estimation** with **safety guarantees** (provably-safe ID).