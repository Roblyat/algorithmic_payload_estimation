### Q8_1 — *Robot Model Identification and Learning: A Modern Perspective* (Annual Review of CRAS, 2024)

- **Task:** Survey of **system identification & model learning** for robots, unifying classical physics-based ID with modern data-driven methods; emphasizes **geometry**, **physical consistency**, and **inductive biases**. 
- **Setting:** Broad: fixed-base manipulators, floating-base/legged systems, contact-rich interaction; offline **and** online/adaptive contexts; links to **differentiable simulation** and **safe/for-control ID**. 
- **Sensors/Data:** Joint states/torques, contact forces (when available), exteroception for kinematics, simulated rollouts; discusses **data sufficiency**, **Fisher information**, and **optimal excitation**. 

- **Method core (what the survey advances):**

    * **Geometric perspective** on inertial parameters via **pseudoinertia LMIs** for **physical consistency**; coordinate-invariant **Riemannian/Bregman** metrics for distances/regularization. 
    * **Geometric information measures** to design **optimal excitation** and assess **practical identifiability** (beyond structural). 
    * **Kinodynamic ID**: joint identification of kinematic + dynamic parameters to avoid structural bias from bad kinematics. 
    * **Discrepancy/residual modeling** (NN/GP) on top of RBD; **physics-informed models** (Lagrangian/Hamiltonian NNs), **equivariance**, **contact-aware learning**, **differentiable simulators**. 

- **Key result (survey takeaways):**

    * Physics-informed + geometric ID methods yield **robust, generalizable models** under limited/noisy data; **simulation-error** criteria often superior to pure equation-error for control use. 

- **Strengths:** Clear unification of **classical ID ↔ modern ML**, actionable tools (LMIs, metrics, info measures), concrete guidance on **data collection**, **regularization**, and **contact modeling** with safety/control in mind. 
- **Weaknesses / assumptions:** Survey-level—less on real-time implementations; some advanced methods (equivariant/graph/diff-physics) remain **simulation-heavy**; open issues in **contact gradients** and **OOD generalization**. 
- **Notes:** Excellent background sections (equation vs **simulation error**), identifiability, and curated references spanning **RBD base parameters**, **physically consistent ID**, **optimal experiment design**, **adaptive control**. 

- **Problem statement (survey’s framing):** How to obtain **accurate, reliable robot models** when data are finite/noisy, dynamics are hybrid (contact), and pure black-box learning lacks **physical guarantees**. 
- **Context / Use case:** Building models for **control & safety** (not just prediction): manipulators/legged robots, contact tasks, sim-to-real; need **sample efficiency**, **robustness**, **consistency**. 
- **SoA / Contribution:** Presents a **modern recipe**: (i) enforce **physical consistency** (LMIs), (ii) use **geometric metrics/regularization**, (iii) design **excitation** with info measures, (iv) **augment physics** with learned residuals/inductive biases, and (v) leverage **diff-physics & equivariance**—bridging ID and control/RL.

---

**Q8_2 — *Inertial Parameter Identification in Robotics: A Survey* (Leboutet et al., Appl. Sci. 2021)**

* **What it is:** A modern survey + benchmark introducing **BIRDy**, an open-source MATLAB toolbox to **compare 17 identification methods** across simulated and real **6-DoF industrial robots (Staubli TX40, Mitsubishi RV2SQ)**. Methods span **IDIM OLS/WLS/IRLS/TLS**, **IV**, **ML**, **OE** (**CLOE/CLIE/DIDIM**), **nonlinear Kalman filters** (EKF/UKF/CDKF and square-root variants), **AdaNN/HTRNN**, and **physically consistent (SDP/LMI) formulations**. 

* **Why it matters:** Gives practitioners **quantitative, apples-to-apples guidance** on method choice under **noise, sampling, controller knowledge, and excitation**—a gap previous piecemeal studies didn’t fill. 

* **Core takeaways (author findings):**

  * **Noise & filtering:** Plain LS-family (incl. AdaNN/HTRNN as OLS surrogates) are **sensitive to encoder noise/derivatives**; tailored **zero-lag filtering/decimation** is often required. 
  * **Top offline picks:** **DIDIM** and **IDIM-IV** generally yield **accurate, noise-robust estimates** with **few iterations**, provided the **closed-loop controller structure/gains are known**. 
  * **OE vs. LS:** **CLIE** tends to outperform **CLOE** (torque outputs more sensitive to parameters than positions under feedback). **OE** is slower due to Jacobians but more noise-tolerant than unfiltered LS. 
  * **Kalman filters:** Can work (joint estimation) but are **tuning-sensitive** and degrade with **sub-sampling**; computational cost rises with sigma-points/particles. 
  * **Physical consistency:** **SDP/LMI** constraints (PC-IDIM/PC-DIDIM/PC-IV) enforce **mass/inertia feasibility**; add compute time but avoid **non-physical** estimates; discuss **marginal physicality** and geometric regularization. 
  * **Trajectories:** Uses **conditioned, Fourier-parametrized exciting motions** and **Monte-Carlo** evaluation; provides practical **figures of merit** (errors, iterations, sims, time). 

* **Strengths (of the survey):**

  * **Unified benchmark** with transparent modeling, excitation design, pre-processing, and metrics. 
  * Covers **both simulation and real robots**, highlighting **controller-knowledge dependence**. 
  * Clarifies **relationships among methods** (e.g., IDIM-IV vs. ML; OE as controller-weighted). 

* **Limitations / cautions:**

  * Focused on **fixed-base serial robots**; parallel/floating-base left as future work. 
  * **Real-world gaps** when **low-level control is unknown**; DIDIM/IV/OE can bias/diverge. 

* **Best-fit uses (practical guidance distilled):**

  * **Offline accurate ID** for control/feedforward & simulation: **DIDIM** or **IDIM-IV**, optionally **PC-constrained**. 
  * **Online/recursive rough ID** or when controller is unknown: **WLS/IRLS** (with careful filtering) or **NKF** (accept tuning load). 

* **Gaps / future:** Extend to **parallel/floating-base**, URDF pipelines, richer sensors (IMUs/F/T), faster symbolic engines; broader study of **noise distributions** and **low-rate sensing**.