## Category Q2 — GP / Hybrid (Model + Learning)
**Scope:** Semi-parametric identification/observers blending **nominal rigid-body models** or controller signals with **GP residuals/regressors**, coupled to **learned observers/classifiers** for contact/wrench/parameter estimation. [Q2_1, Q2_2, Q2_3]  
**Common sensors:** Encoders (q, q̇); controller torques/currents; model terms \(M,C,G,J\) when needed; typically **no wrist F/T** in estimation (used only for ground truth). [Q2_1, Q2_2, Q2_3]  
**Assumptions:** Residuals are **GP-amenable**; sufficient logs for training; Gaussian noise for filtering or regression; task-rate inference feasible. [Q2_1, Q2_2, Q2_3]  
**Typical outputs:** **Sensorless contact detection/force estimation** (partial/full), sometimes stiffness ID via model augmentation. [Q2_1, Q2_2, Q2_3]

**Representative evidence (growing list):**
- Q2_1 — *Decoupling Observer… Enhanced Gaussian Process Model* (**sensorless force**; **EGP + GPDKF**; GP mean+cov in KF; 1 kHz; 3-DoF). [Q2_1]
- Q2_2 — *…Imperfect Dynamic Model: GP Adaptive Disturbance KF* (**sensorless force**; **EGP + VB-adaptive DKF**; best RMSE/transients among DKF/EDKF/GMO/NDO/RFO). [Q2_2]
- Q2_3 — *Human-Robot Contact Detection in Assembly Tasks*** (**contact detection**; **GPR torque regressor + CNN**; generalizes to **new speeds/motions** by retraining **GPR only**; **>99%** balanced accuracy online; no wrist F/T). [Q2_3]

**Strengths (from cards):**
- **Uncertainty-aware semi-parametric observers** (GPDKF/GPADKF) with **no F/T** and faster convergence. [Q2_1, Q2_2]
- **Modular hybrid** detector with **data-efficient generalization** (retrain GPR only) and **real-time** deployment. [Q2_3]

**Weaknesses (from cards):**
- Requires **training data** and GP/CNN hyper-parameter tuning; GP smoothness (SE/ARD) assumptions; limited DoF/wrench demos so far; reliance on controller torque quality. [Q2_1, Q2_2, Q2_3]

**Best-fit contexts:**
- Cost-sensitive pHRI/assembly needing **contact/force awareness without F/T**, with logs available and tasks that periodically change speed/motion profiles. [Q2_1–Q2_3]

**Failure modes:**
- **Distribution shift** beyond what GPR can absorb; nonsmooth/high-frequency residuals; poor torque mapping/sync; insufficient labeling for initial CNN training. [Q2_1–Q2_3]

**Synthesis (Q2):**  
Q2 combines **GP learning** with estimation/classification to overcome model gaps and threshold brittleness: **GPDKF/GPADKF** inject learned **uncertainty** into filters for **sensorless force**, while **GPR→CNN** delivers **contact detection** that **generalizes to new speeds/motions** by retraining **only the regressor**, keeping the classifier stable. [Q2_1, Q2_2, Q2_3]

**Gap notes (Q2):**
- Extend to **full 6-D wrench** and **higher-DoF** robots; **online/sparse GP** for streaming adaptation; principled **domain shift detection** and **auto-retraining** triggers; tighter coupling to **stiffness/PDPI** pipelines. [Q2_1–Q2_3]
