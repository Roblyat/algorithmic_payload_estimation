## Category Q3 — Deep / Physics-Informed Dynamics (PINNs, DeLaN, sequence models)
**Scope:** Learning-based dynamics/ID and interaction estimation using **physics priors** (DeLaN/PINNs) or **model-light deep regressors** (CNN/LSTM/RBFNN/GRNN) with crafted reps and sparsity/meta-heuristics (e.g., **ASGRNN + IALO**). [Q3_1–Q3_6]  
**Common sensors:** Joint states \(q,\dot q,\ddot q\); motor currents / drive signals; typically **no wrist F/T** (sometimes base F/T for special setups; F/T often used only for ground truth). [Q3_1–Q3_6]  
**Assumptions:** Data-rich logs with coverage; moderate smoothness; platform-specific calibration; methods may add **feature selection/sparsity** or **meta-heuristic tuning** to improve robustness and compute. [Q3_1–Q3_6]  
**Typical outputs:** **Inverse dynamics** (torques/currents), **sensorless forces**, **inertial parameters** (mass/CoM), and **admittance commands**. [Q3_1–Q3_6]

**Representative evidence (growing list):**
- Q3_1 — *Extended DeLaN (DeLaN-Motor)* (PINN inverse dynamics; UR10e; \(R^2\!\approx\!0.973\)). [Q3_1]
- Q3_2 — *CNN-based Parameter Identification* (RDPI via DCT→CNN; encoder + PWM proxy; ≥93.55% reconstruction). [Q3_2]
- Q3_3 — *Learning PDPI From Encoder Discrepancies* (mass & CoM without F/T; torque NN + attention-WLS; camera tag). [Q3_3]
- Q3_4 — *EE Force & Joint Torque via Deep Learning* (base F/T + LSTM; 0.153 N sim EE; ≤0.78 Nm real torques). [Q3_4]
- Q3_5 — *Sensorless Admittance With Deadzone* (RBFNN observer + adaptive NN control; incremental learning; UUB). [Q3_5]
- Q3_6 — *ASGRNN Force Observer for Teleoperation*** (**sensorless force**; **sparse features + SV pruning + IALO**; **UR5e**; soft & stiff env.; beats **GPR/MINN/RF**). [Q3_6]

**Strengths (from cards):**
- Physics-informed inverse dynamics without torque sensors. [Q3_1]
- Trajectory-design-light RDPI with DCT→CNN. [Q3_2]
- F/T-free PDPI with analytic attention-WLS. [Q3_3]
- Model-free EE force/torque from base sensing (deep LSTM). [Q3_4]
- Sensorless admittance coping with deadzone (incremental NN; stability). [Q3_5]
- **Model-light, sensorless force estimation** with **automatic feature sparsity** and **fast meta-heuristic tuning** (ASGRNN). [Q3_6]

**Weaknesses (from cards):**
- Current/torque proxy quality; friction form; single-robot scope. [Q3_1]
- 2-DoF limit; proxy assumptions; offline training. [Q3_2]
- Needs camera tag; many steady snapshots; 4-DoF scope. [Q3_3]
- Requires base F/T hardware; data hunger; single platform. [Q3_4]
- Needs free-motion logs & model terms; NN tuning; single platform. [Q3_5]
- **Offline supervised data with ground-truth forces**; tuning of feature threshold/bandwidth; single-platform teleop demo. [Q3_6]

**Best-fit contexts:**
- Cobot control/monitoring without torque sensors. [Q3_1]
- Fast RDPI without heavy excitation design. [Q3_2]
- Small robots needing mass & CoM without wrist F/T. [Q3_3]
- Robots where wrist F/T is impractical but base sensing is feasible. [Q3_4]
- pHRI/compliance with actuator deadzone; bounded motions. [Q3_5]
- **Teleoperation** (surgery/nuclear) needing **sensorless force** from joint signals/currents, **low cost**, and **sparse compute**. [Q3_6]

**Failure modes:**
- Friction/proxy mismatch; distribution shift. [Q3_1, Q3_2, Q3_4]
- Tag occlusions/pose errors. [Q3_3]
- Poor excitation; NN mis-tuning; inaccurate \(M,C,G\). [Q3_5]
- Insufficient workspace/contact coverage in training; mis-set sparsity/thresholds; over-pruned SVs. [Q3_6]

**Synthesis (Q3):**
Q3 now includes a **model-light, sensorless** teleop force observer (ASGRNN) that **learns directly from joint signals/currents**, adds **feature sparsity** and **SV pruning** for compute, and uses **IALO** for rapid hyperparameter search—complementing DeLaN-Motor, DCT→CNN RDPI, encoder-discrepancy PDPI, LSTM base-sensing, and NN-based sensorless admittance. [Q3_1–Q3_6]

**Gap notes (Q3):**
- Toward **online/streaming** adaptation with **uncertainty quantification**; multi-robot generalization; reduce reliance on ground-truth F/T during training; unify with **PDPI** to recover **full inertia** online in teleop tasks. [Q3_1–Q3_6]
