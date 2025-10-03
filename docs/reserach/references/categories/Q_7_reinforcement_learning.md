## 🔎 Q7 -  Reinforcement Learning

---

### Q7_1 — *Estimating an Object’s Inertial Parameters by Robotic Pushing: A Data-Driven Approach* (IROS/letter)

- **Task:** **Pre-grasp object ID** of **2D inertial parameters**—mass (M), planar CoM ((r_x,r_y)), and planar inertia (I_{zz})—from a **single planar push** using **data-driven features** grounded in quasi-static pushing mechanics. 
- **Setting:** Two datasets: (i) **V-REP** sim with **7 Schunk LWA4D** arms, **48,000 pushes** across wide ranges of mass, size, friction (0.2–0.6), velocity (0.01–0.06 m/s); (ii) **MIT M-Cube** real pushing dataset (200k+ pushes, 11 objects, 4 surfaces). 
- **Sensors/Data:** Pusher **forces & moments**, pusher **velocities**, object **planar velocities**; features from **force/torque & velocity signals** (8 signals × 3 windows × {mean, std, RMS} = **72-D**). **Multi-Output Regression Random Forest** regresses ([M,I_{zz},r_x,r_y]). 

- **Method core:** Use **quasi-static** mechanics (limit surface, motion cone) to choose informative signals (F, M, (u), (\omega), pusher velocity), then learn the nonlinear mapping to 2D parameters with a **multi-output RF** (ensemble). Works for **sticking** and **sliding** via motion-cone reasoning encoded in features. 

- **Key result:**

  * **Sim (48k pushes):** mean percent error — **Mass 7.59%**, **Inertia 13.49%**, **CoM-x 12.59%**, **CoM-y 20.61%** (10% test with edge-case masses over-represented). 
  * **M-Cube (real):** mean percent error — **Mass 10.05%**, **Inertia 12.44%**, **CoM-x 12.90%**, **CoM-y 13.32%**. Robust across **surface types** (friction) with small variance. 

- **Strengths:** **Single-push** estimation; **robust to surface friction** via limit-surface–inspired features; **generalizes** across objects/surfaces; simple sensors (F/T and velocities); handles sticking/sliding contact via motion-cone features. [Q7_1] 
- **Weaknesses / assumptions:** Restricted to **planar / quasi-static**; outputs **2D** parameters (not full 3D); performance on cross-dataset **sim→real** is limited without special transfer (40–60% error noted). [Q7_1] 
- **Notes:** Provides full derivation (limit surface ellipsoid, motion cone), dataset construction, and error metrics (average percent difference). Highlights robustness to noise (±5% added). [Q7_1] 

- **Problem statement (paper’s own):** Analytical pushing-based ID needs stringent assumptions/hardware; pure vision often needs priors. Need **accurate, minimal-interaction** estimation of **mass/CoM/inertia** from a **single push**. [Q7_1] 
- **Context / Use case:** **Pre-grasp** assessment in **extreme/industrial** environments (nuclear, SAR) where moving/lifting is risky—estimate inertial properties **before grasp**. [Q7_1] 
- **SoA / Contribution:** First to **accurately estimate all 2D inertial parameters from one push** by combining **physics-guided features** with **multi-output RF**, validated on **sim + real** pushing corpora with low errors and friction robustness. [Q7_1] 

---

### Q7_2 — *Model-based Reinforcement Learning with Parametrized Physical Models and Optimism-Driven Exploration* (ICRA)

- **Task:** **Real-time, sample-efficient online model identification + control** by combining **feature-based least-squares (LS) dynamics ID** with **optimistic MPC** (iLQR/DDP) for exploration. 
- **Setting:** Benchmarks (**pendulum, cartpole, double pendulum**) and a **simulated 7-DoF Barrett WAM** reaching task; runs **in real time**. 
- **Sensors/Data:** ([q, \dot q, \ddot q], \tau)**; features (H(q,\dot q,\ddot q))** derived from **robot morphology** (links/connectivity), e.g., via **SymPyBotics**; no special external sensors. 

- **Method core:**

  * **Feature-based ID:** write dynamics as (H(q,\dot q,\ddot q),\Delta=\tau); estimate parameters **(\hat\Delta=A^{\dagger}b)** by LS; recover forward dynamics ( \hat f(q,\dot q,\tau)=\ddot q). 
  * **Optimism-driven exploration:** augment dynamics with **virtual controls** ( \xi ): ( \ddot q=\hat f(\cdot)+\xi ), penalized by ( \tfrac{1}{m}|\xi|^2) with **(m=c/N)** (optimism decays as samples (N) grow). 
  * **MPC/iLQR (DDP)** with warm starts; **fallback double-integrator** if early passes diverge. 

- **Key result:**

  * **Lowest interaction time** vs prior model-based RL: e.g., **pendulum 3.28±1.17 s** (DDP true: 3.04±0.89), **cartpole 8.31±3.15 s** (true: 7.44±3.26), **double pendulum 4.98±1.83 s** (true: 3.7±0.89); prior PILCO ~**12–50 s**, prior optimism method slower and **not real time**. **7-DoF arm** tasks solved in **~3.6–11 s**, also real time. 

- **Strengths:** **Real-time** online ID + control; **physics-structured features** → **fast LS fitting**; **goal-directed exploration** (optimism) instead of separate PE; scales to **7-DoF**. [Q7_2] 
- **Weaknesses / assumptions:** Evaluated **mostly in sim** (benchmarks + 7-DoF); assumes **known morphology** and good **state/accel estimates**; no explicit uncertainty bounds; safety constraints not enforced. [Q7_2] 
- **Notes:** Includes Algorithm 1; iLQR details; tables comparing completion times; suggests combining linear physics features with richer residual models in future. [Q7_2] 

- **Problem statement (paper’s own):** Dedicated offline ID is inefficient when dynamics **change**; generic statistical models in RL are **sample-hungry**. Need **fast online identification** tightly coupled with **task-driven control**. [Q7_2] 
- **Context / Use case:** Tasks needing **quick adaptation** (new payloads/contacts) where you want **on-the-fly** model learning while **reaching the goal**; no time for separate PE runs. [Q7_2] 
- **SoA / Contribution:** Marries **RBD-style linear regressors** with **optimistic MPC**, achieving **state-of-the-art sample efficiency** and **real-time control** across classic systems and a **7-DoF arm**. [Q7_2] 

---

### Q7_3 — *Preparing for the Unknown: Learning a Universal Policy with Online System Identification* (UP-OSI; Georgia Tech/Google Brain)

- **Task:** **Task-driven online system identification + control**: learn a **Universal Policy (UP)** conditioned on dynamics parameters and an **Online System Identification (OSI)** network that infers those parameters from a short state-action history during execution. 
- **Setting:** Simulated benchmarks (**cart-pole, double inverted pendulum, hopper**) and a **manipulator throwing** task with **unknown object mass**; tests also include **temporally varying friction** and **out-of-training-range** parameters. 
- **Sensors/Data:** State (x=[q,\dot q,\dots]), actions (u); OSI input is a short **history window** ((x_{t-h:t},u_{t-h:t-1})); no special exteroception. 

- **Method core:**

  * **UP:** policy (\pi(x,\mu)) trained with TRPO over a **distribution of models** (parameters (\mu): masses, inertias, frictions, lengths). 
  * **OSI:** network (\phi(x_{t-h:t},u_{t-h:t-1})\to\hat{\mu}) trained **iteratively**: start on “good cases” (control with true (\mu)), then inject **mismatch** rollouts (control with (\hat\mu), simulate with true (\mu)) until UP-OSI performance approaches UP with ground-truth (\mu) (3–5 iterations). 

- **Key result:**

  * Across all tasks, **UP-OSI ≈ UP with true parameters**, and sometimes **outperforms UP-true** **outside** the training range (e.g., long cart-pole + tip mass). It **tracks varying friction online** in hopper and maintains task performance; manipulator **learns to throw** to target height under **unknown block mass**. 

- **Strengths:** **Real-time feasible**, **sample-efficient** (all sim data), handles **changing/unknown dynamics** (mass, friction, geometry), and shows **OOD generalization** by coupling ID and control. [Q7_3] 
- **Weaknesses / assumptions:** Results are **simulation-only**; relies on **good state/acceleration estimates** and **known morphology**; no explicit **safety guarantees** or error bounds; high-dimensional (\mu) remains challenging. [Q7_3] 
- **Notes:** Provides training algorithms, ablations over history length, and plots of **OSI estimates vs ground truth** (e.g., friction). Discusses narrowing the **reality gap** and future real-robot tests. [Q7_3] 

- **Problem statement (paper’s own):** Separate offline ID or model-free RL is **sample-hungry** and fragile to **model mismatch**; need a controller that **identifies while acting** across **unknown/variable** dynamics. [Q7_3] 
- **Context / Use case:** Rapid **adaptation to new payloads/surfaces** (e.g., changing object mass, friction) **during the task** without dedicated PE. [Q7_3] 
- **SoA / Contribution:** Introduces **UP-OSI**, showing that **conditioning policies on (\mu)** + **short-horizon OSI** yields **near-oracle** performance, **handles varying friction**, and **generalizes beyond** training ranges. [Q7_3] 

---

### Q7_4 — *Learning Force Control for Contact-Rich Manipulation Tasks With Rigid Position-Controlled Robots* (RA-L, 2020)

- **Task:** **Learn low-level force control** on **rigid, position-controlled** arms by combining **RL (SAC)** with **classical force controllers** (parallel position/force PID or admittance) plus a **fail-safe** layer for safe real-robot training. 
- **Setting:** **UR3 e-series** with wrist **F/T sensor**; **Gazebo** sim + two real tasks: **ring insertion (0.2 mm clearance)** and **peg-in-pulley (0.05 mm)**. Policy at **20 Hz**; force controller at **500 Hz**. 
- **Sensors/Data:** End-effector pose error (x_e), EE velocity (\dot x), **contact force (F_{\text{ext}})**; actions (a=[a_x, a_p]) where (a_x) shapes the trajectory and (a_p) tunes controller gains/selection. 

- **Method core:**

  * **Parallel position/force PID:** learn (K^x_p) (with (K^x_d=2\sqrt{K^x_p})), learn **force PI** (K^f_p) (with (K^f_i=0.01K^f_p)), and **selection matrix** (S=\text{diag}(s_1…s_6)); reduced controllable dims for stability. 
  * **Admittance:** learn **stiffness** (k_d) per axis and **PD (K^x_p)**; inertia fixed, damping from (\zeta). 
  * **Fail-safe:** per-step checks (IK existence, joint-speed limit, **force limit**) + **reward penalty** for safety violations. 

- **Key result:**

  * **Action-space study (sim peg insertion):** best trade-off models **P-14** (parallel) and **A-13pd** (admittance); too few params learn poorly; too many learn slower. Penalizing safety violations speeds learning and **reduces collisions**. 
  * **Real robot:** both models solve ring-insertion quickly; avg collisions/session **P-14: 45**, **A-13pd: 34**. For **0.05 mm peg**, both succeed after ~13k steps; **A-13pd** has **fewer collisions (≈4) vs P-14 (≈26)** and clearer **on-contact gain modulation** (drop gains at first contact, raise for insertion). 

- **Strengths:** Works on **position-controlled** arms (no torque control); **safe, mostly unsupervised** real-robot training; **learns trajectory + controller gains**; thorough action-space analysis. [Q7_4] 
- **Weaknesses / assumptions:** Needs **wrist F/T**; depends on **hand-tuned base/range** for gains; assumes **known goal pose**; no formal stability guarantees beyond fail-safe; not a PDPI/RDPI method. [Q7_4] 
- **Notes:** Includes full control laws, safety Algorithm 1, reward design, and real-task learning curves showing benefit of safety penalties. [Q7_4] 

- **Problem statement (paper’s own):** RL on **rigid, position-controlled** robots is risky without robust low-level control and supervision; **force-control gains** are task-dependent and hard to tune. [Q7_4] 
- **Context / Use case:** **Contact-rich assembly** on standard industrial arms (no torque mode) needing **learned, safe** force behavior. [Q7_4] 
- **SoA / Contribution:** A practical **RL+force-control** framework with a **fail-safe** that learns **low-level force policies** on real hardware and quantifies the **action-space/learning** trade-offs. [Q7_4] 

---

### 5. **Reinforcement Learning Based Variable Impedance Control for High Precision Human-Robot Collaboration Tasks**

(Meng, Su & Wu, IEEE ICARM 2021)

* **Category**:
  Q3 — **Reinforcement Learning + Variable Impedance Control**
  (Model-free RL to adapt impedance parameters for collaborative assembly.)

* **Problem/Issue**:
  Human-robot collaboration requires **adaptivity to different human operators** and tasks. Fixed impedance controllers are not sufficient for high-precision assembly.

* **Math Background**:

  * Impedance control law with variable parameters (inertia, damping, stiffness).
  * RL optimization via **Proximal Policy Optimization (PPO)**.

* **Methods**:

  * Designed a **Variable Impedance Learning Control (VILC)** strategy.
  * RL agent outputs impedance parameters in real time.
  * PPO ensures stable training and adaptation.

* **Results**:

  * Tested in human-robot collaboration **assembly tasks**.
  * Robot adapted impedance online to operator differences.
  * Achieved robust, precise assembly with reduced training data.

* **Contribution**:

  * First PPO-based variable impedance framework for collaborative assembly.
  * Teach-less approach — no need for demonstrations.
  * Generalizable across different operators and conditions.