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
  - $C_mt =$ Estimation & Modeling Terms
  - $C_ct =$ Robotics Context Terms

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

---
---

## 2) 🔑 Categories with Index Terms

## **1. Classical / Observers**

* momentum observer (MO)
* generalized momentum observer (GMO)
* disturbance observer (DOB)
* reaction force observer (RFOB)
* Kalman filter (KF)
* extended Kalman filter (EKF)
* unscented Kalman filter (UKF)
* state observer
* least squares (LS)
* weighted least squares (WLS)
* iterative reweighted least squares (IRLS)
* recursive least squares (RLS)
* momentum-based observer
* dynamic state observer
* observer
* force observer
* torque observer

 
## **2. Gaussian Process (GP) / Hybrid GP**

* gaussian process regression (GPR)
* sparse gaussian process (SGP, SGPR)
* multi-output gaussian process (MOGP)
* multi-task gaussian process (MTGP)
* gaussian process state space model (GPSSM)
* hybrid gaussian process
* GP residual
* gaussian process dynamics
* GP inverse dynamics
* bayesian nonparametric regression (BNPR)


## **3. Deep Sequence Models (MLP / GRU / TCN / Transformer)**

* neural network inverse dynamics (NN-ID)
* deep learning
* multi layer perceptron (MLP)
* residual network (ResNet)
* long short-term memory (LSTM)
* gated recurrent unit (GRU)
* temporal convolutional network (TCN)
* causal convolution
* dilated convolution
* transformer model
* attention model
* sequence-to-sequence (seq2seq, S2S)
* sequence GAN (SeqGAN, TimeGAN)
* GAN
* Generative Adversarial Networks


## **4. Hybrid / Residual**

* residual learning dynamics
* residual neural network (ResNN)
* hybrid model dynamics
* analytical dynamics neural network (ADNN)
* physics residual
* rigid body dynamics residual (RBD residual)
* Newton Euler residual (NE residual)
* nominal dynamics model (NDM)
* neural correction
* learning inverse dynamics residual (ID residual)
* residual GAN
* GAN
* Generative Adversarial Networks


## **5. Physics-Informed / Differentiable**

* physics-informed neural network (PINN)
* differentiable physics
* differentiable simulation (DiffSim)
* differentiable robot model
* differentiable dynamics
* neural ODE (NODE)
* torchdiffeq
* ODE-net
* physics-guided machine learning robotics (PGML)


## **6. Domain Adaptation / Latent Context**

* domain adaptation (DA)
* transfer learning (TL)
* meta learning (ML)
* context variable dynamics
* latent variable model (LVM)
* amortized inference (AI)
* test time adaptation (TTA)
* online adaptation (OA)
* feature invariance
* domain invariant features (DIF)
* few shot learning (FSL)
* zero shot transfer (ZSL)


## **7. Reinforcement Learning**

* reinforcement
* reinforcement learning
* Isaac Gym differentiable
* Isaac Lab differentiable
* Isaac Gym
* Isaac Lab


## **8. Surveys & Overviews**

* survey
* benchmarking
* review
* overview
* systematic comparison


## **9. (Estimation Goal & Domain)**
## **9a. Estimation & Modeling Terms**

* external force
* force measurement
* force estimation
* force/torque estimation
* wrench estimation
* joint torque estimation
* end-effector force
* end-effector torque
* inertial parameters
* inertial parameter identification (IPI)
* online payload identification
* payload identification
* payload estimation
* object parameter estimation
* parameter identification
* inertia tensor
* inertia tensor estimation
* center of mass (CoM)
* rigid body dynamics
* nonlinear systems
* friction approximation
* nonlinear friction model
* external perturbations
* force torque sensor (F/T sensor)
* noise
* signal noise
* noise estimation
* external force estimation (EFE)
* external torque estimation (ETE)
* torque estimation
* parameter identification differentiable simulation
* payload identification (PI)
* payload estimation (PE)
* contact force

## **9b. Robotics Context Terms**

* robotic manipulator
* robotic arm
* robotic manipulation
* robot payload

---
---

# Research Results Summary

## Results in Numbers / Research Trend

[See detailed results](research_trend.md)

---

The structured literature search identified a substantial number of relevant papers in each query category (Q1–Q6). The table below summarizes the counts of total relevant papers, those published 2022–present, and the subset of “Mature SoA” papers (i.e. peer-reviewed works with citations or high relevance) for each category:

| Query                             | Relev. SoA | Rigid-body | Payload | Both  |
| --------------------------------- | ---------- | ---------- | ------- | ----- |
| **Q1 – Classical / Observers**    | 17         | 7          | 7       | 3     |
| **Q2 – GP / Hybrid GP**           | 4          | 4          | 0       | 0     |
| **Q3 – Deep Sequence Models**     | 8          | 4          | 4       | 0     |
| **Q4 – Physics-Informed / Diff.** | 5          | 5          | 0       | 0     |
| **Q5 – Surveys & Overviews**      | 2          | –          | –       | –     |
| **Total** without Surveys         | **36**     | **20**     | **11**  | **3** |

> **Note:** “Total Papers” excludes non-article references like textbooks.
> “2022–Present” counts publications from 2022 to 2025.
> "Mature SoA Papers" are a selected subset of highly relevant works in 2022–2025, often in top venues or showing notable impact.

---
---

# 📊 Relevant SoA Papers (2021–2025) Classification
- [table only](relevant_SoA.md)

|  n | **Q**  | **Paper**                                                                                                                    | **Year / Venue**     | **Category**| **Focus**         | **Citations** [251130_185300] |
|----|--------| -----------------------------------------------------------------------------------------------------------------------------| -------------------- | ------------| --------------    | ------------- |
|   1|  Q1.1  | Nadeau et al. – *Fast Object Inertial Parameter Identification for Collaborative Robots*                                     | 2022 / ICRA          | Q1          | **Payload**       | 13            |
|   2|  Q1.2  | Kurdas et al. – *Online Payload Identification for Tactile Robots Using the Momentum Observer*                               | 2022 / ICRA          | Q1          | **Payload**       | 16            |
|   3|  Q1.3  | Kommuri et al. – *External Torque Estimation Using Higher Order Sliding-Mode Observer for Robot Manipulators*                | 2022 / T-Mech        | Q1          | **Rigid-body**    | 47            |
|   4|  Q1.4  | Cao et al. – *Contact force and torque sensing for serial manipulator based*                                                 | 2021 / RCIM          | Q1          | **Contact Force** | 34            |
|   5|  Q1.5  | Zhang et al. – *Accurate Payload Dynamics Estimation and Compensation of a Robotic Manipulator without External*             | 2025 / EECR          | Q1          | **Payload**       | 0             |
|   6|  Q1.6  | Hu et al. – *On the Fully Decoupled Rigid-Body Dynamics Identification of Serial Industrial Robots*                          | 2025 / T-RO          | Q1          | **Both**          | 2             |
|   7|  Q1.7  | Liu et al. – *A Two-Stage Payload Dynamic Parameter Identification Method for Interactive*                                   | 2025 / T-ASE         | Q1          | **Payload**       | 1             |
|   8|  Q1.8  | Xu et al. – *Identifying Current Dynamics of Robot Payload*                                                                  | 2025 / TIM           | Q1          | **Both**          | 0             |
|   9|  Q1.9  | Xu et al. – *An accurate identification method based on double weighting for inertial parameters*                            | 2022 / Robotica      | Q1          | **Both**          | 10            |
|  10|  Q1.10 | Duan et al. – *Payload Identification and Gravity/Inertial Compensation for Six-Dimensional Force/Torque*                    | 2022 / Sensors       | Q1          | **Payload**       | 27            |
|  11|  Q1.11 | Wei et al. – *Composite Disturbance Filtering for Interaction Force Estimation With Online Environmental Stiffness*          | 2025 / T-Mech        | Q1          | **Contact Force** | 2             |
|  12|  Q1.12 | Huang et al. – *Toward Sensorless Interaction Force Estimation for Industrial Robots Using High-Order Finite-Time*           | 2022 / T-Mech        | Q1          | **Rigid-body**    | 57            |
|  13|  Q1.13 | Swevers et al. – *Dynamic Model Identification for Industrial Robots*                                                        | 2007 / IEEE CSM      | Q1          | **Both**          | 303           |
|  14|  Q1.14 | Wanke Yu et al. – *A Novel Sliding Mode Momentum Observer for Collaborative Robot Collision Detection*                       | 2022 / ADAII         | Q1          | **Rigid-body**    | 17            |
|  15|  Q1.15 | Dan Zhang et al. – *Dynamic Parameter Identification of Collaborative Robot Based on WLS-RWPSO Algorithm*                    | 2023 / ISAR          | Q1          | **Rigid-body**    | 16            |
|  16|  Q1.16 | Xu et al. – *An online payload identification method based on parameter difference for industrial robots*                    | 2024 / Robotica      | Q1          | **Payload**       | 2             |
|  17|  Q1.17 | Liu et al. – *Sensorless force estimation for industrial robots using disturbance observer and neural learning of friction*  | 2021 / RCIM          | Q1 + Q3     | **Contact Force** | 100           |
|    |        |                                                                                                                              |                      |             |                   |               |
|  18|  Q2.1  | Wei et al. – *Decoupling Observer for Contact Force Estimation of Robot Manipulators Based on Enhanced Gaussian*             | 2022 / CCIS          | Q2 + Q1     | **Contact Force** | 3             |
|  19|  Q2.2  | Wei et al. – *Contact Force Estimation of Robot Manipulators With Imperfect Dynamic Model: On Gaussian Process*              | 2024 / T-ASE         | Q2 + Q1     | **Contact Force** | 19            |
|  20|  Q2.3  | Fathi et al. – *Human-Robot Contact Detection in Assembly Tasks*                                                             | 2022 / ICMERR        | Q2 + Q3     | **Contact Force** | 2             |
|  21|  Q2.4  | Giacomuzzo et al. – *A Comparison Between Gaussian Processes and Neural Networks / GP vs. DeLaN*                             | 2023 / IFAC          | Q2 + Q4     | **Rigid-body**    | 2             |
|    |        |                                                                                                                              |                      |             |                   |               |
|  22|  Q3.1  | Tao et al. – *Robot Hybrid Inverse Dynamics Model Compensation Method Based on the BLL Residual Prediction Algorithm*        | 2025 / Robotica      | Q3          | **Rigid-body**    | 1             |
|  23|  Q3.2  | Lao et al. – *A Learning-Based Approach for Estimating Inertial Properties of Unknown Objects From Encoder Discrepancies*    | 2023 / RA-L          | Q3          | **Payload**       | 3             |
|  24|  Q3.3  | Kružić et al. – *End-Effector Force and Joint Torque Estimation of a 7-DoF Robotic Manipulator Using Deep Learning*          | 2021 / Electronics   | Q3          | **Rigid-body**    | 10            |
|  25|  Q3.4  | Pan et al. – *An adaptive sparse general regression neural network-based force observer for teleoperation system*            | 2023 / Eng. Apps. AI | Q3          | **Rigid-body**    | 9             |
|  26|  Q3.5  | Liang & Kroemer – *Contact Localization for Robot Arms in Motion without Torque Sensing*                                     | 2021 / ICRA          | Q3          | **Contact Force** | 5             |
|  27|  Q3.6  | Taie et al. – *Payload Parameters Identification Using Incremental Ensemble Learning*                                        | 2024 / ICCCR         | Q3          | **Payload**       | 1             |
|  28|  Q3.7  | Taie et al. – *Online Identification of Payload Inertial Parameters Using Ensemble Learning for Collaborative Robots*        | 2024 / RA-L          | Q3          | **Payload**       | 13            |
|  29|  Q3.8  | Taie et al. – *Addressing Catastrophic Forgetting in Payload Parameter Identification Using Incremental Ensemble Learning*   | 2024 / Frontiers     | Q3          | **Payload**       | 0             |
|    |        |                                                                                                                              |                      |             |                   |               |
|  30|  Q4.1  | Wu et al. – *Extended Deep Lagrangian Network for Robotic Arm Dynamics considering Motor Couplings*                          | 2025 / YAC Conf.     | Q4          | **Rigid-body**    | 0             |
|  31|  Q4.2  | Lutter et al. *Combining Physics and Deep Learning to learn Continuous-Time Dynamics Models*                                 | 2023 / arxiv         | Q4          | **Rigid-body**    | -             |
|  32|  Q4.3  | Yang et al. – *A Residual-Driven Decomposed PINNs Method for Dynamics Identification of Robot Manipulators                   | 2025 / RCAR          | Q4 + Q1     | **Rigid-body**    | 0             |
|  33|  Q4.4  | Hu et al. – *A PINN-Based Friction-Inclusive Dynamics Modeling Method for Industrial Robots*                                 | 2024 / IEEE TIE      | Q4 + Q3     | **Rigid-body**    | 8             |
|  34|  Q4.5  | Yang et al. – *Physics-Informed Neural Network for Model Prediction and Dynamics Parameter Identification of Collaborative*  | 2023 / IEEE RAL      | Q4          | **Rigid-body**    | 26            |
|  35|  Q4.6  | Yudie Hu et al. – *Improved deep Lagragian network-enabled momentum observer for collision detection during human-robot coll*| 2026 / RCIM          | Q4          | **Rigid-body**    | 1             |





---

### 📊 Survey's

|    | **Paper**                                                                                                                 | **Year / Venue**       | **Category (Q1–Q6 / Q8)** | **Focus**      | **Citations**          |
|--- | --------------------------------------------------------------------------------------------------------------------------| -----------------------| ------------------------- | -------------- | -----------------------|
|   1| Lee et al. – *Robot Model Identification and Learning: A Modern Perspective*                                              | 2024 / Ann. Rev. Ctrl. | Q8 (Survey)               | **Survey**     | 16                     |
|   2| Leboutet et al. – *Inertial Parameter Identification in Robotics: A Survey*                                               | 2021 / Appl. Sci.      | Q8 (Survey)               | **Survey**     | ~68 (MDPI) / ~100 (GS) |

---

### see links to read paper here:
- [Paper Links to read](relevant_SoA_links.md)

---

### see research trend here:
- [See detailed research trend](research_trend.md)

---

### see citation access date and links here
- [See detailed citation access](250924_233600_citations.md)

## Filter relevant impact State of Art
* **“Mature SoA”** = cited or relevant work. paper with impact for ape [2022-2025]

- **CMD Search done on IEEE**
- **Deep Search ChatGPT on ScienceDirect, Elvister, ArXiv & Goolge Scholar (peer reviewed on Google Scholar**)
- **Delivered Papers seen in table**


![Concept Graph](/docs/reserach/illustrations/250926_concept_graph.drawio.png)

---

## Concept Graph

- **category size depends on how much paper related to it**
- **esitmation/modeling terms and robotics contend size depents on how much paper related to it**
- **reference bubbls size with their amount of citation (maybe in combination with the relese date), showing the impact of each reference**

---


## 🔑 Observations

* **Rigid-body estimation SoA:** Classical observers (KF, SMO), GP-based observers, hybrid DOB+NN, and deep models (LSTM, GRNN, CNN).
* **Payload estimation SoA:** Primarily LS-based dynamic ID (with improvements like double weighting, RRTLS, current-based), ensemble learning, encoder-discrepancy learning, and safe online SysID.
* **Both (robot + payload):** Only a few, most notably **Hu et al. 2025 (FDRDI)** and **De León et al. 2022 (CNN Arm Param ID)**.
* **Trends:** Strong shift from pure observers (Q1) toward hybrid ML/physics-informed (Q4–Q5) and online adaptive/ensemble ML (Q6) for payloads.

---
---

# 📎 Peripheral Papers (Not Central)

These matched your CMD search but are not directly solving your core **payload/rigid-body estimation** problem:

* **Azulay et al. 2024** – *SightGAN: Augmenting Tactile Simulators* (ICRA).

  * Focus: tactile sim-to-real transfer, not payload ID.

* **Xin et al. 2024** – Programmatic Imitation Learning From Unlabeled and Noisy Demonstrations

  * Focus: program synthesis for imitation learning, not F/T or payload.

* **Zheng et al. 2023** – Uncertainty in Bayesian Reinforcement Learning for Robot Manipulation Tasks with Sparse Rewards

    * Issue: sparse rewards harm exploration & stability in robot manipulation
    * Results: improved convergence & stability across 4 manipulation tasks

* **Yuan et al 2025** -Optimization of Adaptive Algorithm for Precise Motion Control of Multi-Degree-of-Freedom Robotic Arms

* **Pezzato et al 2025** - Sampling-Based Model Predictive Control Leveraging Parallelizable Physics Simulations
 - Isaac
 


 
# Categories

1. **Classical**
   - Observers & filters using an analytic model (MO/GMO/DOB/KF/EKF/UKF/LS/RLS/WLS).
   - relevant papers: 1, 2, 4, 5, 6, 7, 9, 12, 16, 17

2. **Hybrid (Physics + Residual)**
   Start from (M,C,G) (or NE/EL) and learn a **correction** (GP or NN) that’s added to the model or fused in a filter.

3. **Pure Deep**
   NN learns dynamics/inverse dynamics **without** explicit physics (MLP/LSTM/GRU/TCN/Transformer).

4. **Physics-Informed**
   NN is **constrained by physics** (e.g., DeLaN/Lagrangian nets, PINNs, differentiable simulation). SINDY, NEURAL ODE

---

Just add up to **three** short tags so you keep it lightweight:

* **Sequence:** `LSTM/GRU`, `TCN`, `Transformer`
* **Use-case:** `contact force`, `collision`, `payload ID`, `inverse dynamics`

---

# One-line decision rules

* **Has nominal (M,C,G) and adds a learned fix?** → **Hybrid**.
* **No explicit physics at all?** → **Pure Deep**.
* **Physics is built into the NN (Lagrangian/PINN/DiffSim)?** → **Physics-Informed**.
* **Classic observers/filters/LS with no learning?** → **Classical**.

---

**“sequence”** means the method **models time series explicitly**—it takes a **window or stream of past samples** and learns the temporal dynamics, not just a single ((q,\dot q,\ddot q)) snapshot.

### What counts as “sequence”

* **RNNs (LSTM/GRU):** ingest one timestep at a time, keep a hidden **state** that carries info from the past → good for variable-length streams and online use.
* **TCN (Temporal Conv Nets):** use **causal 1-D convolutions** over time (often **dilated**) to capture long history with fixed latency → efficient and stable.
* **Transformers:** use **attention** over the window (or stream) to learn long-range temporal dependencies and context → strong but heavier.

### What does *not* count

* A plain **MLP** that sees only the **current** ((q,\dot q,\ddot q)) with no history.
* An MLP that sees a few handcrafted features but **no explicit time window/state**.

### Why it matters in robotics

* **Dynamics are history-dependent** (friction, backlash, compliance). Sequence models can infer these from recent motion.
* **Latency/causality:**

  * **Online control:** use **causal** models (LSTM/GRU/causal TCN) with a short window to keep delay low.
  * **Offline prediction:** you can use **bidirectional** TCN/Transformers for accuracy (but not for real-time control).
---

# PAPER TO CHECK


