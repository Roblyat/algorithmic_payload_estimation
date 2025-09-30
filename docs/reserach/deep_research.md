## 1) Research Strategy

<!-- GitHub render -->
![Query Logic Diagram](/docs/reserach/illustrations/query_logic.drawio.png)

### Categories
- $C_1  =$ Classical / Observers
- $C_2  =$ Gaussian Process (GP) / Hybrid GP
- $C_3  =$ Deep Sequence Models (MLP / GRU / TCN / Transformer)
- $C_4  =$ Hybrid / Residual  
- $C_5  =$ Physics-Informed / Differentiable  
- $C_6  =$ Domain Adaptation / Latent Context
- $C_7. =$ Surveys & Overviews
- $C_8  =$ Reinforcement Learning
- $C_9  =$ Estimation Goal & Domain
  - $C_9a =$ Estimation & Modeling Terms
  - $C_9b =$ Robotics Context Terms

### Query Logic (Generalized Set Intersection)

### Combined Representation

$$
C = \{ C_1, C_2, \dots, C_8 \}
$$

$$
Q = \bigcup_{i=1}^{8} Q_i
$$

$$
Q_i = \left( \bigvee_{c \in C_i} c \right) 
\;\; \land \;\; 
\left( \bigvee_{e \in C9a} e \right) 
\;\; \land \;\; 
\left( \bigvee_{r \in C9b} r \right),
\quad i = 1, 2, \dots, 8
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

| Query                             | Total Papers | 2021–Current | Relev. SoA | Rigid-body | Payload | Both  |
| --------------------------------- | ------------ | ------------ | ---------- | ---------- | ------- | ----- |
| **Q1 – Classical / Observers**    | 15           | 13           | 13         | 4          | 8       | 1     |
| **Q2 – GP / Hybrid GP**           | 3            | 3            | 3          | 3          | 0       | 0     |
| **Q3 – Deep Sequence Models**     | 9            | 7            | 6          | 4          | 1       | 1     |
| **Q4 – Hybrid / Residual**        | 6            | 5            | 3          | 3          | 0       | 0     |
| **Q5 – Physics-Informed / Diff.** | 2            | 2            | 2          | 1          | 1       | 0     |
| **Q6 – Domain Adaptation**        | 12           | 8            | 6          | 2          | 4       | 0     |
| **Q8 – Surveys & Overviews**      | 2            | 1            | 1          | –          | –       | –     |
| **Total** without Surveys         | **47**       | **38**       | **33**     | **17**     | **14**  | **2** |

> **Note:** “Total Papers” excludes non-article references like textbooks.
> “2022–Present” counts publications from 2022 to 2025.
> "Mature SoA Papers" are a selected subset of highly relevant works in 2022–2025, often in top venues or showing notable impact.

---
---

# 📊 Mature SoA Papers (2021–2025) Classification
- [table only](relevant_SoA.md)

|    | **Paper**                                                                                                                    | **Year / Venue**     | **Category**| **Focus**      | **Citations** [250924_233600] |
|----| -----------------------------------------------------------------------------------------------------------------------------| -------------------- | ------------| -------------- | ------------- |
|   1| Nadeau et al. – *Fast Object Inertial Parameter Identification for Collaborative Robots*                                     | 2022 / ICRA          | Q1          | **Payload**    | 6             |
|   2| Nadeau et al. – *The Sum of Its Parts: Visual Part Segmentation for Inertial*                                                | 2023 / ICRA          | Q1          | **Payload**    | 10            |
|   3| Kurdas et al. – *Online Payload Identification for Tactile Robots Using the Momentum Observer*                               | 2022 / ICRA          | Q1          | **Payload**    | 15            |
|   4| Kommuri et al. – *External Torque Estimation Using Higher Order Sliding-Mode Observer for Robot Manipulators*                | 2022 / T-Mech        | Q1          | **Rigid-body** | 47            |
|   5| Cao et al. – *Contact force and torque sensing for serial manipulator based*                                                 | 2021 / RCIM          | Q1          | **Rigid-body** | 29            |
|   6| Zhang et al. – *Accurate Payload Dynamics Estimation and Compensation of a Robotic Manipulator without External*             | 2025 / EECR          | Q1          | **Payload**    | 0             |
|   7| Hu et al. – *On the Fully Decoupled Rigid-Body Dynamics Identification of Serial Industrial Robots*                          | 2025 / T-RO          | Q1          | **Both**       | 8             |
|   8| Liu et al. – *A Two-Stage Payload Dynamic Parameter Identification Method for Interactive*                                   | 2025 / T-ASE         | Q1          | **Payload**    | 0             |
|   9| Xu et al. – *Identifying Current Dynamics of Robot Payload*                                                                  | 2025 / TIM           | Q1          | **Payload**    | 1             |
|  10| Xu et al. – *An accurate identification method based on double weighting for inertial parameters                             | 2022 / Robotica      | Q1          | **Payload**    | 10            |
|  11| Duan et al. – *Payload Identification and Gravity/Inertial Compensation for Six-Dimensional Force/Torque*                    | 2022 / Sensors       | Q1          | **Payload**    | 35            |
|  12| Wei et al. – *Composite Disturbance Filtering for Interaction Force Estimation With Online Environmental Stiffness*          | 2025 / T-Mech        | Q1          | **Rigid-body** | 0             |
|  13| Huang et al. – *Toward Sensorless Interaction Force Estimation for Industrial Robots Using High-Order Finite-Time*           | 2025 / T-Mech        | Q1          | **Rigid-body** | 49            |
|    |                                                    -                                                                         |        -             |     -       |      -         | -             |
|  14| Wei et al. – *Decoupling Observer for Contact Force Estimation of Robot Manipulators Based on Enhanced Gaussian*             | 2022 / CCIS          | Q2          | **Rigid-body** | 3             |
|  15| Wei et al. – *Contact Force Estimation of Robot Manipulators With Imperfect Dynamic Model: On Gaussian Process*              | 2024 / T-ASE         | Q2 + Q4     | **Rigid-body** | 1             |
|  16| Fathi et al. – *Human-Robot Contact Detection in Assembly Tasks*                                                             | 2022 / ICMERR        | Q2          | **Rigid-body** | 0             |
|    |                                                    -                                                                         |        -             |     -       |      -         | -             |
|  17| Wu et al. – *Extended Deep Lagrangian Network for Robotic Arm Dynamics considering Motor Couplings*                          | 2025 / YAC Conf.     | Q3          | **Rigid-body** | 0             |
|  18| De León et al. – *Parameter Identification of a Robot Arm Manipulator Based on a Convolutional Neural Network*               | 2022 / IEEE Access   | Q3          | **Both**       | 11            |
|  19| Lao et al. – *A Learning-Based Approach for Estimating Inertial Properties of Unknown Objects From Encoder Discrepancies*    | 2023 / RA-L          | Q3          | **Payload**    | 4             |
|  20| Kružić et al. – *End-Effector Force and Joint Torque Estimation of a 7-DoF Robotic Manipulator Using Deep Learning*          | 2021 / Electronics   | Q3          | **Rigid-body** | 12            |
|  21| Peng et al. – *Neural-Learning-Based Force Sensorless Admittance Control for Robots With Input Deadzone*                     | 2021 / T-IE          | Q3 + Q4     | **Rigid-body** | 43            |
|  22| Pan et al. – *An adaptive sparse general regression neural network-based force observer for teleoperation system*            | 2023 / Eng. Apps. AI | Q3          | **Rigid-body** | **9**         |
|    |                                                     -                                                                        |        -             |     -       |      -         | -             |
|  23| Liu et al. – *Sensorless force estimation for industrial robots using disturbance observer and neural learning of friction*  | 2021 / RCIM          | Q4 + Q3     | **Rigid-body** | 90            |
|  24| Bao et al. – *Adaptive Neural Trajectory Tracking Control for n-DOF Robotic Manipulators With State Constraints*             | 2023 / T-II          | Q4          | **Rigid-body** | 4             |
|  25| Tao et al. – *Robot Hybrid Inverse Dynamics Model Compensation Method Based on the BLL Residual Prediction Algorithm*        | 2025 / Robotica      | Q4          | **Rigid-body** | 0             |
|    |                                                    -                                                                         |        -             |     -       |      -         | -             |
|  26| Yang et al. – *A Residual-Driven Decomposed PINNs Method for Dynamics Identification of Robot Manipulators                   | 2025 / RCAR          | Q5 + Q1     | **Rigid-body** | 0             |
|  27| Zhang et al. – *Provably-Safe, Online System Identification*                                                                 | 2025 / arXiv         | Q5          | **Payload**    | 0             |
|    |                                                    -                                                                         |        -             |     -       |      -         | -             |
|  28| Liang & Kroemer – *Contact Localization for Robot Arms in Motion without Torque Sensing*                                     | 2021 / ICRA          | Q6          | **Rigid-body** | 1             |
|  29| Taie et al. – *Payload Parameters Identification Using Incremental Ensemble Learning*                                        | 2024 / ICCCR         | Q6          | **Payload**    | 2             |
|  30| Taie et al. – *Online Identification of Payload Inertial Parameters Using Ensemble Learning for Collaborative Robots*        | 2024 / RA-L          | Q6          | **Payload**    | 1             |
|  31| Taie et al. – *Addressing Catastrophic Forgetting in Payload Parameter Identification Using Incremental Ensemble Learning*   | 2024 / Frontiers     | Q6          | **Payload**    | 0             |
|  32| Liu et al. – *External force estimation for robotic manipulator based on particle swarm optimization*                        | 2021 / IJARS         | Q6          | **Rigid-body** | 5             |
|  33| Xu et al. – *An online payload identification method based on parameter difference for industrial robots*                    | 2024 / Robotica      | Q6          | **Payload**    | 2             |
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