---
# Scientific Contribution

## Table of Contents
0. [What we need to do first](#0-what-we-need-to-do-first)
1. [What is on the market](#1-what-is-on-the-market)
2. [Questions](#2-questions)
3. [Important things we need to remember](#3-important-things-we-need-to-remember)

---

## 0) What we need to do first

- Check the 4 papers in here plus their references
- Do big research again and compare to papers we already have
- Check release date of papers we already have (we need 2023-2025, except for important exceptions)
- cmd search --> search with IEEE Keywords, not Author Keywords

---

## 1) What is on the market

**Accurate Payload Dynamics Estimation and Compensation of a Robotic Manipulator without External Motion Measuring Sensors [2025]**  
https://doi-1org-100033c7611df.han.technikum-wien.at/10.1109/EECR64516.2025.11077346

- Classical methods just work as long as payload is manipulated 
- Force sensor gets damaged by contact with environment
- Good SOTA 2025 and classification in SOTA
- Good mathematics
- **Check references of this paper first and check this paper closely!**

**Contact force and torque sensing for serial manipulator based on an adaptive Kalman filter with variable time period [2021]**  
https://doi-1org-10003407611e5.han.technikum-wien.at/10.1016/j.rcim.2021.102210

- Good mathematics 
- Good SOTA for paper's release date 
- Force sensor gets damaged by contact with environment

**Neural-Learning-Based Force Sensorless Admittance Control for Robots With Input Deadzone [2020]**  
https://doi-1org-100033c7611fc.han.technikum-wien.at/10.1109/TIE.2020.2991929

- Good SOTA for NN, based on release date
- Good mathematics

**An adaptive sparse general regression neural network-based force observer for teleoperation system [2023]**  
https://doi-1org-1000340761202.han.technikum-wien.at/10.1016/j.engappai.2022.105689

- NN SOTA
- Okay mathematics
- No robot dynamics

**The Dynamic Model of the UR10 Robot and Its ROS2 Integration [2025]**
https://doi-1org-100033c7614d9.han.technikum-wien.at/10.1109/TII.2025.3534415
- the badest paper i have ever seen 
- all references before 2017, lastest reference is 2017
- this paper just apperars here to verify by Michael/Mohammed if this is a paper worth to read 
- do i need to take papers like this into account? How the fuck this paper passed the review ?????????

---

## 2) Questions

- What about these observers (classical model-based observer methods)? Why can't they work without accurate model dynamics?
    - Disturbance observer
    - Dynamic state observer
    - Momentum-based observer

- How does the best mathematical expression for my problem look?

---

## 3) Important things we need to remember

- We want to eliminate the need for the dynamic parameters of the robot

## 4) What is searched to this point
- "payload estimation" clocked into IEEE Search line --> gave 136 results, matching papers to ape-problem already in citavi

## 5) Index Term Pool

Perfect — let’s make this clean. I’ll add a **new category: “Estimation Goal & Domain”** that contains only unique, well-scoped search terms. I’ll also keep capitalization consistent so you can copy them directly into searches.

---

## 6) 🔑 Categories with Keywords (full, acronyms, no duplicates)

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
* inertial parameter identification (IPI)
* payload identification (PI)
* payload estimation (PE)
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


## **5. Physics-Informed / Differentiable**

* physics-informed neural network (PINN)
* differentiable physics
* differentiable simulation (DiffSim)
* differentiable robot model
* differentiable dynamics
* neural ODE (NODE)
* torchdiffeq
* ODE-net
* Isaac Gym differentiable
* Isaac Lab differentiable
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



## **7. Estimation Goal & Domain**

*(to combine with one of the above categories)*

* external force
* force measurement
* force estimation
* force/torque estimation
* wrench estimation
* joint torque estimation
* end-effector force
* end-effector torque
* inertial parameters
* inertial parameter identification
* online payload identification
* robot payload identification
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
* robotic manipulator
* robotic arm
* robotic manipulation
* external perturbations
* force torque sensor (F/T sensor)
* robot payload
* noise
* signal noise
* noise estimation
* external force estimation (EFE)
* external torque estimation (ETE)
* force estimation
* torque estimation
* wrench estimation
* parameter identification differentiable simulation

---
---

## 7) key references:
thats what i am looking for too exactly, but for example:

- Fast Object Inertial Parameter Identification for Collaborative Robots: 10.1109/ICRA46639.2022.9916213

-An Efficient Parameter Identification Framework: A Case Study on Robot Manipulators:10.1109/TIM.2025.3542857

- On the Fully Decoupled Rigid-Body Dynamics Identification of Serial Industrial Robots: 10.1109/TRO.2025.3578229


concept map:
https://www.researchgate.net/figure/Citation-network-mapping-the-citations-from-reviews-to-RCTs-testing-dietary-fat_fig1_325349621
