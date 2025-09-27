[Back](deep_research.md)

## Analysis of Mature SoA Papers (2022–2025)

Below is the analysis of each *Mature State-of-the-Art* paper (2022–2025), describing its approach and identifying whether its focus is on **rigid-body estimation**, **payload estimation**, or **both**.

--- 35 Paper / C1 - C6 -> 33 / C8 --> 2 

### Nadeau et al. (ICRA 2022)

**“Fast Object Inertial Parameter Identification for Collaborative Robots.”**
- 🔗 [(https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/9916213)]

* **Focus:** Payload estimation
* **Summary:** Addresses low SNR in collaborative robot motion by combining approximate rigid-body dynamics with point-mass discretization to incorporate object shape. Enables faster, robust identification of inertial parameters (mass, CoM, inertia).

---

### Nadeau et al. (ICRA 2023)

**“The Sum of Its Parts: Visual Part Segmentation for Inertial Parameter Identification of Manipulated Objects.”**
- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/10160394]

* **Focus:** Payload estimation
* **Summary:** Combines RGB-D based part segmentation with force-torque sensing. Segments objects into homogeneous parts, estimating each part’s inertial parameters. Requires only slow, stop-and-go motions (safe near humans), yet achieves accurate full parameter identification. Validated on tool dataset and hammer-balancing demo.

---

### Kurdas et al. (ICRA 2022)

**“Online Payload Identification for Tactile Robots Using the Momentum Observer.”**
- [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/9811691]
* **Focus:** Payload estimation
* **Summary:** Real-time ID integrating momentum observer + recursive least-squares + filtering. Tested on Franka Panda; accurately detects changes in payload mass online with lower error than baselines.

---

### Kommuri et al. (IEEE/ASME T-Mech 2022)

**“External Torque Estimation Using Higher Order Sliding-Mode Observer for Robot Manipulators.”**
- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/9382115]

* **Focus:** Rigid-body (external force) estimation
* **Summary:** High-order sliding-mode observer, robust to friction/uncertainties, combined with Luenberger observer. Demonstrated on Sawyer robot; outperforms extended state observer baseline.

---

### Feng Cao et al. (Robotics & CIM 2021)

**“Contact force and torque sensing for serial manipulator based on an adaptive Kalman filter with variable time period”**

- 🔗 [https://www-1sciencedirect-1com-1000340761d0d.han.technikum-wien.at/science/article/pii/S0736584521000934]

* **Focus:** Rigid-body (force) estimation
* **Summary:** Adaptive Kalman filter (variable update rate) estimating contact forces/torques from motor currents. Robust to payload changes; validated on UR5 arm.

---

### Zhang et al. (EECR 2025)

**“Accurate Payload Dynamics Estimation and Compensation of a Robotic Manipulator without External Motion Measuring Sensors.”**
- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/11077346] 

* **Focus:** Payload estimation
* **Summary:** Online ID using only wrist F/T sensor + encoders. Stepwise approach (sensor biases → mass & CoM → inertia tensor). Achieved <10% error in payload mass.

---

### Hu et al. (IEEE T-RO 2025)

**“On the Fully Decoupled Rigid-Body Dynamics Identification of Serial Industrial Robots.”**

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/11029106]

* **Focus:** Both rigid-body and payload
* **Summary:** First method to fully decouple robot vs payload dynamics. Specialized excitation trajectories (S-curves) allow independent estimation. Improves torque prediction and payload CoM estimation.

---

### Liu et al. (IEEE T-ASE 2025)

**“A Two-Stage Payload Dynamic Parameter Identification Method for Interactive Industrial Robots with Large Components.”**

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/10947736]

* **Focus:** Payload estimation (heavy payloads)
* **Summary:** Static + dynamic (RRTLS) estimation. Optimized excitation trajectory. Demonstrated on 35 kg payload with very low error (<3 N, <1.7 Nm).

---

### Xu et al. (IEEE TIM 2025)

**“Identifying Current Dynamics of Robot Payload Based on Iterative Weighting Estimation.”**

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/10944553]

* **Focus:** Payload estimation
* **Summary:** Iterative weighted LS estimator using motor current signals (instead of torque sensors). Achieves lowest errors across methods, improves collision sensitivity.

---

### Xu et al. (Robotica 2022)

**“An accurate identification method based on double weighting for inertial parameters of robot payloads.”**
 - 🔗 [(https://www.cambridge.org/core/journals/robotica/article/abs/an-accurate-identification-method-based-on-double-weighting-for-inertial-parameters-of-robot-payloads/527798F0D816B5094A0A1A7118862C92)]

* **Focus:** Payload estimation
* **Summary:** Double-weighting method reduces outlier effects. Improves mass and CoM estimation accuracy by up to 43× over LS.

---

### Duan et al. (Sensors 2022)

**“Payload Identification and Gravity/Inertial Compensation for Six-Dimensional Force/Torque Sensor with a Fast and Robust Trajectory Design Approach”**

- 🔗 [https://www.mdpi.com/1424-8220/22/2/439]

* **Focus:** Payload estimation
* **Summary:** Trajectory optimization for payload parameter identification, improving accuracy of F/T sensor readings under payload influence.

---

### Wei et al. (IEEE T-ASE 2024)

**“Contact Force Estimation of Robot Manipulators with Imperfect Dynamic Model: On Gaussian Process Adaptive Disturbance Kalman Filter.”**

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/10144490]

* **Focus:** Rigid-body (force) estimation
* **Summary:** GP-augmented disturbance Kalman filter learns unmodeled dynamics. Outperforms traditional disturbance observers.

---

### Wei et al. (IEEE CCIS 2022)

**“Decoupling Observer for Contact Force Estimation of Robot Manipulators Based on Enhanced Gaussian Process Model.”**

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/10016359]

* **Focus:** Rigid-body (force) estimation
* **Summary:** Momentum observer + GP regression. Demonstrated on 3-DOF robot. Robust under uncertainties.

---

### Fathi et al. (ICMERR 2022)

**“Human-Robot Contact Detection in Assembly Tasks (using GP classifier).”**

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/10097827]

* **Focus:** Rigid-body (contact detection)
* **Summary:** GP classifier predicts probability of contact events with uncertainty bounds. More robust than thresholds.

---

### Lao et al. (IEEE RA-L 2023)

**“A Learning-Based Approach for Estimating Inertial Properties of Unknown Objects from Encoder Discrepancies.”**

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/10176292]

* **Focus:** Payload estimation
* **Summary:** Encoder-only CNN with attention mechanism estimates payload mass and CoM. Accurate without force sensors.

---

### Kružić et al. (Electronics 2021)

**End-Effector Force and Joint Torque Estimation of a 7-DoF Robotic Manipulator Using Deep Learning**

- 🔗 [https://www.mdpi.com/2079-9292/10/23/2963]

* **Focus:** Rigid-body (force/torque estimation)
* **Summary:** LSTM sequence models outperform MLPs for temporal dynamics. Key insight: sequence modeling matters.

---

### Pan et al. (Eng. Apps of AI 2023)

**An adaptive sparse general regression neural network-based force observer for teleoperation system**

- 🔗 [https://www-1sciencedirect-1com-1000340761d0d.han.technikum-wien.at/science/article/pii/S0952197622006790]

* **Focus:** Rigid-body (force estimation)
* **Summary:** Sparse adaptive GRNN with feature selection. Outperforms GP and NN baselines. High accuracy in teleoperation.

---

### Liu et al. (Robotics & CIM 2021)

**“Sensorless force estimation for industrial robots using disturbance observer and neural learning of friction approximation.”**

- 🔗 [https://www-1sciencedirect-1com-1000340761d0d.han.technikum-wien.at/science/article/pii/S0736584521000521]

* **Focus:** Hybrid rigid-body estimation
* **Summary:** Disturbance observer + NN for nonlinear friction. Reduced errors by \~66%.

---

### Bao et al. (IEEE T-II 2023)

**“Adaptive Neural Trajectory Tracking Control for n-DOF Robotic Manipulators with State Constraints.”**

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/9925646]

* **Focus:** Hybrid rigid-body estimation
* **Summary:** Controller fusing torque control + RBFNN + disturbance observer. Improves tracking accuracy.

---

### Tao et al. (Robotica 2025)

**“Robot hybrid inverse dynamics model compensation method based on the BLL residual prediction algorithm.”**
- 🔗 [(https://www.cambridge.org/core/journals/robotica/article/robot-hybrid-inverse-dynamics-model-compensation-method-based-on-the-bll-residual-prediction-algorithm/6499FF2BA9499B066EF376E4885A0186)]

* **Focus:** Rigid-body estimation
* **Summary:** Bagging ensemble of LSTMs compensates model residuals. Torque prediction error dropped from 0.5651 Nm → 0.1096 Nm.

---

### Yang et al. (RCAR 2025)

**“A Residual-Driven Decomposed PINNs Method for Dynamics Identification of Robot Manipulators.”**

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/11139811]

* **Focus:** Rigid-body estimation
* **Summary:** Combines LS with PINN residuals. Reduced torque prediction RMSE by 65%.

---

### Zhang et al. (arXiv 2025)

**“Provably-Safe, Online System Identification.”**

- 🔗 [http://arxiv.org/pdf/2504.21486v1]

* **Focus:** Payload estimation
* **Summary:** Interval arithmetic + safe trajectory optimization. Guarantees safety during payload ID.

---

### Taie et al. (IEEE RA-L 2024)

**“Online Identification of Payload Inertial Parameters Using Ensemble Learning for Collaborative Robots.”**

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/10372103]

* **Focus:** Payload estimation
* **Summary:** Online ensemble of NNs updated incrementally. Robust in real-time collaborative settings.

---

### Taie et al. (Frontiers in Robotics & AI 2024)

**“Addressing Catastrophic Forgetting in Payload Parameter Identification Using Incremental Ensemble Learning.”**

- 🔗 [https://www.frontiersin.org/articles/10.3389/frobt.2024.1470163/full]

* **Focus:** Payload estimation
* **Summary:** Incremental ensemble avoids forgetting. Maintains high accuracy across repeated payloads (\~0.007 kg error).

### Taie et al. 2024 ICCCR – *Payload Parameters Identification Using Incremental Ensemble Learning*.

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/10585532]

  * **Focus:** Payload estimation.
  * **Contribution:** Conference version; removes need for excitation trajectories, precursor to their RA-L & Frontiers works.

### De León et al. 2022 – *Parameter Identification of a Robot Arm Manipulator Based on a Convolutional Neural Network

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/9780143]

  * **Focus:** Both rigid-body and payload (maps torque/state signals → inertial parameters using CNN feature extraction).
  * **Contribution:** Introduces a vision-inspired CNN approach; more robust under noise than LS.

### Wu et al. 2025 – *Extended Deep Lagrangian Network for Robotic Arm Dynamics considering Motor Couplings.

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/11150193]

  * **Focus:** Rigid-body estimation (extends physics-informed deep models to capture motor couplings + nonlinear friction).
  * **Contribution:** Improves dynamics prediction on UR10e vs physics-only or NN-only.

### Peng et al. 2021 – Neural-Learning-Based Force Sensorless Admittance Control for Robots With Input Deadzone

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/9089219]

  * **Focus:** Rigid-body estimation (sensorless external torque observer + NN controller).
  * **Contribution:** First NN-based admittance control with deadzone compensation; avoids F/T sensors.

### Liu et al. (IJARS 2021)

**“External force estimation for robotic manipulator based on particle swarm optimization.”**

- 🔗 [(https://doi.org/10.1177/17298814211063744)]

* **Focus:** Rigid-body (force estimation)  
* **Summary:** Uses improved PSO for parameter identification, enabling sensorless external force estimation on Kinova Jaco2. Achieves 0.7 N RMSE. Robust compared to other metaheuristics.

---

### Leboutet et al. (Applied Sciences 2021)

**“Inertial Parameter Identification in Robotics: A Survey.”**

- 🔗 [(https://doi.org/10.3390/app11094303)]

* **Focus:** Survey / Benchmarking  
* **Summary:** Introduces BIRDy Matlab toolbox for systematic benchmarking of ID methods. Compares 17 approaches (LS, ML, IV, DIDIM, CLOE, CLIE, EKF, neural networks, PC methods). Establishes guidelines for method choice.

---

### Lee et al. (Annual Review of Control 2024)

**“Robot Model Identification and Learning: A Modern Perspective.”**  
🔗 [Paper link](https://doi.org/10.1146/annurev-control-061523-102310)

* **Focus:** Survey / Conceptual  
* **Summary:** Provides unified perspective on robot system identification, bridging classical rigid-body ID with modern ML and physics-informed models. Discusses geometry of inertial parameter identification, simulation vs. equation error, and challenges of data collection.

### Huang et al. (IEEE/ASME T-Mech 2025)

**“Toward Sensorless Interaction Force Estimation for Industrial Robots Using High-Order Finite-Time Observers.”**  
- 🔗 [(https://ieeexplore-1ieee-1org-100033c761c8f.han.technikum-wien.at/document/9484422)]

* **Focus:** Rigid-body (force estimation)  
* **Summary:** Proposes a high-order finite-time observer for robust sensorless interaction force estimation in industrial manipulators. Achieves fast convergence and high accuracy under uncertainties. Extends disturbance observer theory.

---

### Wei et al. (IEEE/ASME T-Mech 2025)

**“Composite Disturbance Filtering for Interaction Force Estimation With Online Environmental Stiffness Exploration.”**

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/10659631]

* **Focus:** Rigid-body (interaction force estimation)  
* **Summary:** Introduces a unified framework (EEFO) combining robot dynamics and force generation models with online environmental stiffness exploration. Demonstrated on surgical robot with silicone tissue model. Outperforms DO, NDO, GMO, KF, DKF baselines.

---

### Liang et al 2021** - Contact Localization for Robot Arms in Motion without Torque Sensing

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/9562058]

* **Problem/Issue**: Contact localization usually requires torque sensing.
* **SoA**: Contact particle filter, SVM classifiers, ML models with proprioception.
* **Focus**: **Contact localization via proprioception**.

---

### Xu et al., Robotica 2024 - An online payload identification method based on parameter difference for industrial robots

- 🔗 [https://www.cambridge.org/core/journals/robotica/article/abs/an-online-payload-identification-method-based-on-parameter-difference-for-industrial-robots/23B4C016AAF275A1C4BED7322CDB23FF]

* **Problem/Issue**:  Existing **online payload ID** methods often require external sensors (IMU, F/T) or rely on oversimplified **linear friction models**, limiting accuracy in real deployment.

* **Contribution**:
  * First **sensorless online payload ID** that incorporates **nonlinear friction** and parameter-difference formulation.
  * Outperforms previous methods in both **accuracy** and **practicality** for industrial robots.

---

### Dan Zhang et al. ISAR 2023 - Dynamic Parameter Identification of Collaborative Robot Based on WLS-RWPSO Algorithm

- 🔗 [https://www.mdpi.com/2075-1702/11/2/316]

### Wanke Yu et al. ADAII 2022 - A Novel Sliding Mode Momentum Observer for Collaborative Robot Collision Detection

- 🔗 [https://www.mdpi.com/2075-1702/10/9/818#:~:text=This%20paper%20presents%20a%20novel,accuracy%20to%20ensure%20safe%20PHRI]


## Summary & Trends

* **Payload estimation** dominates the field — crucial for safety and efficient manipulation.
* **Rigid-body external force/torque estimation** remains central for interaction control.
* A few (e.g. *Hu et al. 2025*) combine both.
* The trend: **physics-informed + learning-based methods** (GPs, neural nets, PINNs, ensembles) for robustness and accuracy.
