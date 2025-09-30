---
# Scientific Contribution

## Table of Contents
1. [What is on the market](#1-what-is-on-the-market)
2. [Questions](#2-questions)
3. [Important things we need to remember](#3-important-things-we-need-to-remember)
---

## 1) What is on the market / SIGNIFICANT PAPER / **Key References?**

- CHECK THESE PAPER RELATED WORK --> that finishes research

* **Accurate Payload Dynamics Estimation and Compensation of a Robotic Manipulator without External Motion Measuring Sensors [2025]**  
https://doi-1org-100033c7611df.han.technikum-wien.at/10.1109/EECR64516.2025.11077346

    - Classical methods just work as long as payload is manipulated 
    - Force sensor gets damaged by contact with environment
    - Good SOTA 2025 and classification in SOTA
    - Good mathematics
---

* **An adaptive sparse general regression neural network-based force observer for teleoperation system [2023]**  
https://doi-1org-1000340761202.han.technikum-wien.at/10.1016/j.engappai.2022.105689

    - NN SOTA
    - Okay mathematics
    - No robot dynamics
---

* **On the Fully Decoupled Rigid-Body Dynamics Identification of Serial Industrial Robots** [2025]
https://ieeexplore-1ieee-1org-100033c761aa6.han.technikum-wien.at/document/11029106

    - good mathmatics
    - current sota kalman filter classical observers
---

* **Fast Object Inertial Parameter Identification for Collaborative Robots (ICRA 2022)** [(https://arxiv.org/abs/2203.00830)]
---

* **The Sum of Its Parts: Visual Part Segmentation for Inertial Parameter Identification (ICRA 2023)** [(https://arxiv.org/abs/2302.06685)]
---

* **External Torque Estimation Using Higher-Order Sliding-Mode Observer (IEEE/ASME T-Mech 2022)** 
[(https://www.researchgate.net/publication/350072224_External_Torque_Estimation_using_Higher-order_Sliding_Mode_Observer_for_Robot_Manipulators)]

    - oberver high cited (46) with bad results
---

* **Accurate Identification Method Based on Double Weighting (Robotica 2022)** 
[(https://www.cambridge.org/core/journals/robotica/article/abs/an-accurate-identification-method-based-on-double-weighting-for-inertial-parameters-of-robot-payloads/527798F0D816B5094A0A1A7118862C92)]

    - payload parameter identification !
---

* **Robot Hybrid Inverse Dynamics Model Compensation Based on BLL Residual Prediction (Robotica 2025)** 
[(https://www.cambridge.org/core/journals/robotica/article/robot-hybrid-inverse-dynamics-model-compensation-method-based-on-the-bll-residual-prediction-algorithm/6499FF2BA9499B066EF376E4885A0186)]

    - compared method to predict robot dynamics
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