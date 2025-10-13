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

### Yang et al. (RCAR 2025)

**“A Residual-Driven Decomposed PINNs Method for Dynamics Identification of Robot Manipulators.”**

- 🔗 [https://ieeexplore-1ieee-1org-100033c761d0a.han.technikum-wien.at/document/11139811]

* **Focus:** Rigid-body estimation
* **Summary:** Compares LS with PINN residuals. Reduced torque prediction RMSE by 65%. PINN performs real time well
and deals well with nonlinearity

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


---

# Disposition

## 1) Problem Analysis

### a) Preliminary Working Title

* Summarizes the content of the thesis in brief
* What was done in the thesis?
* Which context/environment is addressed?
* Is the title specific enough?

### b) Problem Description

* Describes the intended task as precisely as possible
* Subdivision into problem statement / motivation & research question

#### 1) Problem Statement / Problem Description

* Explains the subject area & outlines the context
* Description of the relevance from:

  * a scientific perspective
  * an application-oriented perspective

#### 2) Motivation / Scientific Research Gap

* Are there comparable works?
* Do these address the intended tasks?
* Practical relevance:

  * Is the solution needed in industry/companies?
  * Demonstrated through studies, surveys, literature sources, etc.

### c) Checklist

* 3 most important facts about the context
* Why does it exist?
* Why does this thesis need to be written?

---

## 2) Research Question

* What is to be investigated?

  * **Define clearly!!!**
* Represents the substantive basis of the thesis
* Can be subdivided into sub-questions

  * Specification into individual sections
* One central question & concrete sub-questions

### Checklist

* My thesis answers the following questions …
* My thesis is relevant because …
* Why is my thesis relevant for others?
* Are there gaps in the state of the art?
* The answer is not obvious, because …

---

## 3) Methodological Approach

* Describes the scientific procedure in the thesis
* Which methods are used to address the research question(s) & sub-questions?
* Justify the selection of methods

---

## 4) Expected Results

* Rough outline of the expected results (related to research questions & sub-questions)

  * How will the result of each chapter look?

### Checklist

* My thesis shows …
* Results in relation to research question(s) & sub-questions

---

![Focus Check](/docs/reserach/illustrations/focuscheck.png)
![Structure](/docs/reserach/illustrations/structure.png)
![Validate Questions 0](/docs/reserach/illustrations/validate_questions_0.png)
![Validate Questions 1](/docs/reserach/illustrations/validate_questions_1.png)

---
---
---

# 1 Introduction

## 1.1 Motivation

### 1.1.1 Context

* robotic arm manipulators increase more and more
  - since a very long time 📍 *bottom of the funnel* → $\textcolor{orange}{funnel_0}$
* collaborative robotic arm manipulation increases more & more 📍 $\textcolor{orange}{funnel_1}$ $\textcolor{violet}{c_1}$
    - safe manipulation, safe manipulation at payloads, safe collaborative manipulation with & without payload
        - awareness of payload 📍 $\textcolor{orange}{funnel_2}$
            - 📍 awareness fundamental for safe manipulation of payloads
                - and for collaborative payload manipulation

### 1.1.2 Use Case

* camera able to get shape and dimensions of payload, but no information of mass, CoM & inertia
* payload’s mass, CoM & inertia information fundamental for safe manipulation
* payload’s mass, CoM & inertia just identifiable with sensor data
* 🔑 methods to identify robot dynamic parameters and payload parameters is relevant for robotic arm manipulation tasks like pick & place tasks or collaborative manipulation
    - 📍 robot dynamic parameter identification (RDPI) $\textcolor{violet}{c_2}$
        - relevant for general robotic arm movement
    - 📍 payload dynamic parameter identification (PDPI) $\textcolor{violet}{c_2}$
        - relevant for pick & place and collaborative manipulation

    - both dynamic online & including everything $\textcolor{violet}{c_2}$
    - to estimate online what’s going on dynamically $[vel, acc, f/t]$
      - regression respecting friction, non linearity, noise  

* 🗝️ methods relevant to all robotic arm manipulation tasks $\textcolor{violet}{c_1}$
  - payload
  - surgery
  - collaborative
  - regression respecting friction, nonlinearity, noise

  ## 1.2 Problem Statement

  - to get information about the robot or payloads dynmaic parameters is not straight forward. There are problems for example non linearity and noise. 


  ## 1.3 Aim of this work

  ### 1.3.1 Research Question

  - (using a GAN is robust at this, but lacks of anything) what needs to be done to handle that lack?

  ### 1.3.2 Scientific Contribution
  - evaluating a GAN for payload dynmaic parameter identification and impoving the lack of anything.