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

  #### Evidence map (Context)
- robotic arm manipulators increase more and more → [Q0.1], [Q0.2]
- collaborative manipulation increases → [Q1.3]
- awareness of payload is fundamental → [Q1.1], [Q1.2], [Q1.3], [Q1.5], [Q1.6], [Q1.7], [Q1.8/Q1.9], [Q1.10], [Q1.11], [Q1.13], [Q1.14], [Q3.2]
    - get other categories also into this statement!

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

#### Evidence map (Use Case)
- shape from camera; no mass/CoM/inertia → [Q0.4], [Q3.2]
- mass/CoM/inertia needed for safety → [Q1.1], [Q0.4], [Q1.2], [Q1.5], [Q1.7], [Q1.9], [Q1.10], [Q3.2]
- online RDPI/PDPI for pick&place/collab → [Q0.4], [Q1.2], [Q1.5], [Q1.7], [Q3.6], [Q3.7], [3.8]
- online awareness of contact/forces without external wrist F/T → [Q1.3], [Q1.4], [Q1.11], [Q1.12], [Q1.14], [Q2.1], [Q2.2], [Q2.3], [Q3.3], [Q3.4], [Q1.17], [3.5]

  ## 1.2 Problem Statement

  - Problem description (mathematics)
  - Limitations of SoA -> problem statement

  ## 1.3 Aim of this work
  - 251204: rework aim of work, check truth of statements. for example:
    0. These works, however, focus purely on RDPI for a
       fixed robot-tool configuration and do not provide an explicit, measurement-frame model of the
       effective rigid body (robot+tool+payload) or its interaction wrench.
       --> need to keep that statement in mind and check if this really is true, task for later.

  ### 1.3.1 Research Question

  ### 1.3.2 Scientific Contribution