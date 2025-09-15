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