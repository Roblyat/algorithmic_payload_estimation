# Notes

- sim to real with Isaac Sim domain randomization, good approach with domain randomization is has potential to perform on real robot as well
  - bad approaches with domain randomization not worth to test on real robot
---
- additional nonlinearities like drive-train effects (e.g. backlash, temperature) caused by carried payload still remain as unmodeled in this DeLaN + LSTM approach
--- 
- drive-drain effects are stick–slip, small backlash, hysteresis, temperaure etc.
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

  Motivated by the SoA, this work adopts a DeLaN+LSTM architecture as a compromise between physical interpretability and data-driven flexibility. Physics-structured models such as DeLaN and PINNs have consistently shown the strongest robustness for inverse dynamics and friction modelling in industrial manipulators, especially when trained on encoder and motor data alone \cite{Q4_1_extended_delan_motor,Q4_2_lutter2023combiningphysicsdeeplearning,Q4_3_residual_pinns_dynamics_id,Q4_4_10729277}. Recent PINN-based approaches further demonstrate that augmenting a structured core with a temporal residual network (e.g. a TCN) can substantially improve torque prediction and friction compensation \cite{Q4_4_10729277}. At the same time, several studies indicate that LSTM-based residual learners are particularly effective in this domain: they successfully compensate modelling errors on top of rigid-body dynamics and achieve strong performance in joint-torque and end-effector force estimation, outperforming MLP and 1D-convolution baselines \cite{Q3_1_tao_bll,Q3_3_lstm_force_estimation}. Building on these findings, the present thesis follows the DeLaN+residual pattern but replaces the TCN with an LSTM, combining a physics-informed DeLaN backbone for nominal robot–gripper dynamics with a recurrent sequence model that captures history-dependent residual effects.

  ### 1.3.1 Research Questions

  ### 1.3.2 Scientific Contribution

  In this work, a 6D force/torque sensor is used primarily as a **research instrument** to validate the proposed joint–space modelling approach in the end–effector measurement frame.
  During data collection, the FT sensor provides ground–truth flange wrenches (\vec{F}_{\mathrm{meas}}), which are used to validate the accuracy of the combined DeLaN+LSTM model in the sensor frame.
  However, the core models DeLaN and LSTM themselves are formulated and trained **in joint space** using only encoder and motor–current data; the mapping to the end–effector frame is performed afterwards via the Jacobian.

  Conceptually, this means that the FT sensor is **not structurally required** by the method, but only used in this thesis to demonstrate that the learned joint–space model does indeed produce a consistent wrench prediction in the measurement frame.
  If such consistency can be established, a practitioner could in principle follow the same pipeline **without an FT sensor**: train the DeLaN inverse–dynamics model and the sequence model on joint states and motor torques alone, operate the robot without any flange sensing, and perform payload identification purely from joint–space residuals that are subsequently mapped into the end–effector frame.

  In that sensor–free variant, the effective rigid body “robot+gripper” would be defined by the hardware configuration used during data collection (without FT sensor mass and lever arm), and payload–induced residuals could be interpreted directly in joint space and then transformed to the tool frame for PDPI.
  The FT–based experiments in this thesis should therefore be seen as a **validation of the joint–space modelling and frame transformation**, rather than as a strict requirement that all deployments must equip a permanent force/torque sensor.

