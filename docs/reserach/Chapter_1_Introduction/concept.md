# Notes

- sim to real with Isaac Sim domain randomization, good approach with domain randomization is has potential to perform on real robot as well
  - bad approaches with domain randomization not worth to test on real robot
---
- additional nonlinearities like drive-train effects (e.g. backlash, temperature) caused by carried payload still remain as unmodeled in this DeLaN + LSTM approach
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
  - 251204: rework aim of work, check truth of statements. for example:
    0. These works, however, focus purely on RDPI for a
       fixed robot-tool configuration and do not provide an explicit, measurement-frame model of the
       effective rigid body (robot+tool+payload) or its interaction wrench.
       --> need to keep that statement in mind and check if this really is true, task for later.

  ### 1.3.1 Research Question

  ### 1.3.2 Scientific Contribution

  In this work, a 6D force/torque sensor is used primarily as a **research instrument** to validate the proposed joint–space modelling approach in the end–effector measurement frame.
  During data collection, the FT sensor provides ground–truth flange wrenches (\vec{F}_{\mathrm{meas}}), which are used to (i) supervise the residual LSTM in Stage~2 and (ii) quantitatively assess the accuracy of the combined DeLaN+LSTM model in the sensor frame.
  However, the core models themselves are formulated and trained **in joint space** using only encoder and motor–current data; the mapping to the end–effector frame is performed afterwards via the Jacobian.

  Conceptually, this means that the FT sensor is **not structurally required** by the method, but only used in this thesis to demonstrate that the learned joint–space model does indeed produce a consistent wrench prediction in the measurement frame.
  If such consistency can be established, a practitioner could in principle follow the same pipeline **without an FT sensor**: train the DeLaN inverse–dynamics model and the sequence model on joint states and motor torques alone, operate the robot without any flange sensing, and perform payload identification purely from joint–space residuals that are subsequently mapped into the end–effector frame.

  In that sensor–free variant, the effective rigid body “robot+gripper” would be defined by the hardware configuration used during data collection (without FT sensor mass and lever arm), and payload–induced residuals could be interpreted directly in joint space and then transformed to the tool frame for PDPI.
  The FT–based experiments in this thesis should therefore be seen as a **validation of the joint–space modelling and frame transformation**, rather than as a strict requirement that all deployments must equip a permanent force/torque sensor.




Short answer: mathematically you’re doing the same thing as Hu et al., just written in a slightly more “canonical-DeLaN” way (with an explicit Lagrangian), and with a slightly more general friction block.

---

### 1. What Hu et al. actually do (in your notation)

Their dynamics equation is (their Eq. (7), (14), (15))
[
\tau
= M(q),\ddot q

* C(q,\dot q),\dot q
* g(q)
* \tau_f(\dot q),
  ]
  with

- **Inertia matrix via Cholesky network**
  [
  M(q) = \hat L(q;\theta),\hat L(q;\theta)^\top,
  ]
  where (\hat L = l_o(q;\theta) + l_d(q;\theta)) and (l_o,l_d) are MLPs (the “lo” and “ld” subnetworks). Diagonal entries of (l_d) are passed through ReLU + offset to ensure (M(q)\succ 0).

- **Gravity as a neural net**
  [
  g(q) = \hat g(q;\psi),
  ]
  where (\hat g) is another MLP (the “g” subnetwork). Later they use sine activations because every element of (M(q), g(q)) can be written as a linear combination of (\sin(\cdot)) terms for revolute manipulators.

- **Friction model**
  They add an explicit Coulomb–viscous model
  [
  \tau_f(\dot q)
  = f_c ,\mathrm{sgn}(\dot q) + f_v,\dot q,
  ]
  with learned joint-wise parameters (f_c, f_v) (their Eq. (13), (14)).

- **Torque prediction network**
  Plugging these into their “inverse dynamics” network (Eq. (15)), the DeLaN torque map is
  [
  \tau_{\text{pre}}
  = f^{-1}(q,\dot q,\ddot q;\theta,\psi)
  = M_\theta(q)\ddot q

  * C_\theta(q,\dot q)\dot q
  * g_\psi(q)
  * \tau_f(\dot q),
    ]
    where (M_\theta(q)=\hat L\hat L^\top) and
    (C_\theta) is implemented via the kinetic-energy derivatives in (7)/(11).

- **Loss**
  Parameters ((\theta,\psi)) are trained with
  [
  (\theta^*,\psi^*) =
  \arg\min_{\theta,\psi};
  \text{Loss}\bigl(f^{-1}(q,\dot q,\ddot q;\theta,\psi),\tau\bigr)

  * \lambda \Omega(\theta,\psi),
    ]
    i.e. MSE between predicted and measured joint torques plus (L_2) regularisation (their Eq. (12)).

That’s exactly what you are doing in Eq. (\mathcal{L}_{\mathrm{DeLaN}}), just written with your own symbols.

---

### 2. How this lines up with your methods text

Your Stage 1 model:

[
\hat{\tau}_{\mathrm{DeLaN}}
(q,\dot q,\ddot q;\theta,\psi)
==============================

\tau_{\mathrm{cons}}(q,\dot q,\ddot q;\theta)
+
\tau_{\mathrm{fric}}(\dot q;\psi),
]

with

[
\mathcal{L}*\theta(q,\dot q)
=\tfrac12 \dot q^\top M*\theta(q)\dot q - V_\theta(q)
]

and

[
\tau_{\mathrm{cons}}(q,\dot q,\ddot q;\theta)
= M_\theta(q)\ddot q + C_\theta(q,\dot q)\dot q + G_\theta(q).
]

So mapping notation:

* Their (M(q)) ↔ your (M_\theta(q)) (you also use a Cholesky-factor network, exactly as in Eq. (8)–(10) of the paper).
* Their (g(q)) ↔ your (G_\theta(q) = \partial V_\theta / \partial q).
  They approximate gravity directly; you approximate a potential (V_\theta) and get gravity as its gradient. Mathematically, if (g(q)) is conservative there exists a (V(q)) with (\partial V/\partial q=g), so your formulation is just a slightly more “structured” version.
* Their friction (\tau_f = f_c \mathrm{sgn}(\dot q)+f_v \dot q) ↔ your (\tau_{\mathrm{fric}}(\dot q;\psi)).
  If you choose
  [
  \tau_{\mathrm{fric}}(\dot q;\psi)
  = f_c \odot \mathrm{sgn}(\dot q) + f_v \odot \dot q,
  \quad
  \psi = (f_c,f_v),
  ]
  you are *exactly* using their Coulomb–viscous friction model. If instead you let (\tau_{\mathrm{fric}}) be a small NN with input ([\dot q, \mathrm{sgn}(\dot q)]), you are implementing their “f-subnetwork” generalisation (Sec. 4.1, Eq. (15) + discussion of (f) on page 5).

Your torque prediction written as
[
\hat{\tau}*{\mathrm{DeLaN}}(q,\dot q,\ddot q;\theta,\psi)
= f*{\mathrm{DeLaN}}(q,\dot q,\ddot q;\theta,\psi)
]
is identical in structure to their
[
\tau_{\text{pre}} = f^{-1}(q,\dot q,\ddot q;\theta,\psi).
]

Your loss
[
\mathcal{L}*{\mathrm{DeLaN}}(\theta,\psi)
= \frac1N \sum_k |\hat{\tau}*{\mathrm{DeLaN},k} - \tau_{\mathrm{motor},k}|_2^2
]
is the same as their Loss((f^{-1},\tau)), just without writing the (L_2) weight decay term explicitly.

So, conceptually:

* **You and Hu et al. are learning the same objects**: (M(q)), (C(q,\dot q)), (g(q)) and a friction map (\tau_f(\dot q)) from ((q,\dot q,\ddot q) \mapsto \tau).
* Your formulation via a scalar Lagrangian (\mathcal{L}_\theta = T - V) is slightly closer to the original Lutter DeLaN paper; Hu’s implementation collapses this to directly learning (M(q)) and (g(q)), but the resulting torque map is mathematically equivalent.

---

### 3. How this fits with your total two-stage pipeline

Given that:

1. **Stage 1** (your DeLaN) = “improved DeLaN” of Hu et al. (with optional extra structure like sine activations, if you want to be very close to them).
2. **Stage 2** (your LSTM on residual flange wrench) plays the same conceptual role as their “compensatory safety threshold” and other residual tricks: cleaning up the bits that the structured DeLaN can’t model well (backlash, stick–slip, drivetrain weirdness), but *you* do it in wrench space with a sequence model instead of in the observer’s threshold logic.

So you can safely say in the thesis:

* You adopt the *improved DeLaN* architecture of Hu et al. for learning (M_\theta(q)) and (g_\theta(q)) (and optionally their Coulomb–viscous friction block).
* Then you *extend* this with an additional LSTM residual model in the measurement frame.

If you want, I can help you write a small paragraph that explicitly says “we follow the DeLaN variant of Hu et al. for (M) and (g), but keep the original Lagrangian notation for clarity.”
