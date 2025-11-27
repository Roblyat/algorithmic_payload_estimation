### Q1_1 - Fast_Object_Inertial_Parameter_Identification_for_Collaborative_Robots

**Key idea:**
Payload dynamic parameters can be identified **without using a rigid-body dynamics (RBD) model**.
The **PMD method** (point mass discretization) fits point masses into a **known payload geometry** (e.g., *dumbbell shape*).

**Estimation pipeline:**

* Use **gravity-only WLS–NE regressor (Â)** for slow motions.
* Use **full WLS–NE regressor** to enhance **inertia estimation** during dynamic trajectories.

**Findings:**

* **Mass:** very good accuracy.
* **Center of Mass (CoM):** acceptable, *reasonable* given the **short sequence durations** (~1.5 s trajectories).
* **Inertia:** poor accuracy, but still *reasonable* given the **short sequence durations** (~1.5 s trajectories).

---

### Q1_2 - Online_Payload_Identification_for_Tactile_Robots_Using_the_Momentum_Observer

**Concept:** Payload parameter identification without using a force–torque sensor. The robot executes the **same trajectory with and without payload**, and the method detects **payload-induced changes in motion and timeseries**.

**External torque extraction:** A time-consistent joint-space external torque $\tau_{\text{ext}}$ is computed, which **requires a nominal rigid-body model (RBD)**.

**Calibration:** A dedicated **calibration trajectory** is used. The **tool–object (gripper) parameters** are time-independent and denoted:

$$
\phi_{t,o}
$$

The payload parameters are obtained by **subtracting parameters**:

$$
\phi_L - \phi_0 = \tilde{\phi}
$$

where

- $\phi_L$ = parameters with payload  
- $\phi_0$ = parameters without payload  
- $\tilde{\phi}$ = identified payload parameters  

**Estimation method:**

- **Rated Least-Square Newton–Euler regressor** used to estimate $\phi_{\text{payload}}$  
- **Good results** for **mass**, **CoM**, and **inertia**, especially when a calibration run is included

---

### Q1_3 - External_Torque_Estimation_Using_Higher_Order_Sliding-Mode_Observer_for_Robot_Manipulators

**Concept:**  
Joint torque and external force estimation using a **nominal rigid-body model (NRB)**.  
The method compares **measured joint torques** with **computed joint torques** (NRB + controller torques).  
Controller forces are known and mapped to joint torques; external forces can be observed in **end-effector (EE) space** using the Jacobian.

**Assumptions:**  
- gravity model is accurate  
- bounded model errors  
- bounded known friction  

**Estimation method:**  
- **Sliding Mode Observer (SMO)**  
- Accurate in **joint space** and **end-effector space**  
- Provides **good results** for execution validation and external force estimation

---

## Q1.4 — End-Effector Contact Force Estimation (Clean Version)

**Concept:**  
End-effector (EE) contact force estimation using an **Adaptive Moment Filter (AMF)**.  
The method requires a **nominal rigid-body model (NRB)**.

The AMF estimates the EE wrench:

$$
\hat{f}
$$

If needed, the wrench is mapped into joint space using:

$$
\tau_{\text{ext}} = J^\top \hat{f}
$$

The filter also outputs the estimated wrench with covariance:

$$
\Sigma
$$

**Noise and parameter identification (offline):**  
Offline identification is used to determine:

- KF process noise $R$ 
- KF measurement noise $Q$ 
- covariance matrix $\Sigma$  
- motor torque constant $k_t$  
- gear ratio $D$  
- friction parameters $\Psi_f$

**Estimation method:**  
- **Adaptive Kalman Filter (AKF)**  
- Outperforms CKF in **cup-lifting EE-force estimation**  
- **Significantly reduces estimation error**

---

### Q1_5 - Accurate_Payload Dynamics Estimation and Compensation of a Robotic Manipulator without External Motion Measuring Sensors

**Concept:**  
Payload parameter identification **without requiring a nominal rigid-body model (NRB)** and **with an FT sensor**.  
The method uses the **same excitation trajectory** and computes the **offset bias** of the FT sensor.

- In **static poses**: payload **mass** and **CoM** are estimated.  
- In **dynamic motion**: payload **inertia** is estimated using the excitation trajectory.

**Estimation method:**  
- **Least-Square Regressor on torque residuals in the sensor frame**

**Performance:**  
- Works okay; approximately **~10% error** on payload mass  
- **No ground truth** for payload CoM and inertia → **no validation** of CoM & inertia results

--- 

### Q1_6 -  On_the_Fully_Decoupled_Rigid-Body_Dynamics_Identification_of_Serial_Industrial_Robots

**Concept:**  
Robot and payload parameter identification **without requiring a nominal rigid-body model (NRB)** and **without an FT sensor**.  
Uses **reciprocating S-curve symmetric trajectories**.

The method decouples:
- robot parameter / payload parameter estimation  
- joints  
- friction, gravity, and inertia parameters  

Decoupling is achieved using **CV/CA (constant-velocity/constant-acceleration RSC-S trajectories)** for gravitational parameters.

**Estimation method:**  
- **Ordinary Least-Squares (LS) Newton–Euler Regressor** for **RDPI** (robot dynamic parameter identification) and **PDPI** (payload dynamic parameter identification)

**Execution details:**  
- Multiple runs for RSC sections for RDPI & PDPI → enough data for trivial LS  
- **Good results** in RDPI & PDPI on RSC trajectories  
- PDPI ground truth from CAD  
- RDPI validated by comparing **measured** $\hat{\tau}_j$ vs **predicted** $\tilde{\tau}_j$
- Validation performed **offline**

---

### Q1_7 - Two-Stage Payload Dynamic Parameter Identification Method for Interactive Industrial Robots ith Large Components

**Concept:**  
Payload dynamic parameter identification **without requiring a nominal rigid-body model (NRB)** and **with an FT sensor**.  
The payload is directly connected to the **sensor frame $S$**.

A **two-stage LS method** is used:

1. **Static-pose LS** for payload **mass $m$** and **CoM** estimation  
2. **RRTLS** (recursive regularized TLS) inertia estimation using **dynamic Fourier trajectories**

**Experiment setup:**  
- Heavy ≈ 40 kg payload  
- 15 static poses (~2 s each)  
- 5 dynamic trajectories (~10 s each)

**Estimation method:**  
- **LS & RRTLS** for **PDPI** in sensor frame $S$, with payload **directly mounted on $S$**

**Performance:**  
- Strong PDPI results at heavy payloads  
- Good payload compensation & contact force estimation  
- **≈10 s contact estimation time**,  
- **≈40 s payload estimation time**

---

### Q1_8/Q1_9 - An accurate identification method based on double weighting for inertial parameters of robot payloads

**Concept:**  
Robot and payload dynamic parameter identification **without requiring a nominal rigid-body model (NRB)** and **without an FT sensor**, operating in **motor-current space $I_s$**.

Uses the **same Fourier trajectories** with and without payload:

- **RDPI w/o payload** via WLS  
- **PDPI with payload** via staged WLS using the same trajectories

**Estimation method:**  
- **Double-weighting WLS** on RDPI  
  → use same trajectory with & without payload  
  → RDPI → WLS on payload

**Performance:**  
- Good results on payload cylinder vs. CAD ground truth  
- Strong results in joint torque prediction  
- Validation performed **offline**

---

### Q1_10 - Payload Identification and Gravity_Inertial Compensation for Six Dimensional Force_Torque Sensor with a Fast and Robust Trajectory Design Approach

**Concept:** Payload dynamic parameter identification for **better trajectory design**.

- Results are **offline**, with **no ground truth**  
- Used only as a reference for **static-pose payload mass & CoM estimation**

---

### Q1_11 - Composite Disturbance Filtering for Interaction Force Estimation With Online Environmental Stiffness Exploration

**Concept:** Surgery → just for the beginning of the **funnel**.

---

### Q1_12 - Toward Sensorless Interaction Force Estimation for Industrial Robots Using High-Order Finite-Time Observers

**Concept:**  
EE interaction force estimation **without requiring an NRB model**.  
The FT sensor is used **only for ground truth**, not for the estimation itself.

An **offline LS–NE regressor** is used to identify the robot model:

- $M(q)$ — inertia matrix  
- $C(q)$ — Coriolis terms  
- $G(q)$ — gravity terms  

Online **HOFFTO** runs in joint space using the Jacobian $J(q)$, and outputs the estimated EE-interaction force using the **offline-identified robot model** as input.

**Estimation method:**  
- **LS–NE regressor** to estimate RDP **offline** → used as input for the **Higher-Order Finite-Time Observer (HOFFTO)**

**Performance:**  
- Good offline RDP → leads to good HOFFTO predicted joint torque $\tilde{\tau}_j$  
- **Okay** results in EE-interaction force estimation vs. FT-sensor ground truth

---

### Q1_13 - Dynamic_Model_Identification_for_Industrial_Robots

**Concept:**  
Robot and payload dynamic parameter identification **without an NRB model** and **without an FT sensor**.  
All identification occurs in **joint space**.

First, perform **RDPI** using an LS–NE regressor in joint space:

$$
\tau_j = \phi(q, \dot{q}, \ddot{q}) \, \theta
$$

Then, use another LS–NE regressor for payload identification:

$$
\tilde{\tau} = \phi_j^{b} \theta_j^{b} + \phi_L^{e} \theta_L^{e}
$$

→ identifies payload parameters $\theta_L$ = **PDPI**, using the **same trajectories** with and without payload.

**Estimation method:**  
- **LS–NE regressor for RDPI**  
- **LS–NE regressor for PDPI**

**Performance:**  
- Very strong results for both RDPI and PDPI  
- Performed **offline**

---

### Q1_14 - A Novel Sliding Mode Momentum Observer for Collaborative Robot Collision Detection

**Concept:**  
Collision detection that **requires a nominal rigid-body model (NRB)**.  
Performs **binary contact force detection** and **identifies the collided joint index**.

Method:  
An LS–NE regressor (NRB) + controller torques gives the robot torque estimate:

- Known robot torque: $\tilde{\tau}_{\text{robot}}$
- Measured torque: $\tau_{\text{measured}}$

Estimated applied contact force:

$$
\tau_{\text{contact}} = \tau_{\text{measured}} - \tilde{\tau}_{\text{robot}}
$$

Collision decision:

- If $\tau_{\text{contact}} > \text{threshold}$ → **binary contact = true**  
- Joint index identified from $\tau_{\text{contact}} \in \{0, \dots, 6\}$

**Estimation method:**  
- **LS–NE regressor + controller joint torques** for $\tilde{\tau}_{\text{robot}}$  
- Uses $\tau_{\text{measured}}$ for contact force detection & joint index estimation

**Performance:**  
- Good results in **contact force binary detection**  
- Good results in **collided joint index detection**

---

### Q1_15 - Dynamic Parameter Identification of Collaborative Robot Based on WLS-RWPSO Algorithm

**Concept:**  
Robot dynamic parameter identification **without requiring an NRB model**.  
*Goal*: predict joint torques $\tilde{\tau}_j$.

**Estimation method:**  
- **WLS optimized with particle swarm**  
- Dataset is **preprocessed** using a **KLT-denoised** dataset

**Performance:**  
- Very good results in $\tilde{\tau}_j$ joint torque prediction  
- Strong agreement vs. controller applied torque

---

### Q1_16 - An-online-payload-identification-method-based-on-parameter-difference-for-industrial-robots

**Concept:**  
Payload dynamic parameter identification **without requiring an NRB model**.

Workflow:
1. **Offline**: Train an LS–NE regressor for the **robot base parameters** $\theta$.  
2. **Online**:  
   - Compute **measured joint torque**  
   - Subtract predicted **robot torque**  
   - The **residual torque** corresponds to payload torque  
3. Apply **LS–NE regressor** on the residual torque to estimate **payload parameters**

**Estimation method:**  
- **RLS–NE regressor** on joint torque & residual torque for payload

**Performance:**  
- Strong results in joint torque & payload **mass**, **CoM**, and **inertia**  
- Matches **CAD ground truth** closely

---

### Q1_17 - Sensorless force estimation for industrial robots using disturbance observer and neural learning of friction approximation

**Concept:**  
End-effector (EE) force estimation **requiring an NRB model**.  
A **Disturbance Kalman Filter (DKF)** combined with a **NN friction model** is evaluated.

Comparisons include:
- DKF + Stribeck friction  
- DKF + NN friction  
- DKF + NN friction vs. model-based methods  
- DKF + NN friction vs. **general momentum observer**

**Estimation method:**  
- **DKF + NN friction**  
- **DKF + Stribeck friction**  
- model-based estimation  
- general momentum observer

**Performance:**  
- NN friction vs. Stribeck → **NN is better**, still **2.17–7.18 Nm error**  
- DKF good **without external force**  
- With external torque → **0.13 Nm – 8.85 Nm error** → **bad results**

---