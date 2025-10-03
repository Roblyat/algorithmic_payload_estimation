## 1) Data-Science Problem

- **Inputs** per time $t$: joint positions $q_{1..6}$, velocities $\dot q_{1..6}$, accelerations $\ddot q_{1..6}$ (and optionally efforts $I_{1..6}$).
- **Outputs** per time $t$: either **efforts** $I_{1..6}$ (stage 1) or **wrench** at MF $(f_x,f_y,f_z,\tau_x,\tau_y,\tau_z)$ (stage 2).

**Goal:** Estimate the **payload’s contribution** to the wrench at MF while manipulating **robot + payload**, even though training cannot include payload cases. Practically: predict the **gripper-only** wrench $\hat w_{\text{gripper}}$ from robot states, subtract from measured $w_{\text{meas}}$ to isolate $w_{\text{payload}} = w_{\text{meas}} - \hat w_{\text{gripper}}$.

- Two Stage vs. One Stage Mapping

# 2) Approaches — what they are, why they fit, and whether they’re SOTA

Below I expand the table into concrete guidance:

### A) LSTM / GRU (sequence models) — ✅ highly relevant, widely used SOTA for dynamics

* **What**: Recurrent nets that ingest a **window of history** (e.g., last 200–500 ms sampled at 100–250 Hz: 20–125 steps).
* **Why it fits**: Captures **hysteresis**, friction regimes, controller memory, and timing effects your GP misses.
* **How**: Inputs per step: $[q,\dot q,\ddot q]$ (and optionally $I$). Predict current **effort** or **wrench**. You can do **teacher forcing** during training; causal inference at test.
* **SOTA?** Yes for robotic dynamics when data volume is decent and latency is tight. Lightweight GRUs are very deployable in ROS 2.

### B) Temporal Convolution (1D Conv / TCN) — ✅ strong, fast alternative

* **What**: Dilated causal 1D convs over time windows (Temporal Conv Nets).
* **Why**: Parallelizable on GPU, **lower latency** than RNNs, often **equal or better** accuracy on dynamics.
* **SOTA?** Yes; many modern control pipelines use causal Conv1D/TCN as the default.

### C) Transformers (tiny causal) — 🔸 powerful but heavier

* **What**: Self-attention over a short window (say 64–128 steps).
* **Why**: Can model long-range dependencies and multi-output coupling.
* **Trade-off**: Heavier than GRU/TCN; still fine on a 4070 Ti; overkill if window is short.
* **SOTA?** Increasingly common; use **Flash-Attention** and small models for real-time.

### D) MLP / ResNet + BatchNorm on **stacked context** — ✅ simple, strong baseline

* **What**: Concatenate $k$ past steps into one big feature vector, pass through a deep MLP/ResNet.
* **Why**: Simplicity; can already beat your GP, great as a **first baseline**.
* **SOTA?** As a baseline — yes; not the ultimate, but fast and reliable.

### E) Hybrid Dynamics + NN (residual learning) — ✅ best-of-both

* **What**: Use **analytical inverse dynamics** (RNEA) & Jacobian to compute nominal torques/wrench for the robot+gripper. Train a **NN to predict the residual** (unmodeled friction, flex, sensor bias).
* **Why**: Leverages physics to reduce the function the NN must learn → **better generalization** and **data efficiency**.
* **SOTA?** Yes. This is a top pattern in modern robotics (“residual learning”, “Learning from residuals”).

### F) Physics-Informed Neural Nets (PINNs) / differentiable dynamics — ✅ excellent fit

* **What**: Build the dynamics into the **loss** or **forward pass** (differentiable Newton–Euler; PyTorch + Pinocchio or Isaac Gym/PhysX). Penalize violations of equations of motion.
* **Why**: Enforces **physical plausibility**, helps under shift, and supports **latent parameter inference** (learn a per-episode latent $\phi_{\text{payload}}$).
* **SOTA?** Yes for research & increasingly in products.

### G) Domain adaptation / meta-learning — ✅ targeted fix for gripper→payload shift

* **What**: Learn **payload-invariant** representations (adversarial domain confusion), **test-time adaptation** (update normalization/affine layers online), or **meta-learn** to quickly adapt a small latent that encodes payload.
* **Why**: You **cannot train on payload**; you still need to adapt at deployment.
* **SOTA?** Yes; test-time adaptation and latent-context models are current best practice for hidden context shifts.

### H) Deep ensembles / heteroscedastic heads — ✅ safety & robustness

* **What**: Predict mean **and** variance; or ensemble small models.
* **Why**: Gives **uncertainty** for safety logic (slow down/stop when uncertainty spikes).
* **SOTA?** Yes; commonly used in robotics for risk-aware control.

## 4) 🔎 Research Results

<img src="query_logic.png" alt="Query Logic Diagram" width="550" height="">

### Categories
- $C_1 =$ Classical / Observers
- $C_2 =$ Gaussian Process (GP) / Hybrid GP
- $C_3 =$ Deep Sequence Models (MLP / GRU / TCN / Transformer)
- $C_4 =$ Hybrid / Residual  
- $C_5 =$ Physics-Informed / Differentiable  
- $C_6 =$ Domain Adaptation / Latent Context  

### Query Logic (Generalized Set Intersection)

### Combined Representation

$$
C = \{ C_1, C_2, \dots, C_6 \}
$$

$$
Q = \bigcup_{i=1}^{6} Q_i
$$

$$
Q_i = \left( \bigvee_{c \in C_i} c \right) 
\;\; \land \;\; 
\left( \bigvee_{e \in C7a} e \right) 
\;\; \land \;\; 
\left( \bigvee_{r \in C7b} r \right),
\quad i = 1, 2, \dots, 6
$$

---

## 📊 Research Results Table

| Query   | Total Papers | Papers (2022–Current)  | Mature SoA [2022-2025]| Emerging SoA [2025] |
|---------|--------------|------------------------|-----------------------|---------------------|
| $Q_1$   |     459      |          148           |           4           |          0          |
| $Q_2$   |      21      |            8           |           3           |          0          |
| $Q_3$   |     165      |          117           |                       |                     |
| $Q_4$   |              |                        |                       |                     |
| $Q_5$   |              |                        |                       |                     |
| $Q_6$   |              |                        |                       |                     |


## Filter relevant impact State of Art
* **“Mature SoA”** = older but highly cited works [2022-2025]
* **“Emerging SoA”** = [2025]
---
---
<img src="concept_graph.png" alt="Concept Graph" width="750" height="">