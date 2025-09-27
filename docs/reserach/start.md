
---
# Thesis Guidance: Data-Science Problem & Solutions for ROS 2/Isaac Lab

## Table of Contents
1. [Why GP Struggles Here](#1-why-gp-struggles-here-even-with-lots-of-data)
2. [Your Data-Science Problem, Precisely](#2-your-data-science-problem-precisely)
3. [Approaches — SOTA & Fit](#3-approaches--what-they-are-why-they-fit-and-whether-theyre-sota)
4. [Advanced Solutions (Concrete Designs)](#4-the-advanced-solutions-you-asked-me-to-expand-with-concrete-designs)
5. [Practical Plan for ROS 2/Isaac Lab](#5-a-practical-plan-you-can-run-in-ros-2isaac-lab-and-latency-you-can-expect)
6. [Next Steps](#6-what-i-suggest-doing-next-concrete--fast)
7. [Writing plan for the State of the Art chapter](#7-writing-plan-for-the-state-of-the-art-chapter)
8. [Practical tooling and organization](#8-practical-tooling-and-organization)
9. [Timeline you can paste into your disposition](#9-timeline-you-can-paste-into-your-disposition)
10. [What I need from you to start the loop](#10-what-i-need-from-you-to-start-the-loop)

---

## 1) Why GP Struggles Here (Even with Lots of Data)

Even with 200k samples and randomized trajectories, your GP falls down **live** because the problem violates several GP assumptions:

**a) Temporal dependence (non-IID).**
GP regression, as you’re using it, treats each row as independent. Your system is **dynamical**: gear friction, Coulomb/stick-slip, backlash, actuator heating, sensor latency, and controller behavior create **history-dependent** effects. Without sequence context, the same $(q,\dot q,\ddot q)$ can map to different efforts/wrenches.

**b) Non-stationary & heteroscedastic noise.**
Noise variance changes with configuration, speed, direction, payload, and contact state. A single stationary RBF kernel + homoscedastic likelihood struggles; it tends to “average out” and then overfits local pockets.

**c) Multi-output, strongly coupled targets.**
Efforts across 6 joints and f/t components are **correlated**. Scalar-output GPs (or independent heads) ignore cross-covariances → poor generalization. Multi-output GPs exist, but scale badly.

**d) Two-stage error compounding.**
You first estimate **effort** and then feed the *estimated* effort into the **wrench** GP. Any bias/noise in stage 1 gets **amplified** in stage 2, especially when the second mapping is steeper (nonlinear Jacobians + dynamics).

**e) Distribution shift with latent constant (payload).**
During deployment, the hidden variable $\phi_{\text{payload}}$ (mass/CoM/inertia) **shifts the mapping** in a way that’s not “seen” in training (you trained on gripper-only). Classic covariate shift + **latent domain factor**. Vanilla GP has no mechanism to infer or adapt to that hidden constant online.

**f) Extrapolation.**
GPs interpolate beautifully **within** the support of training data. The payload injects affine + nonlinear changes to the mapping (e.g., gravity, inertia). That’s effectively **extrapolation** in function space → unreliable.

**g) Real-time constraints.**
Even sparse GPs with inducing points get expensive if you push sequence context, heteroscedastic likelihoods, or multi-output kernels.

> **Bottom line:** The **physics + deployment constraints** (dynamic, heteroscedastic, multi-output, latent shift, two-stage) don’t match what a plain GP is good at. That’s why you see “great test R²” but **poor live performance**: the test split is IID-ish, the world is not.

---

## 2) Your Data-Science Problem, Precisely

You have synchronized time series of:

- **Inputs** per time $t$: joint positions $q_{1..6}$, velocities $\dot q_{1..6}$, accelerations $\ddot q_{1..6}$ (and optionally efforts $I_{1..6}$).
- **Outputs** per time $t$: either **efforts** $I_{1..6}$ (stage 1) or **wrench** at MF $(f_x,f_y,f_z,\tau_x,\tau_y,\tau_z)$ (stage 2).

**Goal:** Estimate the **payload’s contribution** to the wrench at MF while manipulating **robot + payload**, even though training cannot include payload cases. Practically: predict the **gripper-only** wrench $\hat w_{\text{gripper}}$ from robot states, subtract from measured $w_{\text{meas}}$ to isolate $w_{\text{payload}} = w_{\text{meas}} - \hat w_{\text{gripper}}$.

**Core ML Characterization:**

- **Supervised, multi-output, sequence regression** with **latent, piecewise-constant domain variable** ($\phi_{\text{payload}}$, unknown at test).
- **High-dimensional inputs** (18–24 per step), **strong cross-output structure**, **non-stationary/heteroscedastic** noise.
- **Real-time inference** required; **uncertainty** helpful for safety.
- **Causal/physical constraints** known (Newton–Euler, Jacobians) → huge opportunity for **physics-informed learning** and **residual modeling**.
- **Two-stage mapping** (effort → wrench) if you keep that architecture, but we can also design **single-stage** models that directly map $(q,\dot q,\ddot q, I)$ → wrench.
---

# 3) Approaches — what they are, why they fit, and whether they’re SOTA

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

---

# 4) The “Advanced Solutions” you asked me to expand (with concrete designs)

## 4.1 LSTM / GRU (time-series input)

**Design**

* **Window**: last 0.2–0.5 s (e.g., 50–125 steps @250 Hz).
* **Input per step**: $[q,\dot q,\ddot q]$ (18 dims). Optionally include efforts $I$ (then 24 dims) if you go directly to wrench.
* **Output**:

  * Stage-1: $I_{1..6}$ (efforts), OR
  * Single-stage: $(f_x,f_y,f_z,\tau_x,\tau_y,\tau_z)$
* **Loss**: Huber/L1 + heteroscedastic term (predict $\sigma^2$ too).
* **Deploy**: TorchScript → ROS 2 node; should be <1 ms per step on 4070 Ti, <2–3 ms on CPU if kept small.

**Why it fixes GP issues**

* Encodes **history** → handles friction/backlash/latency.
* Learns **configuration-dependent noise**.
* Can be multi-task (shared trunk → multiple heads) to exploit cross-output structure.

## 4.2 Hybrid Dynamics + NN (Residual)

**Design**

1. Use Pinocchio/Isaac dynamics to compute nominal

$$
\tau_{\text{nom}} = M(q)\ddot q + C(q,\dot q)\dot q + G(q)
$$

and nominal wrench via $w_{\text{nom}} = (J^{-T})\tau_{\text{nom}}$ (or forward dynamics + wrench projection, depending on your stack).
2\) Train a small NN to predict **residuals**:

$$
\Delta \tau = \text{NN}(q,\dot q,\ddot q, \text{short history}), \quad
\Delta w = \text{NN}'(\cdot)
$$

3. Final prediction: $\hat\tau = \tau_{\text{nom}} + \Delta \tau$ or $\hat w = w_{\text{nom}} + \Delta w$.

**Why it fixes GP issues**

* The NN learns only **what the model misses** (stiction, flex, sensor bias), which is **more stationary** and easier than the full map.
* Better **out-of-distribution** behavior when payload perturbs the mapping.

## 4.3 ResNet / MLP + BatchNorm (stacked context)

**Design**

* Stack last $k$ steps into one vector (say $k=10$ → 180–240 dims).
* Deep MLP (e.g., 6–10 layers) with residual blocks + BatchNorm/LayerNorm + dropout.
* Multi-output head (6 or 6).
* Great **baseline**; very fast.

**Why it fixes GP issues**

* Provides **temporal context** and **nonlinear capacity** with simple deployment.

## 4.4 Domain Adaptation (no payload in training)

**Options you can actually use given your constraint**

* **Adversarial invariance**: learn features $h(\cdot)$ that predict wrench/effort while a discriminator tries to predict a proxy “episode ID”; the encoder is trained to **confuse** it → encourages **episode-robust** representation (helps when payload shows up as a new “episode”).
* **Latent context**: introduce a small latent $z$ per episode (constant over a rollout). Train with **amortized inference**: the network infers $z$ from a few initial steps. At deployment, infer $z$ online → acts like “implicit payload parameter.”
* **Test-time adaptation**: update only normalization stats (BatchNorm) or a small adapter layer using entropy minimization / consistency loss on the incoming stream — **no labels needed**.

**Why it fixes GP issues**

* Gives you a way to **handle the hidden payload variable** without ever training on labeled payload data.

## 4.5 Physics-Informed (differentiable Newton–Euler / torchdiffeq)

**Two practical paths**

**(i) Differentiable Newton–Euler in the model**

* Implement the analytical dynamics (Pinocchio in PyTorch or Isaac’s differentiable sim). Build them into the **forward pass**.
* The NN learns a small set of parameters or residual fields; **loss** penalizes errors **and** violations of dynamics.
* Add a **per-episode latent** $z$ that scales mass/inertia-like terms; learn $z$ from the first N timesteps.

**(ii) Neural ODE / torchdiffeq**

* Learn a continuous-time dynamics function $\dot x = f_\theta(x,u,z)$ where $z$ is the payload latent. Integrate with `torchdiffeq`; fit to joint trajectories and observed wrench/effort.
* This gives you a **system-identification** view with a learned latent representing payload.

**Why it helps**

* Hard-codes physics so the model doesn’t “forget” fundamentals; focuses learning on **unmodeled** parts and on **inferring payload latents**.

---

# 5) A practical plan you can run in ROS 2/Isaac Lab (and latency you can expect)

**Data logging (already close to what you do):**

* Sample at **250 Hz** (≥100 Hz minimum). Log $(q,\dot q,\ddot q)$, $I$, MF wrench $w$, controller modes, temperatures if available. Bundle **episodes** (one episode = one trajectory).
* Create **fixed windows** (e.g., 32/64/128 timesteps) with stride 1; normalize per-joint using training stats.

**Model baselines to try, in order:**

1. **MLP-ResNet (stacked 64-step context)** → direct $(q,\dot q,\ddot q) \to I$ and/or $(q,\dot q,\ddot q,I) \to w$.
2. **TCN/GRU** with 64–128 steps (causal, small).
3. **Hybrid residual**: analytic dynamics + residual GRU head.
4. **Latent context** $z$ with amortized inference over first 0.5–1.0 s of each episode.
5. (Optional) **Tiny Transformer** if you need longer horizons.

**Latency (4070 Ti):**

* MLP/TCN/GRU with <1M params: **<0.5 ms** per step on GPU; **1–3 ms** on CPU.
* Plenty for 250 Hz control loops.

**Metrics & eval:**

* Offline: RMSE/MAE per output; **calibration** (sharpness vs reliability) if you predict uncertainty.
* Online: **drift over trajectory**, failure cases (sudden orientation changes), and compute **isolation error** $\| w_{\text{meas}} - \hat w_{\text{gripper}} \|$ distribution.

---

# 6) What I suggest doing next (concrete & fast)

1. **Pick one path now**: MLP-ResNet (stacked 64-step context) **directly** to wrench using inputs $[q,\dot q,\ddot q, I]$. (Skip two-stage GP to avoid compounding errors.)
2. In parallel, stand up a **GRU** version with same inputs and window.
3. Add a **residual head** that adds to **nominal wrench** from your dynamics library.
4. Enable **uncertainty** (heteroscedastic head or 3–5 model ensemble).
5. Add **episode-latent** $z$ (context encoder) if you still see shift under payload.
6. Deploy as a ROS 2 node (TorchScript). Measure latency & isolation error online.

I can start with the data you’ve already got. If you upload:

* A CSV/Parquet of synchronized time windows for train/val/test (or raw streams so I create windows), including $q,\dot q,\ddot q,I$ and MF wrench,
* I’ll run an **in-session** benchmark (MLP vs GRU; and optionally hybrid residual if you provide nominal dynamics output too) and show you the metrics and plots here.

Quick note on your last question: I **can’t** train “in the background” and deliver later. If you share the data **now**, I can train and evaluate **right here in this session** and immediately show results.

---

## Next actionable step

Do you want me to begin with the **MLP-ResNet baseline** on your full dataset (direct $[q,\dot q,\ddot q,I] \to w$) and compare against your GP? If yes, please upload the training CSVs (or raw logs), and I’ll proceed to training and reporting now.



love it — let’s make your literature review rock-solid and aligned with your Disposition. Below is a clear, do-able plan you can run with (IEEE/Elsevier full access helps a lot), plus ready-to-use templates and concrete search strings.

---

# Clean research plan (tailored to your thesis + Disposition)

## 0) Anchor to your Disposition requirements

Your school expects: provisional title, problem/motivation, **research questions**, **methodology**, expected results, outline, literature list, and a timeline. Your lit review must systematically show the gap + justify your method choice. We’ll structure everything to feed those sections directly.&#x20;

---

## 1) Define the exact scope (one paragraph each)

Write these now (short bullets are fine); they will become the first page of your Disposition and the intro of your chapter “State of the Art”.

* **Domain**: External force/torque (wrench) estimation and payload parameter identification for industrial manipulators (UR5-class) in HRC contexts.
* **Goal**: Estimate **payload-only** wrench at MF from robot proprioception (q, q̇, q̈, I) + F/T, **without training on payload data**, in real time, with uncertainty.
* **Constraints**: Latent payload parameters, non-stationarity, sequence dependence, multi-output coupling, deployment in ROS 2/Isaac Lab + real UR5. (These constraints are spelled out in your paper’s background and methods.)&#x20;
* **Target contribution**: Physics-informed, sequence ML with domain adaptation to latent payload, benchmarked vs GP baselines on your sim + real robot.

---

## 2) Formulate crisp research questions (RQs)

Draft at least 3 main RQs + focused sub-RQs that map 1:1 to your experiments & review. Keep them “answerable” within your thesis scope (your Disposition calls for precise, limited questions).&#x20;

**RQ1 (SOTA-methodology):**
What are the current state-of-the-art **methods** for (a) TCP/link external force/torque estimation and (b) payload parameter identification in industrial robots, and how do they handle nonlinearity, sequence dependence, and uncertainty?

* RQ1a: Which **model families** are used (observers, classical ID; GP; deep seq models; hybrid/residual; physics-informed)?
* RQ1b: Which **sensors** (encoders, currents, F/T, IMUs, vision/tactile) and what **latencies**/rates are typical?
* RQ1c: What are the **evaluation protocols** (datasets, metrics, real-time constraints)?

**RQ2 (Gap under latent payload):**
How do existing approaches cope with **domain shift** induced by unknown, constant payload parameters at test time (no payload seen in training)?

* RQ2a: Are there **online adaptation** or **latent-variable** strategies reported (observers, meta-learning, test-time norm updates)?
* RQ2b: What **uncertainty** handling is used (heteroscedastic regression, ensembles) for safety?

**RQ3 (Your solution vs SOTA):**
To what extent do physics-informed sequence models (GRU/TCN/Transformer) with residual dynamics and latent context outperform GP baselines for **payload-only wrench isolation** under strict real-time constraints?

* RQ3a: How do they scale with window size and compute budget?
* RQ3b: What is the isolation error and calibration quality vs GP across motion types (with/without TCP orientation change), mirroring your datasets?&#x20;

---

## 3) Build a taxonomy to sort the literature (your review “map”)

When you read, tag each paper along these axes. This gives your “related work” structure and helps identify the gap.

1. **Robot & task**: manipulator (UR5/UR10/7-DoF), link vs TCP, free motion vs contact.
2. **Estimation target**: TCP wrench; link contact; payload $m, c, J$; combined.
3. **Sensors**: encoders, currents, double encoders, F/T, IMU, vision/tactile.
4. **Model family**:

   * Classical (momentum/disturbance observers, Kalman variants, LS/WLS/IRLS).
   * Probabilistic ML (GP, hybrid GP-stiffness).
   * Deep learning (MLP/ResNet, RNN/LSTM/GRU, TCN, Transformers).
   * **Hybrid/Residual** (analytic dynamics + learned residual).
   * **Physics-informed / differentiable sim**.
5. **Temporal modeling**: per-timestep vs sequence window (length, rate).
6. **Uncertainty**: none; aleatoric; ensembles; Bayesian.
7. **Domain shift handling**: none; explicit payload ID; latent context; test-time adaptation.
8. **Compute & real-time**: model size, latency, deployment stack (ROS/Isaac), rate.
9. **Datasets & metrics**: public vs private; MAE/RMSE/R²; calibration; isolation error.

These categories reflect the needs/gaps laid out in your thesis background & results (e.g., need for sequence models, handling non-stationarity, and isolating payload wrench), so they are defensible in your Disposition.&#x20;

---

## 4) Systematic search protocol (PRISMA-style, lightweight)

**Databases**: IEEE Xplore, ACM DL, ScienceDirect (Elsevier), SpringerLink, arXiv (cs.RO, cs.LG). Your Disposition encourages systematic, iterative searching and careful note-keeping.&#x20;

**4.1 Seed keyword blocks** (combine with AND/OR; restrict to 2015–2025 first):

* Block A (robotics): `("force estimation" OR "wrench estimation" OR "external force" OR "contact force") AND (robot* OR manipulator OR "human-robot interaction" OR HRC)`
* Block B (payload): `("payload identification" OR "payload estimation" OR "inertial parameter identification" OR "mass center inertia") AND (robot* OR manipulator)`
* Block C (methods): `(gaussian process* OR "momentum observer" OR "disturbance observer" OR kalman OR "least squares" OR "deep learning" OR LSTM OR GRU OR transformer OR "temporal convolution" OR "physics-informed" OR "residual learning" OR "differentiable")`
* Block D (sensors): `(current OR effort OR "motor current" OR "double encoder" OR "force torque" OR FT OR IMU)`
* Block E (deployment): `(ROS OR "Isaac" OR "real-time" OR "online")`

**Examples of combined queries**:

* `"wrench estimation" AND manipulator AND (LSTM OR GRU OR "temporal convolution") AND ("force torque" OR FT) AND 2018-2025`
* `"payload identification" AND robot AND ("momentum observer" OR "least squares") AND ("real-time" OR online)`
* `("external force estimation" OR "contact force") AND "gaussian process" AND manipulator`
* `"physics-informed" AND robot dynamics AND (wrench OR torque)`

**4.2 Inclusion criteria**

* Industrial or collaborative robot manipulators.
* Estimates external wrench or payload parameters from proprioception and/or F/T.
* Reports quantitative metrics and some latency/computation details; or widely cited foundational method.
* 2015–2025 (expand earlier if foundational).

**4.3 Exclusion criteria**

* Legged or aerial only (unless the method clearly transfers).
* Pure simulation without sensor realism **and** no deployment discussion.
* Vision-only grasp force estimation unrelated to manipulator dynamics.

**4.4 Screening workflow**

* Phase 1 (Title/Abstract): keep if it fits target + method relevance.
* Phase 2 (Skim): read intro + method figs + results; tag with taxonomy (Sec. 3).
* Phase 3 (Full read): only for the finalists you will cite deeply and/or reproduce.

**4.5 Record-keeping (simple spreadsheet)**
Columns: BibKey, Year, Venue, Robot, Task, Sensors, Model family, Temporal (Y/N, window), Uncertainty, Domain shift strategy, Dataset, Metrics, Latency, Code/data link, **Key ideas**, **What to borrow**, **Limitations**. (Your Disposition stresses systematic storage & notes—this satisfies that.)&#x20;

---

## 5) Immediate “shortlist” targets to expect (by category)

*(You’ll fetch the exact papers via IEEE/Elsevier; list here is what to look for, not external citations.)*

* **Classical/Observers**: momentum/disturbance observers, friction-aware Kalman filters for external torque/wrench; payload/inertial ID via LS/WLS/IRLS.
* **GP / Hybrid GP**: GP for torque/wrench or stiffness + GP hybrids; note scaling and stationarity assumptions.
* **Deep Seq (MLP/GRU/TCN/Transformer)** for inverse dynamics / wrench regression; especially works reporting real-time rates on manipulators.
* **Hybrid/residual**: “nominal RNEA + learned residual” for torque or wrench; double-encoder external torque methods (stiffness/compliance modeling).
* **Physics-informed/differentiable**: PINNs for robot dynamics; differentiable simulators (Isaac-Gym-style) used to fit residual parameters or latent contexts.
* **Domain adaptation/latent context**: test-time adaptation for dynamics; meta-learning for hidden context (payload) with amortized inference.

This mirrors your background/state-of-the-art section and clearly motivates why GP alone underperforms live.&#x20;

---

## 6) How each approach maps to **your** problem (1–2 lines you can paste into your chapter)

* **Observers/KF/LS**: strong physics priors; may struggle with friction & multivalued mappings; great as baseline and for nominal torque/wrench to support residual learning.
* **GP**: good local interpolator, uncertainty “for free”; weak under sequence dependence, heteroscedasticity, and latent payload shift → explains your results.&#x20;
* **MLP/ResNet (stacked context)**: simple, fast, already strong with short windows; good baseline vs GP.
* **GRU/TCN**: capture history (stiction/backlash), scale well, real-time on 4070 Ti/CPU; top practical pick.
* **Tiny Transformer**: models longer dependencies, multi-output coupling; use if you need longer horizons.
* **Hybrid/residual**: best generalization/data-efficiency; add nominal RNEA/Jacobian and learn only the residual → payload shift hurts less.
* **Physics-informed/diff-sim**: enforce dynamics, infer per-episode latent $z$ \~ payload; closes the loop to your thesis goal of isolating **payload-only** wrench.
* **Domain adaptation**: absolutely key since you **cannot** train on payloads; learn invariant features or infer $z$ online.

---

## 7) Writing plan for the State of the Art chapter

Structure your chapter like this (it matches your Disposition and your own paper’s ToC):

1. **Problem framing & constraints** (why the estimation is hard: sequence, heteroscedastic, latent payload, multi-output, real-time).&#x20;
2. **Classical methods** (observers/KF/LS) — strengths/limits.
3. **Probabilistic ML** (GP + hybrids) — where they shine, where they fail (your findings).&#x20;
4. **Deep learning** (MLP/ResNet, GRU/TCN/Transformer) — capabilities in robot dynamics.
5. **Hybrid & physics-informed** — residual learning, differentiable dynamics, and why they align with your constraints.
6. **Domain adaptation/latent context & uncertainty** — safety, generalization to payload.
7. **Synthesis & gap** — what’s still missing and how your approach fills it (your contribution statement).

---

## 8) Practical tooling and organization

* **Reference manager**: Zotero or Mendeley from day 1; keep one shared group library + BibTeX export. (Your Disposition stresses orderly literature management.)&#x20;
* **Foldering**: `/00_screening/`, `/10_selected/`, `/20_notes/`, `/30_figures/`, `/40_bib/`.
* **Notes template per paper** (Markdown):

  * Summary (3–5 lines), Taxonomy tags, Equation/Model sketch, Dataset/metrics, Latency, Strengths, Weaknesses, What I reuse, Open questions.
* **Traceability**: keep a PRISMA-style tally (found → screened → included).
* **Repro list**: mark 3–5 papers you’ll **reproduce/benchmark** against in your env (ROS 2/Isaac/UR5).

---

## 9) Timeline you can paste into your disposition

* **Week 1–2**: Broad search & screening; fill taxonomy sheet; 30–40 candidate papers.
* **Week 3–4**: Deep read top 15; write 60–70% of “State of the Art” draft; pick 3–5 target baselines.
* **Week 5–6**: Implement baselines (observer, GP, MLP/GRU/TCN) + hybrid residual; start experiments.
* **Week 7–8**: Physics-informed + domain adaptation; final experiments; write Results/Discussion.
* **Week 9**: Polish, figures, ablations, threat-to-validity, finalize Disposition → thesis.

---

## 10) What I need from you to start the loop

1. Your **search spreadsheet** (or export) after Phase-1 screening (title/abstract).
2. PDFs of the top 15 finalists (I can ingest and help you summarize/compare).
3. Your shortlist of **methods to reproduce** in Isaac/ROS 2.

I’ll then: (a) help cluster the papers, (b) extract key comparisons, and (c) align them with your contribution narrative and experiments.

---

## Bonus: how this ties back to your current files

* Your **thesis PDF** already articulates the physics & data pipeline and the GP limitation — we’ll reuse this as Chapter intro and to motivate SOTA needs.&#x20;
* Your **MD “start” file** contains the GP critique and the advanced ML roadmap — that maps directly to Sections 3–6 above.&#x20;
* The **Disposition slides** give the checklist and structure we matched throughout (problem, RQs, method, expected results, literature list, timeline).&#x20;

---

### Next actionable step

Run the Phase-1 searches with the query blocks above on IEEE/ScienceDirect, export 30–40 candidates (title/abstract), tag them with the taxonomy, and send me the spreadsheet + PDFs of your top 10–15. Do you want me to provide a ready-made **Google Sheets template** for tagging + PRISMA counting so you can start right away?
