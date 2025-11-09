Short answer: **Yes, totally possible**—especially in Isaac Sim. You can learn a **pure data-driven inverse-dynamics map**
[
\tau(t)=f\big(q(t),\dot q(t),\ddot q(t)\big)
]
and run it **in real time** without giving the network any NE/EL model. Here’s a practical way to make it work well:

# What to train (no physics given)

Go **Pure Deep** (your Category 3). The two best choices for real-time + accuracy:

* **Causal TCN** (temporal conv net): fast, stable, fixed latency, great for friction/history.
* **Small GRU/LSTM**: also fine; slightly trickier to bound latency.

You’ll still feed **current kinematics**; the net learns everything else (gravity, Coriolis, friction) from data.

# Minimal recipe (works in Isaac Sim)

**1) Data generation (rich excitation)**

* In MoveIt2, **don’t just replay tasks**; generate **joint-space Fourier/PRBS trajectories** per joint (respect limits).
* Include **varied speeds/amps**; interleave **payload toggles** if you plan to generalize later.
* Log at **≥500–1000 Hz**: (q,\dot q,\ddot q) (Isaac can give (\ddot q)), and **effort** ((\tau)).

**2) Preprocess**

* **Time align** streams; drop/clip outliers.
* Normalize per joint (z-score).
* Encode angles as **(\sin q,\cos q)** to remove wraparound.
* If (\ddot q) is noisy: use **Savitzky–Golay** or **spectral differentiation**; in sim you can often trust it directly.

**3) Model I/O**

* **Inputs per step:** ([\sin q,\cos q,\dot q,\ddot q]) for all 6 joints.
* **Sequence window:** 50–150 ms history (captures friction/backlash).

  * TCN receptive field ≈ **W** samples; pick W to match your control loop (e.g., 64 samples @1 kHz → 64 ms).
* **Output:** (\hat\tau\in\mathbb{R}^6) (all joints, multi-output head).

**4) Training**

* Loss: MSE (optionally **per-joint weights** if torques differ in scale).
* Regularize with small **L2**; **early stop** on validation NRMSE.
* Mix **slow + fast** segments in each batch so it learns both gravity & dynamic terms.

**5) Real-time deployment**

* Export to **TensorRT/ONNX**; TCN/GRU this size runs **<0.2 ms @1 kHz** on modest GPUs.
* Latency = one window; keep it **fixed & causal**.
* Monitor **RMSE / NRMSE** online; add a deadband if you’ll use residuals for contact later.

# Why this works

* In sim, you control **excitation richness** and **noise**, so a black-box model can learn the mapping well.
* **Sequence modeling** captures **history-dependent** effects (stick–slip, compliance) that a single-shot MLP misses.
* You don’t need NODEs for inverse dynamics: (\tau=f(q,\dot q,\ddot q)) is **algebraic**. NODEs shine when you model **state evolution** (forward dynamics/latent states/irregular sampling). If you do have jittered timestamps, you can **resample/interpolate**—a NODE is optional.

# What about SINDy/PINNs/NODEs here?

* **Neural ODEs:** Yes, they model **nonlinear** dynamics and handle **irregular time** well, but they integrate an ODE—overkill for **inverse dynamics**. Consider a NODE only if you also want a **continuous-time latent state** (e.g., thermal/gear hysteresis) that your inverse map can condition on.

* **SINDy:** Can work **if** you craft a **library that includes NE columns + friction bases** and you have clean (\ddot q). Good for **interpretability** and a compact formula. Harder at 6-DoF unless you use **group sparsity** and **well-designed trajectories**.

* **PINNs:** Great to **identify parameters** by minimizing NE residuals + data loss, or to **denoise/reconstruct** trajectories. For pure inverse-dynamics mapping with real-time constraints, a PINN is usually **heavier to train** and less straightforward than the TCN/GRU route.

# Gotchas & tips

* **Generalization:** if you’ll leave sim, add **domain randomization** (inertias, friction, sensor noise) and **payload variations** during training.
* **Acceleration leakage:** if (\ddot q) is computed from the same (q) stream you’re predicting on, keep processing **causal** to avoid future info leakage.
* **Tool/payload changes:** a pure deep model won’t extrapolate well; plan a **small residual adaptor** or **fast fine-tune** if you’ll change tools later.
* **Evaluation:** report **per-joint RMSE/NRMSE**, and a **torque-prediction latency budget** (window size).

If you want, I can sketch an exact **TCN config** (layers, kernel sizes, receptive field, FLOPs) that will hit 1 kHz on your setup and a minimal PyTorch training loop to get you started.
