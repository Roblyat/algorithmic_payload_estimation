Preprocess

Largely yes — that’s exactly what your current pipeline does when you set:

--lowpass_signals True
--derive_qdd_from_qd True
--lowpass_qdd True
What’s correct in your reasoning
If you have no reliable acceleration in the raw data, you must derive qdd from qd (or from q via 2nd derivative, but that’s usually worse).
Differentiation amplifies high‑frequency noise, so deriving qdd from an unfiltered qd is typically “dangerous” (very noisy qdd).
Filtering qd before differentiating is a common and reasonable approach: it reduces the noise that gets amplified by the derivative.
Filtering qdd after derivation is also common, because even after filtering qd, numerical differentiation can still reintroduce noise.
Two nuances to keep in mind
Filtering q and tau doesn’t directly help qdd, but it can still be beneficial overall:

If you later use q/qd/tau together as model inputs/targets, filtering them consistently can improve learning stability and reduce mismatch (e.g., if tau is noisy).
The “best order” depends on what you want to preserve.

Your order is: low‑pass qd → derive qdd → (optional) low‑pass qdd.
That’s sensible for noise reduction, but it does reduce high‑frequency content and can introduce edge artifacts (though sosfiltfilt avoids phase lag, it can still have boundary effects). So it’s a tradeoff: cleaner qdd vs. potentially smoothing away real fast dynamics.
Bottom line
Your description is correct: filtering then deriving is generally better than deriving then filtering, and filtering qdd after derivation can further stabilize it.

---
---
---

DeLaN

1) What data DeLaN trains on
The input NPZ is trajectory-wise (lists of arrays per trajectory): train_q[i] is (T_i, dof), same for train_qd, train_qdd, train_tau, plus val_*, test_* (load_npz_dataset.py (line 6)).
load_npz_trajectory_dataset() flattens each split by stacking trajectories into one big matrix:
train_q, train_qd, train_qdd, train_tau become (N_train, dof) where N_train = sum_i T_i
same for val_* and test_* (load_npz_dataset.py (line 37)).
Important: training is not sequence-based; it treats each timestep as one supervised sample.
2) What the model is and what it predicts
rbyt_train_delan_jax.py builds a DeLaN Lagrangian network (structured or black-box) and wraps it in delan.dynamics_model (jit’d).
During prediction (and later eval), it produces torques tau_hat from (q, qd, qdd); in the script that’s the [1] output of dynamics_model(...) (see how residual export does it in export_delan_residuals_jax.py (line 70)).
So conceptually:
inputs per sample: q[t], qd[t], qdd[t] → output: tau_hat[t] (same dof).

3) How batching works
It loads the flattened train_* arrays into a replay/batch iterator (ReplayMemory) with minibatch size n_minibatch (rbyt_train_delan_jax.py, around where mem = ReplayMemory(...) is created).
Each epoch iterates over mem and gets batches (q, qd, qdd, tau) with shape roughly (B, dof).
4) The loss it optimizes
It uses delan.inverse_loss_fn(...) from deep_lagrangian_networks with normalization factors computed from train variances:
norm_tau = var(train_tau, axis=0)
norm_qdd = var(train_qa, axis=0) (rbyt_train_delan_jax.py, where loss_fn is defined).
The loss function returns logs including at least:
loss, inverse_mean, forward_mean, energy_mean
The key supervised signal is still: match predicted torques to GT torques (inverse dynamics). The “forward/energy” terms are extra physics-consistency signals (exact weighting is inside deep_lagrangian_networks).
5) The optimizer/update step
Optimizer is AdamW (optax.adamw).
Training step is value_and_grad(loss_fn) → apply updates (rbyt_train_delan_jax.py, update_fn).
Prints every 50 epochs and records training history via DelanTrainRun.record_train_point(...) (that’s what later makes the loss plots).
6) Periodic eval during training (“elbow”)
Every eval_every epochs it computes a simple torque MSE on val if present, else test and records it (rbyt_train_delan_jax.py, where it prints [eval] val_mse=...).
That’s used for the “elbow” plot (train loss vs val/test MSE).
7) Final eval after training
After training finishes, it runs one pass on the full test split, computes MSE/RMSE, and saves plots/metrics via DelanTrainRun.save_eval_metrics(...).

---
---
---

# Maybe 6th parameter setup in cas of ##4
## 4) One key “paper-aligned” note for your DeLaN stability discussion

In the “Combining Physics and Deep Learning …” survey, they explicitly warn that Lagrangian rollouts can become numerically sensitive when the mass matrix eigenvalues approach a small (\epsilon), and emphasize choosing (\epsilon) as large as possible to limit acceleration amplification. 

So if your DeLaN implementation exposes anything like **diagonal epsilon / diagonal shift** (even if not in your current presets), that would be a *very* defensible “6th knob” later—*but only after* the above 5-preset sweep establishes a clean baseline.

## 5) How this connects to your K-domination results (so the story stays coherent)

* K-domination already showed “data regime dominates” and that for (K\ge 32) things stabilize. 
* Best-model approach fixes (K=84) and then asks: “**within the stable regime**, which hyperparameters give the best accuracy-robustness tradeoff across dataset and init seeds?” 
* The proposed aggregate scatter (median ± IQR) is exactly the right visualization for that, showing the most accurate and most stable model in the bottom-left corner of the scatter plots

If you want, paste the exact 5 preset names you plan to encode (e.g., `lutter_like_256`, `lutter_like_128`, …) and I’ll map them to a clean naming scheme that makes the plots and folders thesis-readable.


---
---
---

what i think about the results as a basis for the 2nd experiment is that K-domination results show that variance is dominated by which trajectories are in the split (dataset seed) and initialization (DeLaN seed), the best-model sweep at fixed 
K=84
K=84 should pick hyperparameters that:
- Reduce seed variance (tight IQR; few/no catastrophic outliers).
- Reach good validation error quickly and reliably (stable early stopping behaviour).
- Generalize (validation ranking should correlate with test; your Algorithm 2 uses validation-based selection)

That would mean we dont pick the lowest median, we pick the lowest median with small IQR and low failure rate. 

although we need to put the results into the results section, in this case it is allowed to get this at the beginning of \subsection{Best Model Approch}. Make sure to add the correct Figures of the results section and we are fine. 

So what we aim for the best delan model per hyperparameter set  is:
score(h)=median(val_rmse)+λ⋅IQR(val_rmse)+P⋅1[diverged runs]
where λ∈[0.25,1] is usually enough, and P is a large penalty. Please give what lamda and P are, and what the score(h) metrics tell.

then we plot DeLaN scatter for hyperparameter comparison
- x-axis: median val_rmse across DeLaN seeds (and then across dataset seeds)
- y-axis: IQR(val_rmseval_rmse) across DeLaN seeds (and then across dataset seeds)

at this point we also do the same plot for:

- x-axis: median val_rmse
- y-axis: median test_rmse

to check whether validation is a good selector

Now what 5 parameter setups should we choose based on experiment 1 k-domination:

1) Lutter-like regime (SoftPlus, batch 1024, lr 10−4, wd 10−5, width 256, depth 2) showed up getting stabilized K => 32 and models become comparable we keep that (SoftPlus, batch 1024, lr 1e−4, wd 1e−5, width 256, depth 2) as our baseline model

2) Smaller capacity (tests if 256 is overkill / helps stability):
   Same but **width 128, depth 2**

3) **Deeper capacity (tests representation vs stability):**
   Same as (1) but **width 256, depth 3**
   (often helps if the mapping needs more compositionality; can also destabilize—this is why it’s a sweep)

4) **More regularization (tests generalization + seed variance reduction):**
   Same as (1) but **wd (1\mathrm{e}{-4})**
   (keep lr; just increase weight decay by 10×)

5. **Lower step size (tests optimization stability vs underfit):**
   Same as (1) but **lr (5\mathrm{e}{-5})**
   (useful when training is numerically sensitive; experiment1 notes stability issues can arise from numerical sensitivity around the mass matrix / acceleration amplification, so being conservative with lr is a reasonable stabilizer test) 

 How this connects to K-domination results now:
* Best-model approach fixes (K=84) and then asks: “**within the stable regime**, which hyperparameters give the best accuracy-robustness tradeoff across dataset and init seeds?” 
* The proposed aggregate scatter (median ± IQR) is exactly the right visualization for that, showing the most accurate and most stable model in the bottom-left corner of the scatter plots


## Scatter 1: accuracy vs stability

**Title (paper/thesis):**

* **DeLaN validation accuracy vs seed stability (median vs IQR)**

**Short title (plot header):**

* **val_rmse (median) vs val_rmse (IQR)**

**Filename:**

* `scatter_valrmse_median_vs_iqr__delan.png`

**Caption phrase:**

* “Median validation RMSE versus interquartile range (IQR) across DeLaN and dataset seeds; lower-left indicates accurate and stable hyperparameter settings.”

---

## Scatter 2: validation ↔ test alignment

**Title (paper/thesis):**

* **DeLaN validation vs test performance (median val_rmse vs median test_rmse)**

**Short title (plot header):**

* **val_rmse (median) vs test_rmse (median)**

**Filename:**

* `scatter_valrmse_median_vs_testrmse_median__delan.png`

**Caption phrase:**

* “Median validation RMSE versus median test RMSE across DeLaN and dataset seeds; closeness to the diagonal indicates reliable validation-based model selection.”


---
---
---


okay see the delan best model approach experiment i provided as pdf. do you think we should wait, or do you think we can directly setup stage2, having enough knowledge to set the stage 2 best model algorithm up, even not seen best delan model yet. evaluating lstm with different H, early_stopping different warm up and patience -> because metrics show that lstm with no warm up and 10 patience stop early (5 to 15 epochs) does not have a chance to converge and with to small H=25 also not converge even over many epochs. and maybe getting a good perspective on the pipeline testing with different feature modes for lstm. i already have a setup in my pipeline for this and i can adjust the plots for feautre modes or add new ones as best, if these feature modes may make sense for interpreting. see my current feature modes setup:

Feature modes are defined in feature_builders.py and enforced when building the LSTM window dataset in build_lstm_windows.py. Training (train_residual_lstm.py) just consumes the resulting NPZ and the feature_dim stored in it, so the mode is entirely determined at preprocessing time.

Here are the modes and what goes into each feature vector at every timestep (each signal is shape (T, dof)):

full: concat [q, qd, qdd, tau_hat] → feature_dim = 4*dof
tau_hat: just tau_hat → feature_dim = 1*dof
state: concat [q, qd, qdd] → feature_dim = 3*dof
state_tauhat: concat [qd, qdd, tau_hat] → feature_dim = 3*dof
Where:

q = joint positions
qd = joint velocities
qdd = joint accelerations
tau_hat = model-predicted torque (from DELAN)
In build_lstm_windows.py, those per‑timestep features are stacked into sliding windows of length H, so each LSTM sample is (H, feature_dim) and the target Y is the residual torque r_tau at the window’s last step. feature_mode is saved into the NPZ metadata, but the training script doesn’t read it — it only uses feature_dim and the arrays.

For plotting, lstm_metrics_boxplots.py parses the run folder name and pulls the feature mode from a __feat_<mode>__ token (e.g., __feat_full__). If it doesn’t find that, it uses unknown as the grouping key. That’s why consistent run naming matters for the feature_mode boxplots.

so do you think we can model the lstm best approach loop?

give your thoughts and how you would setup lstm best model approach.
i also provided lstm 2 papers in this field, search them to see related works 