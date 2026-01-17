Keep everything fixed (seed, epochs, H=50), only change `--features`:

* `full` (baseline you already have)
* `tau_hat`
* `state_tauhat`
* `state`

Then compare only **rg_rmse** and **per-joint rg_rmse**.


---
---

## A) What artifacts you currently generate (and what they show)

### 1) DeLaN folder (`shared/models/delan/...`)

**Plots**

* `...__loss_curve.png`
  Training loss vs **epoch** (one curve). This is the objective DeLaN optimizes during training.
* `...__loss_components.png`
  Separate curves for the *components* of the DeLaN loss (the ones you log as `Inv`, `For`, `Power` / `Energy` depending on your implementation).
* `...__elbow_train_vs_test.png`
  “Elbow” view: **train loss** over epochs + **test torque MSE** evaluated every `eval_every`.
* `...__DeLaN_Torque.png`
  For each joint: **GT torque τ** vs **DeLaN predicted torque τ̂** over time (a concatenated/sliced segment).

**Metrics**

* `metrics.json`, `metrics_test.txt`
  DeLaN **test torque MSE/RMSE**, per-joint MSE/RMSE, and timing numbers.

---

### 2) LSTM folder (`shared/models/lstm/...`)

**Plots**

* `loss_curve.png`
  LSTM training curves: `loss` (train) and `val_loss` (validation) over epochs. In your script this is **MSE in the training space you use** (“scaled residuals” in your title).
* `residual_gt_vs_pred.png`
  Per joint: **GT residual r** vs **LSTM predicted residual r̂** for the first `K` samples of `Y_test`.

**Metrics**

* `metrics_train_test_H60.json`
  Contains: run config, dataset shapes, scaler stats, and LSTM residual errors on train/test.
* `train_history_H60.csv`
  Per-epoch train/val loss (and maybe learning rate if you logged it).
* `predictions_test.npz`
  Arrays saved from LSTM-only evaluation (typically `Y_test`, `Y_pred`, maybe `tau_hat`, etc.).
* `scalers_H60.npz`
  Feature scaler + residual scaler parameters used for inverse-transform in eval.

---

### 3) Evaluation folder (`shared/evaluation/...`)

**Plots**

* `residual_gt_vs_pred_test_H60.png`
  Same *idea* as the LSTM plot, but generated from the **reconstructed valid region** of the trajectory residual NPZ (the “combined-eval alignment” slice).
* `torque_gt_vs_delan_vs_combined_test_H60.png`
  Per joint: **GT τ**, **DeLaN τ̂**, and **combined τ_RG = τ̂ + r̂** (your “DeLaN + LSTM” torque).

**Metrics**

* `metrics_test_H60.txt`
  DeLaN torque error, residual error, and combined error on the aligned region.

---

## B) Definitions of every term you mentioned (and the ones you log)

### Training loop basics

* **epoch**: one full pass through the training dataset.
* **batch / minibatch size**: number of samples per gradient update. Larger batch → fewer updates/epoch.
* **train split / test split**: disjoint sets of trajectories (you have 164 train / 41 test).
* **validation split (`val_split`)**: part of the *training* windows held out during LSTM training to monitor overfitting.

---

### Error metrics

Let (y) be ground truth and (\hat y) prediction. For torques, (y=\tau). For residuals, (y=r).

* **MSE** (mean squared error):
  [
  \mathrm{MSE} = \frac{1}{N}\sum_{i=1}^N (y_i - \hat y_i)^2
  ]

* **RMSE** (root mean squared error):
  [
  \mathrm{RMSE} = \sqrt{\mathrm{MSE}}
  ]
  RMSE has the same unit as the signal (Nm for torque).

* **per-joint MSE/RMSE**: same formulas, computed separately for each joint dimension.

**Important detail in your pipeline:** you often compute a **single total MSE** by averaging over *all joints and all samples* (so it’s “global MSE over the full tensor”).

---

### DeLaN-specific logged losses (what they mean conceptually)

Your DeLaN log line looks like:
`Loss=..., Inv=..., For=..., Power=...`

Typical meaning (matching how DeLaN-style training is usually structured):

* **Inv**: inverse dynamics consistency loss (how well predicted τ matches dataset τ).
* **For**: forward dynamics / acceleration consistency loss (how well the learned dynamics reproduce accelerations, or regularize dynamic consistency).
* **Power / Energy**: energy / passivity / power consistency regularizer (penalizes violations of physical consistency, depending on implementation).

Your `loss_components.png` is basically these curves over epochs.

---

### Residual + combined torque naming

* **τ̂ (tau_hat)**: DeLaN predicted torque.
* **r (residual)**: ground truth residual, typically
  [
  r = \tau - \hat\tau
  ]
* **r̂**: LSTM predicted residual.
* **τ_RG / combined**: your corrected torque prediction
  [
  \tau_{\mathrm{RG}} = \hat\tau + \hat r
  ]

---

### “rg_rmse” (why it equals residual RMSE in your eval)

In your `evaluation_metrics_test_H60.txt` you have:

* `res_mse` / `res_rmse`
* `rg_mse` / `rg_rmse`

They are *identical* in your file:

* `res_rmse = 1.675836...`
* `rg_rmse  = 1.675836...`

That’s actually mathematically expected if:

* residual GT (r = \tau - \hat\tau)
* residual pred (\hat r)
* combined pred (\tau_{\mathrm{RG}} = \hat\tau + \hat r)

Then the combined torque error is:
[
\tau - \tau_{\mathrm{RG}} = \tau - (\hat\tau + \hat r) = (\tau - \hat\tau) - \hat r = r - \hat r
]
So **the combined torque error equals the residual prediction error**. Therefore the MSE/RMSE are the same number (just reported under two names).

So:

* `res_rmse` answers: “How well does the LSTM predict residuals?”
* `rg_rmse` answers: “How good is the corrected torque τ_RG vs GT?”
  They’re the same scalar because τ̂ cancels out in the subtraction.

---

## C) What your current numbers say (interpretation)

From `evaluation_metrics_test_H60.txt`:

* **DeLaN torque RMSE**: `delan_rmse = 4.7921`
* **Residual (and combined) RMSE**: `res_rmse = rg_rmse = 1.6758`

That means: **your DeLaN+LSTM correction reduces torque RMSE from ~4.79 Nm → ~1.68 Nm on the evaluated test region**.

Per joint (RMSE):

* DeLaN: `2.0806 11.4468 1.3440 0.6218 0.4772 0.0901`
* Combined (= residual error): `1.6825 3.4275 1.2896 0.6096 0.4808 0.0806`

Interpretation:

* Joint 1 dominates the torque scale/error (biggest RMSE). The combined method drastically improves that joint (11.45 → 3.43).
* Some joints barely change (joint 4 even slightly worse by a hair: 0.4772 → 0.4808), meaning for those joints either:

  * DeLaN was already good and residuals are near noise, or
  * the residual model isn’t helping and adds a tiny bias.

### Feature-mode interpretation (the ablation you’re about to run)

* If **tau_hat-only** achieves almost the same `rg_rmse` as **full**, then the residuals are largely a *function of τ̂ history*, i.e. the LSTM is learning a systematic correction “on top of DeLaN.”
* If **full** (or **state_tauhat**) is clearly better than tau_hat-only, then residuals depend on motion state/history (qd/qdd effects like friction, hysteresis, unmodeled coupling).
* If **state-only** becomes strong, the LSTM is effectively learning dynamics directly (less “residual correction,” more “direct model”), which is important to know but usually not what you want conceptually.

---

## D) What boxplots I’d make across many runs

Assume you’ll have many folders differing by seed / epochs / width / H / feature_mode.

### 1) The “main scoreboard” boxplots (evaluation folder)

Grouped by **feature_mode** (and optionally by H):

* **`rg_rmse`** (overall)
  This is the single most important “end-to-end” number.
* **per-joint `rg_joint_rmse[j]`**
  One boxplot per joint (or a grouped boxplot with 6 boxes per feature_mode).

Also very useful:

* **ΔRMSE improvement** = `delan_rmse - rg_rmse`
* **% improvement** = `(delan_rmse - rg_rmse) / delan_rmse * 100`

These directly show whether residual learning *consistently* improves DeLaN.

### 2) DeLaN-only stability plots (delan folder)

Across runs (seed / hp preset / epochs):

* boxplot of **`eval_test.torque_rmse`**
* boxplot of **per-joint torque_rmse**
* elbow-derived “best epoch” proxy: the epoch where test MSE flattens (optional)

### 3) LSTM-only stability plots (lstm folder)

Across runs:

* boxplot of **`eval_test.rmse_total`** (residual prediction error)
* per-joint residual RMSE boxplots
* optional: best `val_loss` (but be careful: it’s in “training space” and can be misleading if scaling changes)

### 4) Correlation plots (very insightful)

Scatter plots:

* x-axis: `delan_rmse`, y-axis: `rg_rmse`
  Shows whether better DeLaN necessarily implies better combined performance.
* x-axis: `eval_test.rmse_total` (LSTM residual), y-axis: `rg_rmse`
  Should be almost 1:1 if your evaluation region is identical (since `rg_rmse == res_rmse`).

---

## E) Are the two residual plots “exactly the same”?

**Not pixel-identical** (different titles/labels/styling), but **they appear to be plotting the same underlying quantities** for the test set, because:

* Your evaluation file reports `res_rmse = 1.675836...` and per-joint residual RMSE values.
* Your LSTM metrics JSON reports `rmse_total = 1.675836...` and the same per-joint `rmse_per_joint`.

So the **numbers match exactly**, which strongly suggests:

* both scripts are evaluating the same residual prediction target on the same test region, and
* the difference is *how the series is assembled + plot cosmetics* (legend text, titles, maybe order of concatenation).

Why the *plots* might still look a bit different:

* `train_residual_lstm.py` plots `Y_test[:K]` directly from the **windows dataset ordering**.
* `combined_evaluation.py` rebuilds a **trajectory-aligned valid region** and concatenates it; ordering can differ even if the aggregate errors match.
* Titles/labels differ (`Residual torque joint j` vs `Residual joint j`, legend text differs too).

If you want them to look “the same,” the cleanest approach is:

* in evaluation, plot exactly `Y_test` and `Y_pred` by **loading the windows NPZ** (instead of reconstructing), *or*
* ensure evaluation reconstructs windows in the exact same order as `build_lstm_windows.py` (same trajectory loop order, same k-loop, same filtering), and use the same plot titles/legend strings.

---