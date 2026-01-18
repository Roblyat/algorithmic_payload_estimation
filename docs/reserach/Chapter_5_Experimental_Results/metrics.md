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
---
---

Got it — keep the **same artifact layout** (DeLaN artifacts in `shared/models/delan/...`, LSTM artifacts in `shared/models/lstm/...`, pipeline artifacts in `shared/evaluation/...`), but **move plotting code out of the big scripts**.

Below is a clean refactor that matches exactly what you described, based on where the plot/metrics blocks currently live:

* DeLaN plots are currently created inside `rbyt_train_delan_jax.py` (loss curve / components / elbow / torque plot, plus metrics writing blocks).
* LSTM plots are currently created inside `train_residual_lstm.py` (loss_curve + residual_gt_vs_pred). 
* Evaluation plots/metrics are inside `combined_evaluation.py`.

---

## 1) DeLaN: move plots into `payload_estimation/delan/src/delan_plots.py`

### A) Create file: `payload_estimation/delan/src/delan_plots.py`

```python
# payload_estimation/delan/src/delan_plots.py
from __future__ import annotations

import os
import json
from dataclasses import dataclass
from typing import Optional, Sequence, Dict, Any

import numpy as np
import matplotlib.pyplot as plt


@dataclass
class DelanPlotter:
    model_dir: str
    run_name: str

    def _save_fig(self, fig, filename: str, dpi: int = 150) -> str:
        os.makedirs(self.model_dir, exist_ok=True)
        path = os.path.join(self.model_dir, filename)
        fig.savefig(path, dpi=dpi)
        plt.close(fig)
        return path

    def save_training_curves(
        self,
        hist_epoch: Sequence[float],
        hist_loss: Sequence[float],
        hist_inv: Optional[Sequence[float]] = None,
        hist_for: Optional[Sequence[float]] = None,
        hist_energy: Optional[Sequence[float]] = None,
        hist_time: Optional[Sequence[float]] = None,
        *,
        prefix: Optional[str] = None,
    ) -> Dict[str, str]:
        if len(hist_epoch) == 0:
            return {}

        pfx = prefix or self.run_name
        out = {}

        # 1) Loss curve
        fig = plt.figure(figsize=(8, 4), dpi=120)
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(hist_epoch, hist_loss)
        ax.set_title(f"{self.run_name} | Training Loss")
        ax.set_xlabel("Epoch")
        ax.set_ylabel("Loss")
        ax.grid(True, alpha=0.25)
        plt.tight_layout()
        out["loss_curve_png"] = self._save_fig(fig, f"{pfx}__loss_curve.png")

        # 2) Loss components (optional)
        if hist_inv is not None and hist_for is not None and hist_energy is not None:
            fig = plt.figure(figsize=(8, 4), dpi=120)
            ax = fig.add_subplot(1, 1, 1)
            ax.plot(hist_epoch, hist_inv, label="inverse_mean")
            ax.plot(hist_epoch, hist_for, label="forward_mean")
            ax.plot(hist_epoch, hist_energy, label="energy_mean")
            ax.set_title(f"{self.run_name} | Loss Components")
            ax.set_xlabel("Epoch")
            ax.set_ylabel("Value")
            ax.grid(True, alpha=0.25)
            ax.legend()
            plt.tight_layout()
            out["loss_components_png"] = self._save_fig(fig, f"{pfx}__loss_components.png")

        # 3) CSV dump (optional)
        if hist_time is not None and hist_inv is not None and hist_for is not None and hist_energy is not None:
            csv_path = os.path.join(self.model_dir, f"{pfx}__train_history.csv")
            with open(csv_path, "w") as f:
                f.write("epoch,time_s,loss,inverse_mean,forward_mean,energy_mean\n")
                for e, ts, lo, inv, fo, en in zip(hist_epoch, hist_time, hist_loss, hist_inv, hist_for, hist_energy):
                    f.write(f"{e},{ts},{lo},{inv},{fo},{en}\n")
            out["train_history_csv"] = csv_path

        return out

    def save_elbow_plot(
        self,
        hist_epoch: Sequence[float],
        hist_loss: Sequence[float],
        hist_test_epoch: Sequence[float],
        hist_test_mse: Sequence[float],
        *,
        prefix: Optional[str] = None,
    ) -> Optional[str]:
        if len(hist_epoch) == 0 or len(hist_test_epoch) == 0:
            return None

        pfx = prefix or self.run_name
        fig = plt.figure(figsize=(8, 4), dpi=120)
        ax1 = fig.add_subplot(1, 1, 1)

        ax1.plot(hist_epoch, hist_loss, label="train_loss")
        ax1.set_xlabel("Epoch")
        ax1.set_ylabel("Train loss")
        ax1.grid(True, alpha=0.25)

        ax2 = ax1.twinx()
        ax2.plot(hist_test_epoch, hist_test_mse, label="test_mse")
        ax2.set_ylabel("Test torque MSE")

        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc="best")

        ax1.set_title(f"{self.run_name} | Elbow (train loss vs test MSE)")
        plt.tight_layout()
        return self._save_fig(fig, f"{pfx}__elbow_train_vs_test.png")

    def save_torque_plot(
        self,
        tau_gt: np.ndarray,
        tau_pred: np.ndarray,
        *,
        model_choice: str,
        seed: int,
        prefix: Optional[str] = None,
        max_joints: int = 6,
    ) -> str:
        pfx = prefix or self.run_name
        tau_gt = np.asarray(tau_gt)
        tau_pred = np.asarray(tau_pred)
        n_dof = tau_gt.shape[1]

        fig = plt.figure(figsize=(14, 8), dpi=100)
        for j in range(min(n_dof, max_joints)):
            ax = fig.add_subplot(3, 2, j + 1)
            ax.set_title(f"Joint {j}")
            ax.plot(tau_gt[:, j], label="GT", linewidth=1.0)
            ax.plot(tau_pred[:, j], label="DeLaN", linewidth=1.0, alpha=0.85)
            ax.grid(True, alpha=0.2)
            if j == 0:
                ax.legend()

        plt.tight_layout()
        fname = f"{pfx}__{model_choice}__seed{seed}__DeLaN_Torque.png"
        return self._save_fig(fig, fname)

    def save_metrics_txt(self, lines: Sequence[str], filename: str = "metrics_test.txt") -> str:
        os.makedirs(self.model_dir, exist_ok=True)
        path = os.path.join(self.model_dir, filename)
        with open(path, "w") as f:
            for ln in lines:
                f.write(ln.rstrip("\n") + "\n")
        return path

    def save_metrics_json(self, metrics: Dict[str, Any], filename: str = "metrics.json") -> str:
        os.makedirs(self.model_dir, exist_ok=True)
        path = os.path.join(self.model_dir, filename)
        with open(path, "w") as f:
            json.dump(metrics, f, indent=2)
        return path
```

### B) Modify `rbyt_train_delan_jax.py` to use it

In your current `rbyt_train_delan_jax.py`, the blocks you want to remove are exactly the ones that:

* write loss curve / components / csv / elbow 
* write torque plot + metrics txt/json

Add an import near the top (where you already do imports):

```python
import sys
sys.path.append("/workspace/delan/src")
from delan_plots import DelanPlotter
```

Then replace the *entire* plotting section with calls like:

```python
plotter = DelanPlotter(model_dir=model_dir, run_name=run_name)

# training curves (+ csv)
art_train = plotter.save_training_curves(
    hist_epoch, hist_loss,
    hist_inv=hist_inv, hist_for=hist_for, hist_energy=hist_energy,
    hist_time=hist_time,
)

# elbow
elbow_path = plotter.save_elbow_plot(hist_epoch, hist_loss, hist_test_epoch, hist_test_mse)

# torque plot
if plt is not None:
    pred_tau_np = np.array(pred_tau)
    torque_plot_path = plotter.save_torque_plot(
        np.asarray(test_tau), pred_tau_np,
        model_choice=model_choice, seed=seed
    )
```

For the metrics txt/json block: keep the *content* (since you already have the right fields), but write via plotter:

```python
metrics_lines = [
    f"run_name={run_name}",
    f"hp_preset={args.hp_preset}",
    f"hyper={hyper}",
    f"npz={args.npz}",
    f"ckpt={ckpt_path}",
    f"seed={seed}",
    f"model_type={model_choice}",
    f"dt={dt}",
    f"n_dof={n_dof}",
    f"torque_mse={err_tau}",
    f"torque_rmse={err_tau_rmse}",
    "torque_mse_per_joint=" + " ".join([str(x) for x in err_tau_j]),
    "torque_rmse_per_joint=" + " ".join([str(x) for x in err_tau_rmse_j]),
    f"time_per_sample={t_eval}",
]
metrics_path = plotter.save_metrics_txt(metrics_lines, "metrics_test.txt")

metrics_json_path = plotter.save_metrics_json(metrics, "metrics.json")
```

This removes ~100 lines from `rbyt_train_delan_jax.py` while keeping identical artifacts.

---

## 2) LSTM: move plots into `payload_estimation/lstm/src/lstm_plots.py`

### A) Create file: `payload_estimation/lstm/src/lstm_plots.py`

```python
# payload_estimation/lstm/src/lstm_plots.py
from __future__ import annotations
import os
from dataclasses import dataclass
from typing import Sequence, Dict

import numpy as np
import matplotlib.pyplot as plt


@dataclass
class LstmPlotter:
    out_dir: str
    n_dof: int

    def _save_fig(self, fig, filename: str, dpi: int = 150) -> str:
        os.makedirs(self.out_dir, exist_ok=True)
        path = os.path.join(self.out_dir, filename)
        fig.savefig(path, dpi=dpi)
        plt.close(fig)
        return path

    def save_loss_curve(self, train_loss: Sequence[float], val_loss: Sequence[float]) -> str:
        fig = plt.figure(figsize=(10, 4), dpi=120)
        plt.plot(train_loss, label="train")
        plt.plot(val_loss, label="val")
        plt.title("LSTM training loss (scaled residuals)")
        plt.xlabel("epoch")
        plt.ylabel("MSE")
        plt.grid(True, alpha=0.2)
        plt.legend()
        plt.tight_layout()
        return self._save_fig(fig, "loss_curve.png")

    def save_residual_gt_vs_pred(self, Y_gt: np.ndarray, Y_pred: np.ndarray, K: int = 600) -> str:
        K = min(K, Y_gt.shape[0])
        fig = plt.figure(figsize=(14, 8), dpi=120)
        for j in range(self.n_dof):
            ax = fig.add_subplot(3, 2, j + 1)
            ax.plot(Y_gt[:K, j], label="GT", linewidth=1.0)
            ax.plot(Y_pred[:K, j], label="LSTM", linewidth=1.0, alpha=0.85)
            ax.set_title(f"Residual torque joint {j}")
            ax.grid(True, alpha=0.2)
            if j == 0:
                ax.legend()
        plt.tight_layout()
        return self._save_fig(fig, "residual_gt_vs_pred.png")
```

### B) Modify `train_residual_lstm.py`

Your `train_residual_lstm.py` currently contains the plotting code inline and imports matplotlib at top. 

Add:

```python
import sys
sys.path.append("/workspace/lstm/src")
from lstm_plots import LstmPlotter
```

Then replace the block `# ---------- Plots ----------` (your existing loss curve + residual plot) with:

```python
if not args.no_plots:
    plotter = LstmPlotter(out_dir=args.out_dir, n_dof=n_dof)
    plotter.save_loss_curve(history.history["loss"], history.history["val_loss"])
    plotter.save_residual_gt_vs_pred(Y_test, Y_pred, K=600)
```

(Everything else stays where it is: metrics json, scalers, predictions npz, etc.)

---

## 3) Evaluation: add “D) boxplots across many runs” as a separate script in the evaluation container

You already compute and write the evaluation metrics here (global + per-joint) in `combined_evaluation.py`.
And you already write a flat `metrics_test_H60.txt` like this: 

So: **don’t touch `combined_evaluation.py`** for this — just add a new “sweep summarizer” tool that scans folders and makes plots.

### A) Create: `payload_estimation/evaluation/src/metrics_boxplots.py`

This module:

* reads `shared/models/delan/*/metrics.json` (you already write it) 
* reads `shared/models/lstm/*/metrics_train_test_H*.json`
* reads `shared/evaluation/*/metrics_test_H*.txt` 
* outputs: boxplots + correlation scatters into the evaluation folder of the models (already there from combined_evaluation.py")

If you want, I can paste a complete implementation next, but the key design is:

**Dataframe columns to standardize** (one row per run):

* identifiers parsed from folder names:

  * dataset (`ur5`)
  * run tag (`B`)
  * delan run name (`delan_struct_s4_ep500`)
  * feature_mode (`full`, `tau_hat`, …)
  * H, epochs, batch, units, dropout
* metrics:

  * `delan_rmse`, `delan_joint_rmse[j]`
  * `res_rmse`, `res_joint_rmse[j]`
  * `rg_rmse`, `rg_joint_rmse[j]`

### B) Create entry script: `payload_estimation/evaluation/scripts/make_boxplots.py`

CLI example:

```bash
python3 /workspace/evaluation/scripts/make_boxplots.py \
  --delan_root /workspace/shared/models/delan \
  --lstm_root /workspace/shared/models/lstm \
  --eval_root /workspace/shared/evaluation \
  --out_dir /workspace/shared/evaluation/_summary
```

**Plots I’d generate (all boxplots):**

1. `rg_rmse` grouped by `feature_mode` (this is your ablation headline plot)
2. `res_rmse` grouped by `feature_mode` (how hard residual learning is in each setting)
3. `delan_rmse` grouped by `delan_run` (how much stage-1 varies)
4. per-joint boxplots:

   * `rg_joint_rmse[j]` grouped by `feature_mode` (6 plots or one grid)
5. correlation scatters (best in evaluation because it’s “pipeline-level”):

   * x=`delan_rmse`, y=`rg_rmse`  (does better DeLaN always yield better final torque?)
   * x=`res_rmse`, y=`rg_rmse`    (how much residual fit translates to final torque)
   * x=`delan_joint_rmse[j]`, y=`rg_joint_rmse[j]` (per joint)

This keeps evaluation “the place where cross-run analysis lives”, exactly like you want.

---

## 4) Where to put correlation plots?

Put them in the **evaluation container**, because:

* they relate stage-1 ↔ stage-2 ↔ combined
* they compare *runs*, not a single training job

So: implement correlation plots in the same new evaluation “aggregator” script (not in DeLaN or LSTM training scripts).

---

## 5) Quick mapping: “what stays where” after refactor

**1) DeLaN folder** (`shared/models/delan/...`)

* still produced by `rbyt_train_delan_jax.py`
* but plotting code moves out to `payload_estimation/delan/src/delan_plots.py`
* `rbyt_train_delan_jax.py` just *calls* the plotter.

**2) LSTM folder** (`shared/models/lstm/...`)

* still produced by `train_residual_lstm.py`
* but plotting code moves out to `payload_estimation/lstm/src/lstm_plots.py` 

**3) Evaluation folder** (`shared/evaluation/...`)

* still produced by `combined_evaluation.py` (single-run evaluation plots + metrics)
* plus **new** cross-run plots in `shared/evaluation/_summary/...` from `make_boxplots.py`.

---

If you want, paste (or upload) your current folder-naming helper in `app.py` (where you assemble run names). Then I’ll make the aggregator parse your run names *robustly* (regex that matches your exact tokens like `__feat_full__lstm_s4_H60_ep60_b64_u128_do0p2`).

---
---
---

## Tiny note about `rg_rmse` vs `res_rmse` in your setup

In your math, combined torque is:

* `tau_combined = tau_hat + r_hat`
* `tau_gt = tau_hat + r_gt`

So the combined-torque error is:

* `tau_combined - tau_gt = r_hat - r_gt`

That’s why your evaluation file currently shows `rg_rmse == res_rmse` (it’s expected and correct given this definition).

---
---
---

## 5) Small improvement for `metrics_boxplots.py` (optional)

Right now it only reads `shared/evaluation/*/metrics_*_H*.txt` (which is fine). 
If you later want it to also pull DeLaN + LSTM metrics directly:

* DeLaN: `shared/models/delan/*/metrics.json`
* LSTM: `shared/models/lstm/*/metrics_train_test_H*.json`

…then we can extend it by:

* parsing `delan_id` / `lstm_id` from the evaluation folder name (you already do),
* then locating the matching json files and merging into the row.

---
---
---

okay, now the pipeline works fine. i also added this to combine_evaluation.py:
f.write(f"feature_mode={args.features}\n")
f.write(f"residual_npz={args.residual_npz}\n")
f.write(f"lstm_model={args.model}\n")
f.write(f"lstm_scalers={args.scalers}\n")

see:
    # ---- Save metrics ----
    metrics_path = os.path.join(args.out_dir, f"metrics_{split}_H{H}.txt")
    with open(metrics_path, "w") as f:
        f.write(f"split={split}\nH={H}\n")
        f.write(f"delan_mse={delan_mse}\ndelan_rmse={delan_rmse}\n")
        f.write(f"res_mse={r_mse}\nres_rmse={r_rmse}\n")
        f.write(f"rg_mse={rg_mse}\nrg_rmse={rg_rmse}\n")
        f.write("delan_joint_rmse=" + " ".join(map(str, delan_joint.tolist())) + "\n")
        f.write("res_joint_rmse=" + " ".join(map(str, r_joint.tolist())) + "\n")
        f.write("rg_joint_rmse=" + " ".join(map(str, rg_joint.tolist())) + "\n")
        f.write(f"feature_mode={args.features}\n")
        f.write(f"residual_npz={args.residual_npz}\n")
        f.write(f"lstm_model={args.model}\n")
        f.write(f"lstm_scalers={args.scalers}\n")

    print(f"Saved: {metrics_path}")


now we need to do step 5 as well:
## 5) Small improvement for `metrics_boxplots.py` (optional)

Right now it only reads `shared/evaluation/*/metrics_*_H*.txt` (which is fine). 
If you later want it to also pull DeLaN + LSTM metrics directly:

* DeLaN: `shared/models/delan/*/metrics.json`
* LSTM: `shared/models/lstm/*/metrics_train_test_H*.json`

…then we can extend it by:

* parsing `delan_id` / `lstm_id` from the evaluation folder name (you already do),
* then locating the matching json files and merging into the row.

so the combined_evaluation script computes the plots and metrics for the full stage1 and stage2 pipeline prediction, where the delan and lstm training plots and saves the metrics for only stage1 and stage2. this is perfect. 

so now we need to make sure to get the all useful plots to interpred the full pipeline. 
that includes plotting over metrics of stage1 only with different hyperparameters, as well as for stage2 only.

and then for the full pipeline over different runs. 

so please give what you would plot with the metrics_boxplot.py taking the metrics file from delan, lstm and evaluation into account, i provided them.

please give what plots you would do at first, after that shot discussion we perform the code.

