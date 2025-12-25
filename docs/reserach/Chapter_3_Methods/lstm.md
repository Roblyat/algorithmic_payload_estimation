# LSTM
Stage 2 in your diagram is:

1. **Freeze DeLaN**
2. Run DeLaN on each trajectory to get (\hat{\tau}_{\text{DeLaN},k})
3. Compute residuals (r_{\tau,k}=\tau_k-\hat{\tau}_{\text{DeLaN},k})
4. Build sliding windows of length (H)
5. Train an LSTM to predict (r_{\tau,k}), then combine

# Stage 2 Step-by-step

## Step 1 — Freeze DeLaN (save checkpoint to shared/models/delan)

Trained model **persisted in**: `/workspace/shared/models/delan/`

---

## Step 2 — Run DeLaN on each trajectory to compute (\hat\tau_{\text{DeLaN}})

You want trajectory-wise predictions (so we don’t lose boundaries).

### Create a Stage-2 export script in the DeLaN repo

Add a script (in DeLaN repo, e.g.):
`deep_lagrangian_networks/export_ur5_residuals_jax.py`

What it does:

1. Load `delan_ur5_dataset.npz` (trajectory object arrays)
2. Load checkpoint from `/workspace/shared/models/delan/...jax`
3. For each trajectory (i):

   * compute `tau_hat[i] = f_delan(q[i], qd[i], qdd[i])`
   * compute residual `r_tau[i] = tau[i] - tau_hat[i]`
4. Save trajectory-wise residual dataset to processed:

`/workspace/shared/data/processed/ur5_residual_traj.npz`

### Output NPZ structure (trajectory-wise)

This is the key handoff between stage-1 and stage-2 preprocessing:

* `train_t, train_q, train_qd, train_qdd, train_tau`
* `train_tau_hat`  (DeLaN prediction)
* `train_r_tau`    (residual tau)

and the same for test:

* `test_*`

All stored as **object arrays** (one element per trajectory), each element shaped `(T_i, 6)`.

This keeps the segmentation intact.

---

## Step 3 — Compute residuals (r_{\tau,k}=\tau_k-\hat\tau_{\text{DeLaN},k})

This is already done in Step 2 during export, and saved per trajectory:

* `r_tau[i]` has shape `(T_i, 6)`

Important: we still treat **Effort as τ** (same as Stage-1).

---

## Step 4 — Build sliding windows of length (H) (in preprocess container)

Now your preprocess container (plain Python) reads `ur5_residual_traj.npz` and creates training tensors.

### Input (per trajectory)

For a trajectory (i):

* `Q[i]      ∈ R^{T×6}`
* `Qd[i]     ∈ R^{T×6}`
* `Qdd[i]    ∈ R^{T×6}`
* `TauHat[i] ∈ R^{T×6}`
* `RTau[i]   ∈ R^{T×6}`

### Window definition (exactly your thesis)

For each time index (k \ge H-1):

* input window:
  [
  x_k =
  [q_{k-H+1:k},\ \dot q_{k-H+1:k},\ \ddot q_{k-H+1:k},\ \hat\tau_{k-H+1:k}]
  \in \mathbb{R}^{H \times 24}
  ]
* target:
  [
  y_k = r_{\tau,k}\in\mathbb{R}^6
  ]

So:

* `X.shape = (N_windows, H, 24)`
* `Y.shape = (N_windows, 6)`

**Crucial detail:** you build windows **within each trajectory**, never crossing trajectory boundaries.

### Output NPZ structure (window dataset)

Write:

`/workspace/shared/data/processed/ur5_lstm_residual_windows.npz`

Suggested keys:

* `X_train`, `Y_train`
* `X_test`, `Y_test`
* metadata: `H`, `n_dof`, `feature_dim`

Also: **no filtering yet**, just direct measured vel/accel.

---

## Step 5 — Train an LSTM to predict residuals (in LSTM container)

Now the LSTM container loads `ur5_lstm_residual_windows.npz` and trains a Keras model.

A minimal model (matches the tutorial style):

* LSTM → Dropout → LSTM (optional) → Dense(6)

Training details:

* shuffle windows **at batch level** (that’s fine; each sample is already a sequence)
* keep test set fixed

Then at inference:
[
\hat\tau_{\text{RG},k} = \hat\tau_{\text{DeLaN},k} + \hat r_{\tau,k}
]


---
---
---
Alright—Stage 2, step 5. Since you already built **trajectory-safe windows** (`X_*` and `Y_*`) you **do not need** the GfG “create sequences + split” code anymore. You’re already past that part (you split by trajectory, then windowed inside trajectories). What we will reuse from GfG is:

* Keras LSTM architecture pattern (stacked LSTM + dropout)
* `model.fit(...)`
* predict on test set
* metric + plots

Below is a clean, reproducible “train → evaluate → predict residuals → combine with DeLaN” workflow.

---

# 5) Train LSTM on residual torques and combine

## A) Add a training script in the LSTM container

Create: `services/lstm/scripts/train_residual_lstm.py`

```python
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt

import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense, Dropout
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint


def rmse(y_true, y_pred):
    return float(np.sqrt(np.mean((y_true - y_pred) ** 2)))


def per_joint_rmse(y_true, y_pred):
    # returns (6,)
    return np.sqrt(np.mean((y_true - y_pred) ** 2, axis=0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", required=True, help="ur5_lstm_windows_H50.npz")
    ap.add_argument("--out_dir", default="/workspace/shared/models/lstm/residual_lstm_H50")
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--val_split", type=float, default=0.1)
    ap.add_argument("--seed", type=int, default=4)
    ap.add_argument("--units", type=int, default=128)
    ap.add_argument("--dropout", type=float, default=0.2)
    ap.add_argument("--no_plots", action="store_true")
    args = ap.parse_args()

    tf.random.set_seed(args.seed)
    np.random.seed(args.seed)

    os.makedirs(args.out_dir, exist_ok=True)

    d = np.load(args.npz)
    X_train = d["X_train"].astype(np.float32)  # (N, H, 24)
    Y_train = d["Y_train"].astype(np.float32)  # (N, 6)
    X_test  = d["X_test"].astype(np.float32)
    Y_test  = d["Y_test"].astype(np.float32)

    H = int(d["H"])
    feature_dim = int(d["feature_dim"])
    n_dof = int(d["n_dof"])

    print("################################################")
    print("LSTM Residual Dataset:")
    print(f"  npz = {args.npz}")
    print(f"   H  = {H}")
    print(f"  din = {feature_dim}  (expected 24)")
    print(f" dout = {n_dof}        (expected 6)")
    print(f"  X_train = {X_train.shape}, Y_train = {Y_train.shape}")
    print(f"  X_test  = {X_test.shape},  Y_test  = {Y_test.shape}")
    print("################################################")

    # ---- Model (GfG-style, but output=6 instead of 1) ----
    model = Sequential()
    model.add(LSTM(units=args.units, return_sequences=True, input_shape=(H, feature_dim)))
    model.add(Dropout(args.dropout))
    model.add(LSTM(units=args.units))
    model.add(Dropout(args.dropout))
    model.add(Dense(n_dof))  # predict residual torque vector (6)

    model.compile(optimizer="adam", loss="mse")
    model.summary()

    ckpt_path = os.path.join(args.out_dir, "best.keras")
    callbacks = [
        EarlyStopping(monitor="val_loss", patience=10, restore_best_weights=True),
        ModelCheckpoint(ckpt_path, monitor="val_loss", save_best_only=True),
    ]

    history = model.fit(
        X_train, Y_train,
        epochs=args.epochs,
        batch_size=args.batch,
        validation_split=args.val_split,
        shuffle=True,          # OK: each sample is already a window
        callbacks=callbacks,
        verbose=2
    )

    # ---- Evaluate ----
    Y_pred = model.predict(X_test, batch_size=args.batch, verbose=0).astype(np.float32)

    total_rmse = rmse(Y_test, Y_pred)
    joint_rmse = per_joint_rmse(Y_test, Y_pred)

    print("\n################################################")
    print("LSTM Residual Evaluation (test):")
    print(f"Total RMSE: {total_rmse:.4f}")
    print("Per-joint RMSE:", " ".join([f"{x:.4f}" for x in joint_rmse]))
    print("################################################\n")

    # Save predictions for later combination / analysis
    np.savez(
        os.path.join(args.out_dir, "predictions_test.npz"),
        Y_test=Y_test,
        Y_pred=Y_pred,
        H=np.int32(H),
        feature_dim=np.int32(feature_dim),
        n_dof=np.int32(n_dof),
    )
    print(f"Saved predictions: {os.path.join(args.out_dir, 'predictions_test.npz')}")
    print(f"Saved best model:  {ckpt_path}")

    # ---- Plots ----
    if not args.no_plots:
        # 1) Training curve
        plt.figure(figsize=(10, 4), dpi=120)
        plt.plot(history.history["loss"], label="train")
        plt.plot(history.history["val_loss"], label="val")
        plt.title("LSTM training loss")
        plt.xlabel("epoch")
        plt.ylabel("MSE")
        plt.grid(True, alpha=0.2)
        plt.legend()
        out = os.path.join(args.out_dir, "loss_curve.png")
        plt.tight_layout()
        plt.savefig(out, dpi=150)
        print(f"Saved: {out}")

        # 2) Residual GT vs Pred for each joint on test (first K samples)
        K = min(600, Y_test.shape[0])
        fig = plt.figure(figsize=(14, 8), dpi=120)
        for j in range(n_dof):
            ax = fig.add_subplot(3, 2, j + 1)
            ax.plot(Y_test[:K, j], label="GT", linewidth=1.0)
            ax.plot(Y_pred[:K, j], label="LSTM", linewidth=1.0, alpha=0.85)
            ax.set_title(f"Residual torque joint {j}")
            ax.grid(True, alpha=0.2)
            if j == 0:
                ax.legend()
        plt.tight_layout()
        out = os.path.join(args.out_dir, "residual_gt_vs_pred.png")
        plt.savefig(out, dpi=150)
        print(f"Saved: {out}")

        plt.close("all")


if __name__ == "__main__":
    main()
```

### What this does

* trains a stacked LSTM like the tutorial, but predicts **6 outputs** (one residual per joint)
* saves:

  * `best.keras` model
  * `predictions_test.npz`
  * plots (`loss_curve.png`, `residual_gt_vs_pred.png`)
* **no scaling yet** (baseline)

---

## B) Run training (inside the LSTM container)

From host:

```bash
docker compose exec lstm python3 /workspace/scripts/train_residual_lstm.py \
  --npz /workspace/shared/data/processed/ur5_lstm_windows_H50.npz \
  --out_dir /workspace/shared/models/lstm/residual_lstm_H50 \
  --epochs 60 --batch 64
```

You’ll find results on the host in:
`shared/models/lstm/residual_lstm_H50/`

---

# C) Combine LSTM residuals with DeLaN (what “combined model” means)

To *actually* compute:
[
\hat\tau_{\text{RG}} = \hat\tau_{\text{DeLaN}} + \hat r_\tau
]
you need **time-aligned** `tau_hat` and predicted residuals for the same samples.

Right now your windows dataset doesn’t store indices back to the original trajectories/time steps. That’s fine for training, but for “combine + evaluate torque” we need either:

### Option 1 (recommended): do combine/evaluation on the residual trajectory NPZ

Use `ur5_residual_traj.npz` and build windows on-the-fly per trajectory in an evaluation script:

* for each trajectory:

  * build `X_i` windows (shape `(T-H+1, H, 24)`)
  * run LSTM → `r_hat_i` (shape `(T-H+1, 6)`)
  * align to time indices `k=H-1..T-1`
  * `tau_RG_i[k] = tau_hat_i[k] + r_hat_i[k]`
* compare `tau_RG_i[k]` to measured `tau_i[k]`

That produces a clean plot:

* **GT τ vs DeLaN τ̂ vs Combined τ̂_RG** per joint.

I can give you this evaluator script next (runs in the **LSTM** container; loads TF model + loads `ur5_residual_traj.npz`).

### Option 2: store indices during window building

Modify `build_lstm_windows.py` to also store:

* `traj_id`, `k_index` per window
  Then you can reconstruct combined torques directly. This is also good, but Option 1 is simpler for now.

---

# Next step

1. Add `train_residual_lstm.py` (above)
2. Train it once
3. Then I’ll give you **`evaluate_and_combine.py`** that:

   * loads `/workspace/shared/data/processed/ur5_residual_traj.npz`
   * loads `/workspace/shared/models/lstm/residual_lstm_H50/best.keras`
   * outputs plots + MSE/RMSE for:

     * residual prediction
     * torque prediction for DeLaN alone
     * torque prediction for DeLaN+LSTM combined

If you want, tell me whether you prefer:

* **evaluation script in `services/lstm/scripts/`**, or
* evaluation in preprocess (numpy only) but that would require exporting predicted residuals from TF anyway.


### Option 1:
Got it — here’s an **`lstm/scripts/evaluate_and_combine.py`** that:

* loads `ur5_residual_traj.npz` (trajectory-wise)
* loads your trained TF model (`best.keras`)
* **builds windows per trajectory** (no boundary crossing)
* predicts residuals (\hat r_\tau)
* combines: (\hat\tau_{\mathrm{RG}} = \hat\tau_{\mathrm{DeLaN}} + \hat r_\tau)
* computes **RMSE/MSE** for:

  * DeLaN torque only
  * residual prediction
  * combined torque
* saves **plots**:

  * residual GT vs predicted
  * torque GT vs DeLaN vs combined
* (optionally) saves per-trajectory predictions to an NPZ for later analysis

---

## 1) Create `services/lstm/scripts/evaluate_and_combine.py`

```python
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf


def build_windows(feat: np.ndarray, H: int) -> np.ndarray:
    """
    feat: (T, D)
    returns X: (T-H+1, H, D), where each window ends at time k (k>=H-1)
    """
    T, D = feat.shape
    if T < H:
        return np.zeros((0, H, D), dtype=np.float32)

    X = np.zeros((T - H + 1, H, D), dtype=np.float32)
    for i, k in enumerate(range(H - 1, T)):
        X[i] = feat[k - H + 1 : k + 1]
    return X


def mse(y_true, y_pred):
    return float(np.mean((y_true - y_pred) ** 2))


def rmse(y_true, y_pred):
    return float(np.sqrt(np.mean((y_true - y_pred) ** 2)))


def per_joint_rmse(y_true, y_pred):
    return np.sqrt(np.mean((y_true - y_pred) ** 2, axis=0))


def concat_valid_across_trajs(traj_list, H):
    """
    Given list of (T_i, dof) arrays, concatenate only valid indices k>=H-1.
    Returns concatenated array shape (sum_i (T_i-H+1), dof).
    """
    chunks = []
    for a in traj_list:
        a = np.asarray(a, dtype=np.float32)
        if a.shape[0] >= H:
            chunks.append(a[H - 1 :])
    if not chunks:
        return np.zeros((0, 0), dtype=np.float32)
    return np.vstack(chunks).astype(np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--residual_npz", required=True,
                    help="Trajectory-wise residual NPZ (ur5_residual_traj.npz)")
    ap.add_argument("--model", required=True,
                    help="Path to trained keras model (best.keras)")
    ap.add_argument("--out_dir", default="/workspace/shared/models/lstm/eval_combined_H50")
    ap.add_argument("--H", type=int, default=50)
    ap.add_argument("--split", choices=["test", "train"], default="test")
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--max_plot_samples", type=int, default=800)
    ap.add_argument("--save_pred_npz", action="store_true",
                    help="Save per-trajectory predictions to NPZ in out_dir")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    d = np.load(args.residual_npz, allow_pickle=True)
    split = args.split
    H = args.H

    q_list       = list(d[f"{split}_q"])
    qd_list      = list(d[f"{split}_qd"])
    qdd_list     = list(d[f"{split}_qdd"])
    tau_list     = list(d[f"{split}_tau"])
    tau_hat_list = list(d[f"{split}_tau_hat"])
    r_tau_list   = list(d[f"{split}_r_tau"])

    n_traj = len(q_list)
    n_dof = int(np.asarray(q_list[0]).shape[1])
    feature_dim = 4 * n_dof

    print("################################################")
    print("Evaluate & Combine")
    print(f" residual_npz = {args.residual_npz}")
    print(f" model        = {args.model}")
    print(f" split        = {split}")
    print(f" H            = {H}")
    print(f" n_traj       = {n_traj}")
    print(f" n_dof        = {n_dof}")
    print(f" feature_dim  = {feature_dim}")
    print("################################################")

    model = tf.keras.models.load_model(args.model)

    # Store per-trajectory predicted residuals aligned to original time
    r_hat_traj = []
    tau_rg_traj = []

    # Also build concatenated arrays (valid region only) for global metrics/plots
    tau_gt_valid_all = []
    tau_delan_valid_all = []
    tau_rg_valid_all = []
    r_gt_valid_all = []
    r_hat_valid_all = []

    for i in range(n_traj):
        q   = np.asarray(q_list[i], dtype=np.float32)
        qd  = np.asarray(qd_list[i], dtype=np.float32)
        qdd = np.asarray(qdd_list[i], dtype=np.float32)
        tau = np.asarray(tau_list[i], dtype=np.float32)
        tau_hat = np.asarray(tau_hat_list[i], dtype=np.float32)
        r_gt = np.asarray(r_tau_list[i], dtype=np.float32)

        T = q.shape[0]
        if T < H:
            # not enough length for a single window
            r_hat_traj.append(np.full((T, n_dof), np.nan, dtype=np.float32))
            tau_rg_traj.append(np.full((T, n_dof), np.nan, dtype=np.float32))
            continue

        feat = np.concatenate([q, qd, qdd, tau_hat], axis=1).astype(np.float32)  # (T, 24)
        X = build_windows(feat, H)  # (T-H+1, H, 24)

        r_hat_valid = model.predict(X, batch_size=args.batch, verbose=0).astype(np.float32)  # (T-H+1, 6)

        # Align predicted residuals to full timeline (first H-1 undefined)
        r_hat_full = np.full((T, n_dof), np.nan, dtype=np.float32)
        r_hat_full[H - 1 :] = r_hat_valid

        tau_rg_full = np.full((T, n_dof), np.nan, dtype=np.float32)
        tau_rg_full[H - 1 :] = tau_hat[H - 1 :] + r_hat_valid

        r_hat_traj.append(r_hat_full)
        tau_rg_traj.append(tau_rg_full)

        # Collect valid regions for global metrics
        tau_gt_valid_all.append(tau[H - 1 :])
        tau_delan_valid_all.append(tau_hat[H - 1 :])
        tau_rg_valid_all.append(tau_rg_full[H - 1 :])

        r_gt_valid_all.append(r_gt[H - 1 :])
        r_hat_valid_all.append(r_hat_valid)

        if (i + 1) % 25 == 0 or (i + 1) == n_traj:
            print(f"  done {i+1}/{n_traj}", flush=True)

    tau_gt_valid_all = np.vstack(tau_gt_valid_all).astype(np.float32)
    tau_delan_valid_all = np.vstack(tau_delan_valid_all).astype(np.float32)
    tau_rg_valid_all = np.vstack(tau_rg_valid_all).astype(np.float32)
    r_gt_valid_all = np.vstack(r_gt_valid_all).astype(np.float32)
    r_hat_valid_all = np.vstack(r_hat_valid_all).astype(np.float32)

    # ---- Metrics ----
    delan_mse = mse(tau_gt_valid_all, tau_delan_valid_all)
    delan_rmse = rmse(tau_gt_valid_all, tau_delan_valid_all)

    rg_mse = mse(tau_gt_valid_all, tau_rg_valid_all)
    rg_rmse = rmse(tau_gt_valid_all, tau_rg_valid_all)

    r_mse = mse(r_gt_valid_all, r_hat_valid_all)
    r_rmse = rmse(r_gt_valid_all, r_hat_valid_all)

    delan_joint = per_joint_rmse(tau_gt_valid_all, tau_delan_valid_all)
    rg_joint = per_joint_rmse(tau_gt_valid_all, tau_rg_valid_all)
    r_joint = per_joint_rmse(r_gt_valid_all, r_hat_valid_all)

    print("\n################################################")
    print(f"Stage-2 Evaluation ({split}, valid k>=H-1):")
    print(f"DeLaN torque:   MSE={delan_mse:.6e}  RMSE={delan_rmse:.6e}")
    print("  per-joint RMSE:", " ".join([f"{x:.4f}" for x in delan_joint]))
    print(f"Residual LSTM:  MSE={r_mse:.6e}      RMSE={r_rmse:.6e}")
    print("  per-joint RMSE:", " ".join([f"{x:.4f}" for x in r_joint]))
    print(f"Combined torque MSE={rg_mse:.6e}     RMSE={rg_rmse:.6e}")
    print("  per-joint RMSE:", " ".join([f"{x:.4f}" for x in rg_joint]))
    print("################################################\n")

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
    print(f"Saved: {metrics_path}")

    # ---- Optional save predictions ----
    if args.save_pred_npz:
        out_npz = os.path.join(args.out_dir, f"combined_predictions_{split}_H{H}.npz")
        np.savez(
            out_npz,
            r_hat=np.asarray(r_hat_traj, dtype=object),
            tau_rg=np.asarray(tau_rg_traj, dtype=object),
        )
        print(f"Saved: {out_npz}")

    # ---- Plots (first K samples of concatenated valid region) ----
    K = min(args.max_plot_samples, tau_gt_valid_all.shape[0])

    # 1) Residual GT vs Pred
    fig = plt.figure(figsize=(14, 8), dpi=120)
    for j in range(n_dof):
        ax = fig.add_subplot(3, 2, j + 1)
        ax.plot(r_gt_valid_all[:K, j], label="GT residual", linewidth=1.0)
        ax.plot(r_hat_valid_all[:K, j], label="LSTM residual", linewidth=1.0, alpha=0.85)
        ax.set_title(f"Residual joint {j}")
        ax.grid(True, alpha=0.2)
        if j == 0:
            ax.legend()
    plt.tight_layout()
    out = os.path.join(args.out_dir, f"residual_gt_vs_pred_{split}_H{H}.png")
    plt.savefig(out, dpi=150)
    print(f"Saved: {out}")
    plt.close(fig)

    # 2) Torque GT vs DeLaN vs Combined
    fig = plt.figure(figsize=(14, 8), dpi=120)
    for j in range(n_dof):
        ax = fig.add_subplot(3, 2, j + 1)
        ax.plot(tau_gt_valid_all[:K, j], label="GT tau", linewidth=1.0)
        ax.plot(tau_delan_valid_all[:K, j], label="DeLaN tau_hat", linewidth=1.0, alpha=0.85)
        ax.plot(tau_rg_valid_all[:K, j], label="Combined tau_RG", linewidth=1.0, alpha=0.85)
        ax.set_title(f"Torque joint {j}")
        ax.grid(True, alpha=0.2)
        if j == 0:
            ax.legend()
    plt.tight_layout()
    out = os.path.join(args.out_dir, f"torque_gt_vs_delan_vs_combined_{split}_H{H}.png")
    plt.savefig(out, dpi=150)
    print(f"Saved: {out}")
    plt.close(fig)


if __name__ == "__main__":
    main()
```

---

## 2) Run it (from host)

Assuming:

* residual trajectories: `/workspace/shared/data/processed/ur5_residual_traj.npz`
* trained model: `/workspace/shared/models/lstm/residual_lstm_H50/best.keras`

```bash
docker compose exec lstm python3 /workspace/scripts/evaluate_and_combine.py \
  --residual_npz /workspace/shared/data/processed/ur5_residual_traj.npz \
  --model /workspace/shared/models/lstm/residual_lstm_H50/best.keras \
  --out_dir /workspace/shared/models/lstm/residual_lstm_H50/eval_combined \
  --H 50 \
  --split test \
  --save_pred_npz
```

Outputs (on host) in:
`shared/models/lstm/residual_lstm_H50/eval_combined/`

* `metrics_test_H50.txt`
* `residual_gt_vs_pred_test_H50.png`
* `torque_gt_vs_delan_vs_combined_test_H50.png`
* optional `combined_predictions_test_H50.npz`

---

## What “combine” means here (explicit)

* We evaluate only the **valid region** `k >= H-1` (first `H-1` samples are undefined because no full window exists).
* Combined torque:
  [
  \hat\tau_{\mathrm{RG}}[k] = \hat\tau_{\mathrm{DeLaN}}[k] + \hat r_\tau[k]
  ]

---

If you run this and paste the `metrics_test_H50.txt` values, we can decide the next improvement: add **feature-wise mean/std normalization for X** and **per-joint scaling for Y**, in a clean way that’s reversible and logged.
