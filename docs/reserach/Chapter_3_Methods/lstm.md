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