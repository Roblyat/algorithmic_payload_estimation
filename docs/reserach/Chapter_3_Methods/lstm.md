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

Alright—Stage 2, step 5. Since you already built **trajectory-safe windows** (`X_*` and `Y_*`) you **do not need** the GfG “create sequences + split” code anymore. You’re already past that part (you split by trajectory, then windowed inside trajectories). What we will reuse from GfG is:

* Keras LSTM architecture pattern (stacked LSTM + dropout)
* `model.fit(...)`
* predict on test set
* metric + plots

Below is a clean, reproducible “train → evaluate → predict residuals → combine with DeLaN” workflow.

---

# C) Combine LSTM residuals with DeLaN (what “combined model” means)

To *actually* compute:
[
\hat\tau_{\text{RG}} = \hat\tau_{\text{DeLaN}} + \hat r_\tau
]
you need **time-aligned** `tau_hat` and predicted residuals for the same samples.

Right now your windows dataset doesn’t store indices back to the original trajectories/time steps. That’s fine for training, but for “combine + evaluate torque” we need either:

### Option 1: combine/evaluation on the residual trajectory NPZ

Use `ur5_residual_traj.npz` and build windows on-the-fly per trajectory in an evaluation script:

* for each trajectory:

  * build `X_i` windows (shape `(T-H+1, H, 24)`)
  * run LSTM → `r_hat_i` (shape `(T-H+1, 6)`)
  * align to time indices `k=H-1..T-1`
  * `tau_RG_i[k] = tau_hat_i[k] + r_hat_i[k]`
* compare `tau_RG_i[k]` to measured `tau_i[k]`

That produces a clean plot:

* **GT τ vs DeLaN τ̂ vs Combined τ̂_RG** per joint.

### Option 2: store indices during window building

Modify `build_lstm_windows.py` to also store:

* `traj_id`, `k_index` per window
  Then you can reconstruct combined torques directly. This is also good, but Option 1 is simpler for now.

---

**`lstm/scripts/evaluate_and_combine.py`** that:

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

---

## 2) Run it (from host)

Assuming:

* residual trajectories: `/workspace/shared/data/processed/ur5_residual_traj.npz`
* trained model: `/workspace/shared/models/lstm/residual_lstm_H50/best.keras`

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


Yep — let’s add **(1) feature-wise mean/std normalization for X** and **(2) per-joint scaling for Y** in a way that’s:

* **computed from training only**
* **saved to disk** (so reversible)
* **used consistently** in training + evaluation + combine
* **logged** (so your thesis/pipeline is reproducible)

Below are clean drop-in updates for:

* `services/lstm/scripts/train_residual_lstm.py`
* `services/lstm/scripts/evaluate_and_combine.py`

---

## What we’ll do

### X normalization (feature-wise)

Your LSTM input is `(N, H, 24)` where `24 = 4*n_dof`.

Compute on **training only**:

* flatten time: `X_train.reshape(-1, 24)`
* `x_mean` shape `(24,)`
* `x_std` shape `(24,)` with epsilon for safety
* normalize both train and test:
  [
  X' = (X - \mu_X) / \sigma_X
  ]

### Y scaling (per joint)

Your target is `(N, 6)`.

Compute on **training only**:

* `y_mean` shape `(6,)`
* `y_std` shape `(6,)`
* scale:
  [
  Y' = (Y - \mu_Y) / \sigma_Y
  ]
  At inference, invert:
  [
  \hat Y = \hat Y' \cdot \sigma_Y + \mu_Y
  ]

### Save scalers

Write to:
`/workspace/shared/models/lstm/<run_dir>/scalers_H50.npz`

with keys:

* `x_mean, x_std, y_mean, y_std, eps`

---

# 1) Update `train_residual_lstm.py`

### Re-train with scaling

```bash
python3 train_residual_lstm.py \
  --npz /workspace/shared/data/processed/ur5_lstm_windows_H50.npz \
  --out_dir /workspace/shared/models/lstm/residual_lstm_H50_scaled \
  --epochs 60 --batch 64
```

This produces:

* `best.keras`
* `scalers_H50.npz`
* plots

---

# 2) Update `evaluate_and_combine.py` to use scalers

Patch your `evaluate_and_combine.py` like this:

### Add helper functions near top

```python
def apply_x_scaler_feat(feat: np.ndarray, x_mean: np.ndarray, x_std: np.ndarray):
    # feat: (T, D)
    return ((feat - x_mean[None, :]) / x_std[None, :]).astype(np.float32)

def invert_y_scaler(y_scaled: np.ndarray, y_mean: np.ndarray, y_std: np.ndarray):
    return (y_scaled * y_std[None, :] + y_mean[None, :]).astype(np.float32)
```

### Add CLI arg for scalers

In argparse section:

```python
ap.add_argument("--scalers", required=True, help="scalers_H50.npz from training")
```

### Load scalers after loading model

Right after `model = tf.keras.models.load_model(...)`:

```python
sc = np.load(args.scalers)
x_mean = sc["x_mean"].astype(np.float32)
x_std  = sc["x_std"].astype(np.float32)
y_mean = sc["y_mean"].astype(np.float32)
y_std  = sc["y_std"].astype(np.float32)
```

### Normalize features before windowing + invert residual prediction

Inside the trajectory loop, replace:

```python
feat = np.concatenate([q, qd, qdd, tau_hat], axis=1).astype(np.float32)
X = build_windows(feat, H)
r_hat_valid = model.predict(X, ... )
```

with:

```python
feat = np.concatenate([q, qd, qdd, tau_hat], axis=1).astype(np.float32)  # (T, 24)
feat_n = apply_x_scaler_feat(feat, x_mean, x_std)
X = build_windows(feat_n, H)  # windows are normalized

r_hat_valid_scaled = model.predict(X, batch_size=args.batch, verbose=0).astype(np.float32)
r_hat_valid = invert_y_scaler(r_hat_valid_scaled, y_mean, y_std)  # back to physical units
```

Everything else stays the same (combine, metrics, plots).

---

## Run scaled evaluation+combine

```bash
python3 evaluate_and_combine.py \
  --residual_npz /workspace/shared/data/processed/ur5_residual_traj.npz \
  --model /workspace/shared/models/lstm/residual_lstm_H50_scaled/best.keras \
  --scalers /workspace/shared/models/lstm/residual_lstm_H50_scaled/scalers_H50.npz \
  --out_dir /workspace/shared/models/lstm/residual_lstm_H50_scaled/eval_combined \
  --H 50 --split test --save_pred_npz
```

---

# Why this should improve your numbers

Your current metrics show one joint dominates (joint 2 residual RMSE ~2.71). Scaling does two good things:

* **X normalization** prevents the LSTM from over-weighting large-magnitude features (e.g., some joints’ velocities/torques)
* **Y per-joint scaling** ensures the loss treats each joint fairly (same idea as your DeLaN per-joint loss normalization)

So you should see:

* residual RMSE improve (especially large joints)
* combined torque RMSE improve identically (since combined error = residual error)

---

If you want, after you run the scaled version, paste the new `metrics_test_H50.txt` and we can decide whether to:

* increase/decrease `H`
* switch to Huber loss
* add a small Dense “head” with joint-wise weighting
* add a “tau_hat” only vs full feature ablation (to see what’s driving residual prediction)


---
---
---

Those scaled metrics are a **big step up** ✅

* **Residual / RG RMSE**: **1.225 → 0.997** (≈ **−19%**)
* Joint 1 (the big one) residual RMSE: **2.71 → 2.17** (≈ **−20%**)
* Others improved too; joint 3 slightly worse (0.34→0.38) but overall clearly better.

And the plots match the story: the green “combined” track is consistently closer to GT.

---

## Next: “tau_hat only” vs “full features” ablation (recommended)

You want to know whether the LSTM is mostly learning:

* “correction as a function of DeLaN prediction” (i.e. (\hat\tau_{\text{DeLaN}}) alone)
  or
* it needs state history ((q,\dot q,\ddot q)) too.

Your current feature per time step is:
[
x_k = [q_k,\dot q_k,\ddot q_k,\hat\tau_{\text{DeLaN},k}] \in \mathbb{R}^{24}
]

### Ablation variants to test

Let `dof=6`:

1. **tau_hat-only** (recommended first):
   [
   x_k = [\hat\tau_{\text{DeLaN},k}] \in \mathbb{R}^{6}
   ]

2. **state-only**:
   [
   x_k = [q_k,\dot q_k,\ddot q_k] \in \mathbb{R}^{18}
   ]

3. **tau_hat + qd/qdd only** (often strong):
   [
   x_k = [\dot q_k,\ddot q_k,\hat\tau_{\text{DeLaN},k}] \in \mathbb{R}^{18}
   ]

You can implement this with one clean switch.

---

# A) Update window builder to support `--features`

In your `build_lstm_windows.py`, add:

### CLI arg

* `--features {full,tau_hat,state,state_tauhat}`

### Feature slicing

Assuming each trajectory array is `(T,6)`:

```python
def build_features(q, qd, qdd, tau_hat, mode: str):
    if mode == "full":
        return np.concatenate([q, qd, qdd, tau_hat], axis=1)          # (T,24)
    if mode == "tau_hat":
        return tau_hat                                                # (T,6)
    if mode == "state":
        return np.concatenate([q, qd, qdd], axis=1)                   # (T,18)
    if mode == "state_tauhat":
        return np.concatenate([qd, qdd, tau_hat], axis=1)             # (T,18)
    raise ValueError(f"Unknown mode: {mode}")
```

Store in NPZ:

* `feature_mode` (string)
* `feature_dim` accordingly

---

## Run: build tau_hat-only windows

```bash
docker compose exec preprocess python3 /workspace/preprocess/scripts/build_lstm_windows.py \
  --in_npz /workspace/shared/data/processed/ur5_residual_traj.npz \
  --out_npz /workspace/shared/data/processed/ur5_lstm_windows_H50_tauhat.npz \
  --H 50 \
  --features tau_hat
```

---

# B) Training script: no big changes needed

Because it already uses:

* `feature_dim = X_train.shape[-1]`
* feature-wise X scaler
* per-joint Y scaler

So just train on the new window dataset:

```bash
docker compose exec lstm python3 /workspace/scripts/train_residual_lstm.py \
  --npz /workspace/shared/data/processed/ur5_lstm_windows_H50_tauhat.npz \
  --out_dir /workspace/shared/models/lstm/residual_lstm_H50_tauhat_scaled \
  --epochs 60 --batch 64
```

---

# C) Evaluation+combine script

Same idea: it must build features the same way it was trained.

So add arg:

* `--features` (same enum)

and in `evaluate_and_combine.py` use the same `build_features(...)` function before windowing + normalization.

Example run:

```bash
docker compose exec lstm python3 /workspace/scripts/evaluate_and_combine.py \
  --residual_npz /workspace/shared/data/processed/ur5_residual_traj.npz \
  --model /workspace/shared/models/lstm/residual_lstm_H50_tauhat_scaled/best.keras \
  --scalers /workspace/shared/models/lstm/residual_lstm_H50_tauhat_scaled/scalers_H50.npz \
  --out_dir /workspace/shared/models/lstm/residual_lstm_H50_tauhat_scaled/eval_combined \
  --H 50 --split test \
  --features tau_hat
```

---

## How to interpret results

If **tau_hat-only** is close to your **full** performance, then:

* residuals are mostly a *systematic correction* of DeLaN output (e.g. friction/backlash, unmodeled gravity offsets, controller effects)

If **full > tau_hat-only**, then:

* residuals depend strongly on state history (e.g. dynamic friction / stiction / temperature / hysteresis effects that correlate with qd/qdd)

If **state-only** is surprisingly strong:

* DeLaN is less informative than expected, and LSTM is learning dynamics directly (not ideal, but good to know)

---

## Minimal “experiment matrix” I’d run next

Keep everything fixed (seed, epochs, H=50), only change `--features`:

* `full` (baseline you already have)
* `tau_hat`
* `state_tauhat`
* `state`

Then compare only **rg_rmse** and **per-joint rg_rmse**.

---

If you want, paste the filenames you currently have for:

* `build_lstm_windows.py`
* `evaluate_and_combine.py`

…and I’ll give you the exact patch chunks (diff-style) so you can drop them in with zero guessing.