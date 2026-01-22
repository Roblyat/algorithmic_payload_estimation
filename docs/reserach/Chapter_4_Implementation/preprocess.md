## DeLaN preprocess pipeline
#### Downstream usage note (DeLaN vs LSTM)

* For **DeLaN** (per-timestep inverse dynamics), you often **flatten** trajectories into `(N,6)` arrays by concatenating all `T_i`.
* For **sequence models** (LSTM), you typically keep the trajectory structure and sample windows (or pad+mask).

### Inputs

A single **long-format CSV log** where each row is one joint at one timestamp:

* `Time`
* `Joint Name`
* `Position`
* `Velocity`
* `Acceleration`
* `Effort` (used as τ / torque target)

---

### 1) Load raw CSV

Read the CSV into a pandas DataFrame (no extra parsing/cleaning yet).

---

### 2) Select DOF joints (UR5 = 6)

Filter rows to keep only the configured joint names, in a fixed joint order:

```
[dof_joints] = [
  ur5_shoulder_pan_joint,
  ur5_shoulder_lift_joint,
  ur5_elbow_joint,
  ur5_wrist_1_joint,
  ur5_wrist_2_joint,
  ur5_wrist_3_joint
]
```

All other joints are dropped.

---

### 3) Segment into trajectories (add `trajectory_id`)

Goal: define **trajectory boundaries** so train/test split can happen at the trajectory level.

Two modes:

**A) Time-gap segmentation**

* Sort by `Time`
* Compute `dt = Time.diff()`
* Start a new trajectory whenever `dt > time_gap_seconds`
* `trajectory_id` increments each time a gap is detected

**B) Fixed-length segmentation**

* Treat each unique timestamp as one “frame”
* Assign a frame index 0,1,2,...
* Group every `frames_per_trajectory` frames into one trajectory:

  * `trajectory_id = frame_idx // frames_per_trajectory`

Result: the long dataframe now contains an integer `trajectory_id` column.

---

### 4) Build trajectory tensors (pivot long → wide per `trajectory_id`)

For each `trajectory_id = i`:

1. Collect and sort unique timestamps → `t_i` with shape `(T_i,)`
2. For each signal field in `{Position, Velocity, Acceleration, Effort}`:

   * Pivot to wide format:

     * index = `Time`
     * columns = `Joint Name` (ordered by `dof_joints`)
     * values = that signal
   * Reindex rows to `t_i` and columns to the 6-joint order
   * Convert to NumPy
   * **Fail if any NaNs appear** (meaning missing joint samples at some time)

This produces one trajectory object with aligned matrices:

* `t_i`: `(T_i,)`
* `q_i`: `(T_i, 6)` from `Position`
* `qd_i`: `(T_i, 6)` from `Velocity`
* `qdd_i`: `(T_i, 6)` from `Acceleration`
* `tau_i`: `(T_i, 6)` from `Effort`

A string label is assigned like: `"traj_0000"`, `"traj_0001"`, …

Conceptually:

```python
trajs = [
  {
    "label": "traj_0000",
    "t":   t0,    # (T0,)
    "q":   q0,    # (T0,6)
    "qd":  qd0,   # (T0,6)
    "qdd": qdd0,  # (T0,6)
    "tau": tau0,  # (T0,6)
  },
  {
    "label": "traj_0001",
    "t":   t1,    # (T1,)
    "q":   q1,    # (T1,6)
    "qd":  qd1,   # (T1,6)
    "qdd": qdd1,  # (T1,6)
    "tau": tau1,  # (T1,6)
  },
  ...
]
```

---

### 5) Train/test split (trajectory-level)

Shuffle the list of trajectories and split by fraction:

* select `test_fraction` of trajectories → test set
* remaining trajectories → train set

No trajectory is split across train/test.

---

### 6) Save to NPZ (ragged trajectories via object arrays)

Write an NPZ file where each field is stored as a **NumPy object array** (ragged list) of length `n_traj`:

**Train:**

* `train_labels` : `(n_train,)` (strings like `"traj_0007"`)
* `train_t`      : `(n_train,)` where `train_t[i]` is `(T_i,)`
* `train_q`      : `(n_train,)` where `train_q[i]` is `(T_i,6)`
* `train_qd`     : `(n_train,)` where `train_qd[i]` is `(T_i,6)`
* `train_qdd`    : `(n_train,)` where `train_qdd[i]` is `(T_i,6)`
* `train_tau`    : `(n_train,)` where `train_tau[i]` is `(T_i,6)`

**Test:**

* same keys prefixed with `test_...`

So the file is effectively:

```python
npz["train_q"][i]   -> ndarray (T_i, 6)
npz["train_tau"][i] -> ndarray (T_i, 6)
...
```

---