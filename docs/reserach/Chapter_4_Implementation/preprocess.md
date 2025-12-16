## Preprocessing pipeline

### Input dataset (your measured robot logs)

You record a **long-format CSV** with columns:

* `Time`
* `Joint Name`
* `Position`
* `Velocity`
* `Acceleration`
* `Effort` (treated as **τ** for now)

This is **fully measured per joint** (pos/vel/accel/effort). In this first iteration:

* **no filtering** is applied to velocity/acceleration yet
* no torque constant conversion yet (effort ≙ τ)

### 2a — select the 6 UR5 joints

We filter the CSV to only:

* `ur5_shoulder_pan_joint`
* `ur5_shoulder_lift_joint`
* `ur5_elbow_joint`
* `ur5_wrist_1_joint`
* `ur5_wrist_2_joint`
* `ur5_wrist_3_joint`

This defines `n_dof = 6`.

### 2b — pack the log into trajectories

Because your log is continuous and does not contain obvious time gaps, we segment it into trajectories using **fixed-length chunks in “frames”**:

* A **frame** = one unique timestamp across all joints
* We assign a `trajectory_id = frame_index // frames_per_trajectory`

This yields many trajectories (you got `train=164 / test=41`).

### 2c — pivot to wide arrays (T, n_dof)

For each trajectory, we convert long → wide using pivoting:

* index = `Time`
* columns = ordered joint list (the 6 UR5 joints)
* values = one of `Position`, `Velocity`, `Acceleration`, `Effort`

So each trajectory becomes numeric arrays:

* `t`: `(T,)`
* `q`: `(T, 6)` from Position
* `qd`: `(T, 6)` from Velocity
* `qdd`: `(T, 6)` from Acceleration
* `tau`: `(T, 6)` from Effort -> T = k_t*i

### 2d — train/test split by trajectory

We split **by trajectory**, not by rows:

* randomly shuffle trajectory IDs
* assign a fraction (e.g. 20%) of trajectories to test
* rest to train

This avoids leakage of near-identical neighboring samples across splits.

### 2e — save a dataset that training can load

We write `delan_ur5_dataset.npz` containing **trajectory lists** (variable length):

* `train_labels`, `train_t`, `train_q`, `train_qd`, `train_qdd`, `train_tau`
* `test_labels`,  `test_t`,  `test_q`,  `test_qd`,  `test_qdd`,  `test_tau`

Each `*_q` etc. is stored as an **object array** of shape `(n_traj,)`, where each element is an `(T_i, 6)` numpy array.

On the DeLaN side, we load the NPZ and **flatten** by stacking trajectories:

* `train_q = vstack(train_q_list)` → `(N_train, 6)`
* same for `qd, qdd, tau`
* and we force numeric dtype (`float32`) to avoid `dtype=object` issues with JAX.

---