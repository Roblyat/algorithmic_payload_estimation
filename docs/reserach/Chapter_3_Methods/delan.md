## DeLaN training

### 1) Feature transform on positions

The JAX DeLaN model uses a position feature map internally:

* input features include **`cos(q)` and `sin(q)`**, which is helpful for angle wrap-around and keeps features bounded.

### 2) Normalize the loss per joint

They don’t “standard-scale the dataset”; instead they normalize the **loss** per joint using per-joint variance terms:

* `norm_tau = var(train_tau, axis=0)`
* `norm_qdd = var(train_qdd, axis=0)`

and the inverse-dynamics loss is weighted/divided by these, so joints with larger magnitudes don’t dominate training.

### 3) ReplayMemory + random minibatches

Training uses a replay buffer (`ReplayMemory`) which stores the flattened sample arrays and provides minibatches by shuffling indices (internally using a random permutation). This means:

* you train on i.i.d. random minibatches sampled from the pool of `(q, qd, qdd, tau)` samples
* trajectory order isn’t used during optimization (only for splitting/evaluation and potential plotting)

### 4) Constant sampling time matters

A **constant `dt`** is important because:

* many physical preprocessing steps (finite differences, filtering, consistent derivative interpretation) assume uniform sampling
* the repo’s toy dataset asserts constant dt
* even though your first version uses measured velocity/acceleration, once you compute/clean derivatives or integrate models, uniform sampling becomes critical

So later you’ll add:

* resampling to a fixed-rate time grid if needed
* filtering for `qd` and `qdd` (and possibly recompute `qdd` from filtered `qd`)

---

If you want, next step we can add a matching short subsection for “Stage-2 dataset for the LSTM” that defines how you build the sequences $\mathbf{x}_k$ from these flattened arrays.