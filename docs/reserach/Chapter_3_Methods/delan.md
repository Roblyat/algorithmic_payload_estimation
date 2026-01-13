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

### DeLaN example train results:
UR5 Dataset:
  npz = /workspace/shared/data/processed/delan_ur5_dataset.npz
   dt ≈ 0.02439441835857372
  dof = 6
  Train trajectories = 164
  Test trajectories  = 41
  Train samples = 12251
  Test samples  = 3075

![Results DeLaN](/docs/reserach/illustrations/UR5_DeLaN_Torque___Seed=4.png)

---

## 🔧 Fix 1: matplotlib backend selection (Qt5Agg) should depend on `render`

Right now you do `mp.use("Qt5Agg")` unconditionally (inside try), so even when running headless, you’re *still trying* to use Qt. It might work on your machine because you mounted X11, but it’s fragile and will break on servers.

### Better pattern (drop-in)

Change your matplotlib init to:

```python
import matplotlib as mp
plt = None

def _setup_matplotlib(render: bool):
    global plt
    try:
        if render:
            mp.use("Qt5Agg")
        else:
            mp.use("Agg")
        mp.rc('text', usetex=False)
        import matplotlib.pyplot as plt_local
        plt = plt_local
    except Exception:
        plt = None
```

Then **after** you parse args / init env:

```python
_setup_matplotlib(bool(render))
```

Now:

* render=1 → interactive window possible
* render=0 → always headless-safe (still saves PNGs)

---

## 🔧 Fix 2: set `XLA_PYTHON_CLIENT_MEM_FRACTION` *before importing JAX*

You currently set:

```py
import jax
...
os.environ['XLA_PYTHON_CLIENT_MEM_FRACTION'] = '0.4'
```

That’s too late for it to reliably take effect.

### Minimal fix

Move:

```python
import os
os.environ['XLA_PYTHON_CLIENT_MEM_FRACTION'] = '0.4'
```

to the **very top of the file**, before `import jax`.

---
more extremely useful plots:

* **per-joint RMSE bar chart** (DeLaN-only, test split)
* optionally also **per-joint normalized RMSE** (using your `norm_tau`)