# To Do: Implement *trajectory duration* tracking in delan preprocess pipeline

## 1) Split settings

### ✅ Keep this split

* `TEST_FRACTIONS = [0.2]`
  I agree. With your max dataset you get **train=180, val=26, test=51 (257 total)**, which is a strong and realistic split. 
  Dropping `0.3` saves ~50% of the outer sweep budget immediately.

---

## 2) Trajectory budgets `TRAJ_AMOUNTS` (K)

You now have **180 training trajectories available** in the max regime. 
From the old sweep, the low-K regimes mainly served as “sample efficiency curves” but also caused instability/variance.

### My recommendation (balanced + informative)

* **If you want a clean learning-curve but not too expensive:**
  `TRAJ_AMOUNTS = [50, 100, 150, 180]`
* **If you want to still show the “very low data” failure mode once:**
  `TRAJ_AMOUNTS = [25, 50, 100, 150, 180]`
  (but expect 25 to be noisy / less interpretable; it’s mostly for the thesis story)

**Why include 180:** it’s your “best achievable” baseline under the same split, and it’s what you’ll likely cite as the final model regime.

---

## 3) DeLaN preset choice: `lutter_like` vs `lutter_like_256` (and looping 128/256)

Your strong filtered run uses **SoftPlus, batch 1024, lr 1e-4, wd 1e-5, depth 2, width 256**, and it performs well on the big dataset:

* **val RMSE ≈ 0.299**, **test RMSE ≈ 0.323** 

So:

### Recommendation

* For the sweep run: **fix DeLaN to `lutter_like_256`** (w=256, d=2).
* Don’t loop width 128 right now unless you specifically want a compute/capacity ablation.

**Why:** looping 128 and 256 doubles DeLaN cost, and your evidence already shows 256 is a strong/stable regime under the corrected pipeline. 

*(If you later want the ablation: do it only at K=180 and maybe K=100 — not across all K.)*

---

## 4) DeLaN seeds: increase folds since you removed `tf=0.3`

You’re right that DeLaN seed influences outcomes (your earlier boxplots showed that clearly), and now you freed budget by removing one test fraction.

### Recommendation

* Increase to **5 seeds**:
  `DELAN_SEEDS = [0, 1, 2, 3, 4]`

**Why 5 and not 7+:** it’s a good trade: noticeably better “fold coverage” without blowing up runtime, especially since you now train LSTM only on the best DeLaN.

---

## 5) DeLaN epochs (decoupled from K)

Your filtered reference runs that look good are trained with **400 epochs**.
Given you now select by **best val** across seeds, the main risk isn’t “overtraining one seed”, it’s “undertraining all seeds”.

### Recommendation

* Set **`DELAN_EPOCHS = 400`** (constant)

This aligns the sweep with the proven setting you already ran successfully on both datasets.

---

## 6) LSTM hyperparameters (based on your provided run)

Your provided LSTM run is:

* H=150, full features, units=128, dropout=0.2
* epochs=40, batch=64, val_split=0.1
* It reaches best epoch near the end (e.g., best at 37/40 on combined_26), and delivers a very strong test RMSE for the residual model.

### Recommendation: keep the proven regime as default

* `H_LIST = [100, 150]` ✅
* `FEATURE_MODES = ["full"]` ✅
* **Units:** keep `LSTM_UNITS = 128` ✅
* **Dropout:** keep `LSTM_DROPOUT = 0.2` (your proposed 0.1 is more likely to overfit)
* **Batch:** I would **not jump to 256** yet; keep `64` (or try `128` max).
  Your current best run uses 64 and behaves well. 
* **Val split:** keep `LSTM_VAL_SPLIT = 0.1` (you already have a separate trajectory-level val split in the dataset; taking 20% of the training windows again is usually unnecessary)
* **Max epochs:** setting `LSTM_EPOCHS = 120` is fine *because early stopping is default*, but it’s not needed.
  You can keep 120 for safety; early stopping will stop earlier anyway.

### One important interpretation note

In your evaluation JSON, `rg_rmse` equals `res_rmse` for the example runs.
If that’s intentional naming (i.e., `res_rmse` already refers to recombined torque RMSE), then everything is fine. If not, it’s worth confirming that “RG” truly means **DeLaN + predicted residual**, not “residual-only”.

---

## My concrete “sweep_1” settings summary

If you want a strong, efficient next run:

* `TEST_FRACTIONS = [0.2]`
* `TRAJ_AMOUNTS = [50, 100, 150, 180]` *(optionally add 25)*
* `DELAN_HP_PRESET = "lutter_like_256"`
* `DELAN_SEEDS = [0, 1, 2, 3, 4]`
* `DELAN_EPOCHS = 400`
* `H_LIST = [100, 150]`
* `FEATURE_MODES = ["full"]`
* LSTM: `units=128`, `dropout=0.2`, `batch=64 (or 128)`, `val_split=0.1`, `max_epochs=120` (early stopping default)

---
---
---
## Acceleration Filter

Given your **dt ≈ 0.020–0.024 s** (≈ **41–50 Hz** sampling), I’d set a **4th-order Butterworth low-pass** (SOS + `sosfiltfilt`) with a cutoff that’s safely below Nyquist but still preserves real robot motion.

### First compute the constraints

* For **40.9 Hz** sampling: Nyquist ≈ **20.4 Hz**
* For **49.7 Hz** sampling: Nyquist ≈ **24.8 Hz**

So your cutoff must be **< ~20 Hz** in the worst case.

---

## What I would set (practical starting point)

### Default choice (my recommendation)

* **cutoff = 8 Hz**, **order = 4**, **zero-phase** (`sosfiltfilt`)

Why 8 Hz:

* It’s well below Nyquist for both datasets.
* It preserves most joint motion content for typical UR trajectories while strongly attenuating differentiation noise.

### Also run one sensitivity check (so you can justify it in thesis)

Do two additional cutoffs:

* **5 Hz** (more smoothing, safer for acceleration stability)
* **12 Hz** (less smoothing, preserves sharper dynamics)

So you end up with a small, defensible bracket: **5 / 8 / 12 Hz**.

---

## Where to apply it (since you “need acceleration”)

If you are computing (\ddot{q}) from measured (\dot{q}):

Best practice for your case:

1. Filter **(\dot{q})** with the Butterworth (5/8/12 Hz).
2. Differentiate filtered (\dot{q}) → (\ddot{q}).
3. Optionally filter (\ddot{q}) once more with the same filter (often helps).

If you only filter (\ddot{q}) after differentiating raw (\dot{q}), you’ll usually keep more noise than necessary.

---

## Translate that into window lengths (sanity)

At 40–50 Hz:

* H=100 → 2.0–2.5 s
* H=150 → 3.0–3.7 s

These windows are long enough that a cutoff in the 5–12 Hz range won’t “erase” the temporal structure the LSTM needs.

---

## Final recommended setting to implement now

* **Butterworth order:** 4
* **Implementation:** SOS + `sosfiltfilt`
* **Cutoff:** **8 Hz** (and test 5 and 12 Hz as ablation)

If you tell me the *typical trajectory duration* and whether motions are slow/medium/aggressive, I can narrow it to one cutoff with more confidence (e.g., “8 Hz is best” vs “5 Hz is safer”).
