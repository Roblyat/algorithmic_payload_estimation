# “rg_rmse” vs "res_mse"

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
---
---

# std plots delan / lstm /eval

- fill with plot description of plot-setup 

---
---
---

# Plots delan_metrics_boxplots.py / lstm_metrics_boxplots.py / eval_metrics_boxplots.py

* **Stage 1 (DeLaN):** `shared/models/delan/*/metrics.json` (torque RMSE, per-joint, timing, hyper) 
* **Stage 2 (LSTM):** `shared/models/lstm/*/metrics_train_test_H*.json` (residual RMSE, per-joint, train/val history, args) 
* **Pipeline (Eval):** `shared/evaluation/*/metrics_*_H*.txt` (delan_rmse/res_rmse/rg_rmse + per-joint) 

## What you already plot (current `metrics_boxplots.py`) — and why it’s good

Your script currently does the “pipeline-level” essentials :

1. `rg_rmse` by `feature_mode` → headline ablation (final torque quality)
2. `res_rmse` by `feature_mode` → how hard residual learning is per setting
3. `delan_rmse` by `delan_id` → stage-1 variability seen through pipeline eval logs
4. per-joint `rg_joint_rmse_j` by `feature_mode` → which joints benefit/suffer
5. correlation scatters:

   * `delan_rmse` vs `rg_rmse`
   * `res_rmse` vs `rg_rmse`
   * per-joint `delan_joint_rmse_j` vs `rg_joint_rmse_j`

### One important observation (so you don’t over-plot)

In your eval metrics, **`rg_rmse == res_rmse` by construction** (since `tau_pred = tau_hat + r_hat` and `tau_gt - tau_pred = r_gt - r_hat`). You can literally see the equality in the saved metrics file. 
So plotting both is fine for readability, but they aren’t independent.

---

## What I would add once you include DeLaN + LSTM training metrics

### A) Stage-1 only (DeLaN) plots — “physics model quality & stability”

From `delan/*/metrics.json`, you have:

* `eval_test.torque_rmse` + per joint
* timing (`time_per_sample_s`, `hz`)
* `hyper` (seed, width/depth, lr, wd, diag params, etc.)
* dataset info (npz path / dt / dof etc. — you’re adding that)

**Plots I’d do first:**

1. **DeLaN torque RMSE boxplot** grouped by:

   * `model_type` (structured vs black) if present
   * **or** `delan_tag` (seed/epochs/model_short)
     Purpose: stage-1 performance distribution across many runs.

2. **DeLaN per-joint RMSE grid** (6 small panels) grouped by `delan_tag`
   Purpose: identify which joints are consistently hard for DeLaN (often joint2/shoulder).

3. **DeLaN speed vs accuracy scatter**

   * x=`time_per_sample_s` (or hz), y=`torque_rmse`
     Purpose: sanity check if some configs are slower without gains.

4. (Optional but very useful) **Hyperparameter sensitivity**:

   * boxplot `torque_rmse` grouped by `seed` (stability)
   * scatter `epochs_ran` or configured `max_epoch` vs `torque_rmse`
   * scatter `n_width`/`n_depth` vs `torque_rmse`
     Purpose: quickly see what matters in stage-1.

---

### B) Stage-2 only (LSTM) plots — “residual learning quality & generalization”

From `metrics_train_test_H*.json`, you have:

* `eval_test.rmse_total` + per joint (this is residual RMSE in physical units) 
* training summary: `best_val_loss`, `final_train_loss`, `final_val_loss`, `best_epoch` 
* args: `units`, `dropout`, `seed`, etc. 
* and you can parse `feature_mode`, `H` from folder name (like you already do for evaluation).

**Plots I’d do first:**

1. **Residual RMSE boxplot** grouped by:

   * `feature_mode` (core ablation for stage-2)
   * and optionally faceted by `H` (or vice versa)

2. **Per-joint residual RMSE grid** grouped by `feature_mode`
   Purpose: see where LSTM helps vs struggles.

3. **Generalization scatter**:

   * x=`best_val_loss` (or final_val_loss), y=`eval_test.rmse_total`
     Purpose: checks whether training objective correlates with real-unit residual accuracy.

4. **Overfit indicator plot**:

   * y = `(final_val_loss / final_train_loss)` (or difference) grouped by `feature_mode`
     Purpose: find feature modes that overfit more.

---

### C) Pipeline plots (Eval) — expand beyond what you already have

Keep your existing plots. Then add *interpretability* plots that make decisions easier:

1. **“Improvement over DeLaN” plot**

   * `gain = delan_rmse - rg_rmse` (absolute)
   * or `gain_ratio = rg_rmse / delan_rmse` (relative)
     Group by `feature_mode`.
     Purpose: shows whether stage-2 actually helps and by how much.

2. **Per-joint improvement grid**

   * `delan_joint_rmse_j - rg_joint_rmse_j` per joint, grouped by feature_mode
     Purpose: shows which joints benefit from residual learning.

3. **Pipeline correlation that’s actually informative**
   Since `rg_rmse == res_rmse`, the scatter `res_rmse vs rg_rmse` will be almost a diagonal line. 
   Better alternatives:

   * `delan_rmse` vs `gain` (does bad DeLaN create more room for LSTM to help?)
   * `delan_rmse` vs `gain_ratio`
   * `delan_joint_rmse_j` vs `joint_gain_j`

* **`box_gain_by_feature_mode`**: which feature mode actually *helps DeLaN most* (absolute torque RMSE improvement)
* **`box_gain_ratio_by_feature_mode`**: which mode gives best *relative* improvement (robust across datasets/joints)
* **`joint_gain_*` plots**: which joints benefit (often reveals friction / gearbox / shoulder effects)
* **`delan_rmse vs gain`**: whether the LSTM is “repairing a weak physics model” or improving even strong DeLaN runs
* **`delan_joint_rmse vs joint_gain`**: joint-wise version of that story

---

# What you’ll get (summary of plots)

### DeLaN (`shared/models/delan/_plots/`)

* torque RMSE by model type (struct vs black)
* torque RMSE by delan_tag (seed/epochs)
* per-joint RMSE grid by delan_tag
* speed vs accuracy scatter
* RMSE by seed (stability)

### LSTM (`shared/models/lstm/_plots/`)

* residual RMSE by feature_mode (ablation headline)
* residual RMSE by H
* per-joint residual RMSE grid by feature_mode
* best_val_loss vs real-unit RMSE scatter
* overfit ratio (val/train) by feature_mode

### Pipeline Eval (`shared/evaluation/_plots/`)

* rg_rmse, res_rmse by feature_mode
* delan_rmse by delan_tag
* per-joint rg_rmse by feature_mode
* **new:** gain_abs and gain_ratio by feature_mode
* **new:** per-joint gain grid
* **new:** correlation scatters (delan_rmse vs rg_rmse / gain_abs / gain_ratio)





---
---
---

## 4) What I would do in boxplots now that backend exists

You’re right: once backend exists, you want:

* plots for **jax only**
* plots for **torch only**
* plots for **jax vs torch**

### Minimal change to your plotting scripts

In **each** of:

* `delan_metrics_boxplots.py`
* `lstm_metrics_boxplots.py`
* `eval_metrics_boxplots.py`

Add an optional arg:

```py
ap.add_argument("--backend", choices=["all", "jax", "torch"], default="all")
```

Then, while parsing rows, create a column:

* `backend = "jax"` or `"torch"`
  (parse from folder name: you now have `delan_jax_...` / `delan_torch_...`)

Finally:

```py
if args.backend != "all":
    df = df[df["backend"] == args.backend]
```

### The “extra” comparison plots I would add (very useful)

In **eval_metrics_boxplots** (pipeline):

1. `gain = delan_rmse - rg_rmse` boxplot grouped by:

   * x = `feature_mode`
   * hue = `backend`  ✅ (torch vs jax side-by-side)

2. `gain_ratio = rg_rmse / delan_rmse` boxplot:

   * x = `feature_mode`
   * hue = `backend`

3. Scatter:

   * x = `delan_rmse`
   * y = `gain_ratio`
   * color = `backend`

In **delan_metrics_boxplots** (stage 1 only):

* torque_rmse boxplot:

  * x = `model_short` (struct/black)
  * hue = `backend`

In **lstm_metrics_boxplots** (stage 2 only):

* residual rmse boxplot:

  * x = `feature_mode`
  * hue = `backend`
  * (optional facet by H)

This gives you “jax vs torch” immediately, without needing new UI logic beyond one dropdown.

---

## 5) UI buttons in evaluation section for delan/lstm plots (same pattern as eval)

You asked for the same pattern; this is exactly what I’d drop into `evaluation.py` next to your existing “Metrics plots” button:

```py
with e_col3:
    c1, c2, c3 = st.columns(3)

    with c1:
        pad_button()
        if st.button("Eval plots", use_container_width=True):
            run(
                f"{cfg.COMPOSE} exec -T evaluation bash -lc "
                f"\"python3 scripts/eval_metrics_boxplots.py "
                f"--eval_root {paths.evaluation} "
                f"--out_dir {metrics_out} "
                f"\""
            )

    with c2:
        pad_button()
        if st.button("DeLaN plots", use_container_width=True):
            run(
                f"{cfg.COMPOSE} exec -T evaluation bash -lc "
                f"\"python3 scripts/delan_metrics_boxplots.py "
                f"--delan_root {paths.models_delan} "
                f"--out_dir {paths.models_delan}/_plots "
                f"\""
            )

    with c3:
        pad_button()
        if st.button("LSTM plots", use_container_width=True):
            run(
                f"{cfg.COMPOSE} exec -T evaluation bash -lc "
                f"\"python3 scripts/lstm_metrics_boxplots.py "
                f"--lstm_root {paths.models_lstm} "
                f"--out_dir {paths.models_lstm}/_plots "
                f"\""
            )
```

(Then later you can add a `backend` selectbox in the evaluation section and pass `--backend {backend}` into each script.)

---

## Summary of what you should change right now

* **delan.py**

  * add backend selector
  * include backend in `delan_tag` and `delan_id`
  * use correct service + script per backend
  * residual name inherits backend via `delan_tag`

* **lstm.py**

  * include backend in windows filename default
  * rely on `st.session_state["residual_npz"]` as the correct folder-style file path

* **evaluation.py**

  * just use session_state’s residual path; eval output folder already inherits backend if lstm_dir_name does

* **boxplots**

  * add `--backend` filter + “hue=backend” comparisons

If you paste your current `delan.py` naming block (the few lines around where `delan_tag`, `delan_id`, `default_residual_name` are created), I can rewrite that exact block in-place with the backend changes (no refactor, just the affected lines).
