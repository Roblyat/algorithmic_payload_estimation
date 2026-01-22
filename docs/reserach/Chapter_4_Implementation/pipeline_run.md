okay, so now we are set up for a test script i want to do.
we want to perform a script that trains from preprocess -> delan train -> export delan residuals -> build lstm windows -> lstm train -> combined evaluation.

i currently call this by buttons in the ui, i give you a bit more extended insight then just the buttons, so that we can perform a script. 

INFO:

what is hardly set:
- we only train structured models
- we only train delan_jax
- we have derive acceleration = true



how the naming and folders are computed:
base_id = f"{dataset_name}__{run_tag}"

default_delan_npz_name = f"delan_{dataset_name}_dataset.npz"

    npz_in = f"{paths.preprocessed}/{delan_npz_name}"

    delan_tag = f"delan_{delan_backend}_{model_short}_s{seed}_ep{delan_epochs_eff}"

    delan_id = f"{dataset_name}__{run_tag}__{delan_tag}"

    delan_run_dir = f"{paths.models_delan}/{delan_id}"

    default_ckpt_name = f"{delan_id}.{ckpt_ext}"
    
    default_residual_name = f"{base_id}__residual__{delan_tag}.npz"

    res_out = f"{paths.processed}/{residual_name}"

    default_windows_npz_name = f"{base_id}__lstm_windows_H{H}__feat_{feature_mode}__delan_{delan_backend}.npz"

    win_out = f"{paths.processed}/{windows_npz_name}"

    f"{base_id}__{delan_tag}__feat_{feature_mode}__lstm_s{seed_name}_H{H}_ep{epochs_name}_b{batch_name}_u{units_name}_do{do_tag}"
    
    lstm_out = f"{paths.models_lstm}/{lstm_dir_name}"

1:
"Preprocess DeLaN"
        f"{cfg.COMPOSE} exec -T preprocess bash -lc "
        f"\"python3 scripts/build_delan_dataset.py "
        f"--qdd {derive_qdd} "
        f"--col_format {col_format} "
        f"--raw_csv {raw_data_path}/{dataset_name}.{in_format} "
        f"--out_npz {paths.preprocessed}/delan_{dataset_name}_dataset.npz "
        f"\""

2. DeLaN Train
    run(
        f"{cfg.COMPOSE} exec -T {delan_service} bash -lc "
        f"\"python3 {train_script} "
        f"--npz {npz_in} "
        f"-t {model_type} "
        f"-s {seed} "
        f"-r {render_flag} "
        f"--hp_preset {delan_preset} "
        f"{hp_flags} "
        f"--save_path {ckpt}"
        f"\""
    )

    insight:
    delan hyperparameter:
        "n_width": "n_width",
        "n_depth": "n_depth",
        "n_minibatch": "batch",
        "learning_rate": "lr",
        "weight_decay": "wd",
        "max_epoch": "epochs",
        "diagonal_epsilon": "diag_eps",
        "diagonal_shift": "diag_shift",
        "activation": "activation",

    acceptable parameters:
        v_n_width = st.number_input("n_width", 1, 4096, key="val_n_width")
        v_n_depth = st.number_input("n_depth", 1, 16, key="val_n_depth")
        v_batch = st.number_input("n_minibatch", 1, 65536, key="val_n_minibatch")

        v_diag_eps = st.number_input("diagonal_epsilon", format="%.6f", key="val_diagonal_epsilon")
        v_diag_shift = st.number_input("diagonal_shift", format="%.6f", key="val_diagonal_shift")

        v_act = st.selectbox("activation", ["tanh","relu","softplus","gelu","swish"], key="val_activation")

        v_lr = st.number_input("learning_rate", format="%.8f", key="val_learning_rate")
        v_wd = st.number_input("weight_decay", format="%.8f", key="val_weight_decay")
        v_ep = st.number_input("max_epoch", 1, 200000, key="val_max_epoch")

3. Export DeLaN Residual npz
        run(
            f"{cfg.COMPOSE} exec -T {delan_service} bash -lc "
            f"\"python3 {export_script} "
            f"--npz_in {npz_in} "
            f"--ckpt {ckpt} "
            f"--out {res_out}"
            f"\""
        )

4. Build LSTM Windows
        run(
            f"{cfg.COMPOSE} exec -T preprocess bash -lc "
            f"\"python3 scripts/build_lstm_windows.py "
            f"--in_npz {res_out} "
            f"--out_npz {win_out} "
            f"--H {H} "
            f"--features {feature_mode}"
            f"\""
        )

    insight:
    acceptable feature modes:
    FEATURE_MODES: tuple[str, ...] = ("full", "tau_hat", "state", "state_tauhat")

5. Train LSTM
    run(
        f"{cfg.COMPOSE} exec -T lstm bash -lc "
        f"\"python3 scripts/train_residual_lstm.py "
        f"--npz {win_out} "
        f"--out_dir {lstm_out} "
        f"--model_name {lstm_model_name} "
        f"--epochs {epochs} "
        f"--batch {batch} "
        f"--val_split {val_split} "
        f"--seed {lstm_seed} "
        f"--units {units} "
        f"--dropout {dropout} "
        f"--eps {eps}"
        f"{no_plots_flag}"
        f"\""
    )


6. Combined Pipeline evaluation
    run(
        f"{cfg.COMPOSE} exec -T evaluation bash -lc "
        f"\"python3 scripts/combined_evaluation.py "
        f"--residual_npz {res_out} "
        f"--model {lstm_model_path} "
        f"--scalers {lstm_scalers_path} "
        f"--out_dir {eval_out} "
        f"--H {H} "
        f"--split {split} "
        f"--features {feature_mode} "
        f"--save_pred_npz"
        f"\""
    )


______

so as you can see, we can perform a script that runs a loop with the above compose run cmds.
what the loop need to do:

1. have a library of setups we want to run for the datasets, exactly use these numbers taj=[25,50,75,100,150] and test_fraction = [0.2,0.3]

2. a loop that trains:
- for i in taj=[25,50,75,100,150]
     for j in test_fraction = [0.2,0.3]
        preprocess
        for "3 runs with different 3 different seeds"
              train delan
              export residuals
                  for H = 2 intersting H you choose are best
                      for k in feature mode ("full", "tau_hat", "state", "state_tauhat")
                           build lstm windows
                           train lstm
                           combined evaluation


3. collects the full log and saves it and adds clever:

print(####################)
or
print(######################)
print(######################)

or
print(######################)
print(######################)
print(######################)

to split the loop clever in the log.

just take this as an information first, you can ask queastions at first before we perform the script. what else do you need to know?

Here are the **exact, minimal line changes** to implement your two adjustments in the sweep script I gave.

---

You already have everything needed to automate this — we just need to be explicit about **which parts define a “run”** and make sure filenames don’t overwrite each other.

### One important adjustment to your loop

You said you want “random train/test variations sampling trajectories out of the huge dataset”, and also “3 runs with different seeds”.

That only happens if **preprocess is seeded per run** (because the subsampling + split happens in preprocess now). So the clean version is:

* For each `(traj_amount, test_fraction, seed)` → **run preprocess** (produces a specific subset/split NPZ)
* Then use that NPZ for DeLaN/LSTM/eval with the same seed.

So I’ll place **preprocess inside the seed loop** and include `--trajectory_amount`, `--test_fraction`, `--seed`.

---

## Two “interesting” H values

Without overthinking it, the two most useful to compare are:

* `H=25` (short context)
* `H=50` (your current default / medium context)

They typically show whether the residual dynamics are mostly short-memory or need longer windows.

---

## What else I need to know (only if you want the script to be 1:1 with your repo paths)

I can write the script fully with sensible defaults. If any of these differ in your setup, change the constants at the top:

* compose command (you already have it in cfg; in a terminal script we’ll hardcode the same string)
* service names: `preprocess`, `delan_jax`, `lstm`, `evaluation`
* script paths inside containers:

  * preprocess: `scripts/build_delan_dataset.py`, `scripts/build_lstm_windows.py`
  * delan_jax: path to `rbyt_train_delan_jax.py` and export script path
  * evaluation: `scripts/combined_evaluation.py`

If your container script locations differ, tell me and I’ll adapt — otherwise use the defaults below.

---

# Script: `scripts/run_full_sweep.py`

Run this from the **host** (where `docker compose` works). It writes one big logfile and also per-run logs.

```python
#!/usr/bin/env python3
import os
import shlex
import subprocess
from datetime import datetime
from pathlib import Path

# ----------------------------
# USER SETTINGS (edit if needed)
# ----------------------------

COMPOSE = (
    "docker compose -p payload_estimation "
    "--project-directory /workspace "
    "--env-file /workspace/.env "
    "-f /workspace/docker-compose.yml"
)

# Services
SVC_PREPROCESS = "preprocess"
SVC_DELAN = "delan_jax"   # fixed: only delan_jax
SVC_LSTM = "lstm"
SVC_EVAL = "evaluation"

# Base paths inside shared volume (container paths)
RAW_DIR = "/workspace/shared/data/raw"
PREPROCESSED_DIR = "/workspace/shared/data/preprocessed"
PROCESSED_DIR = "/workspace/shared/data/processed"
MODELS_DELAN_DIR = "/workspace/shared/models/delan"
MODELS_LSTM_DIR = "/workspace/shared/models/lstm"
EVAL_DIR = "/workspace/shared/evaluation"

# Dataset settings (your scenario)
DATASET_NAME = "delan_UR3_Load0_dataset_26"  # raw file: RAW_DIR/{DATASET_NAME}.csv
RUN_TAG = "A"
IN_FORMAT = "csv"
COL_FORMAT = "wide"
DERIVE_QDD = True

# Sweep
TRAJ_AMOUNTS = [25, 50, 75, 100, 150]
TEST_FRACTIONS = [0.2, 0.3]
SEEDS = [0, 1, 2]  # 3 runs

H_LIST = [25, 50]
FEATURE_MODES = ["full", "tau_hat", "state", "state_tauhat"]

# Fixed choices you stated
DELAN_MODEL_TYPE = "structured"  # only structured
DELAN_HP_PRESET = "fast_debug"   # change if you want e.g. "paper" / etc.

# If you want to pass manual delan flags (optional), put them here:
DELAN_HP_FLAGS = ""  # e.g. "--n_width 64 --n_depth 2 ..."

# LSTM hyperparams (choose defaults; adjust later if you want)
LSTM_EPOCHS = 50
LSTM_BATCH = 256
LSTM_VAL_SPLIT = 0.2
LSTM_UNITS = 128
LSTM_DROPOUT = 0.1
LSTM_EPS = 1e-8
LSTM_NO_PLOTS = True

# Paths to scripts INSIDE the containers
# preprocess container:
SCRIPT_BUILD_DELAN_DATASET = "scripts/build_delan_dataset.py"
SCRIPT_BUILD_LSTM_WINDOWS = "scripts/build_lstm_windows.py"

# delan_jax container:
SCRIPT_TRAIN_DELAN_JAX = "/workspace/delan_jax/scripts/rbyt_train_delan_jax.py"
SCRIPT_EXPORT_DELAN_RES = "/workspace/delan_jax/scripts/export_delan_residuals_jax.py"

# lstm container:
SCRIPT_TRAIN_LSTM = "scripts/train_residual_lstm.py"

# evaluation container:
SCRIPT_EVAL = "scripts/combined_evaluation.py"


# ----------------------------
# Helper functions
# ----------------------------

def banner(lines, char="#"):
    width = max(len(s) for s in lines) if lines else 0
    bar = char * (width + 8)
    out = [bar]
    for s in lines:
        out.append(f"{char*3} {s.ljust(width)} {char*3}")
    out.append(bar)
    return "\n".join(out)

def run_cmd(cmd, log_file, also_print=True):
    if also_print:
        print(cmd)
    log_file.write("\n" + banner([cmd], char="=") + "\n")
    log_file.flush()

    p = subprocess.run(cmd, shell=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    log_file.write(p.stdout + "\n")
    log_file.flush()
    if also_print:
        print(p.stdout)

    if p.returncode != 0:
        raise RuntimeError(f"Command failed with exit code {p.returncode}")

def compose_exec(service, inner_cmd):
    # -T to avoid TTY issues in CI-like runs
    return f"{COMPOSE} exec -T {service} bash -lc {shlex.quote(inner_cmd)}"

def safe_tag(x):
    return str(x).replace(".", "p")

def main():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    logs_dir = Path("logs_sweeps")
    logs_dir.mkdir(parents=True, exist_ok=True)
    master_log_path = logs_dir / f"sweep_{DATASET_NAME}_{RUN_TAG}_{ts}.log"

    raw_csv = f"{RAW_DIR}/{DATASET_NAME}.{IN_FORMAT}"

    with master_log_path.open("w", encoding="utf-8") as master_log:
        master_log.write(banner([
            f"Sweep started: {ts}",
            f"dataset={DATASET_NAME} run_tag={RUN_TAG}",
            f"raw_csv={raw_csv}",
            f"col_format={COL_FORMAT} derive_qdd={DERIVE_QDD}",
            f"traj_amounts={TRAJ_AMOUNTS} test_fracs={TEST_FRACTIONS} seeds={SEEDS}",
            f"H={H_LIST} feature_modes={FEATURE_MODES}",
            f"DeLaN: backend=jax type={DELAN_MODEL_TYPE} hp_preset={DELAN_HP_PRESET}",
        ], char="#") + "\n")

        for K in TRAJ_AMOUNTS:
            for tf in TEST_FRACTIONS:
                master_log.write("\n" + banner([
                    f"SUBSWEEP: K={K} test_fraction={tf}"
                ], char="#") + "\n")
                master_log.flush()

                for seed in SEEDS:
                    # ---------- Naming ----------
                    base_id = f"{DATASET_NAME}__{RUN_TAG}"

                    # preprocess output NPZ (unique per K/tf/seed)
                    delan_npz_name = f"delan_{DATASET_NAME}_K{K}_tf{safe_tag(tf)}_seed{seed}_dataset.npz"
                    npz_in = f"{PREPROCESSED_DIR}/{delan_npz_name}"

                    # delan tag/id
                    model_short = "struct"
                    delan_epochs_eff = "ep?"  # purely for name; trainer controls epochs via hp_preset/flags
                    delan_tag = f"delan_jax_{model_short}_s{seed}_{delan_epochs_eff}"
                    delan_id = f"{DATASET_NAME}__{RUN_TAG}__{delan_tag}"
                    delan_run_dir = f"{MODELS_DELAN_DIR}/{delan_id}"
                    ckpt = f"{delan_run_dir}/{delan_id}.jax"

                    # residual output (unique per K/tf/seed)
                    residual_name = f"{base_id}__K{K}_tf{safe_tag(tf)}__residual__{delan_tag}.npz"
                    res_out = f"{PROCESSED_DIR}/{residual_name}"

                    # per-run log
                    run_log_path = logs_dir / f"run_K{K}_tf{safe_tag(tf)}_seed{seed}_{ts}.log"
                    with run_log_path.open("w", encoding="utf-8") as run_log:
                        run_log.write(banner([
                            f"RUN: K={K} test_fraction={tf} seed={seed}",
                            f"npz_in={npz_in}",
                            f"ckpt={ckpt}",
                            f"res_out={res_out}",
                        ], char="#") + "\n")

                        # ---------- 1) Preprocess ----------
                        run_log.write("\n" + banner(["1) PREPROCESS"], char="#") + "\n")
                        cmd = compose_exec(
                            SVC_PREPROCESS,
                            f"python3 {SCRIPT_BUILD_DELAN_DATASET} "
                            f"--qdd {str(DERIVE_QDD)} "
                            f"--col_format {COL_FORMAT} "
                            f"--trajectory_amount {K} "
                            f"--test_fraction {tf} "
                            f"--seed {seed} "
                            f"--raw_csv {raw_csv} "
                            f"--out_npz {npz_in}"
                        )
                        run_cmd(cmd, run_log)

                        # ---------- 2) DeLaN Train ----------
                        run_log.write("\n" + banner(["2) DELAN TRAIN"], char="#") + "\n")
                        hp_flags = (DELAN_HP_FLAGS + " ") if DELAN_HP_FLAGS.strip() else ""
                        cmd = compose_exec(
                            SVC_DELAN,
                            f"python3 {SCRIPT_TRAIN_DELAN_JAX} "
                            f"--npz {npz_in} "
                            f"-t {DELAN_MODEL_TYPE} "
                            f"-s {seed} "
                            f"-r 0 "
                            f"--hp_preset {DELAN_HP_PRESET} "
                            f"{hp_flags}"
                            f"--save_path {ckpt}"
                        )
                        run_cmd(cmd, run_log)

                        # ---------- 3) Export residuals ----------
                        run_log.write("\n" + banner(["3) EXPORT RESIDUALS"], char="#") + "\n")
                        cmd = compose_exec(
                            SVC_DELAN,
                            f"python3 {SCRIPT_EXPORT_DELAN_RES} "
                            f"--npz_in {npz_in} "
                            f"--ckpt {ckpt} "
                            f"--out {res_out}"
                        )
                        run_cmd(cmd, run_log)

                        # ---------- 4-6) LSTM windows -> train -> eval ----------
                        for H in H_LIST:
                            for feat in FEATURE_MODES:
                                run_log.write("\n" + banner([
                                    f"BLOCK: H={H} feat={feat}"
                                ], char="#") + "\n")

                                windows_npz_name = (
                                    f"{base_id}__K{K}_tf{safe_tag(tf)}__lstm_windows_H{H}"
                                    f"__feat_{feat}__delan_jax_seed{seed}.npz"
                                )
                                win_out = f"{PROCESSED_DIR}/{windows_npz_name}"

                                # LSTM output dir naming (keep your style, but make it unique)
                                lstm_dir_name = (
                                    f"{base_id}__K{K}_tf{safe_tag(tf)}__{delan_tag}"
                                    f"__feat_{feat}__lstm_s{seed}_H{H}"
                                    f"_ep{LSTM_EPOCHS}_b{LSTM_BATCH}_u{LSTM_UNITS}_do{safe_tag(LSTM_DROPOUT)}"
                                )
                                lstm_out = f"{MODELS_LSTM_DIR}/{lstm_dir_name}"
                                lstm_model_name = "residual_lstm"
                                lstm_model_path = f"{lstm_out}/{lstm_model_name}.pt"
                                lstm_scalers_path = f"{lstm_out}/scalers.npz"

                                eval_out = f"{EVAL_DIR}/{lstm_dir_name}"

                                # 4) Build windows
                                run_log.write("\n" + banner(["4) BUILD LSTM WINDOWS"], char="#") + "\n")
                                cmd = compose_exec(
                                    SVC_PREPROCESS,
                                    f"python3 {SCRIPT_BUILD_LSTM_WINDOWS} "
                                    f"--in_npz {res_out} "
                                    f"--out_npz {win_out} "
                                    f"--H {H} "
                                    f"--features {feat}"
                                )
                                run_cmd(cmd, run_log)

                                # 5) Train LSTM
                                run_log.write("\n" + banner(["5) TRAIN LSTM"], char="#") + "\n")
                                no_plots_flag = "--no_plots" if LSTM_NO_PLOTS else ""
                                cmd = compose_exec(
                                    SVC_LSTM,
                                    f"python3 {SCRIPT_TRAIN_LSTM} "
                                    f"--npz {win_out} "
                                    f"--out_dir {lstm_out} "
                                    f"--model_name {lstm_model_name} "
                                    f"--epochs {LSTM_EPOCHS} "
                                    f"--batch {LSTM_BATCH} "
                                    f"--val_split {LSTM_VAL_SPLIT} "
                                    f"--seed {seed} "
                                    f"--units {LSTM_UNITS} "
                                    f"--dropout {LSTM_DROPOUT} "
                                    f"--eps {LSTM_EPS} "
                                    f"{no_plots_flag}"
                                )
                                run_cmd(cmd, run_log)

                                # 6) Combined evaluation
                                run_log.write("\n" + banner(["6) COMBINED EVALUATION"], char="#") + "\n")
                                cmd = compose_exec(
                                    SVC_EVAL,
                                    f"python3 {SCRIPT_EVAL} "
                                    f"--residual_npz {res_out} "
                                    f"--model {lstm_model_path} "
                                    f"--scalers {lstm_scalers_path} "
                                    f"--out_dir {eval_out} "
                                    f"--H {H} "
                                    f"--split test "
                                    f"--features {feat} "
                                    f"--save_pred_npz"
                                )
                                run_cmd(cmd, run_log)

                    # append run log path to master
                    master_log.write(f"\n[OK] K={K} tf={tf} seed={seed}  log={run_log_path}\n")
                    master_log.flush()

        master_log.write("\n" + banner(["Sweep finished OK"], char="#") + "\n")

    print(f"\nMASTER LOG: {master_log_path}")
    print("Done.")


if __name__ == "__main__":
    main()
```

### How to run

From the host repo root (where your compose command works):

```bash
python3 scripts/run_full_sweep.py
```

---

## Notes / small things you may want to tweak

* `delan_epochs_eff = "ep?"` is just for naming. If you want it accurate, set it from your preset or pass `--epochs` and use that number.
* The script assumes your LSTM trainer produces:

  * `{out_dir}/{model_name}.pt`
  * `{out_dir}/scalers.npz`
    If your filenames differ, tell me what they are and I’ll adjust those two paths.
* If you prefer **preprocess once per (K,tf)** and then 3 model seeds on the same split, move preprocess outside the seed loop. But then you **won’t** get random split variations (only model initialization variation).

---

okay, two adjustments. 1. if it makes sense to train less eppochs on smaller dataset, and more eppochs on lager dataset, then please get that in. 2. the seeds, here are 3 seed, one how the dataset is shuffled in preprocess, the delan seed the seed determines the character assignment between train and test set, which is comparable to a k-fold evaluation, and the lstm seed. but for this first run i assume, data preprocess like we do is good. most interesting to see for delan are different seeds for the first run, most interesting on lstm are window lenght H and feature modes. please give affected lines to adjust this.

---

Here are the **exact, minimal line changes** to implement your two adjustments in the sweep script I gave.

---

## 1) Scale epochs with dataset size (DeLaN + LSTM)

Add these helper functions **once near the top** (after constants):

```python
def delan_epochs_for(K: int) -> int:
    # conservative scaling: small K trains faster, big K trains longer
    if K <= 25:  return 150
    if K <= 50:  return 200
    if K <= 75:  return 250
    if K <= 100: return 300
    return 400  # 150+

def lstm_epochs_for(K: int) -> int:
    if K <= 25:  return 30
    if K <= 50:  return 50
    if K <= 75:  return 60
    if K <= 100: return 80
    return 100
```

Then replace the fixed `LSTM_EPOCHS = 50` usage inside the loop with a per-K value.

### Affected lines inside the loop (where you build delan/lstm names + commands)

**Right after you compute `K` (inside `for K in TRAJ_AMOUNTS:`), add:**

```python
delan_epochs = delan_epochs_for(K)
lstm_epochs = lstm_epochs_for(K)
```

**When naming `delan_tag` replace:**

```python
delan_epochs_eff = "ep?"
delan_tag = f"delan_jax_{model_short}_s{seed}_{delan_epochs_eff}"
```

with:

```python
delan_tag = f"delan_jax_{model_short}_s{delan_seed}_ep{delan_epochs}"
```

**In the DeLaN train command**, add an epochs flag (assuming your trainer supports `--epochs` / `--max_epoch`; if it uses `--max_epoch`, use that name):

Replace:

```python
f"--hp_preset {DELAN_HP_PRESET} "
```

with:

```python
f"--hp_preset {DELAN_HP_PRESET} "
f"--max_epoch {delan_epochs} "
```

**In the LSTM command**, replace:

```python
f"--epochs {LSTM_EPOCHS} "
```

with:

```python
f"--epochs {lstm_epochs} "
```

**And in the LSTM dir name**, replace `LSTM_EPOCHS` with `lstm_epochs`:

```python
f"_ep{lstm_epochs}_b{LSTM_BATCH}_u{LSTM_UNITS}_do{safe_tag(LSTM_DROPOUT)}"
```

---