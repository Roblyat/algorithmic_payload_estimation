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