# Example

## 1.3 Aim of the Work
We aim to design and evaluate a **[chosen family: e.g., Hybrid Residual (Q4)]** approach that:
A) performs **online PDPI** during normal task motion (no dedicated PE),
B) is robust to **sensor noise and unmodeled friction**, and
C) requires **minimal prior modeling**.

### 1.3.1 Research Questions
- RQ1: Can a Q4 residual model maintain unbiased PDPI estimates under low-excitation, noisy conditions?
- RQ2: What fusion of (q, ẋ, τ, F/T, vision) minimizes variance in mass/CoM/inertia online?
- RQ3: How does the method compare to strong baselines from Q1 and Q3 on accuracy, latency, and stability?

### 1.3.2 Scientific Contribution
- **Method:** A residual hybrid PDPI estimator combining [short descriptor], explicitly modeling friction/noise.
- **Theory/Analysis:** Conditions for bounded error without classical PE; noise/friction sensitivity analysis.
- **Empirics:** Benchmarked vs. Q1_*, Q3_* baselines on [robots/datasets]; ablations for sensor sets and motion regimes.
