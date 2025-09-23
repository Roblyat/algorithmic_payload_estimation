### 1. **Uncertainty in Bayesian Reinforcement Learning for Robot Manipulation Tasks with Sparse Rewards** (IEEE ROBIO 2023)

* **Problem/Issue**:
  Conventional deep RL struggles with sparse-reward manipulation tasks. Agents explore poorly, converge slowly, and are unstable under uncertainty.

* **SoA**:

  * RL widely applied in grasping, tool use, HRI.
  * Bayesian Neural Networks (BNNs) offer uncertainty estimates but scale poorly.
  * Dropout-based Bayesian approximations introduced to make DRL uncertainty-aware.

* **Math Background**:

  * MDPs, value functions, policy gradients.
  * Bayesian inference (priors, posteriors over NN weights).
  * KL divergence for reward uncertainty.
  * Monte Carlo sampling (MCMC).

* **Methods**:

  * Proposed **Bayesian Deep RL (BDRL)** framework.
  * Quantifies **3 types of uncertainty**:

    1. **Model uncertainty** (NN weights).
    2. **Aleatoric uncertainty** (data noise).
    3. **Reward function uncertainty** (mis-specified sparse rewards).
  * Combines DRL + Bayesian network for action selection & uncertainty-aware training.

* **Results**:

  * Tested on **4 robot manipulation tasks** (Reach, Push, Pick-and-Place, Slide).
  * Improved convergence, stability, and exploration compared to vanilla DRL.
  * Bayesian networks enabled **conservative actions under uncertainty**, reducing failure.

* **Contribution**:

  * Introduces **uncertainty-aware RL for manipulation** under sparse rewards.
  * Provides generalizable method to accelerate training and stabilize policies.

* **Outlook**:

  * Extend to real-world manipulation & HRI.
  * Improve scalability of Bayesian approximations for larger networks.

* **Focus**:
  **Learning-based policy optimization**, not direct parameter ID or force estimation.

---