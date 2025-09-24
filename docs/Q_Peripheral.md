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

### 2. **Programmatic Imitation Learning from Unlabeled and Noisy Demonstrations (PLUNDER)** (RA-L, 2024)

* **Problem/Issue**:
  Imitation Learning (IL) often requires *labeled*, *noise-free* demos and produces black-box neural policies. These are hard to interpret and adapt.

* **SoA**:

  * Behavior Cloning (NN-based).
  * Inverse RL.
  * Programmatic Imitation Learning (PIL): symbolic program synthesis (LDIPS, PROLEX).
  * GAN-based IfO (GAIfO) for unlabeled demos — but opaque policies.

* **Math Background**:

  * Formulated as **latent variable MAP estimation**:
    $\pi^* = \arg \max_{\pi} \sum_{a_{1:t}} P(z|a,s)P(a|s,\pi)P(\pi)$.
  * Uses **Expectation-Maximization (EM)**:

    * E-step: infer action labels via particle filter.
    * M-step: synthesize probabilistic ASP (Action Selection Policy).
  * Prior: penalizes large ASTs to prevent overfitting.

* **Methods**:

  * **PLUNDER algorithm**: probabilistic PIL with EM loop.
  * Synthesizes interpretable **probabilistic programs** in DSL form.
  * Handles **noisy, unlabeled** human or sim demonstrations.
  * Compared against LDIPS, BC/BC+, GAIfO, Behavior Transformers.

* **Results**:

  * 95% alignment with demos (19% better than next best).
  * 90% task success rate (17% higher).
  * Converges in <10 EM iterations.
  * Robust under noise (outperforms GAIfO & BC).
  * Generated policies interpretable (conditions expressed in logic + probabilities).

* **Contribution**:

  * First **probabilistic PIL** for noisy, unlabeled demos.
  * Bridges gap between **symbolic program synthesis** and **probabilistic modeling**.

* **Outlook**:

  * Apply to **real-world robot data**.
  * Integrate **LLMs** or **neural-guided synthesis** for scalability.
  * Jointly optimize observation models.

* **Focus**:
  **Residual hybridization** of **program synthesis + probabilistic models** → robust, interpretable **robot learning from demos**.

---

### 3. Yuan et al 2025 **Optimization of Adaptive Algorithm for Precise Motion Control of Multi-Degree-of-Freedom Robotic Arms

* **Problem/Issue**: Precise control of multi-DoF arms suffers under uncertainty, noise, and nonlinearities.

* **SoA**: Fuzzy control, heuristic optimization, DRL for real-time adaptation.

* **Math Background**: Adaptive control, fuzzy logic, online optimization, DRL policy learning.

* **Methods**:

  * Combined fuzzy control + deep reinforcement learning.
  * Optimized adaptive algorithm for trajectory tracking.

* **Results**: Improved accuracy + robustness in real-time control.

* **Contribution**: Domain adaptation method for **high-precision trajectory control**.

* **Outlook**: Extend to contact-rich manipulation.

* **Focus**: **Robot control adaptation**, not parameter ID.

---

### 4. **Sampling-Based MPC Leveraging Parallelizable Physics Simulations** (RA-L 2025)

* **Problem/Issue**: Online MPC is limited by computational cost.

* **SoA**: Sampling-based MPC, physics simulators.

* **Math Background**: MPC, contact dynamics, optimization.

* **Methods**:

  * Physics simulations parallelized for sampling-based MPC.
  * Contact modeling integrated into optimization.

* **Results**: Improved real-time MPC, robust to contact.

* **Contribution**: Framework uniting **contact dynamics + MPC + physics simulation**.

* **Outlook**: Whole-body manipulation with fast online MPC.

* **Focus**: **Control adaptation with simulation-in-the-loop**.

---