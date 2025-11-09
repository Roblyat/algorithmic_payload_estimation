# what’s a “parameter value” and what’s “loss”?

* **parameter value (θ):** the numbers inside your model (weights of an NN, coefficients of a polynomial, etc.). Training = finding good θ.
* **loss L(θ):** a score that says how bad the model is on data (e.g., mean-squared error). Training moves θ to **minimize** L(θ). In physics-informed setups (PINNs), the loss can have extra terms that enforce physics.

below are the plots you asked for + short descriptions.

---

## 1) SGD on a simple loss

This shows SGD taking steps on
$$
L(\theta)=(\theta-3)^2
$$
from
$$
\theta_0=-4
$$
with learning rate
$$
\eta=0.3.
$$

**x-axis:** parameter `θ`. **y-axis:** loss `L(θ)`.
Each arrow is one update `θ ← θ − η ∇L`. Steps get smaller near the minimum because the gradient shrinks.

![1) SGD steps on a simple quadratic](/docs/general_information/1%29%20SGD%20steps%20on%20a%20simple%20quadratic.png)

---

## 2) bias–variance: overfitting vs. underfitting (the “elbow” in complexity)

We fit polynomials of degree `0–15` to noisy `sin(x)` and plot **training** vs **validation** MSE.

* Low degree (left): **underfitting** — both errors high (too simple; high bias).
* Increasing degree: validation error **drops** until an **elbow** (sweet spot).
* Too high degree (right): **overfitting** — training error keeps going down, but validation error **turns up** (variance dominates).

**x-axis:** model complexity (polynomial degree). **y-axis:** MSE.
Use the **validation curve’s minimum** as the elbow to pick complexity.

![2) Bias–Variance: Overfitting vs Underfitting](/docs/general_information/2%29%20Bias%E2%80%93Variance%3A%20Overfitting%20vs%20Underfitting.png)

---

## 3) learning curves: do we have enough data?

Fixed model (degree `9`). We grow the training set and track errors.

* With **very little data**, training error is tiny (the model memorizes) but validation error is big → overfitting.
* As **data increases**, the gap **closes** and validation error **drops** until it **plateaus** — an **elbow in data size** (diminishing returns).

**x-axis:** training set size. **y-axis:** MSE.
If the gap stays large even with lots of data → model may be too complex or features noisy.

![3) Learning Curves: Effect of More Data (degree=9)](/docs/general_information/3%29%20Learning%20Curves%3A%20Effect%20of%20More%20Data%20%28degree%3D9%29.png)

---

## 4) balancing a **physics term** in the loss (PINNs-style)

We add a physics penalty to the data loss:
$$
\min_{\theta}; L_{\text{data}} ;+; \lambda,L_{\text{physics}},
$$
here choosing a toy “physics” that encourages `y'' + y = 0` (true for `sin(x)`). We sweep `λ`.

* **Small `λ`**: physics ignored → higher val error.
* **Moderate `λ`**: best balance → **lowest** validation error (the “physics elbow”).
* **Huge `λ`**: physics dominates, data underfit → val error **rises**.

**x-axis:** physics weight `λ` (log scale). **y-axis:** validation MSE.

![4) Balancing Data vs Physics Term](/docs/general_information/4%29%20Balancing%20Data%20vs%20Physics%20Term.png)

---

## how these ideas map to your robotics papers

* **Parameter & loss:** in your payload ID / inverse-dynamics models, `θ` are network weights or identified parameters; **loss** is MSE on torques/parameters, possibly plus **physics** residuals (e.g., `M·q̈ + C·q̇ + G + F_f − τ`) with a weight `λ` or `α`.
* **Elbows:**

  * **complexity elbow**: e.g., LSTM size, polynomial degree — pick where validation error bottoms out.
  * **data elbow**: how much dataset you need for the estimator to stabilize.
  * **physics elbow**: how strongly to weight the physics term so you help generalization **without** strangling data fit.

---

## A) Learning-rate demo (SGD)

**What you see:** three SGD runs on the same quadratic loss with different learning rates.
**x-axis:** parameter value `θ`. **y-axis:** loss `L(θ)`.

* `η = 0.05` (green): tiny, safe steps → slow but steady descent.
* `η = 0.30` (orange): good step size → fast convergence.
* `η = 1.10` (red): too big → oscillates and shoots away (diverges).

![A) Effect of Learning Rate on SGD](/docs/general_information/A%29%20Effect%20of%20Learning%20Rate%20on%20SGD.png)

---

## B) Loss vs epochs (feedforward + backprop in action)

**What you see:** training a tiny MLP (`1–20–1`, ReLU) on noisy `sin(x)` with **mini-batch SGD**.
**x-axis:** epoch. **y-axis:** MSE loss.

* **Training loss** and **validation loss** both fall as we iterate **feedforward → loss → backprop → weight update**.
* The gap between curves shows generalization; a widening gap would hint at overfitting.

![B) Loss vs Epochs for a Tiny MLP (1-20-1, ReLU)](/docs/general_information/B%29%20Loss%20vs%20Epochs%20for%20a%20Tiny%20MLP%20%281-20-1%2C%20ReLU%29.png)

---

## C) Feedforward & backprop schematic (2–2–1 MLP)

**What you see:** a small network graph. **Blue arrows** = **feedforward**:
`z = W·x + b`, `a = σ(z)`, `ŷ = W₂·a + b₂`.
**Red arrows** = **backprop**: start from `∂L/∂ŷ`, use the **chain rule** to get gradients on all weights/biases, then update with SGD.

![C) Feedforward (blue) and Backpropagation (red) in a 2–2–1 MLP](/docs/general_information/C%29%20Feedforward%20%28blue%29%20and%20Backpropagation%20%28red%29%20in%20a%202%E2%80%932%E2%80%931%20MLP.png)

---
