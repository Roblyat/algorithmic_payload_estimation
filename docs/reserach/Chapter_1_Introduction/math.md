# 1) Frames & signals

* $ (q,\dot q,\ddot q) \in \mathbb{R}^6 $: joint position/velocity/acceleration
* $ S $: F/T sensor frame
* $ {}^{S}J(q) \in \mathbb{R}^{6\times 6} $: geometric Jacobian expressed in $S$
* $ M(q),C(q,\dot q),g(q) $: standard RBD terms for the 6-DoF arm
* $ \tau_m \in \mathbb{R}^6 $: motor-side torque estimate from the “effort” signal

  * If effort = current: $ \tau_m = N^\top (k_t \circ I) $ (gear matrix $N$, torque constants $k_t$, element-wise $ \circ $)
  * If effort = joint torque: $ \tau_m \approx \tau_{\text{joint}} $
* $ \tau_f(\dot q) $: joint friction (viscous + Coulomb)
* $ F_S \in \mathbb{R}^6 $: wrench at the sensor (frame $S$)
* $ b_S $: constant F/T bias

---

# 2) Joint-space dynamics with external wrench

$$
\tau_{\text{robot}} = M(q)\ddot q + C(q,\dot q)\dot q + g(q) + \tau_f(\dot q)
$$

$$
\tau_{\text{ext}} = {}^{S}J(q)^{\top} F_S
$$

$$
\tau_m \approx \tau_{\text{robot}} + \tau_{\text{ext}}
$$

Residual joint torque attributable to the flange wrench:

$$
r_\tau = \tau_m - \tau_{\text{robot}} \approx {}^{S}J^{\top} F_S
$$

---

# 3) Convert residual joint torque to 6D wrench in the sensor frame

**(A) Dynamically consistent mapping (recommended):**
$$
\Lambda_S(q) := \big({}^{S}JM^{-1} \ {}^{S}J^{\top}\big)^{-1},
\qquad
\boxed{\hat F_S = \Lambda_S{}^{S}JM^{-1}r_\tau}
$$

---

# 4) Compare to the F/T sensor

Model of the raw sensor reading (y_S):
$$
y_S = \underbrace{\hat F_S}_{\text{from motors}} + b_S + \varepsilon
\quad \text{(no external contacts besides tool dynamics)}
$$

Define the residual you care about:
$$
\boxed{e_S = y_S - \hat F_S}
$$

Bias calibration:
$$
\hat b_S = \mathrm{mean}!\big(y_S - \hat F_S\big)
$$

(Optional) Frame misalignment fit:
$$
\min_{R_{ES},b_S}\sum_k \left|
y_S^{(k)} - \operatorname{blkdiag}(R_{ES},R_{ES})\hat F_S^{(k)} - b_S
\right|^2
$$

---

# 5) (Optional) “Effective rigid body” model for gripper+payload

Parameters:
$$
\phi^\top = \big[m, m c_x, m c_y, m c_z, J_{xx},J_{xy},J_{xz},J_{yy},J_{yz},J_{zz}\big]
;\in; \mathbb{R}^{10}
$$

Spatial inertia at (S) (with ( [c]*\times ) the skew matrix of (c)):
$$
I_S(\phi) =
\begin{bmatrix}
J_C - m[c]*\times[c]*\times & m[c]*\times \
-m[c]_\times & m I_3
\end{bmatrix}
$$

Newton–Euler wrench of the tool in (S) (twist (V_S=[\omega_S v_S]), accel (\dot V_S=[\dot\omega_S \dot v_S])):
$$
\boxed{\hat F_S(\phi) = I_S(\phi)\dot V_S + \operatorname{ad}^\top_{V_S}I_S(\phi)V_S}
$$

---

# 6) Practical real-time recipe (key equations)

Residual joint torque:
$$
r_\tau = \tau_m - \big(M\ddot q + C\dot q + g + \tau_f\big)
$$

Wrench estimate (either of the two):
$$
\hat F_S = \Lambda_S{}^{S}JM^{-1} r_\tau
\qquad\text{or}\qquad
\hat F_S = ({}^{S}J^\top)^{\dagger} r_\tau
$$

Sensor residual:
$$
e_S = y_S - \hat F_S
$$

---

# 7) In short

$$
\boxed{
\hat F_S = \big({}^{S}JM^{-1}{}^{S}J^\top\big)^{-1}{}^{S}JM^{-1}
\big(\tau_m - M\ddot q - C\dot q - g - \tau_f\big)
}
$$

Subtract from the F/T reading: (e_S = y_S - \hat F_S).
