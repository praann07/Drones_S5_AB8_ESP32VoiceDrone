# DMDc Deep Dive: the mathematics of the linear model in this project

**Who this is for.** Someone who already knows what DMDc is (see [`DMDC_EXPLAINED.md`](DMDC_EXPLAINED.md) for the from-scratch version) and wants the full mathematics: where the model comes from, exactly what the estimator minimises, what the singular values of the real data matrix say, what the fitted matrices mean physically, how accurate they are and why, and what a linear model can never do for this drone.

**Where every number comes from.** All figures below were computed from this repository's own data by [`02_identification/dmdc_deep_dive.py`](02_identification/dmdc_deep_dive.py) (read-only, run it with `.venv\Scripts\python.exe 02_identification\dmdc_deep_dive.py`). Where a number comes from somewhere else, the text says so. Claims that were *not* measured are marked **(not verified)**.

**Project:** voice-controlled quadrotor in MuJoCo, course *Data Driven Control of Drones*, School of Artificial Intelligence, Amrita Vishwa Vidyapeetham, Coimbatore.

---

## Contents

0. [Notation](#0-notation)
1. [The identification problem](#1-the-identification-problem)
2. [Physics first: the model DMDc is trying to find](#2-physics-first-the-model-dmdc-is-trying-to-find)
3. [The DMDc estimator](#3-the-dmdc-estimator)
4. [Anatomy of the real data matrix](#4-anatomy-of-the-real-data-matrix)
5. [What was fitted](#5-what-was-fitted)
6. [Statistics: errors, bias, and the smoothing effect](#6-statistics-errors-bias-and-the-smoothing-effect)
7. [Spectral analysis](#7-spectral-analysis)
8. [Controllability and observability](#8-controllability-and-observability)
9. [Prediction error: one step, many steps, and why it fails away from hover](#9-prediction-error-one-step-many-steps-and-why-it-fails-away-from-hover)
10. [The Koopman view, and how DMDc sits next to SINDy](#10-the-koopman-view-and-how-dmdc-sits-next-to-sindy)
11. [A real worked example, by hand](#11-a-real-worked-example-by-hand)
12. [Findings that need your attention](#12-findings-that-need-your-attention)
13. [Honest limits](#13-honest-limits)
14. [Appendices](#14-appendices)

---

## 0. Notation

| Symbol | Meaning | Value in this project |
|---|---|---|
| $\Delta$ | sample time | $0.002$ s (500 Hz) |
| $z\in\mathbb R^6$ | state $(\theta_x,\theta_y,\theta_z,\omega_x,\omega_y,\omega_z)^\top$ | rotation vector $\theta$ (rad) of the body relative to a periodically reset reference, body rates $\omega$ (rad/s) |
| $u\in\mathbb R^4$ | four rotor thrusts | N |
| $\bar u$ | hover thrust per rotor | $9.81/4 = 2.4525$ N |
| $\delta u = u-\bar u\mathbf 1$ | differential thrust | N |
| $N$ | number of snapshot pairs | $95{,}504$ |
| $Z,\;Z^+\in\mathbb R^{6\times N}$ | states at $k$ and at $k+1$ | |
| $\Delta U\in\mathbb R^{4\times N}$ | differential controls at $k$ | |
| $\Omega=\begin{bmatrix}Z\\ \Delta U\end{bmatrix}\in\mathbb R^{10\times N}$ | the data matrix | |
| $G=[A\;B]\in\mathbb R^{6\times 10}$ | the unknowns | $A\in\mathbb R^{6\times6}$, $B\in\mathbb R^{6\times4}$ |
| $\Omega^\dagger$ | Moore–Penrose pseudo-inverse | |
| $\mathbf 1$ | the all-ones vector in $\mathbb R^4$ | the "collective thrust" direction |
| $I_x,I_y,I_z$ | principal moments of inertia | $0.003375,\;0.003375,\;0.00675$ kg m² |
| $L$ | rotor offset along $x$ and along $y$ | $0.106066$ m ($=0.15/\sqrt2$) |
| $k_m$ | yaw torque per newton of thrust | $0.02$ m |
| $c$ | rotational drag, $\tau_{\rm drag}=-c\,\omega$ | $0.008$ N m s/rad |

Time index $k$ is the sample number: $t_k=k\Delta$. A **segment** is a stretch of one recording between two resets of the reference attitude; snapshot pairs are formed *inside* a segment, never across a boundary.

---

## 1. The identification problem

The true plant is a nonlinear continuous-time system with state $z$ and input $u$:

$$\dot z = f(z,u),\qquad f(0,\bar u\mathbf 1)=0 .$$

The point $(z,u)=(0,\bar u\mathbf 1)$ is a **trim**: the drone hovering level. DMDc fits the *linear discrete-time* model

$$z_{k+1}=A\,z_k+B\,\delta u_k .$$

**Why that can work.** Expand $f$ about the trim. Writing $F_z=\partial f/\partial z$ and $F_u=\partial f/\partial u$ at the trim,

$$f(z,u)=F_z\,z+F_u\,\delta u+r(z,\delta u),\qquad \|r\|\le \tfrac12 M\,\big\|(z,\delta u)\big\|^2,$$

where $M$ bounds the second derivative of $f$ on the region visited. The linear model is therefore wrong by an amount that grows **quadratically** with the distance from hover. This is the whole reason DMDc is restricted to near-hover data in this project: the near-hover recordings keep the peak rotation below $20^\circ$ (enforced by `assert_near_hover_bound`, and the worst fit trial reached $17.82^\circ$).

**Assumptions the estimator needs, and the status of each.**

| | Assumption | Status here |
|---|---|---|
| A1 | The dynamics are well approximated by the linear model on the visited region | Measured in §9: the neglected terms are 0.4 % (gyroscopic) and 1.1 % (kinematic) of the linear ones near hover, and 80 % and 17 % outside it |
| A2 | The state is observed directly (no hidden states) | True by construction ($\theta,\omega$ come from the simulator) |
| A3 | The residual is zero-mean noise uncorrelated with the regressors | **Not exactly true**: the residual contains the Savitzky–Golay smoothing error and the unmodelled terms (see §6) |
| A4 | The data are informative, i.e. $\Omega$ has full row rank | **False by exactly one dimension**, and understanding why is the heart of §4 |
| A5 | No measurement noise in the regressors | True for the simulator data; **false for any real sensor** (§6.3 quantifies the damage) |

---

## 2. Physics first: the model DMDc is trying to find

DMDc learns $A,B$ without being told any physics. To judge what it learned we first derive what it *should* learn.

### 2.1 Rotational dynamics (Euler's equation)

For a rigid body with principal inertia $\mathbb I=\mathrm{diag}(I_x,I_y,I_z)$, body rates $\omega$ and applied torque $\tau$,

$$\mathbb I\dot\omega=\tau-\omega\times\mathbb I\omega-c\,\omega .$$

Written out, with $I_x=I_y=I_z/2$:

$$\dot\omega_x=\frac{\tau_x}{I_x}-\frac{I_z-I_y}{I_x}\omega_y\omega_z-\frac{c}{I_x}\omega_x,\quad
\dot\omega_y=\frac{\tau_y}{I_y}+\frac{I_z-I_x}{I_y}\omega_z\omega_x-\frac{c}{I_y}\omega_y,\quad
\dot\omega_z=\frac{\tau_z}{I_z}-\frac{I_y-I_x}{I_z}\omega_x\omega_y-\frac{c}{I_z}\omega_z .$$

With the numbers above, $(I_z-I_y)/I_x=(I_z-I_x)/I_y=1$ and $I_y-I_x=0$: the gyroscopic coupling has coefficient exactly 1 in the $x$ and $y$ equations and is absent from the $z$ equation.

### 2.2 Rotor torques (the mixing matrix)

Rotors sit at $(\pm L,\pm L,0)$ (`quad.xml`): rotor 1 at $(+,+)$, rotor 2 at $(-,-)$, rotor 3 at $(+,-)$, rotor 4 at $(-,+)$. A thrust $f_i$ along $+z$ at position $r_i$ gives torque $r_i\times f_i\hat z$, and the yaw reaction is $\pm k_m f_i$ (rotors 1, 2 positive; 3, 4 negative):

$$\tau=\underbrace{\begin{bmatrix} L&-L&-L&L\\ -L&L&-L&L\\ k_m&k_m&-k_m&-k_m\end{bmatrix}}_{M_\tau}\begin{bmatrix}f_1\\f_2\\f_3\\f_4\end{bmatrix}.$$

The three rows are mutually orthogonal and **each is orthogonal to $\mathbf 1=(1,1,1,1)^\top$**:

$$M_\tau\mathbf 1=0 .$$

This one line is the origin of the rank deficiency studied in §4: adding the same thrust to all four rotors produces no torque, so a torque-only excitation never moves the data along $\mathbf 1$.

### 2.3 Kinematics of the rotation-vector state

The state uses the rotation vector $\theta$ (axis × angle). Its time derivative is

$$\dot\theta=J^{-1}(\theta)\,\omega,\qquad J^{-1}(\theta)=I+\tfrac12[\theta]_\times+\Big(\tfrac1{\|\theta\|^2}-\tfrac{1+\cos\|\theta\|}{2\|\theta\|\sin\|\theta\|}\Big)[\theta]_\times^2 ,$$

where $[\theta]_\times$ is the skew-symmetric cross-product matrix. To first order,

$$\boxed{\;\dot\theta=\omega+\tfrac12\,\theta\times\omega+O(\|\theta\|^2\|\omega\|)\;}$$

**Checked on the data.** Regressing $(\dot\theta-\omega)$ on $\tfrac12\theta\times\omega$ gives a coefficient of $1.000$ on both the near-hover held-out set ($R^2=0.88$) and the full-envelope held-out set ($R^2=0.86$). The remaining variance is the next-order term plus smoothing error. So the *linear* kinematics $\dot\theta=\omega$ is what DMDc must learn, and the neglected piece is $\tfrac12\theta\times\omega$, at most $\tfrac12\|\theta\|\|\omega\|$ in size.

### 2.4 The linearisation at hover

Dropping the second-order terms ($\omega\times\mathbb I\omega$ and $\tfrac12\theta\times\omega$) and writing the torque in terms of the differential thrust, $\tau=M_\tau\,\delta u$:

$$\underbrace{\begin{bmatrix}\dot\theta\\ \dot\omega\end{bmatrix}}_{\dot z}=
\underbrace{\begin{bmatrix}0&I_3\\0&-\mathbb I^{-1}c\end{bmatrix}}_{A_c}z+
\underbrace{\begin{bmatrix}0\\ \mathbb I^{-1}M_\tau\end{bmatrix}}_{B_c}\delta u .$$

The three axes **decouple**. For one axis, with state $(\theta_i,\omega_i)$ and a scalar torque channel,

$$\begin{bmatrix}\dot\theta_i\\ \dot\omega_i\end{bmatrix}=\begin{bmatrix}0&1\\0&-a_i\end{bmatrix}\begin{bmatrix}\theta_i\\ \omega_i\end{bmatrix}+\begin{bmatrix}0\\ b_i\end{bmatrix}v_i,\qquad a_i=\frac{c}{I_i},\quad b_i=\frac{\text{lever}_i}{I_i}.$$

| axis | $a_i=c/I_i$ (1/s) | $b_i=\text{lever}/I_i$ (rad/s² per N on one rotor) |
|---|---|---|
| roll $x$ | $2.3704$ | $31.427$ (lever $L$) |
| pitch $y$ | $2.3704$ | $31.427$ |
| yaw $z$ | $1.1852$ | $2.963$ (lever $k_m$) |

### 2.5 Exact discretisation (zero-order hold)

The controller holds $u$ constant over each $0.002$ s step, so the exact discrete model is $A_d=e^{A_c\Delta}$, $B_d=\int_0^\Delta e^{A_cs}B_c\,ds$. For the one-axis block this has a closed form:

$$A_d=\begin{bmatrix}1&\dfrac{1-e^{-a\Delta}}{a}\\[2mm]0&e^{-a\Delta}\end{bmatrix},\qquad
B_d=b\begin{bmatrix}\dfrac{\Delta}{a}-\dfrac{1-e^{-a\Delta}}{a^2}\\[2mm]\dfrac{1-e^{-a\Delta}}{a}\end{bmatrix}.$$

Series for small $a\Delta$ (here $a\Delta\approx0.005$): $A_d\approx\begin{bmatrix}1&\Delta\\0&1-a\Delta\end{bmatrix}$ and $B_d\approx b\begin{bmatrix}\Delta^2/2\\ \Delta\end{bmatrix}$.

**Predicted values for this drone** (computed from the formulas, not fitted):

| | roll / pitch | yaw |
|---|---|---|
| $A_d[\theta,\omega]$ | $0.0019953$ | $0.0019976$ |
| $A_d[\omega,\omega]=e^{-a\Delta}$ | $0.995270$ | $0.997632$ |
| $B_d[\omega]$ per N on one rotor | $0.062705$ | $0.005919$ |
| $B_d[\theta]$ per N on one rotor | $6.275\times10^{-5}$ | $5.921\times10^{-6}$ |

---

## 3. The DMDc estimator

### 3.1 Data matrices

For each segment with samples $z_0,\dots,z_{m}$ and inputs $u_0,\dots,u_m$, take the pairs $(z_k,\delta u_k)\mapsto z_{k+1}$ for $k=0,\dots,m-1$, and concatenate the columns of all segments:

$$Z=[z_0\;z_1\cdots],\quad Z^+=[z_1\;z_2\cdots],\quad \Delta U=[\delta u_0\;\delta u_1\cdots],\quad \Omega=\begin{bmatrix}Z\\ \Delta U\end{bmatrix}.$$

In this project: 16 near-hover recordings of 6000 samples each, 12 s long. After the Savitzky–Golay edge trim (15 samples at each end) each gives one segment of 5969 pairs, so $N=16\times5969=95{,}504$ and $\Omega\in\mathbb R^{10\times95504}$.

The 16 recordings are 4 axes (roll, pitch, yaw, combined) × 2 signal types (a 0.1 → 15 Hz linear chirp, and a fast pseudo-random binary sequence) × 2 repeats. The held-out set is another 16.

### 3.2 The optimisation problem

DMDc solves

$$\boxed{\;\min_{G\in\mathbb R^{6\times10}}\;\big\|Z^+-G\,\Omega\big\|_F^2\;}\qquad\Longrightarrow\qquad \hat G=Z^+\Omega^\dagger .$$

The squared Frobenius norm splits over the six rows of $Z^+$, so this is **six independent ordinary-least-squares problems** that share one regressor matrix $\Omega$. Row $i$: $\min_g\|(Z^+)_{i\cdot}-g^\top\Omega\|_2^2$.

**Normal equations.** Setting the gradient to zero:

$$\hat G\,\Omega\Omega^\top=Z^+\Omega^\top .$$

If $\Omega\Omega^\top$ is invertible this gives $\hat G=Z^+\Omega^\top(\Omega\Omega^\top)^{-1}$, which equals $Z^+\Omega^\dagger$. If it is singular (our case, §4), the minimiser is not unique, and $Z^+\Omega^\dagger$ is the one of **minimum Frobenius norm** among all minimisers.

**Geometric meaning.** Each row of $Z^+$ is a vector in $\mathbb R^N$. The fitted row is its orthogonal projection onto the 10-dimensional (here 9-dimensional) row space of $\Omega$. The residual is perpendicular to every regressor: $R\,\Omega^\top=0$ with $R=Z^+-\hat G\Omega$.

**Probabilistic meaning.** If $Z^+=G\Omega+E$ with $E$ having independent $\mathcal N(0,\sigma_i^2)$ entries in row $i$, then the least-squares estimate is the maximum-likelihood estimate; and under the weaker assumption of zero-mean uncorrelated errors with equal variance it is the best linear unbiased estimator (Gauss–Markov).

### 3.3 The SVD form

Let $\Omega=U\Sigma V^\top$ be the thin SVD, $U\in\mathbb R^{10\times10}$, $\Sigma=\mathrm{diag}(\sigma_1\ge\dots\ge\sigma_{10})$, $V\in\mathbb R^{N\times10}$. With $r$ the number of retained singular values,

$$\Omega^\dagger=V_r\,\Sigma_r^{-1}\,U_r^\top,\qquad \hat G=Z^+V_r\Sigma_r^{-1}U_r^\top .$$

(Derivation in Appendix A.) Each retained singular direction contributes the term $\sigma_j^{-1}(Z^+v_j)u_j^\top$: the response $Z^+v_j$ along that data direction, divided by how strongly the data excite it. **A small $\sigma_j$ divides noise by a small number**, which is exactly how the huge entries of $B$ in §4 appear.

Truncating at rank $r$ gives the best rank-$r$ approximation of $\Omega$ (Eckart–Young) and acts as a hard-threshold regulariser. Its smooth cousin is ridge regression, $\hat G_\lambda=Z^+\Omega^\top(\Omega\Omega^\top+\lambda I)^{-1}$, whose filter factors are $\sigma_j^2/(\sigma_j^2+\lambda)$ instead of $1$ or $0$.

### 3.4 What the project's code does

`02_identification/dmdc_fit.py` does exactly this:

```python
omega_mat = np.vstack([Z, dU])
G = Zp @ np.linalg.pinv(omega_mat)          # np.linalg.pinv with its default tolerance
A, B = G[:, :6], G[:, 6:]
```

Two details matter.

1. **Control is $\delta u=u-\bar u$.** A linear model describes deviations from a trim, which is what the Taylor expansion in §1 says.
2. **Cross-check against `pydmd`.** The projected DMDc of Proctor, Brunton and Kutz takes the SVD of the *stacked* data $\Omega=\tilde U\tilde\Sigma\tilde V^\top$, splits $\tilde U=\begin{bmatrix}\tilde U_1\\ \tilde U_2\end{bmatrix}$, and with a state basis $\hat U$ from the SVD of $Z^+$ builds

$$\tilde A=\hat U^\top Z^+\tilde V\tilde\Sigma^{-1}\tilde U_1^\top\hat U,\qquad \tilde B=\hat U^\top Z^+\tilde V\tilde\Sigma^{-1}\tilde U_2^\top .$$

With no truncation ($\hat U$ spanning the whole state space) this is algebraically the least-squares solution above, so any disagreement means numerical trouble, not a modelling difference. The code compares them on a **full-rank (rank-6)** segment and records the largest entry difference: $3.68\times10^{-3}$. (A single-axis segment is rank 2 in the state with condition number around $10^{48}$ and is useless as a reference.)

---

## 4. Anatomy of the real data matrix

### 4.1 The singular values

Computed from the real $\Omega$ ($10\times95{,}504$):

| $j$ | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|---|---|---|---|
| $\sigma_j$ | 45.09 | 32.65 | 30.20 | 19.28 | 14.83 | 12.65 | 10.36 | 2.921 | 2.631 | $7.3\times10^{-14}$ |

$\sigma_1/\sigma_{10}=6.15\times10^{14}$. There is a gap of thirteen orders of magnitude between $\sigma_9$ and $\sigma_{10}$, so the numerical rank is **9, not 10**, for every tolerance from $10^{-6}$ to $10^{-12}$.

Splitting the matrix:

| block | singular values | condition number |
|---|---|---|
| $Z$ (states, $6\times N$) | 45.09, 32.65, 30.20, 14.92, 12.64, 10.42 | $4.33$ |
| $\Delta U$ (controls, $4\times N$) | 19.18, 2.965, 2.673, $1.0\times10^{-11}$ | $\approx10^{12}$ |

The states alone are perfectly well conditioned ($\kappa=4.3$). All the trouble is in the control block.

Typical sizes of the columns (RMS over the fit data): $\theta$: $0.105,0.115,0.132$ rad; $\omega$: $0.0441,0.0447,0.0370$ rad/s; $\delta u_i$: $0.0316,0.0316,0.0339,0.0296$ N.

### 4.2 Why the rank is 9: the null direction is $\mathbf 1$

The left singular vector belonging to $\sigma_{10}$ has state block $\approx10^{-17}$ (zero) and input block

$$(0.5,\;0.5,\;0.5,\;0.5)^\top=\tfrac12\mathbf 1 .$$

So $\Omega^\top\!\big(0,\tfrac12\mathbf 1\big)\approx0$, i.e. **the four differential thrusts sum to zero in every column**. Measured: $\max_k\big|\mathbf 1^\top\delta u_k\big|=4.4\times10^{-16}$ N (rounding error), compared with an RMS of $0.0317$ N per rotor. The same holds on the held-out sets ($4.4\times10^{-16}$ near hover, $6.7\times10^{-16}$ on the full envelope).

The reason is §2.2. The excitation signals (`01_simulation/excitation.py`) are generated directly as **per-rotor thrust offsets**, each a signal multiplied by one of the torque sign patterns $(+,-,-,+)$, $(-,+,-,+)$, $(+,+,-,-)$ (the rows of $M_\tau$). Every pattern is orthogonal to $\mathbf 1$ (§2.2), so every offset vector, and every sum of them, has zero total. Nothing ever adds the same thrust to all four rotors. A real data snapshot shows it directly (roll chirp, sample $k=500$, $t=1$ s):

$$u=(2.4348,\;2.4702,\;2.4702,\;2.4348)\text{ N}\;\Longrightarrow\;\delta u=(-0.0177,\;+0.0177,\;+0.0177,\;-0.0177)\text{ N},\quad\mathbf 1^\top\delta u=0 .$$

Hence $\operatorname{rank}\Delta U=3$ (the three torque channels), $\operatorname{rank}\Omega=6+3=9$, and exactly one direction of $B$ cannot be learned.

### 4.3 What the pseudo-inverse does with that direction

`np.linalg.pinv` uses a default cut-off of $10^{-15}\sigma_1=4.5\times10^{-14}$. The singular value $\sigma_{10}=7.3\times10^{-14}$ is *just above* it, so it is **kept**, and the solution contains $\sigma_{10}^{-1}\approx1.4\times10^{13}$ times a rounding-noise component. The result is the raw $B$ of the Stage B gate, which has four identical columns of enormous entries:

$$B_{\rm raw}[\omega_x,:]=(-3.135,\,-3.135,\,-3.135,\,-3.135)\times10^{8},\qquad B_{\rm raw}[\omega_y,:]=(-3.619)\times10^{8},\ \dots$$

Decompose any $B$ into its collective and torque parts with the projector $P_\perp=I_4-\tfrac14\mathbf 1\mathbf 1^\top$:

$$B=\underbrace{B P_\perp}_{\text{identified: torque effects}}+\underbrace{\tfrac14 B\mathbf 1\,\mathbf 1^\top}_{\text{unidentified: pure noise, }\sim10^{8}} .$$

$BP_\perp$ is the meaningful part. Its entries are (rows $\omega_x,\omega_y,\omega_z$, columns rotors 1–4):

$$BP_\perp=\begin{bmatrix}+0.06034&-0.06023&-0.06012&+0.06002\\-0.06005&+0.06019&-0.06029&+0.06016\\+0.00545&+0.00570&-0.00536&-0.00579\end{bmatrix}.$$

The sign patterns $(+,-,-,+)$, $(-,+,-,+)$, $(+,+,-,-)$ are exactly the rows of $M_\tau$ from §2.2. The three $\theta$ rows of $BP_\perp$ are all below $4.5\times10^{-4}$ in magnitude (theory: $6.3\times10^{-5}$).

**Why the huge part is harmless for torque prediction.** The prediction error it causes is $\tfrac14B\mathbf 1\cdot\mathbf 1^\top\delta u$, and $\mathbf 1^\top\delta u\le4.4\times10^{-16}$ N on all the data used here, so even with entries of order $10^{8}$ the product is order $10^{-7}$ per step, below the rounding noise of the state itself. It would be **badly wrong for any change in total thrust** (altitude control), because that direction was never excited.

**A truncated SVD removes it cleanly.** Keeping the 9 singular values above $10^{-8}\sigma_1$ gives $\hat G_{\rm trunc}$ with $\max|B_{\rm trunc}|=0.0602$, $\max|A_{\rm trunc}-A_{\rm pinv}|=2.3\times10^{-7}$ and $\max|B_{\rm trunc}-BP_\perp|=4.0\times10^{-4}$.

| held-out one-step NRMSE | `pinv` (as in the gate) | truncated SVD |
|---|---|---|
| near hover | 0.07475 | 0.07283 |
| full envelope | 0.69998 | 0.69939 |

The two solutions differ only in entries below $5\times10^{-4}$; the 0.002 difference in the near-hover score is therefore driven by those tiny entries and the rounding noise the near-singular solve leaves in them. **(Which entries cause it was not isolated.)** The analysis below uses $\hat G_{\rm trunc}$ wherever a clean $B$ is needed.

---

## 5. What was fitted

### 5.1 The matrix $A$ (from the gate fit)

Order $\theta_x,\theta_y,\theta_z,\omega_x,\omega_y,\omega_z$. Entries rounded to four significant figures:

$$A=\begin{bmatrix}
1.000&1.3\!\times\!10^{-6}&1.1\!\times\!10^{-6}&\mathbf{0.001994}&-6.7\!\times\!10^{-6}&8.5\!\times\!10^{-6}\\
-9.3\!\times\!10^{-7}&1.000&-3.6\!\times\!10^{-7}&6.3\!\times\!10^{-6}&\mathbf{0.001996}&-6.0\!\times\!10^{-6}\\
4.1\!\times\!10^{-8}&-2.8\!\times\!10^{-8}&1.000&-3.3\!\times\!10^{-6}&3.2\!\times\!10^{-6}&\mathbf{0.001999}\\
-1.4\!\times\!10^{-5}&9.5\!\times\!10^{-7}&3.9\!\times\!10^{-6}&\mathbf{0.99543}&-3.6\!\times\!10^{-5}&-9.9\!\times\!10^{-5}\\
9.8\!\times\!10^{-7}&-1.2\!\times\!10^{-5}&6.0\!\times\!10^{-7}&3.1\!\times\!10^{-5}&\mathbf{0.99550}&6.3\!\times\!10^{-5}\\
-7.8\!\times\!10^{-7}&-3.2\!\times\!10^{-7}&-5.8\!\times\!10^{-6}&1.2\!\times\!10^{-5}&3.4\!\times\!10^{-6}&\mathbf{0.99773}
\end{bmatrix}$$

The structure is exactly the block form predicted in §2.5; the bold entries are the physical ones. The unbolded entries are all at most $10^{-4}$; the largest are $\omega_x\leftarrow\omega_z=-9.9\times10^{-5}$ and $\omega_x\leftarrow\omega_y=-3.6\times10^{-5}$. In the exact linearisation these couplings are **zero** (§2.4), so they are estimation error, smoothing error and the leading edge of the neglected nonlinearity.

### 5.2 Fit versus physics

| entry | physics (§2.5) | fitted (gate, smoothed data) | fitted (raw data, §6.4) |
|---|---|---|---|
| $A[\theta_x,\omega_x]$ | $0.0019953$ | $0.0019938$ | |
| $A[\omega_x,\omega_x]$ | $0.995270$ | $0.995430$ | $0.995208$ |
| $A[\omega_y,\omega_y]$ | $0.995270$ | $0.995495$ | $0.995309$ |
| $A[\omega_z,\omega_z]$ | $0.997632$ | $0.997728$ | $0.997630$ |
| roll thrust gain per N (one rotor) | $0.062705$ | $0.060178$ ($\times0.9597$) | $\times1.0024$ |
| pitch thrust gain | $0.062705$ | $0.060172$ ($\times0.9596$) | $\times1.0024$ |
| yaw thrust gain | $0.005919$ | $0.005576$ ($\times0.9421$) | $\times1.0012$ |
| $B[\theta_x,\cdot]$ | $\pm6.3\times10^{-5}$ | entries up to $3\times10^{-4}$ | |

The diagonal drag entries match physics to $1.6\times10^{-4}$ ($0.995430$ vs $0.995270$). The thrust gains are $4\%$ and $6\%$ low on the smoothed data and within $0.24\%$ of physics on the raw data. Section 6.4 explains why.

### 5.3 Standard errors

Under the white-noise model of §3.2, row $i$ of $\hat G$ has covariance $\sigma_i^2(\Omega\Omega^\top)^\dagger$. With the residual standard deviations measured from the fit,

| row | $\theta_x$ | $\theta_y$ | $\theta_z$ | $\omega_x$ | $\omega_y$ | $\omega_z$ |
|---|---|---|---|---|---|---|
| residual std per step | $1.59\times10^{-5}$ | $8.35\times10^{-6}$ | $1.53\times10^{-5}$ | $8.13\times10^{-5}$ | $8.21\times10^{-5}$ | $5.57\times10^{-5}$ |

the standard errors come out at

| quantity | estimate | standard error |
|---|---|---|
| $A[\omega_x,\omega_x]$ | $0.995430$ | $6.2\times10^{-6}$ |
| $A[\omega_y,\omega_y]$ | $0.995495$ | $6.3\times10^{-6}$ |
| $A[\omega_z,\omega_z]$ | $0.997728$ | $5.2\times10^{-6}$ |
| roll gain (per unit-norm torque direction) | $0.12036$ | $2.1\times10^{-5}$ |
| yaw gain (per unit-norm torque direction) | $0.011152$ | $1.4\times10^{-5}$ |

**These standard errors are far too optimistic as statements about the truth.** The 4 % shortfall of the roll gain ($0.12036$ vs $0.12541$) is about $240$ standard errors, which no noise could produce. The errors are **systematic**: the white-noise assumption A3 is violated by the smoothing (§6.4). The numbers are valid only as the *sampling* uncertainty of this particular fit.

---

## 6. Statistics: errors, bias, and the smoothing effect

### 6.1 Unbiasedness and its limits

If the regressors are noise-free and the residual has zero mean conditional on them, $\mathbb E\hat G=G$. If the *true* map is nonlinear, $\hat G$ converges to the best linear approximation **on the distribution of data actually collected**; it is a property of the experiment as much as of the drone. Fitting on $\le20^\circ$ data and testing on the full envelope (§9) is a direct demonstration.

### 6.2 The Savitzky–Golay filter is a linear filter

The preprocessing replaces $\theta$ and $\omega$ by their Savitzky–Golay smooth (window $31$ samples $=62$ ms, polynomial order 3). That is a symmetric FIR filter with coefficients summing to 1 and therefore **zero phase**; its gain at the measured frequencies is

| $f$ (Hz) | 0.5 | 1 | 2 | 5 | 10 | 20 |
|---|---|---|---|---|---|---|
| $\lvert H(f)\rvert$ | 1.000000 | 0.999995 | 0.999919 | 0.99693 | 0.95579 | 0.53996 |

The control $u$ is **not** filtered. So the regression sees a filtered response against an unfiltered input.

### 6.3 Measurement noise (a hypothetical, because the project's data are noise-free)

If the observed state is $\tilde z_k=z_k+n_k$ with white noise $n_k$, then $\tilde z_k$ appears **both** as the regressor in column $k$ and as the target in column $k-1$. Take the scalar autoregression $x_{k+1}=\lambda x_k$ with stationary variance $\sigma_x^2$ and noise variance $\sigma_n^2$:

$$\operatorname{plim}\hat\lambda=\frac{\operatorname{Cov}(\tilde x_{k+1},\tilde x_k)}{\operatorname{Var}(\tilde x_k)}=\lambda\,\frac{\sigma_x^2}{\sigma_x^2+\sigma_n^2}<\lambda .$$

This is **attenuation bias** (errors-in-variables). In the matrix case the estimate of $A$ is shrunk toward zero and, because the input $\delta u$ is clean but the state is not, the estimate of $B$ is pushed up to compensate.

**Instrumental variables fix it.** Use the *previous* state as an instrument, $\Psi=\begin{bmatrix}Z_{k-1}\\ \Delta U\end{bmatrix}$:

$$\hat G_{\rm IV}=Z^+\Psi^\top\big(\Omega\Psi^\top\big)^{-1}.$$

For white noise $n_{k-1}$ is uncorrelated with the noise in $\tilde z_k$ and in $\tilde z_{k+1}$ while still correlated with $z_k$, so $\hat G_{\rm IV}$ is consistent.

**Experiment** (noise *added by this analysis* to the real fit data, same noise realisation in each sample; $\sigma_\theta=0.05\,\sigma_\omega$ s; roll axis shown; true $\lambda=0.99543$, true roll gain $0.12036$):

| $\sigma_\omega$ (rad/s) | $\hat\lambda_{\omega_x}$ least squares | $\hat\lambda_{\omega_x}$ IV | roll gain, LS | roll gain, IV |
|---|---|---|---|---|
| 0 | 0.99543 | 0.99543 | 0.12032 | 0.12032 |
| 0.002 | 0.99319 | 0.99543 | 0.12218 | 0.12037 |
| 0.005 | 0.98157 | 0.99547 | 0.13158 | 0.12034 |
| 0.010 | 0.94185 | 0.99589 | 0.16472 | 0.12085 |
| 0.020 | 0.81549 | 0.99322 | 0.26742 | 0.12254 |

For reference, the rms of $\omega$ in the data is $0.037$–$0.045$ rad/s, so $\sigma_\omega=0.01$ is about 22 % noise. Least squares then reports a drag pole of $0.9419$ (a time constant of $0.033$ s instead of $0.44$ s) and a thrust gain $37\%$ too high, while IV stays within $0.05\%$ and $0.4\%$. At $\sigma_\omega=0.02$ IV is noisier ($0.99322$) because the instrument is weaker. **This is the case for any real-hardware use of DMDc**, and it does not show up in the project's simulator data. The project's separate Stage C noise ablation tests the *flight controller's* noise tolerance, not this estimator bias.

### 6.4 The smoothing biases the thrust gain by a few percent

Regress $\omega_{k+1}$ on $(\omega_k,\;v_k)$ with the **raw, unsmoothed** simulator rates ($v_k$ the roll-channel input $\mathbf t^\top\delta u_k$ for the unit direction $\mathbf t=\tfrac12(1,-1,-1,1)$):

| axis | $\lambda$ raw | $\lambda$ physics | $\lambda$ smoothed fit | gain raw / physics | gain smoothed / physics | residual std raw | residual std smoothed |
|---|---|---|---|---|---|---|---|
| roll | 0.995208 | 0.995270 | 0.995430 | **1.0024** | 0.9597 | $7.0\times10^{-6}$ | $8.1\times10^{-5}$ |
| pitch | 0.995309 | 0.995270 | 0.995495 | **1.0024** | 0.9596 | $6.3\times10^{-6}$ | $8.2\times10^{-5}$ |
| yaw | 0.997630 | 0.997632 | 0.997728 | **1.0012** | 0.9421 | $9.3\times10^{-18}$ | $5.6\times10^{-5}$ |

On raw data the model recovers physics to $0.24\%$ in gain and $6\times10^{-5}$ in the drag pole, with a residual ten times smaller (and, for yaw, at rounding level, because the simulator's yaw dynamics are exactly linear). On the smoothed data the gain is $4$–$6\%$ low.

**Mechanism.** If $\omega_s=h\ast\omega$ with $h$ the Savitzky–Golay kernel, then

$$\omega_s[k+1]-\omega_s[k]=\big(h\ast\Delta\omega\big)[k]\approx\Delta\,(h\ast(b\,v-a\,\omega))[k].$$

The regression of the left side on the unfiltered $v_k$ recovers $b$ weighted by $H(f)$ at the frequencies where the input has energy. The excitation is a **linear chirp from 0.1 to 15 Hz** (`excitation.py`), so each frequency gets equal time. The mean Savitzky–Golay gain over that band is

$$\overline{|H|}=\frac1{14.9}\int_{0.1}^{15}|H(f)|\,df=0.9585 ,$$

against a measured roll gain ratio of $0.9597$. That agreement (0.1 %) supports the mechanism for roll and pitch. The yaw ratio (0.9421) is *not* explained by this number alone **(not verified)**: the yaw recordings use a different amplitude and the yaw response weights the spectrum differently.

The damage is limited: the gains are 4–6 % low, but the identified model still gives a one-step NRMSE of 0.075 and an 11.9 s rollout error of 0.4 %.

---

## 7. Spectral analysis

### 7.1 Eigenvalues of $A$

Computed from the truncated-SVD fit (its $A$ differs from the gate's $A$ by at most $2.3\times10^{-7}$ in any entry, so the eigenvalues agree to about $10^{-6}$):

| $\lambda$ | $\lvert\lambda\rvert$ | $s=\ln\lambda/\Delta$ (1/s) | meaning |
|---|---|---|---|
| $0.99999\pm1.6\!\times\!10^{-6}i$, $0.99999$ | $\approx0.999995$ | $-0.0026\pm0.0008i$, $-0.0028$ | the three **angle integrators** ($\dot\theta=\omega$) |
| $0.99547\pm9.4\!\times\!10^{-6}i$ | $0.99547$ | $-2.2710\pm0.0047i$ | roll and pitch rate decay, time constant $1/2.271=0.440$ s |
| $0.99773$ | $0.99773$ | $-1.1349$ | yaw rate decay, time constant $0.881$ s |

Spectral radius: $0.9999947$. Physics (§2.4): $\{0,0,0,-a_x,-a_y,-a_z\}=\{0,0,0,-2.370,-2.370,-1.185\}$ in continuous time. The fitted values are $-2.271$ and $-1.135$, about 4 % smaller in magnitude — the same smoothing effect as in §6.4.

**The integrator eigenvalues are not exactly 1.** They are $0.999995$, i.e. $\mathrm{Re}\,s=-0.0026\ \mathrm{s}^{-1}$ (a time constant of about 380 s, roughly 900 times slower than the 0.44 s decay of the other modes). The true value in the linear model is $1$ exactly; the fitted deviation of $5\times10^{-6}$ per step is estimation error. Strictly, then, the fitted model is *asymptotically* stable with an extremely slow mode, where the physics is *marginally* stable. For every practical purpose the two are the same.

### 7.2 Structure: why the fitted spectrum looks like this

$A$ is, up to the tiny couplings, block-diagonal over the three axes, and each block is the $2\times2$ matrix of §2.5, upper-triangular with eigenvalues $1$ and $e^{-a\Delta}$:

$$\lambda_1=1,\ v_1=\begin{bmatrix}1\\0\end{bmatrix};\qquad \lambda_2=e^{-a\Delta},\ v_2\propto\begin{bmatrix}1\\-a\end{bmatrix}\ \ (\text{to first order}).$$

The second mode has $\omega=-a\theta$: a tilt and a rate in a fixed ratio, so that $\dot\theta=\omega=-a\theta$ and **both decay together**, at rate $a$. The first mode is a pure tilt with no rate: it stays forever, and that is why a free rigid body does not return to level.

**The eigenvectors the computer returns look different** (columns with several axes mixed). That is a consequence of *near-degeneracy*: the three integrator eigenvalues differ by $\sim10^{-6}$ and the roll and pitch decay eigenvalues by $6.5\times10^{-5}$. Eigenvectors inside a (nearly) repeated eigenvalue are determined only up to an arbitrary mixture, and the $10^{-5}$ cross-couplings of §5.1 are enough to select one. This is why the roll and pitch decay modes appear as a complex-conjugate pair ($0.99547\pm9.4\times10^{-6}i$) instead of two real eigenvalues. The condition number of the eigenvector matrix is $9.2$, so the decomposition is well conditioned even though the individual eigenvectors are arbitrary.

**Boundedness without eigenvalue 1 being asymptotic.** Because each axis' eigenvalue 1 is *semisimple* (no Jordan block), $\|A^k\|$ stays bounded. Measured: $\|A^k\|_2$ rises from $1$ to a maximum of $1.316$ near $k\approx2500$ and then falls to $1.296$ at $k=5500$ (11 s). If it had been a Jordan block it would grow like $k$.

### 7.3 Continuous time from the fit

The continuous-time model follows from the matrix logarithm of the augmented matrix. Since $e^{[A_c\,B_c;\,0\,0]\Delta}=\begin{bmatrix}A&B\\0&I\end{bmatrix}$,

$$\begin{bmatrix}A_c&B_c\\0&0\end{bmatrix}=\frac1\Delta\log\begin{bmatrix}A&B\\0&I\end{bmatrix}.$$

Results (truncated fit, principal logarithm):

| | fitted | physics | ratio |
|---|---|---|---|
| $A_c[\theta_x,\omega_x]$ | $0.99917$ | $1$ | $0.9992$ |
| $A_c[\theta_y,\omega_y]$ | $1.00031$ | $1$ | |
| $A_c[\theta_z,\omega_z]$ | $1.00040$ | $1$ | |
| $A_c[\omega_x,\omega_x]$ | $-2.2903$ | $-2.3704$ | $0.966$ |
| $A_c[\omega_y,\omega_y]$ | $-2.2575$ | $-2.3704$ | $0.952$ |
| $A_c[\omega_z,\omega_z]$ | $-1.1373$ | $-1.1852$ | $0.960$ |
| $B_c[\omega_x,\text{rotor }1]$ | $30.147$ | $31.427$ | $0.959$ |
| $B_c[\omega_z,\text{rotor }1]$ | $2.843$ | $2.963$ | $0.960$ |

The kinematic entries come out at $1\pm0.001$ — DMDc *rediscovered* $\dot\theta=\omega$ from data, with nothing but the pairs $(z_k,z_{k+1})$. The Euler estimate $(A-I)/\Delta$ and the exact logarithm differ by at most $5.2\times10^{-3}$ in any entry: an $O(\Delta)$ difference (Appendix B).

**Cross-couplings in $A_c$** are all small: the largest are $\omega_x\leftarrow\omega_z=-0.050$ and $\omega_x\leftarrow\omega_y=-0.018$, against the diagonal $-2.29$. In the exact linearisation they are zero.

---

## 8. Controllability and observability

*This section is an analysis done for this document on the fitted near-hover model. It is not part of the project's validation gates, and the README lists a formal reachability study as future work.*

### 8.1 Definitions and tests

For $z_{k+1}=Az_k+Bu_k$ the reachable set after $n$ steps is the range of the **Kalman controllability matrix**

$$\mathcal C=\big[\,B\;\;AB\;\;A^2B\;\cdots\;A^{n-1}B\,\big].$$

The pair $(A,B)$ is controllable iff $\operatorname{rank}\mathcal C=n$. The **PBH test** is equivalent: for every eigenvalue $\lambda$ of $A$, $\operatorname{rank}[\lambda I-A\;\;B]=n$. Observability is the dual statement for $(A,C)$; here the whole state is measured, $C=I$, so the pair is observable trivially.

Because $\Delta U$ has rank 3 (§4.2), the identified input matrix has **three effective inputs**: the torques about $x,y,z$. The fourth direction (collective thrust) acts on altitude, which is not in this six-state model. So the analysis uses $B_\tau=B_{\rm trunc}\,T^\top$ with $T$ the three unit torque directions $\tfrac12(1,-1,-1,1)$, $\tfrac12(-1,1,-1,1)$, $\tfrac12(1,1,-1,-1)$.

### 8.2 Results

| test | result |
|---|---|
| rank of the discrete $\mathcal C=[B_\tau\ AB_\tau\cdots A^5B_\tau]$ ($6\times18$) | **6** (full) |
| singular values of that matrix | $0.2914,\ 0.2914,\ 0.02766,\ 1.01\!\times\!10^{-3},\ 1.01\!\times\!10^{-3},\ 9.5\!\times\!10^{-5}$ |
| rank of the continuous $\mathcal C$ (from $A_c,B_c$) | **6** |
| its singular values | $4586,\ 4295,\ 26.0,\ 24.4,\ 24.1,\ 3.71$ |
| PBH rank at each of the six eigenvalues | $6,6,6,6,6,6$ |
| observability ($C=I$) | rank 6 |

Both versions have full rank. The condition numbers of the six singular values are $3.1\times10^{3}$ (discrete) and $1.2\times10^{3}$ (continuous). The *discrete* singular values are small in absolute size only because each $0.002$ s step has a tiny influence. The rank is what matters, and it is 6.

### 8.3 How hard is it to steer? The Gramian

The finite-horizon controllability Gramian $W(T)=\sum_{k<T/\Delta}A^kB_\tau B_\tau^\top(A^\top)^k$ gives the minimum control energy to reach a state $x$ at rest: $E_{\min}=x^\top W^{-1}x$ (sum of squared inputs over the horizon).

| horizon | eigenvalues of $\Delta\cdot W$ |
|---|---|
| 0.5 s | $1.2\!\times\!10^{-6},\ 4.1\!\times\!10^{-5},\ 1.27\!\times\!10^{-4},\ 1.28\!\times\!10^{-4},\ 3.00\!\times\!10^{-3},\ 3.04\!\times\!10^{-3}$ |
| 2.0 s | $2.8\!\times\!10^{-5},\ 1.1\!\times\!10^{-4},\ 2.08\!\times\!10^{-3},\ 2.10\!\times\!10^{-3},\ 4.8\!\times\!10^{-3},\ 4.9\!\times\!10^{-3}$ |

The spread within a horizon (ratio $\approx2500$ at 0.5 s) says some directions of state space are far costlier to reach than others; lengthening the horizon to 2 s improves the worst direction by a factor of $\approx23$.

**A concrete number.** Reaching a $10^\circ$ roll from rest in $0.5$ s (the inner-loop settling time) needs
$$\sum_{k=1}^{250}\|u_k\|^2=0.456\ \text{N}^2,$$
that is $0.043$ N rms per step in the unit torque channel, about $1.7\%$ of the $2.45$ N hover thrust of a single rotor. Attitude authority is nowhere near a limit for such manoeuvres. **(This is a linear-model estimate.** The 17.8° peak and motor saturation at 6 N are not part of it.)

---

## 9. Prediction error: one step, many steps, and why it fails away from hover

### 9.1 The metrics

The gate's **one-step NRMSE** is

$$\mathrm{NRMSE}=\sqrt{\frac{\sum_k\|\hat z_{k+1}-z_{k+1}\|^2}{\sum_k\|z_{k+1}-z_k\|^2}},\qquad \hat z_{k+1}=Az_k+B\delta u_k .$$

The denominator is the **actual change in one step**, not the size of the state. This matters: over $2$ ms the state barely moves, so an error normalised by $\|z\|$ looks tiny for *any* model, including "nothing changes", which scores exactly $1$ on this metric. A number of $0.075$ therefore means the one-step error is about 7.5 % of the size of the one-step motion.

The **derivative $R^2$** is $1-\sum\|\dot z-\hat{\dot z}\|^2/\sum\|\dot z-\bar{\dot z}\|^2$ with $\hat{\dot z}=(\hat z_{k+1}-z_k)/\Delta$.

| | derivative $R^2$ | one-step NRMSE |
|---|---|---|
| DMDc, near-hover held-out (**reproduced today**) | 0.99386 | 0.07475 |
| DMDc, full-envelope held-out | not stored | 0.69998 |
| SINDy, full-envelope held-out (stored) | 0.99457 | 0.07855 |

(See §12 for why this DMDc row differs from the figure quoted elsewhere in the repository.)

### 9.2 Error accumulation over a rollout

Run the model open loop, feeding it the recorded inputs, from the true initial state: $\hat z_{k+1}=A\hat z_k+B\delta u_k$. With $e_k=\hat z_k-z_k$ and $\delta_k$ the one-step residual,

$$e_{k+1}=A\,e_k+\delta_k\qquad\Longrightarrow\qquad e_n=\sum_{j=0}^{n-1}A^{\,n-1-j}\,\delta_j ,\qquad \|e_n\|\le\sup_j\|A^j\|\sum_{j<n}\|\delta_j\|.$$

If the residuals were *coherent* (same sign every step) the sum would grow like $n\|\delta\|$; if they were independent it would grow like $\sqrt n\|\delta\|$.

Numbers for the near-hover held-out set: $\|\delta\|\approx1.3\times10^{-4}$ per step (the root-sum-square of the six residual standard deviations in §5.3, using the fit residuals as a stand-in for the held-out ones), $\sup_j\|A^j\|=1.316$ (the largest value among the sampled $k=0,500,\dots,5500$, §7.2), $n=5970$ steps (11.94 s).

| | value |
|---|---|
| coherent bound $n\,\sup\|A^j\|\,\|\delta\|$ | $\approx1.0$ |
| random-walk estimate $\sqrt n\,\sup\|A^j\|\,\|\delta\|$ | $\approx0.013$ |
| **measured**, 16 held-out segments of 11.94 s: error norm at the end of the rollout | mean $0.0042$, max $0.0073$ |
| **measured**: largest error norm at any time in the rollout | mean $0.0065$, max $0.0119$ |
| for scale: rms state norm $\|z\|$ | $0.146$ |

The measured error sits at the *random-walk* scale ($0.012$ against $0.013$), a factor of $\sim100$ below the coherent bound: the residuals behave like uncorrelated noise, not like a drift. This is a consistency check on the error model, not a proof. The worst final error of $0.0073$ is about $5\%$ of the rms state size. All 16 rollouts stayed bounded.

### 9.3 Why a linear model fails away from hover

Section 1 predicted a remainder growing like the square of the distance from hover. Measured on held-out data, the size of the two neglected terms relative to the linear terms they sit beside:

| | near hover | full envelope |
|---|---|---|
| rms $\lVert\omega\rVert$ | $0.050$ rad/s | $4.16$ rad/s |
| rms $\lVert\theta\rVert$ | $7.9^\circ$ | $80.4^\circ$ |
| gyroscopic term $\omega\times\mathbb I\omega$ vs. $\dot\omega$: rms ratio | **0.0039** | **0.799** |
| kinematic term $\tfrac12\theta\times\omega$ vs. $\dot\theta$: rms ratio | **0.0111** | **0.173** |

Near hover the neglected terms are 0.4 % and 1.1 % — negligible, so the linear model is accurate. Over the full envelope the gyroscopic term is **80 %** of the rms angular acceleration, and the kinematic term 17 % of $\dot\theta$. A linear model cannot represent a product of two states, so it explains only part of the motion, and the one-step NRMSE rises from $0.075$ to $0.70$. This is not a bug; it is the quadratic remainder of §1, measured.

---

## 10. The Koopman view, and how DMDc sits next to SINDy

### 10.1 Koopman operator

For the flow $\varphi_\Delta$ of $\dot z=f(z,u_0)$ the **Koopman operator** acts on scalar observables $g$ by $(\mathcal K^\Delta g)(z)=g(\varphi_\Delta(z))$. It is linear on an infinite-dimensional space even though $f$ is nonlinear. DMD approximates $\mathcal K^\Delta$ restricted to the span of the observables $g_i(z)=z_i$ (the state coordinates themselves). That restriction is exact only if the span is *invariant*: $z_i(\varphi_\Delta(z))\in\operatorname{span}\{z_1,\dots,z_n\}$. For the real drone,

$$z(\varphi_\Delta(z))=A_dz+O(\|z\|^2),$$

so the coordinates are **approximately** invariant near hover, with an error governed by the quadratic terms of §9.3. DMDc is the version with inputs (the observable set is the state plus the input).

### 10.2 Extended DMD and the link to SINDy

**EDMD** enlarges the span with a dictionary $\psi(z)=[z;\;z_iz_j;\dots]$ and fits a linear model $\psi(z_{k+1})\approx K\psi(z_k)$. **SINDy** fits a *continuous-time* model $\dot z=\Xi^\top\Theta(z,u)$ with a dictionary $\Theta$ and an $\ell_0$-style sparsity penalty (here STLSQ). They sit on one ladder:

| model | dictionary | time | sparsity |
|---|---|---|---|
| **DMDc** (this project) | $[z;\,\delta u]$ | discrete, one step | none |
| continuous-time linear regression | $[z;\,\delta u]$ | $\dot z$ | none |
| SINDy (this project, selected) | $26$ candidate terms per equation (linear terms plus products) | $\dot z$ | STLSQ, 44 of 156 coefficients kept |
| EDMD | any dictionary | discrete | none |

If SINDy is given only the linear terms and no thresholding, it is continuous-time DMDc, and by Appendix B its $\Xi$ matches $(A-I)/\Delta$, $B/\Delta$ to $O(\Delta)$: measured in §7.3 as $5\times10^{-3}$.

### 10.3 Head-to-head (from the project's validation stage)

| | DMDc | SINDy |
|---|---|---|
| parameters | $36+24=60$ dense entries, of which $6\times9=54$ are identifiable (§4) | 156 candidate coefficients, 44 non-zero |
| valid region | near hover | full tested envelope |
| near-hover held-out one-step NRMSE | 0.0825 in `head_to_head_near_hover.csv` (but see §12) | 0.0789 |
| outside hover | 0.70 | selected, drives the controller |

(See §12 before quoting the DMDc number.)

---

## 11. A real worked example, by hand

This example uses **real numbers** from the project: the roll-axis chirp recording `A_hover_roll_chirp_000` (raw, unsmoothed), four moments chosen far apart in time. It is small enough to solve on paper and it exercises every step of the estimator.

**The model for one channel.** From §2.5, with $v_k=\mathbf t^\top\delta u_k$ and $\mathbf t=\tfrac12(1,-1,-1,1)$:

$$\omega_{k+1}=\lambda\,\omega_k+b\,v_k ,\qquad\text{unknowns }(\lambda,b).$$

### Step 1: the raw numbers

| $k$ | $t$ (s) | $u_1,u_2,u_3,u_4$ (N) | $v_k$ | $\omega_k$ (rad/s) | $\omega_{k+1}$ (rad/s) |
|---|---|---|---|---|---|
| 500 | 1.000 | 2.43480, 2.47020, 2.47020, 2.43480 | $-0.03539718$ | $+0.12743900$ | $+0.12238514$ |
| 1500 | 3.000 | 2.44081, 2.46419, 2.46419, 2.44081 | $-0.02338013$ | $-0.07415437$ | $-0.07674189$ |
| 2500 | 5.000 | 2.45485, 2.45015, 2.45015, 2.45485 | $+0.00469894$ | $-0.05655812$ | $-0.05569929$ |
| 3500 | 7.000 | 2.46489, 2.44011, 2.44011, 2.46489 | $+0.02478076$ | $-0.03019729$ | $-0.02693900$ |

(Check of §4.2 on the first row: $\delta u=u-2.4525=(-0.0177,+0.0177,+0.0177,-0.0177)$, so $\mathbf 1^\top\delta u=0$.)

### Step 2: the data matrices

$$\Omega=\begin{bmatrix}\omega_k\\v_k\end{bmatrix}=\begin{bmatrix}0.12743900&-0.07415437&-0.05655812&-0.03019729\\-0.03539718&-0.02338013&0.00469894&0.02478076\end{bmatrix},\quad Z^+=\begin{bmatrix}0.12238514&-0.07674189&-0.05569929&-0.02693900\end{bmatrix}.$$

### Step 3: the normal equations $\hat G\,\Omega\Omega^\top=Z^+\Omega^\top$

$$\Omega\Omega^\top=\begin{bmatrix}\sum\omega^2&\sum\omega v\\\sum\omega v&\sum v^2\end{bmatrix}=\begin{bmatrix}0.025850267&-0.003791317\\-0.003791317&0.002435757\end{bmatrix},\qquad Z^+\Omega^\top=\begin{bmatrix}0.025251118&-0.003467150\end{bmatrix}.$$

Determinant: $\det=0.025850267\times0.002435757-(0.003791317)^2=6.2965\times10^{-5}-1.4374\times10^{-5}=4.8591\times10^{-5}$.

Solve the $2\times2$ system by Cramer's rule:

$$\lambda=\frac{0.002435757\times0.025251118-(-0.003791317)\times(-0.003467150)}{4.8591\times10^{-5}}=\mathbf{0.99525923},$$

$$b=\frac{0.025850267\times(-0.003467150)-(-0.003791317)\times0.025251118}{4.8591\times10^{-5}}=\mathbf{0.12570782}.$$

**Check on the data.** Predictions $\lambda\omega_k+bv_k$ are $0.122385139,\ -0.076741887,\ -0.055699298,\ -0.026938996$; the targets are $0.12238514,\ -0.07674189,\ -0.05569929,\ -0.02693900$. They agree to the 8 digits printed: the residual is $\sim10^{-16}$. Four equations, two unknowns, and every equation satisfied: **the raw simulator data are exactly linear in this axis**, so the least-squares solution is exact.

### Step 4: the same answer from the SVD

$$\Omega=U\Sigma V^\top,\qquad \Sigma=\mathrm{diag}(0.162631,\;0.042862),\qquad U=\begin{bmatrix}-0.98776&0.15595\\0.15595&0.98776\end{bmatrix}.$$

The singular values are the square roots of the eigenvalues of $\Omega\Omega^\top$ ($0.026449$ and $0.001837$), and $\kappa(\Omega)=3.79$, so the problem is well conditioned. Then $\hat G=Z^+V\Sigma^{-1}U^\top$ gives $(\lambda,b)=(0.99525926,\ 0.12570785)$, the same to seven digits (the difference is the rounding of the printed table).

### Step 5: compare with the whole recording

The same two numbers from all $5999$ pairs of this recording:

$$(\lambda,b)=(0.99525926,\;0.12570785).$$

Four hand-picked pairs reproduce the 5999-pair answer to the digits shown, again because the data are exactly linear.

### Step 6: compare with physics

| | fitted (4 pairs) | physics (§2.5) | ratio |
|---|---|---|---|
| $\lambda=e^{-a\Delta}$ | $0.99525926$ | $0.99527048$ | $0.99999$ |
| $b$ (per unit-norm torque direction) | $0.12570785$ | $0.12541$ | $1.0024$ |

So the drag pole is recovered to $1.1\times10^{-5}$ and the thrust gain to $0.24\%$. Converting: $a=-\ln\lambda/\Delta=2.3760$ s$^{-1}$ versus $C/I_x=2.3704$ s$^{-1}$.

### Step 7: scaling up

The project's fit is this *same machinery* with the matrices enlarged: six states and four inputs instead of one and one, $95{,}504$ columns instead of $4$, and **smoothed** data instead of raw. That last change is why the project's roll gain is $0.9597$ of physics instead of $1.0024$ (§6.4). The rank deficiency of §4 does not appear in the one-channel example because the single input $v_k$ already lives in the identifiable subspace; in the four-rotor version the missing direction $\mathbf 1$ is what costs the extra, unidentifiable column.

---

## 12. Findings that need your attention

These came up while preparing this document. They are facts about the repository, listed so they are not discovered during review.

1. **The stored DMDc score changed on 21 October, and the documentation still quotes the old value.**
   - `data/processed/identification_results.csv` was refreshed in commit `fb71aa1` (2026-10-21, "Results, data and plots refresh"): DMDc near-hover derivative $R^2=0.99386$, one-step NRMSE $0.07475$.
   - The earlier version of the same file (commit `5a170d0`, 2026-09-09) held $R^2=0.99255$, NRMSE $0.08249$. The SINDy row is **identical** in both versions ($0.99457$ / $0.07855$).
   - The README, `CLAUDE.md`, `ONBOARDING.md`, `RESULTS.md`, `PROJECT_EXPLAINED.md`, `DMDC_EXPLAINED.md`, `INTRO_DRONE_MUJOCO.md` and the stored `head_to_head_near_hover.csv` and `model_selection.md` all still say $0.9926$ / $0.0825$.
   - **Reproduced today** with the code and data in this folder: $0.07475$ and $R^2=0.99386$ (identical to the refreshed file). The cause of the change between the two runs was **not determined**. One suspect worth checking is that the near-singular direction of §4.3 makes the raw `pinv` solution sensitive to the linear-algebra library: switching to the truncated SVD already moves the score from $0.0748$ to $0.0728$.
   - The head-to-head statement "SINDy matches DMDc in DMDc's own regime" holds either way (SINDy $0.0789$ vs DMDc $0.0747$ or $0.0825$; on the new number DMDc is marginally *better*, which would change the wording of "SINDy matches or slightly beats it").

2. **The thrust gains are 4–6 % low because of the smoothing, not the drone** (§6.4). A fit on raw data recovers physics to 0.24 %.

3. **The fitted angle-integrator eigenvalues are $0.999995$, not 1** (§7.1). The model is, strictly, asymptotically stable with a ~380 s mode, where physics is marginal.

4. **A controllability check exists now** (§8; full rank, PBH passes). Other documents in the repository still list controllability and reachability as not done; this analysis is not part of the validation gates, so they are not wrong, but they are now incomplete.

5. **The comment in `dmdc_fit.py` about "removing the uniform-thrust direction"** (also noted in `DMDC_EXPLAINED.md`) describes subtracting the constant hover thrust, which does not remove the direction; the data leave it unidentified (§4.2).

---

## 13. Honest limits

- **Near hover only.** Outside $\le20^\circ$ the one-step NRMSE is $0.70$ (§9.3); the linear model cannot hold the gyroscopic product terms.
- **$B$ is identified only on the torque subspace** (§4). Collective thrust (altitude) is not in this model.
- **Smoothing biases the thrust gain by 4–6 %** and inflates the residual by a factor of 10 (§6.4). A fit on raw data would be more accurate.
- **No measurement noise.** The data are noise-free simulator output. With realistic sensor noise plain least squares would be biased (§6.3: a drag pole of $0.942$ instead of $0.995$ at 22 % noise); an instrumental-variable or total-least-squares estimator would be needed. **This was not applied to the project's pipeline.**
- **The linear model drives nothing.** The flight controller's gains come from the SINDy hover-Jacobian; DMDc is the linear baseline and an eigenvalue analysis tool.
- **The standard errors of §5.3 are not confidence intervals for the truth**, because the residual is not white noise.
- **The tiny cross-couplings ($\le10^{-4}$) in $A$ are not physical**; they are estimation and smoothing error.
- **The explanation for the 0.0825 → 0.0747 change is not established** (§12, item 1).
- **No Koopman lifting was fitted.** §10 is exposition, not a result.

---

## 14. Appendices

### A. The pseudo-inverse through the SVD

Let $\Omega=U\Sigma V^\top$ with $\Sigma=\mathrm{diag}(\sigma_1,\dots,\sigma_r,0,\dots)$. The pseudo-inverse is $\Omega^\dagger=V\Sigma^\dagger U^\top$ where $\Sigma^\dagger$ inverts the non-zero entries. It satisfies the four Penrose conditions: $\Omega\Omega^\dagger\Omega=\Omega$, $\Omega^\dagger\Omega\Omega^\dagger=\Omega^\dagger$, and both $\Omega\Omega^\dagger$ and $\Omega^\dagger\Omega$ are symmetric.

*Least squares.* For any $G$, $\|Z^+-G\Omega\|_F^2=\|Z^+-G\Omega\Omega^\dagger\Omega\|_F^2$ is minimised when $G\Omega$ equals the projection of $Z^+$ onto the row space of $\Omega$, namely $Z^+\Omega^\dagger\Omega$. So $\hat G=Z^+\Omega^\dagger$ is a minimiser, and for any other minimiser $G'=\hat G+W$ with $W\Omega=0$ we get $\|G'\|_F^2=\|\hat G\|_F^2+\|W\|_F^2$ (the cross term vanishes because the rows of $\hat G$ lie in the row space of $\Omega^\top$, orthogonal to the rows of $W$). Hence $\hat G$ is the minimum-norm minimiser.

*Why truncation helps.* Each retained direction adds $\sigma_j^{-1}(Z^+v_j)u_j^\top$. A noise error $\eta$ in $Z^+v_j$ is amplified by $\sigma_j^{-1}$. For $\sigma_{10}=7.3\times10^{-14}$ that is $1.4\times10^{13}$.

### B. Euler versus exact discretisation

For $\dot z=A_cz+B_cu$ with a zero-order hold, $A=e^{A_c\Delta}=I+\Delta A_c+\tfrac12\Delta^2A_c^2+\cdots$ and $B=\big(\Delta I+\tfrac12\Delta^2A_c+\cdots\big)B_c$. So

$$\frac{A-I}{\Delta}=A_c+\tfrac12\Delta A_c^2+O(\Delta^2),\qquad \frac{B}{\Delta}=B_c+\tfrac12\Delta A_cB_c+O(\Delta^2).$$

With $\|A_c\|\approx2.3$ and $\Delta=0.002$: $\tfrac12\Delta\|A_c\|^2\approx5.3\times10^{-3}$, which is the measured $5.2\times10^{-3}$ difference between $(A-I)/\Delta$ and $A_c$ in §7.3.

### C. The Savitzky–Golay kernel

For window $W=31$ and order $p=3$ the smoothing coefficients $h_j$ ($j=-15,\dots,15$) are the first row of $(X^\top X)^{-1}X^\top$ with $X_{ji}=j^i$, $i=0..3$. They are symmetric ($h_j=h_{-j}$) and sum to 1. The frequency response is $H(f)=\sum_jh_je^{-2\pi if j\Delta}$, real by symmetry, which is why the filter has zero phase and why the preprocessing does not delay the data. Its gain is $1$ at low frequency and falls to $0.956$ at $10$ Hz, $0.81$ at $15$ Hz and $0.54$ at $20$ Hz.

### D. Reproduction

| What | How |
|---|---|
| Every number in §4–§9 | `.venv\Scripts\python.exe 02_identification\dmdc_deep_dive.py` (a few minutes; prints sections A–L) |
| The Stage B gate fit | `.venv\Scripts\python.exe 02_identification\run_identification.py` |
| The DMDc estimator | `02_identification/dmdc_fit.py` |
| Smoothing | `02_identification/preprocessing.py` |
| Excitation signals | `01_simulation/excitation.py`, `01_simulation/run_excitation.py` |
| Plant constants | `01_simulation/models/quad.xml`, `01_simulation/sim_driver.py` |

Section letters in the script output: A data sizes, B SVD anatomy, C standard errors, D physics prediction, E continuous time, F spectrum, G controllability, H rollouts, I noise bias and IV, J smoothing response, K neglected terms, L raw-versus-smoothed.
