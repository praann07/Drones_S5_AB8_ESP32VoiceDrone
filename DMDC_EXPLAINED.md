# DMDc Explained From Scratch, and How It Is Used in This Project

**DMDc** stands for **Dynamic Mode Decomposition with control**. This document explains it from the very beginning (no prior knowledge assumed), builds up the mathematics step by step, then shows exactly what DMDc does in *this* project, with the real matrices and numbers we measured.

If you are short on time: read **Part 1**, **Part 6** and **Part 9**.

---

## Contents

1. [The idea in plain words](#part-1--the-idea-in-plain-words)
2. [Building blocks: state, control, snapshots](#part-2--building-blocks)
3. [From DMD to DMDc](#part-3--from-dmd-to-dmdc)
4. [How the numbers are found (least squares, pseudo-inverse, SVD)](#part-4--how-the-numbers-are-found)
5. [A tiny worked example you can check by hand](#part-5--a-tiny-worked-example)
6. [Why a straight-line model can describe a drone (near hover)](#part-6--why-a-straight-line-model-can-describe-a-drone)
7. [DMDc in this project: the setup](#part-7--dmdc-in-this-project-the-setup)
8. [DMDc in this project: the fitted matrices, explained](#part-8--the-fitted-matrices-explained)
9. [DMDc in this project: the results](#part-9--the-results)
10. [How DMDc relates to SINDy](#part-10--how-dmdc-relates-to-sindy)
11. [Things that went wrong, and honest limits](#part-11--honest-limits)
12. [Questions a reviewer might ask](#part-12--questions-a-reviewer-might-ask)
13. [Where the code is, and how to reproduce the numbers](#part-13--code-and-reproduction)

---

## Part 1 · The idea in plain words

Imagine watching a shopping trolley being pushed in a car park, filming it, and wanting to predict where it will be one second later **without knowing any physics**. You could notice: "whenever it is moving like *this* and I push like *that*, one moment later it is moving like *this other thing*." If that relationship is roughly proportional (twice the push gives twice the change), you can capture it with a **table of multipliers**. That table is a **matrix**.

**DMDc does exactly that.** It takes recordings of a system and finds two matrices, `A` and `B`, such that

```
 next state  =  A × (state now)  +  B × (what I did now)
```

- `A` says **how the system carries on by itself**: how it drifts, decays or oscillates when nobody touches it.
- `B` says **how my actions change it**: what a push does.
- No physics equations are needed. The matrices come from **data**.
- It is called a **linear** model because the answer is always a straight proportional combination: no squares, no products of variables, no curves.

**The catch:** real drones are not perfectly linear. So a linear model like this is only trustworthy in a **small neighbourhood**. For our drone that neighbourhood is **hovering**. The rest of this document explains why that works, and what happens outside it.

---

## Part 2 · Building blocks

### State
The **state** is the shortest list of numbers that fully describes the system right now. For the drone's rotation we use six numbers:

```
 z = ( θx, θy, θz,  ωx, ωy, ωz )
       └─ tilt ──┘  └ rotation rate ┘
```

`θ` is how far the drone is tilted around each axis (radians). `ω` is how fast it is turning around each axis (radians per second).

### Control
The **control** (or input) is what we are allowed to choose: the four propeller thrusts `u = (u1, u2, u3, u4)` in newtons.

### Time steps
The simulation moves in tiny steps of `dt = 0.002 s` (500 per second). Write `z[k]` for the state at step `k`, and `z[k+1]` for the state one step later.

### Snapshots
A **snapshot** is one recorded moment: the state, the control, and the state one step later. If we record a flight of 5,000 steps we get about 5,000 snapshots. We stack them as columns of matrices:

```
 Z   = [ z[0]   z[1]   z[2]  ...  z[N-1] ]      the states         (6 rows,  N columns)
 Z⁺  = [ z[1]   z[2]   z[3]  ...  z[N]   ]      the states one step later  (6 rows, N columns)
 U   = [ u[0]   u[1]   u[2]  ...  u[N-1] ]      the controls       (4 rows,  N columns)
```

`Z⁺` is simply `Z` shifted by one step. The whole method is about the relationship between these three tables.

---

## Part 3 · From DMD to DMDc

**DMD** (Dynamic Mode Decomposition) is the *no-control* version. It assumes the system just evolves on its own:

```
 z[k+1] = A · z[k]            find A so that  Z⁺ ≈ A·Z
```

It was invented for fluid flows: film the swirling water, find the matrix `A`, and its **eigenvectors** ("modes") show the coherent patterns, and its **eigenvalues** show how fast each pattern grows, decays or oscillates.

**DMDc** adds the actions:

```
 z[k+1] = A · z[k]  +  B · u[k]            find A and B so that  Z⁺ ≈ A·Z + B·U
```

Why this matters for a drone: a drone does not evolve alone. We push it with propellers. If you ignored `u`, the model would wrongly blame the system for changes that we caused.

**Stacking trick.** Put the unknowns side by side and the knowns on top of each other:

```
                    ┌ Z ┐
   Z⁺  ≈  [ A  B ]  │   │         or, in short:     Z⁺  ≈  G · Ω
                    └ U ┘
```

`G = [A B]` is one wide matrix of unknowns, and `Ω = [Z; U]` stacks states over controls. Now there is **one** unknown matrix and one equation.

**Who proposed it:** Proctor, Brunton and Kutz, *Dynamic Mode Decomposition with Control*, SIAM Journal on Applied Dynamical Systems, 2016.

---

## Part 4 · How the numbers are found

### 4.1 Least squares
We want `G` such that `Z⁺ ≈ G·Ω`. With thousands of snapshots there are far more equations than unknowns, and noise means no `G` fits them all exactly. So we pick the `G` that makes the **total squared error** smallest:

```
 G  =  argmin_G  ‖ Z⁺ − G·Ω ‖²
```

### 4.2 The pseudo-inverse
The solution has a closed form using the **pseudo-inverse** `Ω†` (the best possible "inverse" of a matrix that is not square):

```
 G = Z⁺ · Ω†            then   A = first 6 columns of G,   B = last 4 columns of G
```

This is exactly what the project's code does: `G = Zp @ np.linalg.pinv(vstack([Z, dU]))`.

### 4.3 The SVD (why it matters)
The pseudo-inverse is computed with the **singular value decomposition**:

```
 Ω = U Σ Vᵀ          Ω† = V Σ⁻¹ Uᵀ
```

`Σ` holds **singular values**, which measure how much real information `Ω` carries along each direction. Two things follow:

- **Tiny singular values are dangerous.** Dividing by them in `Σ⁻¹` blows noise up enormously. A direction that the data never explored has a singular value near zero, and the solution there is meaningless.
- **Truncation.** The original DMDc algorithm keeps only the largest `r` singular values (`svd_rank = r`), which gives a compact "reduced" model. We use `pydmd`, a standard library, only as a **cross-check** (Part 7), and compute the main answer by direct least squares.

When no truncation is applied, the direct least-squares solution and the SVD-based DMDc are **mathematically the same estimator**. That is why comparing them is a useful safety check.

---

## Part 5 · A tiny worked example

A one-number system you can verify by hand. Suppose the truth is `x[k+1] = 0.9·x[k] + 0.5·u[k]` but we do not know the 0.9 or the 0.5. We start at `x = 1` and apply controls `1, 0, 2, −1`:

| k | x[k] | u[k] | x[k+1] (measured) |
|---|---|---|---|
| 0 | 1.0000 | 1 | 1.4000 |
| 1 | 1.4000 | 0 | 1.2600 |
| 2 | 1.2600 | 2 | 2.1340 |
| 3 | 2.1340 | −1 | 1.4206 |

Each row is one equation `x[k+1] = a·x[k] + b·u[k]` in the unknowns `a` and `b`:

```
 1.4000 = a·1.0000 + b·( 1)
 1.2600 = a·1.4000 + b·( 0)        →  a = 1.26 / 1.4 = 0.9   immediately
 2.1340 = a·1.2600 + b·( 2)        →  b = (2.134 − 0.9·1.26) / 2 = 0.5
 1.4206 = a·2.1340 + b·(−1)        →  0.9·2.134 − 0.5 = 1.4206  ✓
```

Four equations, two unknowns, and they agree perfectly: `a = 0.9`, `b = 0.5`. With real noisy data the equations disagree slightly, and least squares finds the `a` and `b` that disagree the *least*. Our drone has the same structure with a 6-number state and 4 controls, so `A` is 6×6 and `B` is 6×4.

**Reading the answer:** `a = 0.9 < 1` means the state shrinks by 10 % every step when left alone, so it settles. If `a` were above 1 the system would blow up.

---

## Part 6 · Why a straight-line model can describe a drone

The drone's real rotation equation is **nonlinear**:

```
 θ̇ = ω  (+ small tilt-dependent corrections)
 I·ω̇ = M·u − ω × (I·ω) − c·ω            ← contains ω × (I·ω): a product of the unknowns
```

Products like `ω × (I·ω)` are not allowed in a linear model. So why does DMDc work at all?

**Because near hover, the product terms are tiny.** If the drone is almost still, `ω` is small, so `ω × (I·ω)` (which is ω *squared*) is far smaller than the other terms. Mathematically this is a **first-order Taylor expansion** around the hover point `(z*, u*)`:

```
 ż = f(z, u)    ≈   f(z*, u*)  +  J·(z − z*)  +  Bc·(u − u*)
                    └── 0 at hover ──┘
 J  = ∂f/∂z at hover       Bc = ∂f/∂u at hover      (the "Jacobians")
```

With deviations `δz = z − z*` and `δu = u − u*`, we get a **linear** equation `δż = J·δz + Bc·δu`. Over one time step `dt` this becomes the discrete form DMDc fits:

```
 A  =  e^(J·dt)  ≈  I + J·dt               B  ≈  Bc·dt
```

So `A` should look like "the identity plus a small correction", and `B` should be small numbers proportional to `dt`. You will see exactly this in Part 8.

**The limit.** Move away from hover (big rotation rates) and the neglected product terms grow, the straight-line approximation breaks, and the model's predictions get worse. This is the reason DMDc is only claimed valid **near hover** in this project.

---

## Part 7 · DMDc in this project: the setup

### What DMDc is used for here (and what it is not)
- **It is not the model that flies the drone.** The flight controller's gains come from **SINDy**, the model selected after validation.
- **It is the linear comparison.** DMDc is the second identification method fitted on the same data, so we can ask a fair question: *does the sparse nonlinear model (SINDy) beat a plain linear one, and where?*
- It also provides an exact **eigenvalue** analysis (stability), because a linear model's stability can be read straight from `A`.

### The regime: near hover only
The design decision (`docs/TDD.md` section 5) is to fit DMDc **only on small-amplitude trials**, with a hard rule that the peak rotation stays under **20°**. A dedicated "near hover" family of excitation trials was recorded for this, and an assertion in the data-generation code enforces the bound. The worst trial actually reached **17.8°**. No Koopman or EDMD lift is used, by explicit choice (Part 10), so that SINDy versus DMDc isolates one question: *nonlinear and sparse versus linear and dense.*

### The data
| | Count |
|---|---|
| Near-hover trials used to fit DMDc | 16 |
| Near-hover trials held out for testing | 16 |
| Time step | 0.002 s (500 Hz) |
| State | 6 numbers (θ, ω) |
| Control | 4 propeller thrusts |

"Held out" means **entire separate recordings** that DMDc never saw. We never cut one recording in two, because the two halves would look alike and the test would be too easy.

### Three details that matter
1. **Control is the difference from hover.** We feed DMDc `du = u − 2.4525 N` (hover thrust per propeller, `9.81 ÷ 4`), not the raw thrust. A linear model describes *deviations* from a trim point, which is what the Taylor expansion in Part 6 says.
2. **Snapshot pairs never cross a boundary.** Each recording is split into segments, and consecutive pairs `(z[k], z[k+1])` are formed **inside** a segment only. A pair that straddles a reset would invent a transition that never physically happened.
3. **A second opinion.** The result is cross-checked against `pydmd` (the standard DMDc library). The cross-check must use a segment that is *full rank* in the state, because a single-axis trial leaves 4 of 6 state variables exactly zero, which makes its snapshot matrix rank 2 with a condition number around 10⁴⁸. On a full-rank (rank 6) segment, the two answers agree to a maximum entry difference of **0.0037**.

---

## Part 8 · The fitted matrices, explained

These are the actual fitted values from this project's data (rounded). Rows and columns are ordered `θx θy θz ωx ωy ωz`.

### The matrix A (6 × 6)

```
          θx     θy     θz     ωx      ωy      ωz
  θx   [ 1      0      0      0.002   0       0      ]
  θy   [ 0      1      0      0       0.002   0      ]
  θz   [ 0      0      1      0       0       0.002  ]
  ωx   [ 0      0      0      0.9954  0      -0.0001 ]
  ωy   [ 0      0      0      0       0.9955  0.0001 ]
  ωz   [ 0      0      0      0       0       0.9977 ]
```

**Reading it in words:**

| Part of A | Value | What it means |
|---|---|---|
| Top-left block | identity | The tilt does not decay by itself (no spring pulls a free body back). |
| Top-right block | `0.002` on the diagonal | **`θ[k+1] = θ[k] + 0.002·ω[k]`**: angle grows by rotation rate × dt. This is simply `θ̇ = ω` written in steps. DMDc *rediscovered* it from data, and `0.002` is exactly `dt`. |
| Bottom-right diagonal | `0.9954, 0.9955, 0.9977` | **Drag.** Each step the rotation rate keeps about 99.5 % of itself, so it decays slowly. Converting: `ln(0.9954)/0.002 ≈ −2.3 per second` for roll and pitch, and `≈ −1.15 per second` for yaw. These match the drag rates SINDy found independently. |
| Everything else | ≈ 0 | Near hover the axes do not influence each other (the cross-coupling is second order). |

### The matrix B (6 × 4), the part that needs care

The **real, meaningful** part of `B` (the effect of propeller thrusts on the rotation rates, per newton of thrust change) is:

```
              u1       u2       u3       u4
  ωx  [  0.0603  -0.0602  -0.0601   0.0600 ]      roll
  ωy  [ -0.0601   0.0602  -0.0603   0.0602 ]      pitch
  ωz  [  0.0054   0.0057  -0.0054  -0.0058 ]      yaw
```

and the rows for `θx, θy, θz` are essentially zero (below 0.0005): thrust does not change the tilt *directly* within one step, only through the rotation rate.

**Check against physics.** The drone's inertia is `Ix = Iy = 0.003375` and `Iz = 0.00675` kg·m², the arm factor is `0.106` m and the yaw coefficient is `0.02` m. So the expected effect of one newton on one propeller over one step is:

```
 roll / pitch :  dt · 0.106 / 0.003375  =  0.0629          learned: 0.0600 – 0.0603   (about 4 % low)
 yaw          :  dt · 0.020 / 0.00675   =  0.0059          learned: 0.0054 – 0.0058   (within a few %)
```

The **signs** also match the mixer geometry: for roll, propellers 1 and 4 push one way and 2 and 3 the other (`+ − − +`). So DMDc, with no physics given, recovered the drone's control effectiveness to within a few percent. That is the sign that the linear fit is meaningful.

**A numerical artifact in the raw B (important honesty).** If you print the raw fitted `B`, you will see enormous numbers (around 10⁷ to 10⁸), identical in all four columns. This is **not** physics. In the near-hover recordings the four thrusts always add up to the same total (the excitation only creates torques, not extra lift), so the data never varies in the "all four propellers together" direction. That direction has a singular value of exactly zero (Part 4.3), the pseudo-inverse divides noise by a near-zero number there, and a huge meaningless component appears **only along that one direction**.

- It does **no harm for the use here**, because the test data also has zero component in that direction, so predictions are unaffected (the one-step score of 0.0825 is computed with it present).
- It would be **wrong to use this `B` for any change in total thrust**, for example altitude control. That direction was never identified. The table above is the trustworthy part of `B`.

### The eigenvalues of A

| Eigenvalue (discrete time) | Count | Meaning |
|---|---|---|
| `1.0000` | 3 | **Marginal.** One per axis: the tilt angle simply accumulates rotation. A free rigid body has no restoring force, so these are expected, not a defect. |
| `0.9955` | 2 | Stable: roll and pitch rotation rates decay by drag. |
| `0.9977` | 1 | Stable: yaw rate decays by (weaker) drag. |

The rule for a discrete-time linear system is: **stable if every `|λ| < 1`, unstable if any `|λ| > 1`, and marginal if exactly on 1.** Here none exceed 1 (3 are on it by physics), so the identified model does not blow up.

---

## Part 9 · The results

All scores are on **held-out near-hover trials** unless stated.

| Question | Answer |
|---|---|
| How well does DMDc predict the next 2 ms? | **R² = 0.9926** on the rate-of-change, one-step **NRMSE = 0.0825** |
| What does NRMSE mean here? | Error relative to how much the state *changes* in one step. A model that does nothing scores exactly **1.0**, so 0.0825 is about 8 % of the change. |
| Does it blow up when predicting 11 s on its own? | **No: 0 of 16 rollouts diverged**, final error about 0.0037 |
| How does it compare with SINDy in DMDc's own regime? | SINDy **0.0789** versus DMDc **0.0825**: SINDy matches or slightly beats it |
| What happens outside hover? | It **falls to 0.70** (NRMSE on the full flight envelope), because the neglected nonlinear terms are no longer small |
| Number of parameters | `A` (36) + `B` (24) = **60**, all dense (versus SINDy's 44 non-zero coefficients) |

**The honest conclusion we state on every DMDc plot:** DMDc is **accurate near hover and valid only there.** The drop to 0.70 outside hover is not a bug. It is the expected, reported behaviour of a linear model, and it is the main reason SINDy was chosen.

---

## Part 10 · How DMDc relates to SINDy

A common confusion: *"Is DMDc for the linear case and SINDy for the nonlinear case, or the other way round?"* The correct statement:

| | **DMDc** | **SINDy** |
|---|---|---|
| Model form | Linear: `z[k+1] = A z[k] + B u[k]` | Nonlinear: `ż = Θ(z, u)·Ξ` with a library of terms (products, etc.) |
| Time | Discrete (one step to the next) | Continuous (the rate of change) |
| Sparsity | None: every entry of `A` and `B` is used | Yes: most coefficients forced to exactly zero |
| What it can represent | Straight-line (proportional) relationships only | Curved and product relationships, if they are in the library |
| Where it is valid | Near the point it was fitted (hover) | The full flight envelope tested |
| Cost | One matrix solve | A short loop of solves (STLSQ) |
| Role in this project | The linear baseline and an exact eigenvalue check | **The selected model** that sets the controller gains |

**They are not opposites of the same coin. They sit on one ladder.** If you give SINDy a library that contains *only* the linear terms and turn off its sparsity, you get essentially a continuous-time version of DMDc. SINDy is the generalisation: it adds the curved terms (`ωi·ωj`, `θi·ωj`) and the ability to throw away unneeded ones.

**Why neither "wins" automatically.** Near hover both give almost the same prediction, which is a good sign: the physics is linear there. They differ outside hover, where only the nonlinear model survives.

**Koopman and EDMD (not used here).** DMDc can be extended to nonlinear systems by first "lifting" the state into a bigger set of functions (such as its squares and products), then fitting a linear model in that larger space. This is called **extended DMD (EDMD)** and is linked to **Koopman operator** theory. We deliberately did **not** do this, so that the comparison stays clean. It is listed as future work.

---

## Part 11 · Honest limits

- **Near hover only.** Outside the 20° small-angle regime the one-step score falls from 0.0825 to 0.70.
- **`B` has an unidentified direction.** The four thrusts always summed to the same total in the near-hover data, so the "all propellers together" direction of `B` is numerically meaningless (very large values). The torque part of `B`, shown in Part 8, is correct and matches physics. A comment in `02_identification/dmdc_fit.py` describes the difference-from-hover step as removing the uniform-thrust direction. In fact it subtracts the constant hover thrust; the direction is still present, and it is the data (not the code) that leaves it unidentified.
- **Ideal data.** The recordings come from a simulator with exact measurements, so noise behaviour was tested separately (the noise ablation), not here.
- **No control action is taken from DMDc.** It is a baseline and an analysis tool in this project, not part of the flying controller.
- **The cross-check agrees to 0.0037, not to machine precision.** That is the maximum entry difference between `pydmd` and the direct solve on one rank-6 segment, which is small relative to the entries of `A`, but it is a measured tolerance, not an exact match.

---

## Part 12 · Questions a reviewer might ask

**What is DMDc in one sentence?**
It fits a linear model `next state = A·state + B·control` to recorded data by least squares, so it learns both how the system drifts by itself and how our actions change it.

**Why use it if the drone is nonlinear?**
Near hover the nonlinear terms are tiny (they are products of small quantities), so a linear model is a good local approximation, in the same sense that a curve looks straight under a microscope.

**Where is the maths of that approximation?**
A first-order Taylor expansion around hover, `δż = J·δz + Bc·δu`, discretised as `A = e^(J·dt) ≈ I + J·dt`, `B ≈ Bc·dt` (Part 6). The fitted `A` indeed looks like identity plus a small correction, and its top-right block is exactly `dt = 0.002`.

**How do you know the learned `A` and `B` are physically right?**
Two ways. The decay rates in `A` (about −2.3 and −1.15 per second) match what SINDy found independently. The torque part of `B` matches inertia-based predictions to a few percent, with the correct sign pattern for roll, pitch and yaw.

**Why is DMDc not the model that flies the drone?**
It is accurate only near hover (0.70 outside), while SINDy covers the full tested envelope and matches DMDc even inside DMDc's own regime (0.0789 versus 0.0825).

**What are the eigenvalues telling you?**
Three of six sit exactly on the unit circle, which is correct for a free rigid body (the tilt only accumulates rotation, with no restoring force). The other three are inside the circle (0.9955, 0.9955, 0.9977), meaning the rotation rates decay. So the identified model is stable in the sense that matters and does not blow up.

**What does "held out" mean?**
Whole separate recordings that the fit never saw. It is not a slice of one recording.

**What is the difference between DMD and DMDc?**
DMD models a system left alone. DMDc adds the control input `B·u`, which is essential for a drone because we push it continuously.

**Is there anything wrong with the raw `B`?**
Its all-propellers-together direction is unidentified and numerically huge. It does not affect the near-hover predictions or the torque entries, and it must not be reused for altitude control (Part 8).

**Did you try the nonlinear extension?**
No. EDMD/Koopman lifts were excluded on purpose, and are listed as future work.

---

## Part 13 · Code and reproduction

| What | Where |
|---|---|
| The DMDc model (fit, one-step prediction, eigenvalues, `pydmd` cross-check) | `02_identification/dmdc_fit.py` (class `DMDcModel`) |
| Where it is fitted and scored | `02_identification/run_identification.py` |
| Rollouts, eigenvalue plot, head-to-head with SINDy | `03_validation/run_validation.py` |
| The decision to use near hover only | `docs/TDD.md`, section 5 |
| The near-hover trial family and its 20° assertion | `01_simulation/run_excitation.py` (`assert_near_hover_bound`) |
| Plots | `results/plots_2d/eigenvalue_spectrum.png`, `results/plots_2d/head_to_head_near_hover.png` |

Reproduce the numbers:

```powershell
.\.venv\Scripts\python.exe 02_identification\run_identification.py   # fits SINDy and DMDc, prints R² and NRMSE
.\.venv\Scripts\python.exe 03_validation\run_validation.py           # rollouts, eigenvalues, head-to-head
```

**Reference:** J. L. Proctor, S. L. Brunton, J. N. Kutz, *Dynamic Mode Decomposition with Control*, SIAM Journal on Applied Dynamical Systems, 2016.
