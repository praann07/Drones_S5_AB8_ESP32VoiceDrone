# B-Spline Trajectory Planning for a Drone

## 1. Introduction

A **B-spline** is a mathematical curve used to create a smooth path from a set of control points. In a drone project, it can be used to generate a smooth 3D trajectory between selected waypoints.

A B-spline defines the desired path. A separate flight controller is still needed to make the simulated or physical drone follow that path. A B-spline also does not automatically avoid obstacles; collision and clearance checks must be performed separately.

## 2. Important terminology

- **Control point** (`P_i`): A geometric point that influences the shape of the curve.
- **Knot vector** (`U`): An ordered sequence of parameter values that organises the curve's parameter intervals.
- **Curve parameter** (`u`): A value used to evaluate a location along the curve.
- **Degree** (`p`): The polynomial degree of the B-spline. For example, a cubic B-spline has `p = 3`.
- **Basis function** (`N_{i,p}(u)`): A weight that determines how much a control point contributes to the curve at parameter `u`.
- **Blending**: Combining two points in weighted proportions to calculate a new point between them.
- **de Boor's algorithm**: A recursive algorithm that evaluates a point on a B-spline by repeatedly blending intermediate points.

**Important distinction:** Control points are geometric positions; knots are numerical parameter values. They are not the same thing.

## 3. B-spline curve equation

The main B-spline equation is:

\[
\boxed{\mathbf{C}(u)=\sum_{i=0}^{n}N_{i,p}(u)\mathbf{P}_i}
\]

Where:

- `C(u)` is the point on the curve at parameter `u`.
- `P_i` is the `i`-th control point.
- `N_{i,p}(u)` is the basis-function weight for control point `i`.
- `p` is the degree of the curve.
- `n + 1` is the number of control points.

For a 3D drone trajectory, each control point has three coordinates:

\[
\mathbf{P}_i=\begin{bmatrix}x_i\\y_i\\z_i\end{bmatrix}
\]

Therefore:

\[
\mathbf{C}(u)=
\begin{bmatrix}
\sum_{i=0}^{n}N_{i,p}(u)x_i\\
\sum_{i=0}^{n}N_{i,p}(u)y_i\\
\sum_{i=0}^{n}N_{i,p}(u)z_i
\end{bmatrix}
\]

This gives the desired drone position `(x(u), y(u), z(u))` for a given parameter `u`.

### How to understand the equation

Each control point contributes to the curve according to its basis-function weight. The equation adds all these weighted contributions to calculate the curve position.

## 4. Cox–de Boor basis functions

Basis functions are calculated recursively, starting with degree zero.

### 4.1 Degree-zero basis function

\[
N_{i,0}(u)=
\begin{cases}
1, & u_i\leq u<u_{i+1}\\
0, & \text{otherwise}
\end{cases}
\]

At degree zero, the basis function is active only within its knot interval.

- `u` is the parameter being evaluated.
- `u_i` and `u_{i+1}` are consecutive knot values.
- `i` is the basis-function index.

### 4.2 Higher-degree basis function

For degree `p > 0`, the Cox–de Boor recursion is:

\[
\boxed{
\begin{aligned}
N_{i,p}(u)={}&
\frac{u-u_i}{u_{i+p}-u_i}N_{i,p-1}(u)\\
&+\frac{u_{i+p+1}-u}{u_{i+p+1}-u_{i+1}}
N_{i+1,p-1}(u)
\end{aligned}
}
\]

A term with a zero denominator is defined as zero.

Define the weights:

\[
a=\frac{u-u_i}{u_{i+p}-u_i},
\qquad
b=\frac{u_{i+p+1}-u}{u_{i+p+1}-u_{i+1}}
\]

Then the recurrence can be written as:

\[
N_{i,p}(u)=aN_{i,p-1}(u)+bN_{i+1,p-1}(u)
\]

**In simple terms:** Start with simple interval functions and combine lower-degree basis functions recursively to obtain higher-degree basis functions. These weights determine how the control points influence the curve.

## 5. Knot vector

A knot vector is an ordered sequence:

\[
U=[u_0,u_1,\ldots,u_m]
\]

The index relationship is:

\[
\boxed{m=n+p+1}
\]

Here, `m` is the index of the last knot, so there are `m + 1` knots in total. There are `n + 1` control points and the curve degree is `p`.

### Example: clamped cubic B-spline

Suppose there are six control points and the degree is three:

\[
n=5,\qquad p=3
\]

Then:

\[
m=5+3+1=9
\]

There are ten knots. One possible clamped knot vector is:

\[
U=[0,0,0,0,1,2,3,3,3,3]
\]

For a clamped cubic B-spline, the first and last knot values each repeat `p + 1 = 4` times. This makes the curve meet the first and last control points.

The positive-length knot spans in this example are `[0, 1]`, `[1, 2]`, and `[2, 3]`. Repeated knot values do not create extra positive-length spans.

## 6. What does blending mean?

**Blending means combining two points in weighted proportions to calculate a new point.**

For points `A` and `B`, linear interpolation is:

\[
\boxed{P=(1-\alpha)A+\alpha B}
\]

where `alpha` is the blending weight.

Suppose `A = 0` and `B = 10`:

- If `alpha = 0`, then `P = 0`, so the result is `A`.
- If `alpha = 0.5`, then `P = 5`, halfway between `A` and `B`.
- If `alpha = 1`, then `P = 10`, so the result is `B`.

The two weights add to one:

\[
(1-\alpha)+\alpha=1
\]

When `0 <= alpha <= 1`, both weights are non-negative, and the result lies between the two points (or on the line segment between two vector points).

### Why is the blending weight between zero and one?

In de Boor's algorithm, the parameter `u` is selected within the relevant knot span. For a positive denominator, the numerator in the blending-weight formula is non-negative and no greater than the denominator. Thus:

\[
0\leq\alpha_{i,r}\leq1
\]

This guarantee applies when evaluating in the correct knot span; arbitrary values of `u` do not necessarily give weights in this range.

## 7. de Boor's algorithm

de Boor's algorithm evaluates a B-spline point by recursively blending neighbouring intermediate points.

### 7.1 Initial values

The relevant control points are used as the initial intermediate points:

\[
d_i^{(0)}=\mathbf{P}_i
\]

### 7.2 Recurrence formula

\[
\boxed{
d_i^{(r)}(u)=
(1-\alpha_{i,r})d_{i-1}^{(r-1)}(u)
+\alpha_{i,r}d_i^{(r-1)}(u)
}
\]

The blending weight is:

\[
\boxed{
\alpha_{i,r}=
\frac{u-u_i}{u_{i+p-r+1}-u_i}
}
\]

Terms:

- `d_i^(r)(u)` is an intermediate point at recursion level `r`.
- `i` identifies the point index.
- `r` is the recursion level, from `1` through `p`.
- `alpha_(i,r)` is the knot-dependent blending weight.
- `p` is the B-spline degree.
- `u` is the parameter at which the curve is evaluated.

The algorithm repeatedly calculates intermediate points:

\[
d^{(0)}\rightarrow d^{(1)}\rightarrow d^{(2)}
\rightarrow\cdots\rightarrow d^{(p)}
\]

For a cubic B-spline, there are three recursion levels. The final point at the appropriate index for the selected knot span is the evaluated curve point.

### 7.3 Small blending example

Suppose the two points being blended are `A = 0` and `B = 10`, with `alpha = 0.5`:

\[
P=(1-0.5)(0)+0.5(10)=5
\]

The new intermediate point is `5`. de Boor's algorithm performs repeated, knot-dependent blends of this type to evaluate a B-spline curve.

### Meaning of `d_i^(r)`

- `P_i`: an original control point.
- `d_i^(1)`: an intermediate point after the first blending level.
- `d_i^(2)`: an intermediate point after the second blending level.
- `d_i^(p)`: the final point at the final recursion level, using the appropriate index for the selected knot span.

**Drone application:** de Boor's algorithm can calculate the desired positions along the planned trajectory.

## 8. First derivative of a B-spline

The first derivative describes how the curve position changes as parameter `u` changes.

\[
\boxed{
\mathbf{C}'(u)=\sum_{i=0}^{n}N'_{i,p}(u)\mathbf{P}_i
}
\]

The derivative of a basis function can be calculated using:

\[
\boxed{
N'_{i,p}(u)=
\frac{p}{u_{i+p}-u_i}N_{i,p-1}(u)
-\frac{p}{u_{i+p+1}-u_{i+1}}N_{i+1,p-1}(u)
}
\]

A term with a zero denominator is defined as zero.

### Derivation

Start with:

\[
\mathbf{C}(u)=\sum_{i=0}^{n}N_{i,p}(u)\mathbf{P}_i
\]

Differentiate with respect to `u`. Since control points are fixed:

\[
\frac{d\mathbf{C}(u)}{du}
=\sum_{i=0}^{n}\frac{dN_{i,p}(u)}{du}\mathbf{P}_i
\]

Therefore:

\[
\mathbf{C}'(u)=\sum_{i=0}^{n}N'_{i,p}(u)\mathbf{P}_i
\]

`C'(u)` is the tangent vector to the curve. It describes direction and change with respect to the curve parameter; it is not necessarily physical velocity in metres per second.

## 9. Second derivative of a B-spline

The second derivative is:

\[
\boxed{
\mathbf{C}''(u)=\sum_{i=0}^{n}N''_{i,p}(u)\mathbf{P}_i
}
\]

### Derivation

Differentiate the first derivative:

\[
\mathbf{C}'(u)=\sum_{i=0}^{n}N'_{i,p}(u)\mathbf{P}_i
\]

Taking the derivative again gives:

\[
\frac{d\mathbf{C}'(u)}{du}
=\sum_{i=0}^{n}N''_{i,p}(u)\mathbf{P}_i
\]

Hence:

\[
\mathbf{C}''(u)=\frac{d^2\mathbf{C}(u)}{du^2}
\]

`C''(u)` helps analyse how the curve bends. Physical acceleration also depends on how quickly the drone moves along the curve.

## 10. Physical velocity and acceleration

To express motion over time, let the curve parameter depend on time: `u = u(t)`.

### 10.1 Velocity

\[
\boxed{\mathbf{v}(t)=\mathbf{C}'(u)\dot{u}}
\]

Using the chain rule:

\[
\mathbf{v}(t)=\frac{d\mathbf{C}}{dt}
=\frac{d\mathbf{C}}{du}\frac{du}{dt}
\]

Therefore:

\[
\mathbf{v}(t)=\mathbf{C}'(u)\dot{u}
\]

### 10.2 Acceleration

\[
\boxed{
\mathbf{a}(t)=
\mathbf{C}''(u)\dot{u}^{\,2}
+\mathbf{C}'(u)\ddot{u}
}
\]

### Derivation

Differentiate velocity:

\[
\mathbf{a}(t)=\frac{d}{dt}
\left[\mathbf{C}'(u)\dot{u}\right]
\]

Applying the product rule and chain rule:

\[
\mathbf{a}(t)=
\frac{d\mathbf{C}'(u)}{dt}\dot{u}
+\mathbf{C}'(u)\ddot{u}
\]

Since:

\[
\frac{d\mathbf{C}'(u)}{dt}
=\mathbf{C}''(u)\dot{u}
\]

we obtain:

\[
\mathbf{a}(t)=
\mathbf{C}''(u)\dot{u}^{\,2}
+\mathbf{C}'(u)\ddot{u}
\]

Terms:

- `v(t)`: physical velocity, usually measured in metres per second.
- `a(t)`: physical acceleration, usually measured in metres per second squared.
- `du/dt` or `u_dot`: rate at which the curve parameter changes.
- `d²u/dt²` or `u_ddot`: rate of change of the parameter rate.

**Drone application:** These equations connect the geometric trajectory to a time-based motion plan.

## 11. Curvature and turning limits

Curvature measures how sharply a trajectory bends.

For a 3D curve:

\[
\boxed{
\kappa(u)=
\frac{\|\mathbf{C}'(u)\times\mathbf{C}''(u)\|}
{\|\mathbf{C}'(u)\|^3}
}
\]

Terms:

- `C'(u)`: first derivative, or tangent vector.
- `C''(u)`: second derivative.
- `×`: vector cross product.
- `||...||`: vector magnitude.
- `kappa(u)`: curvature.

At physical speed `v`, the normal (lateral) acceleration required to follow a curved path is:

\[
\boxed{a_{\text{normal}}=v^2\kappa}
\]

A tighter turn has larger curvature. If the drone travels too quickly around a tight curve, the required acceleration may exceed its limits. The trajectory or speed profile can be adjusted accordingly.

## 12. Obstacle-clearance constraint

A B-spline does not automatically avoid obstacles. The generated path must be checked for collision and clearance.

For a spherical obstacle:

\[
\boxed{\|\mathbf{C}(u)-\mathbf{O}\|\geq R_{\text{safe}}}
\]

Terms:

- `C(u)`: drone's planned position.
- `O`: centre of the obstacle.
- `||C(u) - O||`: distance between the drone reference point and the obstacle centre.
- `R_safe`: obstacle radius plus the drone's effective radius and a safety margin.

The distance between two 3D points is:

\[
d=\sqrt{(x-x_o)^2+(y-y_o)^2+(z-z_o)^2}
\]

Substituting the trajectory coordinates gives:

\[
d(u)=\|\mathbf{C}(u)-\mathbf{O}\|
\]

To maintain clearance:

\[
d(u)\geq R_{\text{safe}}
\]

The trajectory should be checked across its length, not only at its waypoints. If clearance is insufficient, the path must be adjusted or replanned.

## 13. How the formulations fit together

1. **Choose control points:** Define the desired route.
2. **Construct the B-spline:** Use the curve equation and basis functions to create a smooth trajectory.
3. **Evaluate positions:** Use de Boor's algorithm to calculate positions on the curve.
4. **Analyse motion:** Use derivatives to obtain curve directions and, with a time parameterisation, velocity and acceleration.
5. **Check curvature and clearance:** Ensure the path respects motion limits and avoids obstacles.
6. **Follow the trajectory:** A separate drone controller tracks the desired positions.

## 14. Quick revision table

| Formulation | What it calculates | Why it matters for the drone |
|---|---|---|
| B-spline curve equation | Position on the curve | Generates the 3D path |
| Cox–de Boor recursion | Basis-function weights | Constructs the curve |
| Knot vector | Parameter intervals | Organises curve evaluation |
| de Boor's algorithm | A specific curve point | Calculates trajectory positions |
| First derivative | Change with respect to `u` | Gives the tangent direction |
| Second derivative | Second change with respect to `u` | Helps analyse bending |
| Time derivatives | Velocity and acceleration | Supports motion planning |
| Curvature | Sharpness of a turn | Helps check motion limits |
| Clearance constraint | Distance from obstacles | Helps check path safety |

## 15. Short explanation for a project review

“Our drone project uses B-spline formulations to generate a smooth 3D trajectory from control points. The Cox–de Boor recursion calculates basis functions, while de Boor's algorithm evaluates positions on the curve. The first and second derivatives help us analyse trajectory direction and motion. Curvature and obstacle-clearance checks help determine whether the planned trajectory is suitable. A separate flight controller is responsible for making the drone follow the trajectory.”

---

## References for further reading

- Carl de Boor, *A Practical Guide to Splines*, Springer.
- Les Piegl and Wayne Tiller, *The NURBS Book*, Springer.
- B-spline and de Boor algorithm documentation from established numerical computing libraries.
