"""
Sparse Identification of Nonlinear Dynamics (SINDy) for 6-DOF Quadrotor Dynamics.
Discovers governing differential equations (inertia ratios, aerodynamic drag, control derivatives)
directly from flight telemetry data using Sequentially Thresholded Least Squares (STLSQ).
"""

import numpy as np


class SINDyModel:
    def __init__(self, threshold=0.015, max_iter=15, alpha_ridge=1e-5):
        self.threshold = threshold
        self.max_iter = max_iter
        self.alpha_ridge = alpha_ridge
        self.feature_names = []
        self.state_names = []
        self.xi = None  # Coefficient matrix [n_features, n_targets]
        self.r2_scores = {}
        self.rmse_scores = {}

    def _build_library(self, X, U, wind=None):
        """
        Constructs the candidate feature library Theta(X, U) tailored to quadrotor physics.
        X: [N, n_states] -> [p, q, r, roll, pitch, yaw, vn, ve, vup]
        U: [N, 4] -> [u1, u2, u3, u4]
        """
        N = X.shape[0]
        feats = []
        names = []

        # 1. Constant term (Bias)
        feats.append(np.ones((N, 1)))
        names.append("1")

        # Extract states
        p = X[:, 0:1]
        q = X[:, 1:2]
        r = X[:, 2:3]
        roll = X[:, 3:4]
        pitch = X[:, 4:5]
        yaw = X[:, 5:6]
        vn = X[:, 6:7]
        ve = X[:, 7:8]
        vup = X[:, 8:9]

        # Extract motor inputs
        u1 = U[:, 0:1]
        u2 = U[:, 1:2]
        u3 = U[:, 2:3]
        u4 = U[:, 3:4]

        # Control mixtures (Roll, Pitch, Yaw moments & Thrust)
        u_thrust = u1 + u2 + u3 + u4
        u_roll = u1 - u2 - u3 + u4
        u_pitch = u1 - u2 + u3 - u4
        u_yaw = -u1 - u2 + u3 + u4

        # 2. Linear State & Control Features (Coordinate Invariant)
        linear_dict = {
            "p": p, "q": q, "r": r,
            "roll": roll, "pitch": pitch,
            "vn": vn, "ve": ve, "vup": vup,
            "u_thrust": u_thrust, "u_roll": u_roll, "u_pitch": u_pitch, "u_yaw": u_yaw,
            "u1": u1, "u2": u2, "u3": u3, "u4": u4
        }
        for k, v in linear_dict.items():
            feats.append(v)
            names.append(k)

        # 3. Gyroscopic Coupling terms (Euler Equations: (Iy - Iz)/Ix * q*r)
        feats.append(q * r)
        names.append("q*r")
        feats.append(p * r)
        names.append("p*r")
        feats.append(p * q)
        names.append("p*q")

        # 4. Aerodynamic Damping / Drag terms (v * |v|, p * |p|, etc.)
        feats.append(p * np.abs(p))
        names.append("p*|p|")
        feats.append(q * np.abs(q))
        names.append("q*|q|")
        feats.append(r * np.abs(r))
        names.append("r*|r|")
        feats.append(vn * np.abs(vn))
        names.append("vn*|vn|")
        feats.append(ve * np.abs(ve))
        names.append("ve*|ve|")
        feats.append(vup * np.abs(vup))
        names.append("vup*|vup|")

        # 5. Trigonometric Attitude Projection (Gravity & Thrust orientation)
        feats.append(np.sin(roll))
        names.append("sin(roll)")
        feats.append(np.sin(pitch))
        names.append("sin(pitch)")
        feats.append(np.sin(yaw))
        names.append("sin(yaw)")
        feats.append(np.cos(yaw))
        names.append("cos(yaw)")
        feats.append(np.cos(roll) * np.cos(pitch))
        names.append("cos(roll)*cos(pitch)")
        feats.append(np.sin(pitch) * u_thrust)
        names.append("sin(pitch)*u_thrust")
        feats.append(np.sin(roll) * u_thrust)
        names.append("sin(roll)*u_thrust")
        feats.append(np.cos(roll) * np.cos(pitch) * u_thrust)
        names.append("cos(roll)*cos(pitch)*u_thrust")

        # 6. Euler Angle Kinematics basis (q*cos(roll), r*sin(roll), r/cos(pitch))
        feats.append(q * np.cos(roll))
        names.append("q*cos(roll)")
        feats.append(r * np.sin(roll))
        names.append("r*sin(roll)")
        feats.append(r * np.cos(roll))
        names.append("r*cos(roll)")
        feats.append(p + q * np.sin(roll) * np.tan(np.clip(pitch, -1.2, 1.2)))
        names.append("p+q*sin(roll)*tan(pitch)")

        Theta = np.hstack(feats)
        self.feature_names = names
        return Theta

    def _stlsq(self, Theta, X_dot):
        """
        Sequentially Thresholded Least Squares (STLSQ) optimization.
        Solves: min ||Theta * Xi - X_dot||_2 + alpha * ||Xi||_2 s.t. Sparsity
        """
        n_features = Theta.shape[1]
        n_targets = X_dot.shape[1]

        # Initial Ridge Regression
        I = np.eye(n_features)
        Xi = np.linalg.solve(Theta.T @ Theta + self.alpha_ridge * I, Theta.T @ X_dot)

        for _ in range(self.max_iter):
            small_inds = np.abs(Xi) < self.threshold
            Xi[small_inds] = 0.0

            # Re-solve for each target on non-zero support
            for j in range(n_targets):
                big_inds = ~small_inds[:, j]
                if np.sum(big_inds) == 0:
                    continue
                Theta_active = Theta[:, big_inds]
                I_active = np.eye(np.sum(big_inds))
                Xi[big_inds, j] = np.linalg.solve(
                    Theta_active.T @ Theta_active + self.alpha_ridge * I_active,
                    Theta_active.T @ X_dot[:, j]
                )

        return Xi

    def fit(self, X, U, dt, state_names=None):
        """
        Fits the SINDy model to flight data (X, U).
        X: [N, n_states]
        U: [N, n_inputs]
        dt: float time-step (e.g. 0.004 s)
        """
        self.state_names = state_names or [f"x{i}" for i in range(X.shape[1])]

        # Compute numerical time derivative dX/dt using central differences
        X_dot = np.zeros_like(X)
        X_dot[1:-1] = (X[2:] - X[:-2]) / (2.0 * dt)
        X_dot[0] = (X[1] - X[0]) / dt
        X_dot[-1] = (X[-1] - X[-2]) / dt

        Theta = self._build_library(X, U)
        self.xi = self._stlsq(Theta, X_dot)

        # Compute Metrics
        X_dot_pred = Theta @ self.xi
        for j, name in enumerate(self.state_names):
            y_true = X_dot[:, j]
            y_pred = X_dot_pred[:, j]
            ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
            ss_res = np.sum((y_true - y_pred) ** 2)
            r2 = 1.0 - (ss_res / (ss_tot + 1e-9))
            rmse = np.sqrt(np.mean((y_true - y_pred) ** 2))
            self.r2_scores[name] = max(0.0, float(r2))
            self.rmse_scores[name] = float(rmse)

        return self

    def predict_derivative(self, x, u):
        """
        Evaluates dX/dt given current state x and control input u.
        """
        x_2d = np.atleast_2d(x)
        u_2d = np.atleast_2d(u)
        Theta = self._build_library(x_2d, u_2d)
        return (Theta @ self.xi).squeeze()

    def simulate(self, x0, U_seq, dt):
        """
        Numerically stable simulation of the discovered SINDy dynamic model.
        """
        N = U_seq.shape[0]
        n = len(x0)
        X_sim = np.zeros((N, n))
        X_sim[0] = x0
        
        # Physical quadrotor bounds: [p, q, r, roll, pitch, yaw, vn, ve, vup]
        lower_b = np.array([-15.0, -15.0, -15.0, -1.2, -1.2, -10.0, -25.0, -25.0, -25.0])
        upper_b = np.array([15.0, 15.0, 15.0, 1.2, 1.2, 10.0, 25.0, 25.0, 25.0])
        
        for k in range(N - 1):
            x_k = np.clip(X_sim[k], lower_b, upper_b)
            u_k = U_seq[k]
            
            # Predict and clamp derivative
            dxdt = self.predict_derivative(x_k, u_k)
            dxdt = np.nan_to_num(dxdt, nan=0.0, posinf=50.0, neginf=-50.0)
            dxdt = np.clip(dxdt, -80.0, 80.0)
            
            x_next = x_k + dt * dxdt
            X_sim[k + 1] = np.clip(x_next, lower_b, upper_b)

        return X_sim

    def get_equations_report(self):
        """
        Generates readable ASCII and LaTeX representations of discovered ODEs.
        """
        lines = []
        lines.append("=" * 70)
        lines.append(">>> SINDy DISCOVERED GOVERNING DIFFERENTIAL EQUATIONS")
        lines.append("=" * 70)
        lines.append("")

        for j, state_name in enumerate(self.state_names):
            coeffs = self.xi[:, j]
            terms = []
            for i, c in enumerate(coeffs):
                if abs(c) > 1e-4:
                    feat = self.feature_names[i]
                    if feat == "1":
                        terms.append(f"{c:+.4f}")
                    else:
                        terms.append(f"{c:+.4f} * {feat}")
            
            eq_str = " ".join(terms) if terms else "0.0000"
            r2 = self.r2_scores.get(state_name, 0.0)
            rmse = self.rmse_scores.get(state_name, 0.0)
            lines.append(f"d({state_name})/dt = {eq_str}")
            lines.append(f"  └── R² Score: {r2:.4f} | Derivative RMSE: {rmse:.5f}")
            lines.append("")

        lines.append("=" * 70)
        return "\n".join(lines)
