"""
Dynamic Mode Decomposition with Control (DMDc) for 6-DOF Quadrotor Dynamics.
Extracts input-output state transition matrices (A, B), continuous eigenvalue spectrum,
stability poles, and coherent spatiotemporal dynamic modes from flight data.
"""

import numpy as np
from scipy import linalg


class DMDcModel:
    def __init__(self, rank=None, energy_threshold=0.999):
        self.rank = rank
        self.energy_threshold = energy_threshold
        self.dt = None
        self.A = None  # Discrete State Matrix [n, n]
        self.B = None  # Discrete Input Matrix [n, q]
        self.A_cont = None  # Continuous State Matrix (ln(A)/dt)
        self.B_cont = None
        self.eigenvalues = None  # Discrete eigenvalues
        self.continuous_poles = None  # Continuous poles (rad/s)
        self.damping_ratios = None
        self.natural_frequencies = None
        self.modes = None
        self.state_names = []
        self.input_names = []
        self.r2_scores = {}
        self.rmse_scores = {}

    def fit(self, X, U, dt, state_names=None, input_names=None):
        """
        Fits DMDc matrices A and B from state snapshots X and control snapshots U.
        X: [N, n] (states across time)
        U: [N, q] (controls across time)
        dt: sample period in seconds
        """
        self.dt = dt
        N, n = X.shape
        _, q = U.shape
        self.state_names = state_names or [f"x{i}" for i in range(n)]
        self.input_names = input_names or [f"u{i}" for i in range(q)]

        # Construct snapshot matrices (transposed to [states, time_samples])
        X1 = X[:-1].T  # [n, N-1]
        X2 = X[1:].T   # [n, N-1]
        Upsilon = U[:-1].T  # [q, N-1]

        # Stack states and inputs: Omega = [X1; Upsilon]
        Omega = np.vstack([X1, Upsilon])  # [n + q, N-1]

        # SVD of Omega
        U_omega, s_omega, Vh_omega = linalg.svd(Omega, full_matrices=False)
        V_omega = Vh_omega.T.conj()

        # Determine Truncation Rank
        if self.rank is None:
            cum_energy = np.cumsum(s_omega**2) / np.sum(s_omega**2)
            r = np.searchsorted(cum_energy, self.energy_threshold) + 1
            r = max(2, min(r, len(s_omega)))
        else:
            r = min(self.rank, len(s_omega))

        # Truncate
        U_r = U_omega[:, :r]
        s_r = s_omega[:r]
        V_r = V_omega[:, :r]

        # Split U_r into state part and control part
        U1 = U_r[:n, :]  # [n, r]
        U2 = U_r[n:, :]  # [q, r]

        # Compute Total Dynamic Operator G = [A, B] = X2 * V_r * diag(1/s_r) * U_r^*
        s_inv = np.diag(1.0 / s_r)
        G = X2 @ V_r @ s_inv @ U_r.T.conj()  # [n, n + q]

        self.A = G[:, :n].real
        self.B = G[:, n:].real

        # Continuous-time state matrix approximation
        try:
            self.A_cont = (linalg.logm(self.A) / dt).real
            self.B_cont = (np.linalg.pinv(self.A - np.eye(n)) @ self.A_cont @ self.B).real
        except Exception:
            self.A_cont = (self.A - np.eye(n)) / dt
            self.B_cont = self.B / dt

        # Eigenvalues and Dynamic Modes
        eigvals, eigvecs = np.linalg.eig(self.A)
        self.eigenvalues = eigvals
        self.modes = eigvecs

        # Continuous Poles, Natural Frequencies, Damping Ratios
        cont_poles = np.log(eigvals.astype(complex)) / dt
        self.continuous_poles = cont_poles
        omega_n = np.abs(cont_poles)
        self.natural_frequencies = omega_n
        with np.errstate(divide='ignore', invalid='ignore'):
            zeta = -cont_poles.real / np.where(omega_n == 0, 1e-9, omega_n)
        self.damping_ratios = np.clip(zeta, -1.0, 1.0)

        # Validation Metrics across training trajectory
        X_pred = np.zeros_like(X)
        X_pred[0] = X[0]
        for k in range(N - 1):
            X_pred[k + 1] = self.A @ X_pred[k] + self.B @ U[k]

        for j, name in enumerate(self.state_names):
            y_true = X[:, j]
            y_pred = X_pred[:, j]
            ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
            ss_res = np.sum((y_true - y_pred) ** 2)
            r2 = 1.0 - (ss_res / (ss_tot + 1e-9))
            rmse = np.sqrt(np.mean((y_true - y_pred) ** 2))
            self.r2_scores[name] = max(0.0, float(r2))
            self.rmse_scores[name] = float(rmse)

        return self

    def simulate(self, x0, U_seq):
        """
        Simulates the identified linear dynamic model: x_{k+1} = A*x_k + B*u_k
        """
        N = U_seq.shape[0]
        n = len(x0)
        X_sim = np.zeros((N, n))
        X_sim[0] = x0
        
        for k in range(N - 1):
            X_sim[k + 1] = self.A @ X_sim[k] + self.B @ U_seq[k]

        return X_sim

    def get_summary_report(self):
        """
        Returns a formatted analytical summary of the identified DMDc model.
        """
        lines = []
        lines.append("=" * 70)
        lines.append(">>> DMDc (DYNAMIC MODE DECOMPOSITION WITH CONTROL) REPORT")
        lines.append("=" * 70)
        lines.append("")
        lines.append(f"Identified Discrete System Matrix A ({self.A.shape[0]}x{self.A.shape[1]}):")
        lines.append(np.array2string(self.A, precision=4, suppress_small=True))
        lines.append("")
        lines.append(f"Identified Control Input Matrix B ({self.B.shape[0]}x{self.B.shape[1]}):")
        lines.append(np.array2string(self.B, precision=4, suppress_small=True))
        lines.append("")
        lines.append("Dynamic Mode Spectrum & Stability Analysis:")
        lines.append(f"{'Mode #':<8} {'Discrete Eigenvalue (z)':<28} {'Continuous Pole (s)':<26} {'Freq (Hz)':<12} {'Damping ζ'}")
        lines.append("-" * 88)

        for i, (z, s, wn, zt) in enumerate(zip(self.eigenvalues, self.continuous_poles, self.natural_frequencies, self.damping_ratios)):
            z_str = f"{z.real:+.4f} {z.imag:+.4f}j (|z|={abs(z):.4f})"
            s_str = f"{s.real:+.3f} {s.imag:+.3f}j"
            freq_hz = wn / (2.0 * np.pi)
            lines.append(f"Mode {i+1:<3} {z_str:<28} {s_str:<26} {freq_hz:<12.3f} {zt:+.4f}")

        lines.append("")
        lines.append("=" * 70)
        return "\n".join(lines)
