"""
ekf.py  (improved)
==================
EKF for BFMC — designed to be used as the *local* tracker
after the particle filter has converged to a good hypothesis.

Changes vs original
-------------------
* Joseph-form covariance update   → numerically stable
* Chi-squared gating per landmark → principled outlier rejection
* Separate update_pose() to accept an external pose seed (from PF)
* Heading-aware graph snapping    → doesn't fight the road direction
* Velocity state removed          → simpler 3-DOF model [x, y, θ]
"""

from __future__ import annotations

import numpy as np
from typing import Optional, Tuple, List


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def normalize_angle(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


# ---------------------------------------------------------------------------
# EKF
# ---------------------------------------------------------------------------

class EKF:
    """
    State: x = [x, y, θ]   (position + heading)

    Prediction uses a unicycle model:
        x'  = x + v·cos(θ)·dt
        y'  = y + v·sin(θ)·dt
        θ'  = θ + ω·dt
    """

    def __init__(
        self,
        init_x: float = 0.0,
        init_y: float = 0.0,
        init_theta: float = 0.0,
        init_sigma_xy: float = 0.5,
        init_sigma_theta: float = 0.5,
    ):
        self.x = np.array([init_x, init_y, init_theta], dtype=float)

        self.P = np.diag([
            init_sigma_xy**2,
            init_sigma_xy**2,
            init_sigma_theta**2,
        ])

        # Process noise (tune per-vehicle)
        self._Q_base = np.diag([0.02**2, 0.02**2, 0.01**2])

        # Measurement noise
        self._R_bearing_range = np.diag([0.05**2, 0.10**2])

        # Graph correction noise (soft constraint)
        self._R_graph = np.diag([0.08**2, 0.08**2])

        # Chi-squared gate: 95 % for 2 DOF (bearing + range)
        self._chi2_gate = 5.991
        # Chi-squared gate for graph (2 DOF)
        self._chi2_graph_gate = 5.991

    # -----------------------------------------------------------------------
    # External pose seed (from particle filter)
    # -----------------------------------------------------------------------

    def seed_from_particle_filter(
        self,
        x: float,
        y: float,
        theta: float,
        cov_3x3: np.ndarray,
    ) -> None:
        """
        Hard-reset the EKF to the PF's best estimate.
        Called once when the PF declares convergence.
        """
        self.x = np.array([x, y, theta], dtype=float)
        self.P = cov_3x3.copy()

    # -----------------------------------------------------------------------
    # Predict
    # -----------------------------------------------------------------------

    def predict(
        self,
        v: float,
        omega: float,
        dt: float,
        extra_sigma_v: float = 0.05,
        extra_sigma_omega: float = 0.05,
    ) -> None:
        """
        Unicycle prediction with velocity-scaled process noise.

        extra_sigma_v / extra_sigma_omega capture residual IMU bias
        uncertainty on top of the base process noise.
        """
        th = self.x[2]

        # Linearise motion model
        F = np.array([
            [1, 0, -v * np.sin(th) * dt],
            [0, 1,  v * np.cos(th) * dt],
            [0, 0,  1],
        ])

        # Propagate state
        self.x[0] += v * np.cos(th) * dt
        self.x[1] += v * np.sin(th) * dt
        self.x[2]  = normalize_angle(self.x[2] + omega * dt)

        # Velocity-adaptive process noise
        Q = self._Q_base.copy()
        Q[0, 0] += (extra_sigma_v * np.cos(th) * dt)**2
        Q[1, 1] += (extra_sigma_v * np.sin(th) * dt)**2
        Q[2, 2] += (extra_sigma_omega * dt)**2

        self.P = F @ self.P @ F.T + Q

    # -----------------------------------------------------------------------
    # Measurement update: bearing + range
    # -----------------------------------------------------------------------

    def update_bearing_range(
        self,
        z: np.ndarray,          # [bearing, range]
        landmark: dict,         # {"x": ..., "y": ...}
    ) -> bool:
        """
        EKF update for a single (bearing, range) observation.
        Returns True if the update passed the chi-squared gate.
        """
        dx = landmark["x"] - self.x[0]
        dy = landmark["y"] - self.x[1]
        pred_range   = np.hypot(dx, dy)
        pred_bearing = normalize_angle(np.arctan2(dy, dx) - self.x[2])

        H = self.compute_measurement_jacobian(landmark)

        # Innovation
        innov = np.array([
            normalize_angle(z[0] - pred_bearing),
            z[1] - pred_range,
        ])

        # Innovation covariance
        S = H @ self.P @ H.T + self._R_bearing_range

        # Chi-squared gate
        maha2 = innov.T @ np.linalg.inv(S) @ innov
        if maha2 > self._chi2_gate:
            return False   # outlier, skip

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        self.x    = self.x + K @ innov
        self.x[2] = normalize_angle(self.x[2])

        # Joseph-form covariance update (numerically stable)
        I_KH = np.eye(3) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self._R_bearing_range @ K.T

        return True

    # -----------------------------------------------------------------------
    # Measurement update: graph projection
    # -----------------------------------------------------------------------

    def update_graph(
        self,
        proj_x: float,
        proj_y: float,
        heading_from_edge: Optional[float] = None,
    ) -> bool:
        """
        Soft constraint that pulls the estimate onto the road graph.

        Parameters
        ----------
        proj_x / proj_y  : projection point on nearest edge
        heading_from_edge: if provided, also correct heading toward road direction
        """
        # --- Position correction (2-DOF) ---
        H_pos = np.array([
            [1, 0, 0],
            [0, 1, 0],
        ])
        innov_pos = np.array([proj_x - self.x[0], proj_y - self.x[1]])
        S_pos = H_pos @ self.P @ H_pos.T + self._R_graph
        maha2 = innov_pos.T @ np.linalg.inv(S_pos) @ innov_pos

        if maha2 > self._chi2_graph_gate:
            return False   # too far from road, skip

        K_pos = self.P @ H_pos.T @ np.linalg.inv(S_pos)
        self.x    = self.x + K_pos @ innov_pos
        self.x[2] = normalize_angle(self.x[2])
        I_KH = np.eye(3) - K_pos @ H_pos
        self.P = I_KH @ self.P @ I_KH.T + K_pos @ self._R_graph @ K_pos.T

        # --- Optional heading correction ---
        if heading_from_edge is not None:
            H_h = np.array([[0, 0, 1]])
            innov_h = normalize_angle(heading_from_edge - self.x[2])
            R_h = np.array([[0.15**2]])   # heading correction is less trusted
            S_h = H_h @ self.P @ H_h.T + R_h
            K_h = self.P @ H_h.T @ np.linalg.inv(S_h)
            self.x    = self.x + (K_h * innov_h).flatten()
            self.x[2] = normalize_angle(self.x[2])
            I_KH = np.eye(3) - K_h @ H_h
            self.P = I_KH @ self.P @ I_KH.T + K_h @ R_h @ K_h.T

        return True

    # -----------------------------------------------------------------------
    # Jacobian
    # -----------------------------------------------------------------------

    def compute_measurement_jacobian(self, landmark: dict) -> np.ndarray:
        """
        H matrix for (bearing, range) w.r.t. [x, y, θ].
        """
        dx = landmark["x"] - self.x[0]
        dy = landmark["y"] - self.x[1]
        r2 = dx**2 + dy**2 + 1e-9
        r  = np.sqrt(r2)

        H = np.array([
            [  dy / r2,  -dx / r2,  -1.0],   # d(bearing)/d(x,y,θ)
            [-dx / r,    -dy / r,    0.0],    # d(range)/d(x,y,θ)
        ])
        return H

    # -----------------------------------------------------------------------
    # Diagnostics
    # -----------------------------------------------------------------------

    @property
    def sigma_xy(self) -> float:
        return float(np.sqrt(0.5 * (self.P[0, 0] + self.P[1, 1])))

    @property
    def sigma_theta(self) -> float:
        return float(np.sqrt(self.P[2, 2]))

    def __repr__(self) -> str:
        return (
            f"EKF(x={self.x[0]:.3f}, y={self.x[1]:.3f}, "
            f"θ={np.degrees(self.x[2]):.1f}°, "
            f"σ_xy={self.sigma_xy:.3f})"
        )