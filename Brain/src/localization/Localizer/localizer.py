"""
localizer.py
============
Hybrid PF + EKF localizer for BFMC.

State machine
-------------
GLOBAL  → particles spread over entire graph (unknown start)
         transitions to LOCAL once PF converges

LOCAL   → EKF runs as primary estimator; PF idles or runs lightly
         transitions back to GLOBAL if kidnap detected

RECOVER → lightweight PF re-seeded near EKF position + random fraction
         (fast recovery when soft kidnap detected)

Usage
-----
    loc = Localizer(landmarks, graph)

    # Every timestep:
    loc.predict(v, omega, dt)
    loc.update(detections)          # list[Detection]
    x, y, theta = loc.pose          # best estimate
    mode = loc.mode                 # "GLOBAL" / "LOCAL" / "RECOVER"
"""

from __future__ import annotations

import numpy as np
from enum import Enum
from typing import List, Tuple

from src.localization.Localizer.particle_filter import ParticleFilter, Detection, snap_to_graph
from src.localization.Localizer.ekf import EKF, normalize_angle
import networkx as nx


class Mode(str, Enum):
    GLOBAL  = "GLOBAL"    # PF exploring, EKF not yet trusted
    LOCAL   = "LOCAL"     # EKF primary, PF idle
    RECOVER = "RECOVER"   # PF recovering near last EKF position


class Localizer:
    """
    Orchestrates the PF → EKF handoff and recovery.

    Parameters
    ----------
    landmarks        : list of landmark dicts
    graph            : NetworkX road graph
    n_particles      : starting particle count
    pf_alpha         : motion noise for PF (a1..a4)
    converge_xy      : xy std threshold to declare PF convergence [m]
    converge_theta   : heading std threshold to declare convergence [rad]
    kidnap_threshold : EKF residual that triggers RECOVER mode
    graph_snap_dist  : max dist for road-snapping [m]
    """

    def __init__(
        self,
        landmarks: List[dict],
        graph: nx.Graph,
        n_particles: int = 300,
        pf_alpha: Tuple[float, float, float, float] = (0.05, 0.01, 0.01, 0.05),
        converge_xy: float = 0.30,
        converge_theta: float = 0.30,
        kidnap_threshold: float = 3.0,
        graph_snap_dist: float = 0.40,
    ):
        self.landmarks = landmarks
        self.G = graph
        self.graph_snap_dist = graph_snap_dist

        self._converge_xy    = converge_xy
        self._converge_theta = converge_theta
        self._kidnap_thr     = kidnap_threshold

        # ---- Particle filter (global by default) ----
        self.pf = ParticleFilter(
            landmarks=landmarks,
            graph=graph,
            n_particles=n_particles,
            alpha=pf_alpha,
            graph_snap_dist=graph_snap_dist,
            global_init=True,
        )

        # ---- EKF (initialised lazily from PF) ----
        self.ekf = EKF()

        self._mode = Mode.GLOBAL

        # Internal bookkeeping
        self._consecutive_bad = 0   # steps with large EKF residuals
        self._bad_threshold   = 5   # how many bad steps before RECOVER

    # -----------------------------------------------------------------------
    # Public interface
    # -----------------------------------------------------------------------

    @property
    def mode(self) -> str:
        return self._mode.value

    @property
    def pose(self) -> Tuple[float, float, float]:
        """Best pose estimate given the current mode."""
        if self._mode == Mode.LOCAL:
            return float(self.ekf.x[0]), float(self.ekf.x[1]), float(self.ekf.x[2])
        else:
            return self.pf.estimate()

    @property
    def covariance(self) -> np.ndarray:
        if self._mode == Mode.LOCAL:
            return self.ekf.P.copy()
        return self.pf.covariance()

    # -----------------------------------------------------------------------

    def predict(self, v: float, omega: float, dt: float) -> None:
        """Motion update — always run on both PF and EKF."""
        self.pf.predict(v, omega, dt)

        if self._mode in (Mode.LOCAL, Mode.RECOVER):
            self.ekf.predict(v, omega, dt)

    def update(self, detections: List[Detection]) -> None:
        """
        Measurement update + state-machine transitions.
        """
        # ---- PF update (always in GLOBAL / RECOVER; lightly in LOCAL) ----
        if self._mode in (Mode.GLOBAL, Mode.RECOVER):
            self.pf.update(detections)
            self.pf.resample()

            if self._mode == Mode.GLOBAL and self.pf.is_converged(
                self._converge_xy, self._converge_theta
            ):
                self._transition_to_local()

            elif self._mode == Mode.RECOVER and self.pf.is_converged(
                self._converge_xy * 1.5, self._converge_theta * 1.5
            ):
                self._transition_to_local()

        # ---- EKF update (always in LOCAL) ----
        if self._mode == Mode.LOCAL:
            # run PF lightly as a sanity check (use only 50 particles)
            pf_detections = detections  # same observations
            self._lite_pf_check(pf_detections)

            # associate + update EKF with each detection
            for det in detections:
                lm, error = self._associate(det)
                if lm is not None:
                    ok = self.ekf.update_bearing_range(
                        np.array([det.bearing, det.distance]),
                        lm,
                    )
                    if not ok:
                        self._consecutive_bad += 1
                    else:
                        self._consecutive_bad = max(0, self._consecutive_bad - 1)

            # graph snapping
            self._apply_graph_snap()

            # kidnap detection
            if self._consecutive_bad >= self._bad_threshold:
                self._transition_to_recover()

    # -----------------------------------------------------------------------
    # Graph snapping (EKF only)
    # -----------------------------------------------------------------------

    def _apply_graph_snap(self) -> None:
        px, py, p_theta = self.pose
        sx, sy, dist, _ = snap_to_graph(self.G, px, py, self.graph_snap_dist, particle_theta=p_theta)

        if sx is None:
            return

        # Compute road heading from the nearest edge
        road_heading = self._edge_heading(px, py)
        self.ekf.update_graph(sx, sy, heading_from_edge=road_heading)

    def _edge_heading(self, x: float, y: float) -> float | None:
        best_dist = 1e9
        best_heading = None

        for u, v in self.G.edges():
            x1, y1 = float(self.G.nodes[u]["x"]), float(self.G.nodes[u]["y"])
            x2, y2 = float(self.G.nodes[v]["x"]), float(self.G.nodes[v]["y"])

            dx, dy = x2 - x1, y2 - y1
            len2 = dx**2 + dy**2
            if len2 < 1e-9:
                continue

            t = np.clip(((x - x1) * dx + (y - y1) * dy) / len2, 0, 1)
            px = x1 + t * dx
            py = y1 + t * dy
            dist = np.hypot(x - px, y - py)

            if dist < best_dist:
                best_dist = dist
                best_heading = np.arctan2(dy, dx)

        return best_heading

    # -----------------------------------------------------------------------
    # Data association (EKF mode — no landmark IDs)
    # -----------------------------------------------------------------------

    def _associate(self, det: Detection):
        """
        Find the most likely landmark for this detection using only
        geometry (bearing + range) and type.

        Returns (landmark_dict, mahalanobis_distance) or (None, inf).
        """
        best_lm    = None
        best_maha  = np.inf
        chi2_gate  = 5.991   # 95 % chi-squared, 2 DOF

        for lm in self.landmarks:
            if lm.get("type") != det.lm_type:
                continue

            dx = lm["x"] - self.ekf.x[0]
            dy = lm["y"] - self.ekf.x[1]
            pred_range   = np.hypot(dx, dy)
            pred_bearing = normalize_angle(np.arctan2(dy, dx) - self.ekf.x[2])

            # coarse range gate first (cheap)
            if abs(det.distance - pred_range) > 0.4:
                continue

            innov = np.array([
                normalize_angle(det.bearing - pred_bearing),
                det.distance - pred_range,
            ])

            H = self.ekf.compute_measurement_jacobian(lm)
            R = self.ekf._R_bearing_range
            S = H @ self.ekf.P @ H.T + R

            try:
                maha2 = innov.T @ np.linalg.inv(S) @ innov
            except np.linalg.LinAlgError:
                continue

            if maha2 < chi2_gate and maha2 < best_maha:
                best_maha = maha2
                best_lm   = lm

        return best_lm, best_maha

    # -----------------------------------------------------------------------
    # Lightweight PF consistency check
    # -----------------------------------------------------------------------

    def _lite_pf_check(self, detections: List[Detection]) -> None:
        """
        Run the PF lightly to detect if the EKF has drifted far from
        the particle cloud.  If the EKF pose is an outlier vs the PF,
        increment the bad counter.
        """
        if not self.pf.particles:
            return

        # PF keeps running its motion update externally (via predict())
        # Here we just compare EKF pose to PF weighted mean
        px, py, pt = self.pf.estimate()
        ex, ey, et = self.pose

        dist = np.hypot(ex - px, ey - py)
        hdiff = abs(normalize_angle(et - pt))

        if dist > 1.0 or hdiff > np.deg2rad(45):
            self._consecutive_bad += 2

    # -----------------------------------------------------------------------
    # State machine transitions
    # -----------------------------------------------------------------------

    def _transition_to_local(self) -> None:
        x, y, theta = self.pf.estimate()
        cov = self.pf.covariance()
        self.ekf.seed_from_particle_filter(x, y, theta, cov)
        self._mode = Mode.LOCAL
        self._consecutive_bad = 0
        print(f"[LOCALIZER] → LOCAL  pose=({x:.2f}, {y:.2f}, {np.degrees(theta):.1f}°)")

    def _transition_to_recover(self) -> None:
        x, y, theta = self.ekf.x
        self.pf.init_at(x, y, theta, sigma_xy=0.8, sigma_theta=0.5, n=300)
        # add some global particles for kidnap recovery
        global_inj = self.pf._init_global(50)
        self.pf.particles.extend(global_inj)
        self.pf.particles = ParticleFilter._normalise(self.pf.particles)
        self._mode = Mode.RECOVER
        self._consecutive_bad = 0
        print(f"[LOCALIZER] → RECOVER  last_ekf=({x:.2f}, {y:.2f})")