"""
particle_filter.py
==================
Adaptive Monte Carlo Localization (AMCL) for BFMC.

Key features:
- ID-free data association
- KLD-based adaptive particle count
- Kidnap recovery (loop-closure)
- Graph-snapped motion and resampling
- Strict Directed-Edge enforcement
- Mix-MCL (Measurement Model Inversion)
- Visibility-Based First-Sight & Singular Teleportation
"""

from __future__ import annotations

import numpy as np
import networkx as nx
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict


# ---------------------------------------------------------------------------
# Utility Functions
# ---------------------------------------------------------------------------

def normalize_angle(a: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def angle_diff(a: float, b: float) -> float:
    """Calculate the shortest difference between two angles."""
    return normalize_angle(a - b)


# ---------------------------------------------------------------------------
# Data Structures
# ---------------------------------------------------------------------------

@dataclass
class Particle:
    """Represents a single hypothesis in the state space."""
    x: float
    y: float
    theta: float
    weight: float = 1.0


@dataclass
class Detection:
    """Single landmark observation from the perception system."""
    lm_type: str    # e.g., "stop_sign", "crosswalk"
    bearing: float  # Radians in car frame
    distance: float # Metres


# ---------------------------------------------------------------------------
# Geometric & Statistical Helpers
# ---------------------------------------------------------------------------

def snap_to_graph(
    G: nx.Graph,
    x: float,
    y: float,
    max_dist: float = 0.5,
    particle_theta: Optional[float] = None  # NEW: Added to resolve bidirectional roads
) -> Tuple[Optional[float], Optional[float], float, Optional[float]]:
    """
    Project (x, y) onto the nearest graph edge.
    Returns (proj_x, proj_y, dist, edge_heading). 
    """
    best_cost = 1e9
    best_dist = 1e9
    best_proj = (x, y)
    best_heading = None

    for u, v in G.edges():
        x1, y1 = float(G.nodes[u]["x"]), float(G.nodes[u]["y"])
        x2, y2 = float(G.nodes[v]["x"]), float(G.nodes[v]["y"])

        dx, dy = x2 - x1, y2 - y1
        length_sq = dx**2 + dy**2
        if length_sq < 1e-9:
            continue

        # Project point onto line segment
        t = np.clip(((x - x1) * dx + (y - y1) * dy) / length_sq, 0.0, 1.0)
        px, py = x1 + t * dx, y1 + t * dy
        dist = np.hypot(x - px, y - py)

        if dist > max_dist:
            continue

        edge_heading = np.arctan2(dy, dx)
        
        cost = dist
        if particle_theta is not None:
            if abs(angle_diff(particle_theta, edge_heading)) > np.pi / 2:
                cost += 1000.0  # Massive penalty for the wrong-way edge

        if cost < best_cost:
            best_cost = cost
            best_dist = dist
            best_proj = (px, py)
            best_heading = edge_heading

    if best_dist <= max_dist:
        return best_proj[0], best_proj[1], best_dist, best_heading
    return None, None, best_dist, None


def kld_particle_count(
    n_unique_bins: int,
    epsilon: float = 0.05,
    delta: float = 0.01,
    z_delta: float = 2.3263,  # Quantile for 1 - delta = 0.99
    min_n: int = 10,
    max_n: int = 50,
) -> int:
    """Compute KLD-sampling particle count based on occupied histogram bins."""
    if n_unique_bins < 2:
        return max_n

    k = n_unique_bins
    term = 1 - 2 / (9 * (k - 1)) + z_delta * np.sqrt(2 / (9 * (k - 1)))
    n = ((k - 1) / (2 * epsilon)) * (term ** 3)
    
    return int(np.clip(n, min_n, max_n))


# ---------------------------------------------------------------------------
# Models (Motion & Measurement)
# ---------------------------------------------------------------------------

def sample_motion(
    particle: Particle,
    v: float,
    omega: float,
    dt: float,
    alpha: Tuple[float, float, float, float] = (0.05, 0.01, 0.01, 0.05),
) -> Particle:
    """Noisy unicycle motion model."""
    a1, a2, a3, a4 = alpha

    v_hat = v + np.random.normal(0, np.sqrt(a1 * v**2 + a2 * omega**2))
    omega_hat = omega + np.random.normal(0, np.sqrt(a3 * v**2 + a4 * omega**2))
    gamma_hat = np.random.normal(0, np.sqrt(a1 * v**2 + a2 * omega**2))

    new_x = particle.x + v_hat * np.cos(particle.theta) * dt
    new_y = particle.y + v_hat * np.sin(particle.theta) * dt
    new_theta = normalize_angle(particle.theta + omega_hat * dt + gamma_hat * dt)

    return Particle(new_x, new_y, new_theta, particle.weight)


def likelihood_field(
    particle: Particle,
    detections: List[Detection],
    landmarks: List[Dict],
    lm_weights: Dict[str, float],
    max_range: float = 3.0,
) -> float:
    """Measurement model: Calculate likelihood of detections given particle pose."""
    SIGMA_BEARING = 0.25 # Forgiving angle
    SIGMA_RANGE = 0.5   # Forgiving distance
    P_NO_MATCH = 1e-15   # Severe penalty for contradiction, but won't crash float math

    log_w = 0.0

    for det in detections:
        lls = []
        for lm in landmarks:
            if lm.get("type") != det.lm_type:
                continue

            dx, dy = lm["x"] - particle.x, lm["y"] - particle.y
            pred_range = np.hypot(dx, dy)
            if pred_range > max_range:
                continue

            pred_bearing = normalize_angle(np.arctan2(dy, dx) - particle.theta)

            fov_score = np.exp(-0.5 * (pred_bearing / np.deg2rad(40)) ** 2)
            if fov_score < 1e-3:
                continue

            angle_to_particle = np.arctan2(-dy, -dx)
            visibility_score = 0.0
            for f in lm.get("facing", []):
                f_rad = np.deg2rad(f) if abs(f) > np.pi * 2 else f
                diff = abs(angle_diff(angle_to_particle, f_rad))
                score = np.exp(-0.5 * (diff / np.deg2rad(30)) ** 2)
                visibility_score = max(visibility_score, score)

            if visibility_score < 1e-3:
                continue
            
            dr = det.distance - pred_range
            db = angle_diff(det.bearing, pred_bearing)
            
            ll = (
                -0.5 * (dr / SIGMA_RANGE) ** 2
                -0.5 * (db / SIGMA_BEARING) ** 2
                - np.log(SIGMA_RANGE * SIGMA_BEARING * 2 * np.pi)
            )
            ll += np.log(fov_score + 1e-9) + np.log(visibility_score + 1e-9)
            lls.append(ll)

        if lls:
            det_ll = np.logaddexp.reduce(lls) - np.log(len(lls))
        else:
            det_ll = np.log(P_NO_MATCH) # Contradiction penalty applied here!

        rarity_multiplier = lm_weights.get(det.lm_type, 1.0)
        log_w += det_ll * rarity_multiplier

    return np.exp(np.clip(log_w, -100, 0))


# ---------------------------------------------------------------------------
# Particle Filter Class
# ---------------------------------------------------------------------------

class ParticleFilter:
    """Adaptive MCL for BFMC localization."""

    def __init__(
        self,
        landmarks: List[Dict],
        graph: nx.Graph,
        n_particles: int = 300,
        alpha: Tuple[float, float, float, float] = (0.05, 0.01, 0.01, 0.05),
        graph_snap_dist: float = 0.3,
        global_init: bool = True,
    ):
        self.landmarks = landmarks
        self.G = graph
        self.alpha = alpha
        self.graph_snap_dist = graph_snap_dist

        self._bin_size_xy = 0.3
        self._bin_size_theta = np.deg2rad(20)

        # Track last seen detections for Mix-MCL
        self.last_detections: List[Detection] = []

        type_counts = {}
        for lm in self.landmarks:
            t = lm.get("type", "unknown")
            type_counts[t] = type_counts.get(t, 0) + 1
            
        total_lms = len(self.landmarks)
        self.lm_weights = {}
        for t, count in type_counts.items():
            self.lm_weights[t] = 1.0 + np.log(total_lms / count)

        self.particles = self._init_global(n_particles) if global_init else []
        self.neff = float(n_particles)

        self._w_slow, self._w_fast = 0.0, 0.0
        self._alpha_slow, self._alpha_fast = 0.001, 0.1

        # ==========================================
        # First-Sight & Teleportation Config
        # ==========================================
        self.first_meaningful_seen = False
        self.permanently_locked = False
        
        # Landmarks that do NOT trigger the "first sight" rule
        self.IGNORE_FIRST_LIST = ["trafficlight", "priority", "stop", "STOP_LINE", "crosswalk", "greenlight", "zebra"]
        
        # Map camera labels to the SPECIFIC landmark 'type' or 'name' in your JSON
        self.FIRST_SIGHT_MAP = {
            "crosswalk": "crosswalk_sign_6",
            "oneway": "oneway_1",
            "noentry": "no_entry_1"
        }

    def _init_global(self, n: int) -> List[Particle]:
        """
        Initializes 100% of the particles converged exactly at node 263, 
        facing the direction of the 264 -> 263 edge.
        """
        particles = []
        target_u, target_v = "416", "417"
        
        if target_u in self.G.nodes and target_v in self.G.nodes:
            x1, y1 = float(self.G.nodes[target_u]["x"]), float(self.G.nodes[target_u]["y"])
            x2, y2 = float(self.G.nodes[target_v]["x"]), float(self.G.nodes[target_v]["y"])
            
            base_theta = np.arctan2(y2 - y1, x2 - x1)
            
            # Pinpoint the exact spot of node 263
            base_x = x2
            base_y = y2
            
            for _ in range(n):
                # 2cm variance pinpoint drop (forces instant convergence)
                x = base_x + np.random.normal(0, 0.02)
                y = base_y + np.random.normal(0, 0.02)
                theta = normalize_angle(base_theta + np.random.normal(0, 0.01))
                particles.append(Particle(x, y, theta, 1.0))
                
            # Lock the particle filter state so it doesn't immediately scatter
            # if it sees a contradictory landmark on frame 1
            self.first_meaningful_seen = True
            self.permanently_locked = True
            
            print(f">>> [PARTICLE FILTER] Initialized {n} particles converged at node {target_v}.")
            return self._normalise(particles)
            
        else:
            print(f"\033[1;91m>>> [PF ERROR] Nodes {target_u} or {target_v} missing from graph! Falling back to random.\033[0m")
            # Fallback to completely random initialization if nodes are missing
            SPAWN_NODES = ["1", "2", "3", "4", "6", "9", "10", "11", "12", "13", "14", "15", "16", "17", "18", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "31", "32", "33", "34", "35", "36", "37", "38", "39", "40", "42", "43", "44", "45", "46", "47", "48", "75", "74", "73", "72", "70", "71", "76", "77", "78", "91", "90", "88", "89", "94", "95", "96", "102", "101", "100", "99", "97", "98", "103", "104", "105", "106", "107", "108", "109", "110", "111", "112", "113", "114", "115", "116", "117", "118", "119", "120", "121", "122", "123", "124", "125", "126", "127", "128", "129", "130", "131", "132", "133", "134", "135", "136", "137", "138", "139", "140", "141", "142", "143", "144", "145", "146", "147", "148", "149", "150", "184", "185", "186", "187", "188", "189", "190", "191", "201", "202", "203", "204", "205", "206", "207", "208", "209", "210", "211", "212", "213", "214", "215", "216", "217", "218", "219", "220", "221", "222", "264", "265", "266", "300", "301"]
            allowed_edges = [e for e in self.G.edges() if e[0] in SPAWN_NODES]
            
            for _ in range(n):
                u, v = allowed_edges[np.random.randint(len(allowed_edges))]
                x1, y1 = float(self.G.nodes[u]["x"]), float(self.G.nodes[u]["y"])
                x2, y2 = float(self.G.nodes[v]["x"]), float(self.G.nodes[v]["y"])
                t = np.random.uniform(0, 1)
                x, y = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
                theta = np.arctan2(y2 - y1, x2 - x1) + np.random.normal(0, 0.1)
                particles.append(Particle(x, y, theta, 1.0))
                
            return self._normalise(particles)

    def _get_visible_spawns_for_lm(self, target_lm: Dict, detection: Detection) -> List[Tuple[float, float, float]]:
        """Helper to find physical road points where a specific map landmark is visible."""
        lx, ly = float(target_lm["x"]), float(target_lm["y"])
        valid_spawns = []
        
        for u, v in self.G.edges():
            x1, y1 = float(self.G.nodes[u]["x"]), float(self.G.nodes[u]["y"])
            x2, y2 = float(self.G.nodes[v]["x"]), float(self.G.nodes[v]["y"])
            
            dx, dy = x2 - x1, y2 - y1
            edge_len = np.hypot(dx, dy)
            if edge_len < 1e-9: continue
            
            edge_heading = np.arctan2(dy, dx)
            num_samples = max(5, int(edge_len * 10))
            
            for t in np.linspace(0, 1, num_samples):
                px = x1 + t * dx
                py = y1 + t * dy
                
                dist_to_lm = np.hypot(lx - px, ly - py)
                if dist_to_lm > 3.0: 
                    continue
                    
                angle_to_lm = np.arctan2(ly - py, lx - px)
                bearing_in_cam = angle_diff(angle_to_lm, edge_heading)
                
                if abs(bearing_in_cam) < np.deg2rad(50):
                    is_facing = True
                    if "facing" in target_lm and target_lm["facing"]:
                        angle_to_particle = np.arctan2(py - ly, px - lx)
                        for f in target_lm["facing"]:
                            f_rad = np.deg2rad(f) if abs(f) > np.pi * 2 else f
                            if abs(angle_diff(angle_to_particle, f_rad)) < np.deg2rad(60): 
                                is_facing = True
                                break
                        else:
                            is_facing = False
                            
                    if is_facing:
                        valid_spawns.append((px, py, edge_heading))
                        
        return valid_spawns

    def _teleport_to_visible_edges(self, target_lm_id: str, detection: Detection):
        target_lm = next((lm for lm in self.landmarks if str(lm.get("id")) == target_lm_id), None)
        if not target_lm: return

        valid_spawns = self._get_visible_spawns_for_lm(target_lm, detection)
        if not valid_spawns: return
            
        new_particles = []
        target_count = len(self.particles) if self.particles else 300
        
        # --- INSTANT CONVERGENCE FIX ---
        # Instead of spreading randomly over ALL valid areas, grab the exact center of the visibility zone
        best_spawn = valid_spawns[len(valid_spawns) // 2]
        base_px, base_py, base_heading = best_spawn

        for _ in range(target_count):
            # Pinpoint drop (2cm variance) forces instant convergence < 0.4 thresholds
            nx_pos = base_px + np.random.normal(0, 0.02)
            ny_pos = base_py + np.random.normal(0, 0.02)
            ntheta = normalize_angle(base_heading + np.random.normal(0, 0.01))
            new_particles.append(Particle(nx_pos, ny_pos, ntheta, 1.0))
            
        self.particles = self._normalise(new_particles)
        self.neff = float(len(self.particles))
        
        # Reset trackers so it doesn't instantly de-converge due to math whiplash
        self._w_fast = 1.0
        self._w_slow = 1.0
        
        print(f"\033[1;92m>>> [PF] SUCCESS! Locked {len(new_particles)} particles exactly to {target_lm_id}.\033[0m")

    def init_at(self, x, y, theta, sigma_xy=0.5, sigma_theta=0.3, n=300):
        self.particles = []
        max_attempts = n * 5
        attempts = 0
        
        while len(self.particles) < n and attempts < max_attempts:
            attempts += 1
            px = x + np.random.normal(0, sigma_xy)
            py = y + np.random.normal(0, sigma_xy)
            
            sx, sy, _, _ = snap_to_graph(self.G, px, py, max_dist=1.0, particle_theta=theta)
            
            if sx is not None:
                pt = normalize_angle(theta + np.random.normal(0, sigma_theta))
                self.particles.append(Particle(sx, sy, pt))
                
        self.particles = self._normalise(self.particles)

    def predict(self, v: float, omega: float, dt: float):
        new_particles = []
        for p in self.particles:
            p_new = sample_motion(p, v, omega, dt, self.alpha)
            sx, sy, _, edge_heading = snap_to_graph(self.G, p_new.x, p_new.y, 0.3, p_new.theta)
            
            if sx is not None:
                p_new.x, p_new.y = sx, sy
                if edge_heading is not None and abs(angle_diff(p_new.theta, edge_heading)) < np.pi/2:
                    diff = angle_diff(edge_heading, p_new.theta)
                    p_new.theta = normalize_angle(p_new.theta + 0.1 * diff) 

                new_particles.append(p_new)
        
        self.particles = new_particles if new_particles else self._init_global(len(self.particles))

    def update(self, detections: List[Detection]):
        self.last_detections = detections if detections else []
        
        # 1. FIRST-SIGHT TELEPORTATION
        for det in detections:
            cam_label = det.lm_type.lower()
            
            if not self.first_meaningful_seen and cam_label not in self.IGNORE_FIRST_LIST:
                target_map_id = None
                for key, val in self.FIRST_SIGHT_MAP.items():
                    if key in cam_label:
                        target_map_id = val
                        break
                
                if target_map_id:
                    self.first_meaningful_seen = True 
                    self.permanently_locked = True
                    print(f"\033[1;95m>>> [PF] First sight: {cam_label.upper()}! Teleporting to {target_map_id}...\033[0m")
                    self._teleport_to_visible_edges(target_map_id, det)
                    return 

        # 2. SINGULAR LANDMARK CONVERGENCE
        for det in detections:
            count = sum(1 for lm in self.landmarks if lm.get("type") == det.lm_type)
            if count == 1:
                print(f"\033[1;95m>>> [PF] Saw Singular Landmark ({det.lm_type})! Forcing convergence.\033[0m")
                self._teleport_to_visible_edges(str(next(lm["id"] for lm in self.landmarks if lm.get("type") == det.lm_type)), det)
                return 
            
        # 3. START LINE
        start_det = next((d for d in detections if d.lm_type.lower() in ["start"]), None)
        if start_det is not None:
            target_count = len(self.particles) if self.particles else 300
            new_particles = []
            
            # --- TARGET EXACTLY NODE 263 ---
            target_u, target_v = "264", "263" 
            
            if target_u in self.G.nodes and target_v in self.G.nodes:
                x1, y1 = float(self.G.nodes[target_u]["x"]), float(self.G.nodes[target_u]["y"])
                x2, y2 = float(self.G.nodes[target_v]["x"]), float(self.G.nodes[target_v]["y"])
                base_theta = np.arctan2(y2 - y1, x2 - x1)
                
                # --- INSTANT CONVERGENCE FIX ---
                base_x = x2 # Pinpoint the exact spot of node 263
                base_y = y2
                
                for _ in range(target_count):
                    # 2cm variance pinpoint drop
                    x = base_x + np.random.normal(0, 0.02)
                    y = base_y + np.random.normal(0, 0.02)
                    theta = normalize_angle(base_theta + np.random.normal(0, 0.01))
                    new_particles.append(Particle(x, y, theta, 1.0))
            
            if new_particles:
                self.particles = self._normalise(new_particles)
                self.neff = float(len(self.particles))
                self.first_meaningful_seen = True # Lock first sight so it doesn't jump later
                self.permanently_locked = True
                
                self._w_fast = 1.0
                self._w_slow = 1.0
                
                print(f">>> [PARTICLE FILTER] 'Start' detected! Snapping {len(self.particles)} particles exactly to node {target_v}.")
                return
            
        # 4. STANDARD LIKELIHOOD 
        SIGMA_GRAPH, MAX_DIST = 0.15, 0.40  # More forgiving track snapping 
        
        for p in self.particles:
            w = likelihood_field(p, detections, self.landmarks, self.lm_weights) if detections else 1.0
            _, _, dist, edge_heading = snap_to_graph(self.G, p.x, p.y, self.graph_snap_dist, p.theta)
            
            w *= 1e-6 if dist > MAX_DIST else np.exp(-0.5 * (dist / SIGMA_GRAPH) ** 2)
            
            if edge_heading is not None:
                heading_diff = abs(angle_diff(p.theta, edge_heading))
                if heading_diff > np.pi / 2:
                    w *= 1e-6 
            
            p.weight *= w

        self.particles = self._normalise(self.particles)
        
        # Fast/Slow MAV tracking for contradictions
        if detections:
            w_avg = 1.0 / max(len(self.particles), 1)
            self._w_slow += self._alpha_slow * (w_avg - self._w_slow)
            self._w_fast += self._alpha_fast * (w_avg - self._w_fast)
        
        weights = np.array([p.weight for p in self.particles])
        self.neff = 1.0 / (np.sum(weights**2) + 1e-12)

    def resample(self):
        if not self.particles: return

        is_locked_in = self.is_converged(xy_thresh=0.4, theta_thresh=0.4)
        n_target = self._kld_count()

        # If w_fast crashes relative to w_slow, w_diff spikes > 0.3. This means CONTRADICTION.
        w_diff = max(0.0, 1.0 - self._w_fast / (self._w_slow + 1e-9))
        
        injections = []

        # Only deconverge/inject if we saw something explicitly contradictory, OR we haven't locked in yet
        if (not is_locked_in or w_diff > 0.3) and self.last_detections and not self.permanently_locked:
            if w_diff > 0.3:
                print("\033[1;93m>>> [PF] Map contradiction detected! Deconverging and scattering...\033[0m")
            
            seen_types = set()
            for det in self.last_detections:
                if det.lm_type in seen_types: continue
                seen_types.add(det.lm_type)
                
                # Smart Scatter (Inversely Proportional)
                matching_lms = [lm for lm in self.landmarks if lm.get("type") == det.lm_type]
                C = len(matching_lms)
                if C == 0: continue
                
                p_total = 0.15 / C
                n_inject_total = int(n_target * p_total)
                n_per_instance = max(1, n_inject_total // C)
                
                for lm in matching_lms:
                    spawns = self._get_visible_spawns_for_lm(lm, det)
                    if not spawns: continue
                    for _ in range(n_per_instance):
                        px, py, edge_heading = spawns[np.random.randint(len(spawns))]
                        nx_pos = px + np.random.normal(0, 0.05)
                        ny_pos = py + np.random.normal(0, 0.05)
                        ntheta = normalize_angle(edge_heading + np.random.normal(0, 0.03))
                        injections.append(Particle(nx_pos, ny_pos, ntheta, 1.0))
        
        n_resample = n_target - len(injections)
        
        weights = np.array([p.weight for p in self.particles])
        weights /= weights.sum()
        
        new_particles = []
        r, c, i = np.random.uniform(0, 1.0 / max(n_resample, 1)), weights[0], 0

        # Normal Resampling
        for m in range(n_resample):
            u = r + m / n_resample
            while u > c and i < len(self.particles) - 1:
                i += 1
                c += weights[i]
            
            p = self.particles[i]
            sx, sy, dist, _ = snap_to_graph(self.G, p.x, p.y, self.graph_snap_dist, p.theta)
            
            if sx is not None:
                blend = max(0.0, 1.0 - dist / self.graph_snap_dist)
                nx = p.x + blend * (sx - p.x) + np.random.normal(0, 0.02)
                ny = p.y + blend * (sy - p.y) + np.random.normal(0, 0.02)
                new_particles.append(Particle(nx, ny, p.theta, 1.0))
            else:
                fsx, fsy, _, _ = snap_to_graph(self.G, p.x, p.y, max_dist=1.0, particle_theta=p.theta)
                if fsx is not None:
                    new_particles.append(Particle(fsx, fsy, p.theta, 1.0))

        # Add the contradictory scatter particles back into the pool
        new_particles.extend(injections)
        self.particles = self._normalise(new_particles)

    def estimate(self) -> Tuple[float, float, float]:
        if not self.particles: return 0.0, 0.0, 0.0
        weights = np.array([p.weight for p in self.particles])
        x = np.average([p.x for p in self.particles], weights=weights)
        y = np.average([p.y for p in self.particles], weights=weights)
        sin_t = np.average([np.sin(p.theta) for p in self.particles], weights=weights)
        cos_t = np.average([np.cos(p.theta) for p in self.particles], weights=weights)
        return x, y, np.arctan2(sin_t, cos_t)

    def best_particle(self) -> Particle:
        return max(self.particles, key=lambda p: p.weight)

    def covariance(self) -> np.ndarray:
        x, y, theta = self.estimate()
        weights = np.array([p.weight for p in self.particles])
        weights /= weights.sum()
        cov = np.zeros((3, 3))
        for p, w in zip(self.particles, weights):
            diff = np.array([p.x - x, p.y - y, normalize_angle(p.theta - theta)])
            cov += w * np.outer(diff, diff)
        return cov

    def is_converged(self, xy_thresh=0.4, theta_thresh=0.4) -> bool:
        cov = self.covariance()
        return (np.sqrt(cov[0, 0]) < xy_thresh and 
                np.sqrt(cov[1, 1]) < xy_thresh and 
                np.sqrt(cov[2, 2]) < theta_thresh)

    @staticmethod
    def _normalise(particles: List[Particle]) -> List[Particle]:
        total = sum(p.weight for p in particles)
        if total < 1e-12:
            for p in particles: p.weight = 1.0 / len(particles)
        else:
            for p in particles: p.weight /= total
        return particles

    def _kld_count(self) -> int:
        bins = set()
        for p in self.particles:
            bx, by = int(p.x / self._bin_size_xy), int(p.y / self._bin_size_xy)
            bt = int(p.theta / self._bin_size_theta)
            bins.add((bx, by, bt))
        return kld_particle_count(len(bins))