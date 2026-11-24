"""
threadLocalizer.py 
========================================
Real-time Localization Thread for BFMC
Integrates the Particle Filter + EKF localizer with vehicle data.
"""

import ast
import time
import os
import networkx as nx
from pathlib import Path

import numpy as np

# --- Localizer Imports ---
from src.localization.Localizer.map_loader import load_landmarks, load_point_landmarks
from src.localization.Localizer.localizer import Localizer
from src.localization.Localizer.particle_filter import ParticleFilter, Detection, snap_to_graph

# --- Message Handlers ---
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (mainCamera, CurrentSpeed, ImuData, ObjectDetect, LiveAnimation, LocalizerPose)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender


class threadLocalizer(ThreadWithStop):
    """This thread handles Localizer."""

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        
        # State variables
        self.vehicle_speed = None
        self.lastValidSpeed = 0.0

        self.yaw_rate = None
        self.prev_yaw = None 

        self.last_time = time.time()
        self.init_success = False  # <-- GUARD VARIABLE

        # Setup Map and Localizer
        self._init_localizer()

        self.subscribe()
        super(threadLocalizer, self).__init__()

    def _init_localizer(self):
        """Helper to keep __init__ clean. Loads heavy map data with absolute paths."""
        print("\033[1;97m[ Localizer ]  :\033[0m \033[1;92mINFO\033[0m - Initializing map and graph...", flush=True)
        try:
            
            # 1. Build the absolute paths to the map files
            graph_path = Path("/home/pi/Documents/Brain/src/utils/Competition_track_graph.graphml")
            landmarks_path = Path("/home/pi/Documents/Brain/src/utils/landmarks.json")

            if not graph_path.exists():
                raise FileNotFoundError(f"Graph file not found at: {graph_path}")
            if not landmarks_path.exists():
                raise FileNotFoundError(f"Landmarks file not found at: {landmarks_path}")

            # 2. Load Map Data
            self.G = nx.read_graphml(graph_path)
            raw_data = load_landmarks(str(landmarks_path))
            all_points = load_point_landmarks(raw_data)
            self.landmarks = all_points[:500] 
            
            # 3. Initialize Localizer
            self.localizer = Localizer(
                landmarks=self.landmarks,
                graph=self.G,
                n_particles=50,
                pf_alpha=(0.20, 0.10, 0.10, 0.20),
                converge_xy=0.2,
                converge_theta=0.2,
                graph_snap_dist=0.30,
            )
            
            self.init_success = True
            print("\033[1;97m[ Localizer ]  :\033[0m \033[1;92mINFO\033[0m - Map and filter loaded successfully.", flush=True)
            
        except Exception as e:
            print(f"\033[1;97m[ Localizer ] :\033[0m \033[1;91mERROR\033[0m - ERROR during initialization: {e}\033[0m", flush=True)

    def subscribe(self):
        self.currentSpeedSubscriber = messageHandlerSubscriber(self.queuesList, CurrentSpeed, "lastOnly", True)
        self.imuSubscriber = messageHandlerSubscriber(self.queuesList, ImuData, "lastOnly", True)
        self.objectDetectSubscriber = messageHandlerSubscriber(self.queuesList, ObjectDetect, "lastOnly", True)
        self.liveAnimationSender = messageHandlerSender(self.queuesList, LiveAnimation)
        self.poseSender = messageHandlerSender(self.queuesList, LocalizerPose)

    def state_change_handler(self):
        pass

    def thread_work(self):
        """This function runs continuously in a loop provided by ThreadWithStop."""
        # GUARD: If initialization failed, do not run the filter math.
        if not self.init_success:
            time.sleep(1) # Sleep to prevent CPU spamming while dead
            return

        # Calculate dynamic delta time (DT)
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # --- A. FETCH REAL VEHICLE DATA ---
        
        # 1. Speed Data
        if self.currentSpeedSubscriber.is_data_in_pipe():
            currentSpeedRecv = self.currentSpeedSubscriber.receive()
            if currentSpeedRecv is not None:
                self.vehicle_speed = currentSpeedRecv
                self.vehicle_speed /= 1000  # Convert mm/s to m/s
                self.lastValidSpeed = self.vehicle_speed
        
        v_est = float(self.vehicle_speed) if self.vehicle_speed is not None else float(self.lastValidSpeed)

        # 2. IMU Data
        omega_raw = 0.0
        if self.imuSubscriber.is_data_in_pipe():
            imu_data_recv = self.imuSubscriber.receive()
            if imu_data_recv is not None:
                try:
                    data = ast.literal_eval(imu_data_recv) 
                    current_yaw_deg = float(data['yaw'])
                    
                    if self.prev_yaw is not None:
                        # Calculate the change in angle
                        diff = current_yaw_deg - self.prev_yaw

                        # Handle the "Wrap-around"
                        if diff > 180: diff -= 360
                        if diff < -180: diff += 360

                        # Convert degrees to radians
                        diff_rad = np.deg2rad(diff)
                        
                        # Calculate Rate: Omega = delta_theta / dt
                        if dt > 0:
                            omega_raw = diff_rad / dt
                            
                    self.prev_yaw = current_yaw_deg
                    
                except Exception as e:
                    print(f"[ Localizer ] IMU Parse Error: {e}")
        
        # 3. Camera Detections
        detections = [] 
        if self.objectDetectSubscriber.is_data_in_pipe():
            obj_data = self.objectDetectSubscriber.receive()
            if obj_data is not None:
                for obj in obj_data:
                    label = obj.get('label', '')
                    
                    # Constraint: Ignore mid_stop_line / EnterIntersection
                    if label == "EnterIntersection":
                        continue
                    
                    # Only accept detections that have a valid distance measurement
                    dist = obj.get('distance_m')
                    bearing = obj.get('bearing_rad')
                    
                    if dist is not None and bearing is not None:
                        detections.append(Detection(
                            lm_type=label, 
                            bearing=bearing, 
                            distance=dist
                        ))
                        
                # Optional Debug: Print to verify the localizer is seeing the valid detections
                if len(detections) > 0 and self.debugging:
                    det_strs = [f"{d.lm_type} ({d.distance:.2f}m, {np.rad2deg(d.bearing):.1f}°)" for d in detections]
                    # print(f"\033[1;94m[ Localizer ] Detections: {', '.join(det_strs)}\033[0m", flush=True)

        # --- B. FILTER EXECUTION ---
        try:
            self.localizer.predict(v_est, omega_raw, dt)
            self.localizer.update(detections)

            if self.localizer.mode in ("GLOBAL", "RECOVER"):
                x_est, y_est, theta_est = self.localizer.pf.estimate() 
            else:
                x_est, y_est, theta_est = self.localizer.pose

            if self.debugging:
                self.logging.debug(f"Loc: x={x_est:.2f}, y={y_est:.2f}, mode={self.localizer.mode}")

            particles = self.localizer.pf.particles
            animation_payload = {
                "particles": [[round(float(p.x), 3), round(float(p.y), 3), round(float(p.weight), 4)] for p in particles],
                "pose": [round(float(x_est), 3), round(float(y_est), 3), round(float(theta_est), 4)],
                "mode": str(self.localizer.mode),
            }
            self.liveAnimationSender.send(animation_payload)
            pose_payload = {
                "pose": [float(x_est), float(y_est), float(theta_est)],
                "mode": str(self.localizer.mode)
            }
            self.poseSender.send(pose_payload)

        except Exception as e:
            print(f"\033[1;91m[ Localizer ] :\033[0m \033[1;91mERROR\033[0m - ERROR - Filter step failed: {e}\033[0m", flush=True)

        time.sleep(0.1)