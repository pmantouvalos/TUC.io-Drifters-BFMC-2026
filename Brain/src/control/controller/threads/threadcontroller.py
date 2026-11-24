import time
import math
import socket
import json
import ast
import numpy as np
from collections import deque, Counter

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.control.utils.controlMode import ControlMode
from src.statemachine.stateMachine import StateMachine
from src.data.TrafficCommunication.useful.sharedMem import sharedMem
from src.pathplanning.pathPlanner import PathPlanner

from src.utils.messages.allMessages import (
    CurrentSpeed,          # Speed telemetry for the server
    ImuData,               # Orientation/IMU telemetry for the server
    SteerMotor,
    SpeedMotor,
    VisionManeuver,
    StateChange,
    Klem,
    JoyAxes,
    JoyButton,
    TOFRear,
    TOFLeft,
    TOFRight,
    TOFForward,            # Front-facing Time-of-Flight
    ObjectDetect,
    ParkingScanGate,
    Semaphores,            # V2X — semaphore states
    Location,              # V2X — localization
    NavigateTo,            # Dashboard target node designation
    CurrentNode,
    IntersectionMask,      # Controller nearest node mask (for dashboard)
    LeftLineClassifier,    # Left lane boundary (solid/dashed)
    RightLineClassifier,   # Right lane boundary (solid/dashed)
    NavCommand
)


class TrajectoryFollower:
    """
    Mathematical trajectory generator and follower for smooth intersection turns.
    Utilizes cubic bezier coefficients and a bicycle kinematic model.
    """
    def __init__(self):
        # Left Turn Coefficients (Generated with K=0.75, start=0.55, end=0.30)
        self.coeffs_x_left = [1.3825, -2.3929, 0.0000, 4.9777]
        self.coeffs_y_left = [1.6260, -4.2475, 3.6171, 6.0044]

        # Right Turn Coefficients (Generated with K=1.0, start=0.48, end=0.40)
        self.coeffs_x_right = [-0.8862, 1.5251, 0.0000, 2.7935]
        self.coeffs_y_right = [1.0374, -2.6658, 2.2196, 6.0511]

        # Forward / Straight Coefficients (Generated with K=1.0, start=0.5, end=0.5)
        self.coeffs_x_forward = [0.0000, 0.0000, 0.0000, 5.0074]
        self.coeffs_y_forward = [0.0165, 0.0000, 1.6324, 3.1578]

        self.L_WHEELBASE = 0.255  # Wheelbase in meters
        self.reset("forward")

        self.ENABLE_FEED_FORWARD = False
        

    def reset(self, direction="forward"):
        self.is_active = False
        self.t_current = 0.0
        self.car_pos_local = np.array([0.0, 0.0])
        self.current_direction = direction

        if direction == "right":
            self.coeffs_x = self.coeffs_x_right
            self.coeffs_y = self.coeffs_y_right
        elif direction == "left":
            self.coeffs_x = self.coeffs_x_left
            self.coeffs_y = self.coeffs_y_left
        else: 
            self.coeffs_x = self.coeffs_x_forward
            self.coeffs_y = self.coeffs_y_forward
            
        self.START_POS_TRACK = np.array([self.coeffs_x[3], self.coeffs_y[3]])
        
        dx = 3*self.coeffs_x[0]*0**2 + 2*self.coeffs_x[1]*0 + self.coeffs_x[2]
        dy = 3*self.coeffs_y[0]*0**2 + 2*self.coeffs_y[1]*0 + self.coeffs_y[2]
        self.car_yaw = math.atan2(dy, dx)
        self.last_time = time.time()
        self.math_steering_rad = 0.0 

    def start_turn(self, direction="forward"):
        self.reset(direction)
        self.is_active = True
        self.last_time = time.time()

    def get_path_point(self, t):
        x = self.coeffs_x[0]*t**3 + self.coeffs_x[1]*t**2 + self.coeffs_x[2]*t + self.coeffs_x[3]
        y = self.coeffs_y[0]*t**3 + self.coeffs_y[1]*t**2 + self.coeffs_y[2]*t + self.coeffs_y[3]
        return np.array([x, y])

    def update(self, stanley_gain):
        if not self.is_active: return 0.0
        
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        v_m_s = 0.20
        
        # 1. Update Kinematics (Bicycle Model)
        self.car_yaw += (v_m_s / self.L_WHEELBASE) * math.tan(self.math_steering_rad) * dt
        self.car_pos_local[0] += v_m_s * math.cos(self.car_yaw) * dt
        self.car_pos_local[1] += v_m_s * math.sin(self.car_yaw) * dt

        current_global_pos = self.car_pos_local + self.START_POS_TRACK
        
        # 2. Find nearest point on curve (t_current)
        t_search = np.linspace(self.t_current, min(1.0, self.t_current + 0.2), 50)
        dists = [np.linalg.norm(current_global_pos - self.get_path_point(t)) for t in t_search]
        self.t_current = t_search[np.argmin(dists)]

        if self.t_current >= 0.98: 
            self.is_active = False

        # 3. Calculate Math Errors 
        current_point = self.get_path_point(self.t_current)
        dx_curr = 3*self.coeffs_x[0]*self.t_current**2 + 2*self.coeffs_x[1]*self.t_current + self.coeffs_x[2]
        dy_curr = 3*self.coeffs_y[0]*self.t_current**2 + 2*self.coeffs_y[1]*self.t_current + self.coeffs_y[2]
        current_yaw = math.atan2(dy_curr, dx_curr)
        
        e = ((current_global_pos[0] - current_point[0]) * math.sin(current_yaw) - 
             (current_global_pos[1] - current_point[1]) * math.cos(current_yaw))
        # 0.025 left, 0.15 right
        if self.current_direction == "left":
            lookahead = 0.025
        elif self.current_direction == "right":
            lookahead = 0.15
        else:
            lookahead = 0.1  # forward/straight default

        t_target = self.t_current + lookahead
        dx_targ = 3*self.coeffs_x[0]*t_target**2 + 2*self.coeffs_x[1]*t_target + self.coeffs_x[2]
        dy_targ = 3*self.coeffs_y[0]*t_target**2 + 2*self.coeffs_y[1]*t_target + self.coeffs_y[2]
        target_yaw = math.atan2(dy_targ, dx_targ)
        
        theta_e_rad = (target_yaw - self.car_yaw + math.pi) % (2 * math.pi) - math.pi

        # 4. Pure Mathematical Stanley Control
        delta = theta_e_rad + math.atan2(stanley_gain * e, v_m_s)
        
        # 5. 5-Degree Feed-Forward Kick
        if self.t_current < 0.20 and self.ENABLE_FEED_FORWARD:
            if self.current_direction == "right":
                delta -= math.radians(25.0) 
            elif self.current_direction == "left":
                delta += math.radians(5.0) 

        MAX_STEER = math.radians(25)
        delta = max(-MAX_STEER, min(MAX_STEER, delta))
        self.math_steering_rad = delta
        
        steer_deg = math.degrees(-delta)
        
        return steer_deg
    
    
# ═════════════════════════════════════════════════════════════════════════════
# CONFIGURATION CONSTANTS
# ═════════════════════════════════════════════════════════════════════════════

AXES_MAX = 1023
STEERING_MIN = -250
STEERING_MAX = 250
THROTTLE_MIN = -500
THROTTLE_MAX = 500
STEERING_DEADBAND = 15
THROTTLE_DEADBAND = 20

NORMAL_SPEED = 200
HIGHWAY_SPEED = 400
PARKING_SPEED = 150
OVERTAKE_SPEED = 500

PARK_SIGN_SPEED = 150
PARK_BLIND_DURATION = 1.5
PARK_ENTER_BLIND_DURATION = 4.0
PARK_SCAN_DELAY = 0.3
PARK_FREE_CONFIRM = 1.2
PARK_ALL_FREE_DELAY = 2.5
PARK_HALF_LEN_TIME = 0.8
PARK_REVERSE_SPEED = -150
PARK_STEER_ANGLE = 250
PARK_CORRECT_ANGLE = -250
PARK_SAFE_REAR = 70  # (mm) distance to consider car safely in the spot
PARK_REVERSE_TIMEOUT = 3.5
PARK_CORRECT_TIMEOUT = 3.0
PARK_FWD_SPEED = 100
PARK_FWD_TIMEOUT = 2.5
PARK_STOP_WAIT = 5.0
UNPARK_SPEED = 150
UNPARK_DURATION = 3.0
PARK_COOLDOWN = 60.0
PARK_FREE_SPOT_TIME = (740 / PARK_SIGN_SPEED)  # Reference

PARK_OCC_CONFIRM_FRAMES = 3
PARK_FREE_CONFIRM_FRAMES = 2
PARK_EXTRA_FWD_TIME = 2.0
PARK_CRAWL_SPEED = 150

TOF_BUFFER_SIZE = 7
TOF_FREE_HIGH = 300  
TOF_FREE_LOW = 300  
PARK_GRACE_FRAMES = 4

STEER_LEFT_TIME = 1
STEER_RIGHT_TIME = 1
RIGHT_STEER = 250
LEFT_STEER = -500

passing_from_left = False
passing_from_right = False

GRAPHML_PATH = "src/utils/Competition_track_graph.graphml"
WAYPOINT_THRESHOLD = 0.35  # meters

ROLLING_CHECKPOINTS = [
    "192", "248", "257", "54", "291", "200", 
    "6", "301", "42", "201", "290",
]

OBSTACLE_IDS = {
    "Stop": 1,
    "Priority": 2,
    "Parking": 3,
    "Crosswalk": 4,
    "STOP_LINE": 4,
    "HighwayEntry": 5,
    "HighwayExit": 6,
    "Roundabout": 7,
    "Pedestrian": 11,
    "Car": 10,
}


def constrain_num(val, min_val, max_val):
    return min(max_val, max(min_val, val))


def map_range(x, in_min, in_max, out_min, out_max):
    tmp_x = min(in_max, max(in_min, x))
    return (tmp_x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class threadcontroller(ThreadWithStop):
    """
    Main controller thread processing:
      - Lane keeping (Stanley controller parameters mapped from Oak-D)
      - Object detection behaviors (Stop signs, crosswalks, pedestrians, highway, parking)
      - V2X semaphore tracking (Stops on RED)
      - V2X localization via DWM1001
      - Dynamic Path Planning (Dijkstra)
      - Telemetry distribution via UDP
    """

    detections = []
    stop_buffer = np.array([0, 0, 0, 0, 0])
    index = 0
    seen_stop = False

    def __init__(self, sharedMemory, queueList, logging, debugging=False):
        try:
            super(threadcontroller, self).__init__(pause=0.03)
            self.queuesList = queueList
            self.logging = logging
            self.debugging = debugging
            self.seen_stop_sign = False

            self.stateMachine = StateMachine.get_instance()
            self.control_mode = ControlMode.INIT
            self.act_enabled = False
            self._klem_received = False

            self.highway_flag = False
            self._has_started = False
            self.sharedMemory = sharedMemory

            # ── Subscribers ───────────────────────────────────────────────
            self.klSubscriber = messageHandlerSubscriber(self.queuesList, Klem, "lastOnly", True)
            self.joyAxesSubscriber = messageHandlerSubscriber(self.queuesList, JoyAxes, "lastOnly", True)
            self.joyButtonSubscriber = messageHandlerSubscriber(self.queuesList, JoyButton, "lastOnly", True)
            self.stateChangeSubscriber = messageHandlerSubscriber(self.queuesList, StateChange, "lastOnly", True)
            self.visionManeuverSubscriber = messageHandlerSubscriber(self.queuesList, VisionManeuver, "lastOnly", True)
            self.tofRearSubscriber = messageHandlerSubscriber(self.queuesList, TOFRear, "lastOnly", True)
            self.tofLeftSubscriber = messageHandlerSubscriber(self.queuesList, TOFLeft, "lastOnly", True)
            self.tofRightSubscriber = messageHandlerSubscriber(self.queuesList, TOFRight, "lastOnly", True)
            self.tofForwardSubscriber = messageHandlerSubscriber(self.queuesList, TOFForward, "lastOnly", True)
            self.objectDetectSubscriber = messageHandlerSubscriber(self.queuesList, ObjectDetect, "lastOnly", True)
            self.currentSpeedSubscriber = messageHandlerSubscriber(self.queuesList, CurrentSpeed, "lastOnly", True)
            self.imuSubscriber = messageHandlerSubscriber(self.queuesList, ImuData, "lastOnly", True)
            self.leftLineClassifierSubscriber = messageHandlerSubscriber(self.queuesList, LeftLineClassifier, "lastOnly", True)
            self.rightLineClassifierSubscriber = messageHandlerSubscriber(self.queuesList, RightLineClassifier, "lastOnly", True)
            self.navCommandSubscriber = messageHandlerSubscriber(self.queuesList, NavCommand, "lastOnly", True)

            # ── V2X Subscribers ────────────────────────────────────────────
            self.semaphoresSubscriber = messageHandlerSubscriber(self.queuesList, Semaphores, "lastOnly", True)
            self.locationSubscriber = messageHandlerSubscriber(self.queuesList, Location, "lastOnly", True)
            self.navigateToSubscriber = messageHandlerSubscriber(self.queuesList, NavigateTo, "lastOnly", True)
            self.intersectionMaskSubscriber = messageHandlerSubscriber(self.queuesList, IntersectionMask, "lastOnly", True)
            
            # ── Cached ToF ────────────────────────────────────────────────
            self.tof_rear = 9999
            self.tof_left = 9999
            self.tof_right = 9999
            self.tof_forward = 9999
            
            # ── ToF Robustness ────────────────────────────────────────────
            self._tof_left_buf = deque(maxlen=TOF_BUFFER_SIZE)
            self._tof_right_buf = deque(maxlen=TOF_BUFFER_SIZE)
            self._tof_left_free = False
            self._tof_right_free = False
            self._park_grace_counter = 0

            # ── Publishers ────────────────────────────────────────────────
            self.steerMotorSender = messageHandlerSender(self.queuesList, SteerMotor)
            self.speedMotorSender = messageHandlerSender(self.queuesList, SpeedMotor)
            self.parkingScanSender = messageHandlerSender(self.queuesList, ParkingScanGate)
            self.currentNodeSender = messageHandlerSender(self.queuesList, CurrentNode)
            self._last_published_node = None

            # ── V2X State ──────────────────────────────────────────────────
            self.semaphore_states = {}
            self.nearest_semaphore_state = -1
            self.car_x = 0.0
            self.car_y = 0.0
            self.loc_quality = 0

            self.shared_mem = None
            self._last_speed = 0
            self._last_steering_cmd = "0"
            self._last_detection = None
            self.car_last_seen_time = None
            
            self.next_instruction = "right"
            self.lane_preference = "center"
            
            self.INTERSECTION_NODES = [
                "94", "21", "9", "103", "33", "45", "76", "200", "85", "57",
                "207", "78", "210", "59", "22", "48", "35", "95", "105", "96",
                "11", "104", "36", "46", "47", "290", "77", "60", "23"
            ]

            # ── Roundabout State ──────────────────────────────────────────
            self.target_roundabout_exit = 2
            self.passed_exits = 0
            self._roundabout_yield_start = None
            self._roundabout_exit_start = None
            self._right_line_was_visible = True
            self._right_line_lost_frames = 0

            # ── IMU State ─────────────────────────────────────────────────
            self.current_yaw = 0.0
            self.accumulated_yaw = 0.0
            self.last_yaw = 0.0

            self.speed_state = None
            self.trajectory_follower = TrajectoryFollower()

            # ── Path Planning ──────────────────────────────────────────────
            try:
                self.planner = PathPlanner(GRAPHML_PATH)
            except Exception as e:
                print(f"\033[1;97m[ Controller Thread ] :\033[0m \033[1;93mWARN\033[0m - PathPlanner: {e}")
                self.planner = None

            self.current_path = []
            self.current_node_idx = 0
            self._checkpoint_idx = 0
            self.target_node_id = ROLLING_CHECKPOINTS[0]
            self.path_heading = None
            self._fixed_target = False

            # ── Parking State ─────────────────────────────────────────────
            self._park_sign_seen_time = None
            self._park_scan_active = False
            self._park_seen_wall = False
            self._park_side = None
            self._park_free_confirm = None
            self._park_all_free_timer = None
            self._park_position_start = None
            self._park_reverse_start = None
            self._park_correct_start = None
            self._park_fwd_start = None
            self._park_stop_time = None
            self._unpark_start_time = None
            self._parking_completed = False
            self._parking_done_time = None
            self._parking_spot_detection_time = None
            self._parking_move_fwd_bfr_maneuver = None
            self._park_blind_enter_time = None
            self._last_parking_condition = None

            self._searching_for_stop_line = False
            self._last_printed_mode = None
            self._stop_timer = None 

            self._occ_frames_left = 0
            self._occ_frames_right = 0
            self._free_frames_left = 0
            self._free_frames_right = 0
            self._wall_seen_left = False
            self._wall_seen_right = False
            self._park_commit_time = None
            self._park_align_start = None
            self._free_timer_left = None
            self._free_timer_right = None

            # ── Intersection Timers ────────────────────────────────────────
            self.fwd_start = None
            self.turn_start = None
            self._intersection_scan_start = None
            self._mask_buffer = []

            # ── Overtake Timers ───────────────────────────────────────────
            self.overtake_start = None
            self.overtake_correct = None

            # ── Rolling Semaphore Fingerprinting ──────────────────────────
            self._semaphore_suspects = []
            self._pending_visual_state = None
            self._current_visual_state = None
            self._visual_state_timer = None
            self._converged_semaphore_id = None
            self._blind_fallback_timer = None
            self._fingerprint_active = False

            self.subscribe_list = [
                self.klSubscriber, self.joyAxesSubscriber, self.joyButtonSubscriber,
                self.stateChangeSubscriber, self.visionManeuverSubscriber, self.tofRearSubscriber,
                self.tofLeftSubscriber, self.tofRightSubscriber, self.tofForwardSubscriber,
                self.semaphoresSubscriber, self.locationSubscriber, self.navigateToSubscriber,
                self.currentSpeedSubscriber, self.imuSubscriber, self.intersectionMaskSubscriber,
                self.leftLineClassifierSubscriber, self.rightLineClassifierSubscriber, self.navCommandSubscriber
            ]

            # ── Telemetry Output ──────────────────────────────────────────
            try:
                self._telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._telem_sock.setblocking(False)
                self._telem_ip = "192.168.50.100"
                self._telem_port = 9002
            except Exception:
                self._telem_sock = None

            print("\033[1;97m[ Controller Thread ] :\033[0m \033[1;92mINFO\033[0m - Controller initialized")

        except Exception as e:
            self.try_unsubscribe()
            print(f"\033[1;97m[ Controller Thread ] :\033[0m \033[1;91mERROR\033[0m - Failed: {e}")

    # ─────────────────────────────────────────────────────────────────────────
    # STATE CHANGE HANDLER
    # ─────────────────────────────────────────────────────────────────────────

    def state_change_handler(self):
        message = self.stateChangeSubscriber.receive()
        if message is not None:
            match message:
                case "DEFAULT" | "STOP" | "LEGACY":
                    self.control_mode = ControlMode.INIT
                    print(f"\033[1;97m[ Controller Thread ] :\033[0m \033[1;92mINFO\033[0m - Mode -> {message}")
                case "MANUAL":
                    self.control_mode = ControlMode.MANUAL
                    print("\033[1;97m[ Controller Thread ] :\033[0m \033[1;92mINFO\033[0m - Mode -> MANUAL")
                case "AUTO":
                    self.control_mode = ControlMode.AUTO
                    print("\033[1;97m[ Controller Thread ] :\033[0m \033[1;92mINFO\033[0m - Mode -> AUTO")

    # ─────────────────────────────────────────────────────────────────────────
    # LIFECYCLE
    # ─────────────────────────────────────────────────────────────────────────

    def try_unsubscribe(self):
        try:
            for sub in self.subscribe_list:
                try:
                    sub.unsubscribe()
                except Exception:
                    pass
        except Exception:
            pass

    def stop(self):
        self.try_unsubscribe()
        self.control_mode = ControlMode.INIT
        self.steerMotorSender.send("0")
        self.speedMotorSender.send("0")
        self.stateMachine.request_mode("dashboard_stop_button")
        super(threadcontroller, self).stop()

    def stopTOF(self):
        self.control_mode = ControlMode.AUTO
        self.steerMotorSender.send("0")
        self.speedMotorSender.send("0")
        self.stateMachine.request_mode("TOF_stop")
        super(threadcontroller, self).stopTOF()

    # ─────────────────────────────────────────────────────────────────────────
    # HELPERS — Motor
    # ─────────────────────────────────────────────────────────────────────────
    
    def _log_once(self, message):
        """Logs mode transitions cleanly without spamming the console."""
        if self.control_mode != self._last_printed_mode:
            print(f"\033[1;34m[ MODE ]\033[0m {message}")
            self._last_printed_mode = self.control_mode

    def _send_speed(self, value: int):
        self._last_speed = value
        self.speedMotorSender.send(str(int(value)))

    def _apply_lane_keeping(self, visionRecv, axesRecv, speed_override=None, side="center"):
        if speed_override is not None:
            self._send_speed(speed_override)
        elif axesRecv is not None:
            tv = map_range(float(axesRecv["throttle"]), 0.0, AXES_MAX, THROTTLE_MIN, THROTTLE_MAX)
            if abs(tv) < THROTTLE_DEADBAND:
                tv = 0.0
            self._send_speed(int(tv))

        if not hasattr(self, "_last_steering_cmd"):
            self._last_steering_cmd = "0"

        if visionRecv is not None:
            key = {
                "left": "steering_left",
                "right": "steering_right",
                "center": "steering_center",
            }.get(side, "steering_center")
            
            raw = visionRecv.get(key) or visionRecv.get("steering", "250")
            sv = constrain_num(
                map_range(float(raw), 0.0, 500.0, float(STEERING_MIN), float(STEERING_MAX)),
                STEERING_MIN,
                STEERING_MAX,
            )
            self._last_steering_cmd = str(int(sv))

        self.steerMotorSender.send(self._last_steering_cmd)

    # ─────────────────────────────────────────────────────────────────────────
    # HELPERS — Parking
    # ─────────────────────────────────────────────────────────────────────────

    def _set_scan_gate(self, active: bool):
        if self._park_scan_active != active:
            self._park_scan_active = active
            self.parkingScanSender.send(active)
            print(f">>> [SCAN GATE] {'ON' if active else 'OFF'}")

    def _update_tof_state(self, side: str, raw_value):
        buf = self._tof_left_buf if side == "left" else self._tof_right_buf
        state_attr = "_tof_left_free" if side == "left" else "_tof_right_free"

        buf.append(raw_value)
        current = getattr(self, state_attr)

        if current is True:
            if raw_value < TOF_FREE_LOW:
                setattr(self, state_attr, False)
                buf.clear()
        else:
            if len(buf) >= max(1, TOF_BUFFER_SIZE // 2):
                median = float(np.median(buf))
                if median > TOF_FREE_HIGH:
                    setattr(self, state_attr, True)

        return getattr(self, state_attr)

    def _reset_parking_timers(self):
        self._park_sign_seen_time = None
        self._park_seen_wall = False
        self._park_side = None
        self._park_free_confirm = None
        self._park_all_free_timer = None
        self._park_position_start = None
        self._park_reverse_start = None
        self._park_correct_start = None
        self._park_fwd_start = None
        self._park_stop_time = None
        self._parking_spot_detection_time = None
        self._parking_move_fwd_bfr_maneuver = None
        self._park_blind_enter_time = None
        self._last_parking_condition = None
        self._park_grace_counter = 0
        self._occ_frames_left = 0
        self._occ_frames_right = 0
        self._free_frames_left = 0
        self._free_frames_right = 0
        self._wall_seen_left = False
        self._wall_seen_right = False
        self._park_commit_time = None
        self._park_align_start = None
        self._free_timer_left = None
        self._free_timer_right = None
        self._set_scan_gate(False)

    def _commit_to_bay(self, side: str, reason: str):
        print("=" * 55)
        print(f">>> [FIND_SPOT] SPOT {side.upper()} SELECTED ({reason})")
        print("=" * 55)
        self._park_side = side
        self._park_free_confirm = None
        self._park_all_free_timer = None
        self._park_seen_wall = False
        self._park_position_start = None
        self.control_mode = ControlMode.PARKING_POSITION

    # ─────────────────────────────────────────────────────────────────────────
    # HELPERS — V2X + Path Planning
    # ─────────────────────────────────────────────────────────────────────────

    def _update_semaphores(self, recv):
        if recv is None:
            return
        sid = recv.get("id", "?")
        self.semaphore_states[sid] = recv
        if self.semaphore_states:
            nearest = min(
                self.semaphore_states.values(),
                key=lambda s: (s["x"] - self.car_x) ** 2 + (s["y"] - self.car_y) ** 2,
            )
            self.nearest_semaphore_state = nearest["state"]

    def _update_location(self, recv):
        if recv is None:
            return
        print(f"[LOC] x={recv['x']:.3f} y={recv['y']:.3f}")
        self.car_x = recv["x"]
        self.car_y = recv["y"]
        self.loc_quality = recv["quality"]

        # --- Intersection Queue Skipper ---
        if self.planner and getattr(self.planner, 'G', None):
            best_dist = float('inf')
            nearest_int_node = None
            
            for node in getattr(self, 'INTERSECTION_NODES', []):
                if node in self.planner.G.nodes:
                    nx_x = float(self.planner.G.nodes[node]["x"])
                    nx_y = float(self.planner.G.nodes[node]["y"])
                    dist = np.hypot(nx_x - self.car_x, nx_y - self.car_y)
                    if dist < best_dist:
                        best_dist = dist
                        nearest_int_node = node
            
            if best_dist < 1.0:
                if getattr(self, '_current_near_int_node', None) is None:
                    self._current_near_int_node = nearest_int_node
                    
            elif best_dist >= 1.0 and getattr(self, '_current_near_int_node', None) is not None:
                passed_node = self._current_near_int_node
                self._current_near_int_node = None 
                
                if self.control_mode in [ControlMode.AUTO, ControlMode.MOVE]:
                    if time.time() > getattr(self, "_ignore_intersections_until", 0):
                        print(f"\033[1;93m>>> [NAV] Passed intersection {passed_node} without turning. Skipping command: {self.next_instruction.upper()}\033[0m")
                        
                        if hasattr(self, 'maneuver_queue') and len(self.maneuver_queue) > 0:
                            next_cmd = self.maneuver_queue.pop(0)
                            self.next_instruction = next_cmd.replace("TURN_", "").replace("GO_", "").lower()
                            print(f">>> [NAV] Chamber reloaded with next: {self.next_instruction.upper()}")
                        else:
                            self.next_instruction = "straight"

        print("ATTEMPTING SHARED MEMORY")
        if self.shared_mem is not None:
            self.shared_mem.insert("devicePos", [self.car_x, self.car_y])
            self.shared_mem.insert("deviceSpeed", [abs(self._last_speed)])
            print(f"DEVICE POS: {[self.car_x, self.car_y]}")
        else:
            print("SHARED MEMORY NONE")

    def _update_path_progress(self):
        if self.planner is None:
            return

        if not self.current_path:
            nearest = self.planner.find_nearest_node(self.car_x, self.car_y)
            if nearest and nearest != self.target_node_id:
                self.current_path = self.planner.dijkstra(nearest, self.target_node_id)
                self.current_node_idx = 0
                if self.debugging:
                    print(f"[ PathPlanner ] {self.planner.path_info(self.current_path)}")
            return

        if self.current_node_idx < len(self.current_path):
            wid = self.current_path[self.current_node_idx]
            dist = self.planner.distance_to_node(self.car_x, self.car_y, wid)
            if dist < WAYPOINT_THRESHOLD:
                self.current_node_idx += 1

        if self.current_node_idx >= len(self.current_path):
            dist_cp = self.planner.distance_to_node(self.car_x, self.car_y, self.target_node_id)
            if dist_cp < WAYPOINT_THRESHOLD * 2:
                print(f"\033[1;97m[ PathPlanner ] :\033[0m \033[1;92mINFO\033[0m - Target node {self.target_node_id} REACHED")
                if self._fixed_target:
                    self._fixed_target = False
                    self._send_speed(0)
                    self.steerMotorSender.send("0")
                    print("\033[1;97m[ PathPlanner ] :\033[0m \033[1;92mINFO\033[0m - Fixed-target navigation complete. Car stopped.")
                    return
                self._advance_checkpoint()
                return

        self.path_heading = self.planner.heading_to_next(self.current_path, self.current_node_idx)

    def _report_obstacles(self):
        if self.shared_mem is None:
            return
        for det in self.detections:
            obs_id = OBSTACLE_IDS.get(det.get("label", ""))
            if obs_id is not None:
                self.shared_mem.insert("historyData", [obs_id, self.car_x, self.car_y])

    def _send_telemetry(self):
        if self._telem_sock is None:
            return
        try:
            payload = json.dumps(
                {
                    "mode": self.control_mode.name,
                    "detections": self.detections,
                    "car_x": self.car_x,
                    "car_y": self.car_y,
                    "checkpoint": self._checkpoint_idx,
                    "target_node": self.target_node_id,
                    "path_len": len(self.current_path),
                    "heading": self.path_heading,
                }
            ).encode()
            self._telem_sock.sendto(payload, (self._telem_ip, self._telem_port))
        except Exception:
            pass

    # ─────────────────────────────────────────────────────────────────────────
    # MAIN LOOP
    # ─────────────────────────────────────────────────────────────────────────

    def thread_work(self):
        try:
            self.state_change_handler()

            klemRecv = self.klSubscriber.receive()
            buttonRecv = self.joyButtonSubscriber.receive()
            axesRecv = self.joyAxesSubscriber.receive()
            visionRecv = self.visionManeuverSubscriber.receive()
            tofRearRecv = self.tofRearSubscriber.receive()
            tofLeftRecv = self.tofLeftSubscriber.receive()
            tofRightRecv = self.tofRightSubscriber.receive()
            tofForwardRecv = self.tofForwardSubscriber.receive()
            objectDetectRecv = self.objectDetectSubscriber.receive()
            semaphoresRecv = self.semaphoresSubscriber.receive()
            locationRecv = self.locationSubscriber.receive()
            navigateToRecv = self.navigateToSubscriber.receive()
            currentSpeedRecv = self.currentSpeedSubscriber.receive()
            imuRecv = self.imuSubscriber.receive()
            intersectionMaskRecv = self.intersectionMaskSubscriber.receive()
            leftLineClassifierRecv = self.leftLineClassifierSubscriber.receive()
            rightLineClassifierRecv = self.rightLineClassifierSubscriber.receive()

            if imuRecv is not None:
                data = ast.literal_eval(imuRecv)  
                yaw = float(data["yaw"])
                self.current_yaw = yaw  
                self.sharedMemory.insert("deviceRot", [yaw])

            if currentSpeedRecv is not None:
                currentSpeedRecv = currentSpeedRecv / 10
                self.sharedMemory.insert("deviceSpeed", [round(currentSpeedRecv), 2])  

            # ── Navigator Commands ─────────────────────────────────────────
            if self.navCommandSubscriber.is_data_in_pipe(): 
                navCommandRecv = self.navCommandSubscriber.receive()
                
                sequence = navCommandRecv.get("maneuver_sequence")
                lane_pref = navCommandRecv.get("lane_preference")
                stop_cmd = navCommandRecv.get("command")
                
                if lane_pref is not None:
                    self.lane_preference = lane_pref

                if stop_cmd == "STOP_AT_DESTINATION":
                    print("\033[1;95m>>> [ NAVIGATOR ] DESTINATION REACHED. STOPPING.\033[0m")
                    self.control_mode = ControlMode.INIT

                if sequence is not None:
                    print(f"\033[1;95m>>> [ NAVIGATOR ] Queue Loaded: {sequence}\033[0m")
                    self.maneuver_queue = sequence
                    
                    if len(self.maneuver_queue) > 0:
                        next_cmd = self.maneuver_queue.pop(0)
                        self.next_instruction = next_cmd.replace("TURN_", "").replace("GO_", "").lower()

            # ── Cache ToF ─────────────────────────────────────────────────
            if tofRearRecv is not None:
                self.tof_rear = tofRearRecv
            if tofForwardRecv is not None:
                self.tof_forward = tofForwardRecv
            if tofLeftRecv is not None:
                self.tof_left = tofLeftRecv
                self._update_tof_state("left", tofLeftRecv)
            if tofRightRecv is not None:
                self.tof_right = tofRightRecv
                self._update_tof_state("right", tofRightRecv)

            # ── Update detections ─────────────────────────────────────────
            if objectDetectRecv is not None:
                self.detections = (
                    objectDetectRecv
                    if isinstance(objectDetectRecv, list)
                    else [objectDetectRecv]
                )
                detected_labels = [d.get("label") for d in self.detections if d is not None]
                self._last_detection = self.detections 
                self._report_obstacles()
            else:
                self.detections = self._last_detection if self._last_detection is not None else []

            # ── V2X updates ───────────────────────────────────────────────
            self._update_semaphores(semaphoresRecv)
            self._update_location(locationRecv)

            # ── Navigate-to command from dashboard ────────────────────────
            if navigateToRecv is not None:
                target = str(navigateToRecv.get("target", "")).strip()
                if target and self.planner and target in self.planner._pos:
                    self.target_node_id = target
                    self.current_path = []
                    self.current_node_idx = 0
                    self._fixed_target = True
                    print(f"\033[1;97m[ PathPlanner ] :\033[0m \033[1;92mINFO\033[0m - NavigateTo: node {target}")
                else:
                    print(f"\033[1;97m[ PathPlanner ] :\033[0m \033[1;93mWARN\033[0m - NavigateTo: unknown node '{target}'")

            # =================================================================
            # PREPARE
            # =================================================================
            if klemRecv is not None:
                self._klem_received = True
                prev = self.act_enabled
                match str(klemRecv):
                    case "30":
                        self.act_enabled = True
                    case _:
                        self.act_enabled = False
                if prev != self.act_enabled:
                    print(f"\033[1;96m[KLEM]\033[0m act_enabled={self.act_enabled}")
            else:
                if not self._klem_received:
                    self.act_enabled = True

            if buttonRecv is not None:
                match int(buttonRecv):
                    case 0:
                        self.stateMachine.request_mode("dashboard_stop_button")
                    case 1:
                        self.stateMachine.request_mode("dashboard_manual_button")
                    case 2:
                        self.stateMachine.request_mode("dashboard_legacy_button")
                    case 3:
                        self.stateMachine.request_mode("dashboard_auto_button")

            # =================================================================
            # ACT — State Machine Logic
            # =================================================================
            match self.control_mode:

                case ControlMode.INIT:
                    self.steerMotorSender.send("0")
                    self._send_speed(0)
                    self.control_mode = ControlMode.IDLE
                    time.sleep(0.03)

                case ControlMode.IDLE:
                    self.control_mode = ControlMode.START

                case ControlMode.MANUAL:
                    if self.act_enabled and axesRecv is not None:
                        sv = map_range(float(axesRecv["steering"]), 0.0, AXES_MAX, STEERING_MIN, STEERING_MAX)
                        tv = map_range(float(axesRecv["throttle"]), 0.0, AXES_MAX, THROTTLE_MIN, THROTTLE_MAX)
                        if abs(sv) < STEERING_DEADBAND:
                            sv = 0.0
                        if abs(tv) < THROTTLE_DEADBAND:
                            tv = 0.0
                        self.steerMotorSender.send(str(int(constrain_num(sv, STEERING_MIN, STEERING_MAX))))
                        self._send_speed(int(constrain_num(tv, THROTTLE_MIN, THROTTLE_MAX)))
                        time.sleep(0.03)

                case ControlMode.AUTO:
                    self._log_once("AUTO mode")
                    if self.act_enabled:
                        if time.time() < getattr(self, "_ignore_intersections_until", 0):
                            self.detections = [
                                obj for obj in self.detections 
                                if obj is not None and obj.get("label") not in ["Stop", "Priority", "TrafficLight", "STOP_LINE", "Crosswalk"]
                            ]

                        if self.nearest_semaphore_state == 0:  
                            self.steerMotorSender.send("0")
                            self._send_speed(0)
                            if self.debugging:
                                print("[ SEMAPHORE ] RED -> STOP")
                            self._send_telemetry()
                            return

                        self._apply_lane_keeping(visionRecv, axesRecv, NORMAL_SPEED, side=self.lane_preference)

                        if any(
                            obj is not None and obj.get("label") == "Pedestrian" and obj.get("proximity") == "CLOSE"
                            for obj in self.detections
                        ) and not any(
                            obj is not None and obj.get("label") == "Crosswalk" for obj in self.detections
                        ):
                            self.control_mode = ControlMode.SIMPLE_PEDESTRIAN

                        elif (
                            any(
                                obj is not None and obj.get("label") == "Stop" and obj.get("proximity") in ("CLOSE", "MEDIUM", "FAR")
                                for obj in self.detections
                            )
                            or self._searching_for_stop_line
                        ):
                            if any(
                                obj is not None and obj.get("label") == "Stop" and obj.get("proximity") in ("CLOSE", "MEDIUM")
                                for obj in self.detections
                            ):
                                print(">>> Stop sign detected -> searching for stop line")
                            self._searching_for_stop_line = True
                            self.control_mode = ControlMode.FIND_STOP_LINE

                        elif any(
                            obj is not None and obj.get("label") == "Pedestrian" and obj.get("proximity") in ("CLOSE", "MEDIUM")
                            for obj in self.detections
                        ) and any(
                            obj is not None and obj.get("label") == "Crosswalk" and obj.get("proximity") in ("CLOSE", "MEDIUM")
                            for obj in self.detections
                        ):
                            self.control_mode = ControlMode.MOVE_FOR_PEDESTRIAN_IN_CROSSWALK

                        elif any(
                            obj is not None and "traffic" in obj.get("label", "").lower() and obj.get("proximity") in ("CLOSE", "MEDIUM", "FAR")
                            for obj in self.detections
                        ) and not any(
                            obj is not None and obj.get("label") == "GreenLight" for obj in self.detections
                        ):
                            print(">>> Traffic light detected but NOT green -> prepare to stop at stop line")
                            self._searching_for_stop_line = True
                            self.control_mode = ControlMode.FIND_STOP_LINE

                        elif any(
                            obj is not None and obj.get("label") == "Priority" and obj.get("proximity") in ("CLOSE", "MEDIUM", "FAR")
                            for obj in self.detections
                        ):
                            self._log_once("PRIORITY SIGN -> ENTER INTERSECTION")
                            self.control_mode = ControlMode.ENTER_INTERSECTION

                        elif any(
                            obj is not None and obj.get("label") == "HighwayEntry" and obj.get("proximity") == "CLOSE"
                            for obj in self.detections
                        ):
                            self.control_mode = ControlMode.ENTER_HIGHWAY

                        elif any(
                            obj is not None and obj.get("label") == "Parking" and obj.get("proximity") in ("CLOSE")
                            for obj in self.detections
                        ):
                            self._log_once("PARKING SIGN DETECTED -> prepare to park")
                            self.control_mode = ControlMode.ENTER_PARKING

                        elif any(
                            obj is not None and obj.get("label") == "Car" and obj.get("distance_m") is not None
                            and (obj.get("distance_m") < 0.5 or self.tof_forward < 700)
                            and obj.get("bearing_deg") is not None and abs(obj.get("bearing_deg")) < 10
                            for obj in self.detections
                        ):
                            self._log_once("OVERTAKE: Car detected ahead within 0.7m and 20° bearing -> prepare to overtake")

                            if(leftLineClassifierRecv == "dashed"):
                                self._log_once("OVERTAKE (VEER LEFT)")
                                print(self.tof_forward)
                                self.control_mode = ControlMode.OVERTAKE_LEFT
                            elif(rightLineClassifierRecv == "dashed"):
                                self._log_once("OVERTAKE (VEER RIGHT)")
                                print(self.tof_forward)
                                self.control_mode = ControlMode.OVERTAKE_RIGHT
                            else:
                                self.control_mode = ControlMode.AVOID_CAR

                        elif any(
                            obj is not None and obj.get("label") == "Roundabout" and obj.get("proximity") in ("CLOSE", "MEDIUM")
                            for obj in self.detections
                        ):
                            self._log_once("ROUNDABOUT SIGN DETECTED -> Dynamic Approach")
                            instruction = str(self.next_instruction).lower().strip()
                            
                            if instruction == "right":
                                self.target_roundabout_exit = 1
                            elif instruction in ["front", "straight"]:
                                print(f">>> [ROUNDABOUT] Command: '{instruction}'. Target: Straight (Exit 2)")
                                self.target_roundabout_exit = 2
                            elif instruction == "left":
                                self.target_roundabout_exit = 3
                            elif instruction == "u_turn":
                                self.target_roundabout_exit = 4
                            else:
                                print(f">>> [ROUNDABOUT] NavCommand: '{instruction}'. Using DEFAULT: Straight (Exit 2)")
                                self.target_roundabout_exit = 2
                                
                            self.control_mode = ControlMode.ROUNDABOUT_APPROACH

                        else:
                            self._park_sign_seen_time = None
                            self.control_mode = ControlMode.AUTO

                case ControlMode.FIND_STOP_LINE:
                    self._log_once("FIND STOP LINE")
                    self._apply_lane_keeping(visionRecv, axesRecv, NORMAL_SPEED)

                    if not self._fingerprint_active:
                        self._semaphore_suspects = list(self.semaphore_states.keys())
                        self._fingerprint_active = True
                        self._converged_semaphore_id = None
                        print(f">>> [FINGERPRINT] Started tracking. Suspects: {self._semaphore_suspects}")

                    if self._fingerprint_active and not self._converged_semaphore_id:
                        sees_green = any(obj is not None and obj.get("label") == "GreenLight" for obj in self.detections)
                        sees_any_light = any(obj is not None and "traffic" in obj.get("label", "").lower() for obj in self.detections)
                        
                        raw_visual_state = None
                        if sees_green:
                            raw_visual_state = "green"
                        elif sees_any_light:
                            raw_visual_state = "not_green"

                        if raw_visual_state is not None:
                            if raw_visual_state == self._pending_visual_state:
                                if time.time() - self._visual_state_timer >= 0.2:
                                    self._current_visual_state = raw_visual_state
                            else:
                                self._pending_visual_state = raw_visual_state
                                self._visual_state_timer = time.time()

                        if self._current_visual_state is not None:
                            valid_suspects = []
                            for sem_id in self._semaphore_suspects:
                                server_data = self.semaphore_states.get(sem_id)
                                if not server_data: continue

                                server_mapped_state = "green" if server_data["state"] == "green" else "not_green"

                                if server_mapped_state == self._current_visual_state:
                                    valid_suspects.append(sem_id)

                            if len(valid_suspects) < len(self._semaphore_suspects):
                                print(f">>> [FINGERPRINT] Filtering... Remaining suspects: {valid_suspects}")
                                self._semaphore_suspects = valid_suspects

                            if len(self._semaphore_suspects) == 1:
                                self._converged_semaphore_id = self._semaphore_suspects[0]
                                self.car_x = self.semaphore_states[self._converged_semaphore_id]["x"]
                                self.car_y = self.semaphore_states[self._converged_semaphore_id]["y"]
                                if self.shared_mem is not None:
                                    self.shared_mem.insert("devicePos", [self.car_x, self.car_y])
                                    
                                print("=" * 60)
                                print(f">>> [FINGERPRINT] CONVERGED! Semaphore ID: {self._converged_semaphore_id}")
                                print("=" * 60)
                                
                            elif len(self._semaphore_suspects) == 0:
                                print(">>> [FINGERPRINT] WARN: All suspects eliminated. Resetting list.")
                                self._semaphore_suspects = list(self.semaphore_states.keys())
                                self._current_visual_state = None

                    line_seen = any(
                        obj is not None and obj.get("label") == "STOP_LINE" and obj.get("proximity") in ("CLOSE", "MEDIUM")
                        for obj in self.detections
                    )

                    if line_seen:
                        self._searching_for_stop_line = False  
                        if any(
                            obj is not None and obj.get("label") == "TrafficLight" and obj.get("proximity") in ("CLOSE", "MEDIUM")
                            for obj in self.detections
                        ):
                            print(">>> [FIND_STOP_LINE] -> TRAFFIC_LIGHT")
                            self.control_mode = ControlMode.TRAFFIC_LIGHT
                        else:
                            print(">>> [FIND_STOP_LINE] -> STOP_SIGN")
                            self.control_mode = ControlMode.STOP_SIGN

                case ControlMode.STOP_SIGN:
                    self._log_once("STOP SIGN")
                    self._searching_for_stop_line = False
                    self._current_near_int_node = None
                    
                    if self._stop_timer is None:
                        self._stop_timer = time.time()
                        self._send_speed(0)
                        self.steerMotorSender.send("0")
                        print(f">>> [STOP_SIGN] Stopped — waiting 3s. Intended path: {self.next_instruction.upper()}")

                    if time.time() - self._stop_timer < 3.0:
                        self._send_speed(0)
                        self.steerMotorSender.send("0")

                        if any(
                            obj is not None and obj.get("label") == "Car" and obj.get("proximity") in ("CLOSE", "MEDIUM")
                            for obj in self.detections
                        ):
                            print(">>> [STOP_SIGN] Crossing car detected -> holding position and resetting timer")
                            self._stop_timer = time.time()
                    else:
                        print(f">>> [STOP_SIGN] 3s timer finished! Executing: {self.next_instruction.upper()}")
                        self._stop_timer = None
                        self.fwd_start = time.time()
                        
                        if self.next_instruction == "left":
                            self.control_mode = ControlMode.INTERSECTION_LEFT
                        elif self.next_instruction == "right":
                            self.control_mode = ControlMode.INTERSECTION_RIGHT
                        else:
                            self.control_mode = ControlMode.INTERSECTION_FWD

                case ControlMode.TRAFFIC_LIGHT:
                    self._log_once("TRAFFIC LIGHT")
                    self._send_speed(0)
                    self.steerMotorSender.send("0")
                    self._current_near_int_node = None

                    if self._converged_semaphore_id is not None:
                        server_data = self.semaphore_states.get(self._converged_semaphore_id)
                        
                        if server_data and server_data["state"] == "green":
                            print(f">>> [TRAFFIC_LIGHT] Server confirms ID {self._converged_semaphore_id} is GREEN! GO!")
                            self._fingerprint_active = False 
                            self._converged_semaphore_id = None
                            self.control_mode = ControlMode.ENTER_INTERSECTION
                        else:
                            pass
                    else:
                        sees_green = any(obj is not None and obj.get("label") == "GreenLight" for obj in self.detections)
                        sees_any_light = any(obj is not None and "traffic" in obj.get("label", "").lower() for obj in self.detections)
                        
                        if sees_green:
                            print(">>> [TRAFFIC_LIGHT] Visually Green (Un-fingerprinted)! GO!")
                            self._fingerprint_active = False
                            self._blind_fallback_timer = None
                            self.control_mode = ControlMode.ENTER_INTERSECTION
                        elif sees_any_light:
                            self._blind_fallback_timer = None
                        else:
                            if self._blind_fallback_timer is None:
                                self._blind_fallback_timer = time.time()
                                print(">>> [TRAFFIC_LIGHT] Blind spot reached (No light visible) & Not Converged. Starting 2s fallback timer...")

                            elapsed_blind = time.time() - self._blind_fallback_timer
                            if elapsed_blind >= 2.0:
                                print(">>> [TRAFFIC_LIGHT] 2s Fallback Timer expired! Proceeding blind.")
                                self._blind_fallback_timer = None
                                self._fingerprint_active = False
                                self.control_mode = ControlMode.ENTER_INTERSECTION

                case ControlMode.SIMPLE_PEDESTRIAN:
                    self._log_once("STOP PEDESTRIAN")
                    self._send_speed(0)
                    time.sleep(0.5) 
                    if any(
                        obj is not None and obj.get("label") == "Pedestrian" and obj.get("proximity") in ("CLOSE")
                        for obj in self.detections
                    ):
                        self.control_mode = ControlMode.SIMPLE_PEDESTRIAN
                    else:
                        self.control_mode = ControlMode.AUTO

                case ControlMode.MOVE:
                    self._log_once("MOVE")
                    if self.act_enabled:
                        self._apply_lane_keeping(visionRecv, axesRecv, NORMAL_SPEED)
                        if objectDetectRecv is not None:
                            if not any(
                                obj is not None and obj.get("label") == "Stop" and obj.get("proximity") == "CLOSE"
                                for obj in self.detections
                            ):
                                self.control_mode = ControlMode.AUTO
                        else:
                            self.control_mode = ControlMode.MOVE

                case ControlMode.MOVE_FOR_PEDESTRIAN_IN_CROSSWALK:
                    self._log_once("ENTERING CROSSWALK SEQUENCE")

                    ped = any(
                        obj is not None and obj.get("label") == "Pedestrian" and obj.get("proximity") in ("CLOSE", "MEDIUM")
                        for obj in self.detections
                    )
                    stop_line = any(
                        obj is not None and obj.get("label") == "STOP_LINE" and obj.get("proximity") in ("CLOSE")
                        for obj in self.detections
                    )
                    cw = any(
                        obj is not None and obj.get("label") == "Crosswalk" for obj in self.detections
                    )

                    if not hasattr(self, "ped_state"):
                        self.ped_state = "APPROACHING"

                    if self.act_enabled:
                        if self.ped_state == "APPROACHING":
                            if stop_line:
                                self._send_speed(0)
                                self.steerMotorSender.send("0")
                                self.ped_state = "WAITING"
                                print(">>> Stage: At Stop Line. Waiting for pedestrian...")
                            else:
                                self._apply_lane_keeping(visionRecv, axesRecv, NORMAL_SPEED // 2)

                        elif self.ped_state == "WAITING":
                            if not ped:
                                print(">>> Stage: Pedestrian gone! Starting to move...")
                                self.ped_state = "LEAVING"
                            else:
                                self._send_speed(0)
                                self.steerMotorSender.send("0")

                        elif self.ped_state == "LEAVING":
                            self._apply_lane_keeping(visionRecv, axesRecv, NORMAL_SPEED)
                            if not cw and not stop_line:
                                print(">>> Stage: Crosswalk cleared. Switching to AUTO")
                                self.ped_state = "APPROACHING" 
                                self.control_mode = ControlMode.AUTO

                case ControlMode.CROSSWALK:
                    self._log_once("CROSSWALK")
                    self._apply_lane_keeping(visionRecv, axesRecv, NORMAL_SPEED)
                    self.control_mode = ControlMode.AUTO

                case ControlMode.ENTER_HIGHWAY:
                    self._log_once("ENTER HIGHWAY AND TRY TO TURN")
                    self._apply_lane_keeping(visionRecv, axesRecv, NORMAL_SPEED, side="right")
                    
                    if not any(
                            obj is not None and obj.get("label") == "HighwayEntry"
                            for obj in self.detections
                        ):
                        if self.overtake_start is None:     
                            self.overtake_start = time.time()
                        
                        elapsed = time.time() - self.overtake_start
                        if elapsed >= 5:
                            self.overtake_start = None
                            self.control_mode = ControlMode.ENTER_HIGHWAY_2
                    else:
                        self.control_mode = ControlMode.ENTER_HIGHWAY

                case ControlMode.ENTER_HIGHWAY_2:
                    self._log_once("ENTER HIGHWAY FOR REAL")
                    self.highway_flag = True
                    self._apply_lane_keeping(visionRecv, axesRecv, HIGHWAY_SPEED)

                    if any(
                            obj is not None and obj.get("label") == "Car" and obj.get("distance_m") is not None
                            and obj.get("distance_m") < 0.7 and obj.get("bearing_deg") is not None
                            and abs(obj.get("bearing_deg")) < 20 for obj in self.detections
                        ):
                            self._log_once("OVERTAKE: Car detected ahead within 0.7m and 20° bearing -> prepare to overtake")

                            if(leftLineClassifierRecv == "dashed"):
                                self._log_once("OVERTAKE (VEER LEFT)")
                                self.control_mode = ControlMode.OVERTAKE_LEFT
                            elif(rightLineClassifierRecv == "dashed"):
                                self._log_once("OVERTAKE (VEER RIGHT)")
                                self.control_mode = ControlMode.OVERTAKE_RIGHT
                   
                    elif any(
                        obj is not None and obj.get("label") == "HighwayExit" and obj.get("proximity") == "CLOSE"
                        for obj in self.detections
                    ):
                        self.control_mode = ControlMode.EXIT_HIGHWAY
                    else:
                        self.control_mode = ControlMode.ENTER_HIGHWAY_2

                case ControlMode.EXIT_HIGHWAY:
                    self._log_once("EXIT HIGHWAY")
                    self.highway_flag = False
                    self.control_mode = ControlMode.AUTO

                case ControlMode.OVERTAKE_RIGHT:
                    global passing_from_right 
                    passing_from_right = True
                    dt = 0

                    if self.overtake_start is None:
                        self.overtake_start = time.time()
                  
                    self.steerMotorSender.send(str(250))
                    
                    if(self.highway_flag):
                        self._log_once(" OVERTAKE WITH HIGHWAY SPEED")
                        self._send_speed(250)
                        dt = 400 / 250  
                    else:
                        self._log_once(" OVERTAKE WITH NORMAL SPEED")
                        self._send_speed(250)
                        dt = 400 / 250  

                    elapsed = time.time() - self.overtake_start

                    if elapsed >= dt:
                        self.overtake_start = None
                        self.control_mode = ControlMode.CONTINUE_AHEAD

                case ControlMode.OVERTAKE_LEFT:  
                    global passing_from_left
                    passing_from_left = True
                    dt = 0
                    if self.overtake_start is None:
                            self.overtake_start = time.time()
                  
                    self.steerMotorSender.send(str(-250))
                    
                    if(self.highway_flag):
                        self._log_once(" OVERTAKE WITH HIGHWAY SPEED")
                        self._send_speed(250)
                        dt = 400 / 250  
                    else:
                        self._log_once(" OVERTAKE WITH NORMAL SPEED")
                        self._send_speed(250)
                        dt = 400 / 250 

                    elapsed = time.time() - self.overtake_start
                    
                    if elapsed >= dt:
                        self.overtake_start = None
                        self.control_mode = ControlMode.CONTINUE_AHEAD

                case ControlMode.CONTINUE_AHEAD:
                    self._log_once("CONTINUE AHEAD (PASSING CAR)")
                    
                    if self.overtake_start is None:
                        self.overtake_start = time.time()
                        self._seen_car_on_right = False  
                        self._seen_car_on_left = False  
                        self.car_last_seen_time = time.time()

                    if(self.highway_flag):
                        self._apply_lane_keeping(visionRecv, axesRecv, speed_override=250)
                    else:
                        self._apply_lane_keeping(visionRecv, axesRecv, speed_override=250)

                    elapsed = time.time() - self.overtake_start
                    min_pass_time = 0.5  
                    
                    if passing_from_right:
                        if self.tof_right is not None and self.tof_right < 400:
                            self._seen_car_on_right = True
                    else:
                        if self.tof_left is not None and self.tof_left < 400:
                            self._seen_car_on_left = True

                    car_in_view_now = any(
                        obj is not None and obj.get("label") == "Car"
                        for obj in self.detections
                    )

                    current_time = time.time()

                    if car_in_view_now:
                        self.car_last_seen_time = current_time

                    car_lost_timeout = 1  
                    car_in_view = (
                        self.car_last_seen_time is not None
                        and (current_time - self.car_last_seen_time) < car_lost_timeout
                    )

                    if elapsed >= min_pass_time and not car_in_view:
                        if passing_from_right:
                            right_tof_is_clear = self.tof_right is not None and self.tof_right > 300
                            if right_tof_is_clear and (self._seen_car_on_right or elapsed > 5.5):
                                print(f">>> [OVERTAKE] Car passed & right side clear (ToF: {self.tof_right}). Merging back.")
                                self.overtake_start = None
                                self.control_mode = ControlMode.VEER_RIGHT
                        else:
                            left_tof_is_clear = self.tof_left is not None and self.tof_left > 300
                            if left_tof_is_clear and (self._seen_car_on_left or elapsed > 5.5):
                                print(f">>> [OVERTAKE] Car passed & left side clear (ToF: {self.tof_left}). Merging back.")
                                self.overtake_start = None
                                self.control_mode = ControlMode.VEER_LEFT

                case ControlMode.VEER_RIGHT:  
                    self._log_once("VEER RIGHT (MERGING)")
                    dt = 0
                    if self.overtake_start is None:
                        print(f"\r>>> HARD TO STARBOARD | tof fwd: {self.tof_forward} | tof right: {self.tof_right}")
                        self.overtake_start = time.time()

                    self.steerMotorSender.send(str(-250))
                    if(self.highway_flag):
                        dt = 300 / 250  
                        self._send_speed(250)  
                    else:
                        dt = 300 / 250  
                        self._send_speed(250)

                    elapsed = time.time() - self.overtake_start
                    
                    if elapsed >= dt:
                        self.overtake_start = None
                        print(">>> [OVERTAKE] Complete. Returning to AUTO.")
                        if self.highway_flag:
                            self.control_mode = ControlMode.ENTER_HIGHWAY_2
                        else:
                            self.control_mode = ControlMode.AUTO
                            
                case ControlMode.VEER_LEFT:  
                    self._log_once("VEER LEFT (MERGING)")
                    dt = 0
                    if self.overtake_start is None:
                        print(f"\r>>> HARD TO PORT | tof fwd: {self.tof_forward} | tof left: {self.tof_left}")
                        self.overtake_start = time.time()
                    
                    self.steerMotorSender.send(str(250))

                    if(self.highway_flag):
                        dt = 400 / 250  
                        self._send_speed(250) 
                    else:
                        dt = 400 / 250  
                        self._send_speed(250)

                    elapsed = time.time() - self.overtake_start
                    
                    if elapsed >= dt:
                        self.overtake_start = None
                        print(">>> [OVERTAKE] Complete. Returning to AUTO.")
                        if self.highway_flag:
                            self.control_mode = ControlMode.ENTER_HIGHWAY_2
                        else:
                            self.control_mode = ControlMode.AUTO

                case ControlMode.PRIORITY:
                    self.control_mode = ControlMode.ENTER_INTERSECTION

                case ControlMode.ENTER_INTERSECTION:
                    self._log_once("ENTER INTERSECTION (SCANNING PHASE)")

                    stop_line_present = any(
                        obj is not None and obj.get("label") == "STOP_LINE"
                        for obj in self.detections
                    )

                    if self._intersection_scan_start is None:
                        self._intersection_scan_start = time.time()
                        self._saw_stop_line = False
                        self._last_stop_line_time = time.time()
                        self._current_near_int_node = None 
                        print(f">>> [ENTER_INTERSECTION] Approaching intersection. Intended path: {self.next_instruction.upper()}")

                    if self.act_enabled:
                        if stop_line_present:
                            self._send_speed(NORMAL_SPEED)
                            self.steerMotorSender.send("0")
                        else:
                            self._apply_lane_keeping(visionRecv, axesRecv, NORMAL_SPEED)

                    if stop_line_present:
                        self._saw_stop_line = True
                        self._last_stop_line_time = time.time()

                    trigger_maneuver = False
                    
                    if stop_line_present:
                        print("\n>>> [TRIGGER] Stop line spotted! Executing maneuver instantly.")
                        trigger_maneuver = True
                    elif time.time() - self._intersection_scan_start >= 12.0:
                        print("\n>>> [TRIGGER] Timeout reached!")
                        trigger_maneuver = True

                    if trigger_maneuver:
                        print(f">>> Executing intersection maneuver: {self.next_instruction.upper()}")
                        
                        self._intersection_scan_start = None
                        self.fwd_start = time.time()

                        if self.next_instruction == "left":
                            self.control_mode = ControlMode.INTERSECTION_LEFT
                        elif self.next_instruction == "right":
                            self.control_mode = ControlMode.INTERSECTION_RIGHT
                        else:
                            self.control_mode = ControlMode.INTERSECTION_FWD

                case ControlMode.INTERSECTION_FWD:
                    self._log_once("INTERSECTION: GOING STRAIGHT")
                    self._send_speed(200)
                    if not self.trajectory_follower.is_active:
                        print(">>> Going Forward")
                        self.trajectory_follower.start_turn("forward")
                        self.fwd_start = time.time() 
                    
                    stanley_gain = 1.75
                    steer_deg = self.trajectory_follower.update(stanley_gain)
                    
                    motor_steer = int(map_range(steer_deg, -23.0, 23.0, STEERING_MIN, STEERING_MAX))
                    self.steerMotorSender.send(str(constrain_num(motor_steer, STEERING_MIN, STEERING_MAX)))
                    
                    if not self.trajectory_follower.is_active or (time.time() - self.fwd_start > 8.0):
                        print(f">>> [INTERSECTION] Turn complete. Loading next command...")
                        
                        if hasattr(self, 'maneuver_queue') and len(self.maneuver_queue) > 0:
                            next_cmd = self.maneuver_queue.pop(0)
                            self.next_instruction = next_cmd.replace("TURN_", "").replace("GO_", "").lower()
                            print(f">>> [INTERSECTION] Chamber loaded with: {self.next_instruction.upper()}")
                        else:
                            self.next_instruction = "straight" 
                        self.fwd_start = None
                        self._ignore_intersections_until = time.time() + 3.0
                        self.control_mode = ControlMode.AUTO

                case ControlMode.INTERSECTION_LEFT:
                    self._log_once("INTERSECTION: LEFT TURN (TRAJECTORY)")
                    self._send_speed(200) 
                    
                    if not self.trajectory_follower.is_active:
                        print(">>> Executing Mathematical Left Turn Curve")
                        self.trajectory_follower.start_turn("left")
                        self.fwd_start = time.time() 
                    
                    stanley_gain = 0.75
                    steer_deg = self.trajectory_follower.update(stanley_gain)
                    
                    motor_steer = int(map_range(steer_deg, -23.0, 23.0, STEERING_MIN, STEERING_MAX))
                    self.steerMotorSender.send(str(constrain_num(motor_steer, STEERING_MIN, STEERING_MAX)))
                    
                    if not self.trajectory_follower.is_active or (time.time() - self.fwd_start > 8.0):
                        print(f">>> [INTERSECTION] Turn complete. Loading next command...")
                        
                        if hasattr(self, 'maneuver_queue') and len(self.maneuver_queue) > 0:
                            next_cmd = self.maneuver_queue.pop(0)
                            self.next_instruction = next_cmd.replace("TURN_", "").replace("GO_", "").lower()
                            print(f">>> [INTERSECTION] Chamber loaded with: {self.next_instruction.upper()}")
                        else:
                            self.next_instruction = "straight" 
                        self.fwd_start = None
                        self._ignore_intersections_until = time.time() + 3.0
                        self.control_mode = ControlMode.AUTO

                case ControlMode.INTERSECTION_RIGHT:
                    self._log_once("INTERSECTION: RIGHT TURN (TRAJECTORY)")
                    self._send_speed(200)  
                    
                    if not self.trajectory_follower.is_active:
                        print(">>> Executing Mathematical Right Turn Curve")
                        self.trajectory_follower.start_turn("right")
                        self.fwd_start = time.time() 
                    
                    stanley_gain = 1.0
                    steer_deg = self.trajectory_follower.update(stanley_gain)
                    
                    motor_steer = int(map_range(steer_deg, -23.0, 23.0, STEERING_MIN, STEERING_MAX))
                    self.steerMotorSender.send(str(constrain_num(motor_steer, STEERING_MIN, STEERING_MAX)))
                    
                    if not self.trajectory_follower.is_active or (time.time() - self.fwd_start > 8.0):
                        print(f">>> [INTERSECTION] Turn complete. Loading next command...")
                        
                        if hasattr(self, 'maneuver_queue') and len(self.maneuver_queue) > 0:
                            next_cmd = self.maneuver_queue.pop(0)
                            self.next_instruction = next_cmd.replace("TURN_", "").replace("GO_", "").lower()
                            print(f">>> [INTERSECTION] Chamber loaded with: {self.next_instruction.upper()}")
                        else:
                            self.next_instruction = "straight" 
                        self.fwd_start = None
                        self._ignore_intersections_until = time.time() + 3.0
                        self.control_mode = ControlMode.AUTO

                case ControlMode.ENTER_PARKING:
                    if self.act_enabled:
                        t_since_done = (
                            time.time() - self._parking_done_time
                            if self._parking_done_time is not None
                            else 9999
                        )
                        if self._parking_completed and t_since_done < PARK_COOLDOWN:
                            print(f">>> [AUTO] Parking sign IGNORED (cooldown {t_since_done:.0f}s/{PARK_COOLDOWN}s)")
                            self.control_mode = ControlMode.AUTO
                        elif self._parking_completed and t_since_done >= PARK_COOLDOWN:
                            print(f">>> [AUTO] New parking zone -> reset")
                            self._parking_completed = False
                            self._parking_done_time = None
                            self.control_mode = ControlMode.AUTO
                        else:
                            self._apply_lane_keeping(visionRecv, axesRecv, speed_override=PARK_SIGN_SPEED)
                            if self._park_sign_seen_time is None:
                                self._park_sign_seen_time = time.time()
                                self._set_scan_gate(False)
                                print("=" * 60)
                                print(f">>> [PARK] Parking Sign! Speed={PARK_SIGN_SPEED} | Blind {PARK_BLIND_DURATION}s")
                                print("=" * 60)
                                
                            elapsed_sign = time.time() - self._park_sign_seen_time
                            if (elapsed_sign >= PARK_SCAN_DELAY and not self._park_scan_active):
                                self._set_scan_gate(True)
                                
                            if elapsed_sign >= PARK_BLIND_DURATION:
                                print(">>> [PARK] -> ENTER_PARKING")
                                self._park_sign_seen_time = None
                                self._park_seen_wall = False
                                self._park_side = None
                                self._park_free_confirm = None
                                self._park_all_free_timer = None
                                self._park_position_start = None
                                self._park_reverse_start = None
                                self._park_correct_start = None
                                self._park_fwd_start = None
                                self._park_stop_time = None

                            if self._park_blind_enter_time is None:
                                self._park_blind_enter_time = time.time()
                                print(f">>> [ENTER_PARKING] Blind drive {PARK_ENTER_BLIND_DURATION}s — no spot scanning")

                            if (time.time() - self._park_blind_enter_time >= PARK_ENTER_BLIND_DURATION):
                                print(">>> [ENTER_PARKING] -> FIND_PARKING_SPOT")
                                self._park_blind_enter_time = None
                                self._parking_spot_detection_time = None
                                self._last_parking_condition = None
                                self._occ_frames_left = 0
                                self._occ_frames_right = 0
                                self._free_frames_left = 0
                                self._free_frames_right = 0
                                self._wall_seen_left = False
                                self._wall_seen_right = False
                                self._park_commit_time = None
                                self._free_timer_left = None
                                self._free_timer_right = None
                                self.control_mode = ControlMode.FIND_PARKING_SPOT

                case ControlMode.FIND_PARKING_SPOT:
                    self._log_once("FIND PARKING SPOT")

                    if self.act_enabled:
                        self._apply_lane_keeping(visionRecv, axesRecv, speed_override=PARK_SIGN_SPEED)

                        left_occ = not self._tof_left_free
                        right_occ = not self._tof_right_free

                        if self._park_commit_time is not None:
                            elapsed_commit = time.time() - self._park_commit_time
                            print(f">>> [FIND_SPOT] Crawl {elapsed_commit:.2f}s / {PARK_EXTRA_FWD_TIME}s")
                            self._send_speed(PARK_CRAWL_SPEED)
                            self.steerMotorSender.send("0")
                            if elapsed_commit >= PARK_EXTRA_FWD_TIME:
                                self._send_speed(0)
                                self._commit_to_bay(self._park_side, "transition-confirmed")

                        else:
                            if left_occ:
                                self._occ_frames_left += 1
                                if self._occ_frames_left >= PARK_OCC_CONFIRM_FRAMES:
                                    self._wall_seen_left = True
                                self._free_timer_left = None
                            else:
                                self._occ_frames_left = 0
                                if self._free_timer_left is None:
                                    self._free_timer_left = time.time()

                            if right_occ:
                                self._occ_frames_right += 1
                                if self._occ_frames_right >= PARK_OCC_CONFIRM_FRAMES:
                                    self._wall_seen_right = True
                                self._free_timer_right = None
                            else:
                                self._occ_frames_right = 0
                                if self._free_timer_right is None:
                                    self._free_timer_right = time.time()

                            free_secs_left = (time.time() - self._free_timer_left if self._free_timer_left is not None else 0.0)
                            free_secs_right = (time.time() - self._free_timer_right if self._free_timer_right is not None else 0.0)

                            if self._parking_spot_detection_time is None:
                                self._parking_spot_detection_time = time.time()
                            elapsed_scan = (time.time() - self._parking_spot_detection_time)
                            
                            in_zone = (
                                self._wall_seen_left
                                or self._wall_seen_right
                                or elapsed_scan >= PARK_ENTER_BLIND_DURATION
                            )

                            left_spot_found = (in_zone and free_secs_left >= PARK_FREE_SPOT_TIME)
                            right_spot_found = (in_zone and free_secs_right >= PARK_FREE_SPOT_TIME)

                            print(
                                f">>> [FIND_SPOT] "
                                f"L={'OCC' if left_occ else f'FREE {free_secs_left:.1f}s'}(wall={self._wall_seen_left}) left tof={self.tof_left} "
                                f"R={'OCC' if right_occ else f'FREE {free_secs_right:.1f}s'}(wall={self._wall_seen_right}) right tof={self.tof_right} " 
                                f"in_zone={in_zone}"
                            )

                            if left_spot_found or right_spot_found:
                                chosen = "left" if left_spot_found else "right"
                                if left_spot_found and right_spot_found:
                                    chosen = "left"
                                self._park_side = chosen
                                self._park_commit_time = time.time()
                                print(f">>> [FIND_SPOT] SPOT FOUND -> {chosen.upper()} | crawl {PARK_EXTRA_FWD_TIME}s @ {PARK_CRAWL_SPEED}")

                case ControlMode.PARKING_POSITION:
                    if self.act_enabled:
                        self._apply_lane_keeping(visionRecv, axesRecv, speed_override=PARK_SIGN_SPEED)
                        if self._park_position_start is None:
                            self._park_position_start = time.time()
                            print(f">>> [PARKING_POSITION] {PARK_HALF_LEN_TIME}s side={self._park_side}")
                            
                        elapsed = time.time() - self._park_position_start
                        if elapsed >= PARK_HALF_LEN_TIME:
                            print(">>> [PARKING_POSITION] -> PARKING_REVERSE")
                            self._send_speed(0)
                            self._park_position_start = None
                            self._park_reverse_start = None
                            self._set_scan_gate(False)
                            self.control_mode = ControlMode.PARKING_REVERSE

                case ControlMode.PARKING_REVERSE:
                    STEER_IN = (PARK_STEER_ANGLE if self._park_side == "right" else -PARK_STEER_ANGLE)
                    if self.act_enabled:
                        if self._park_reverse_start is None:
                            self._park_reverse_start = time.time()
                            print(f">>> [PARKING_REVERSE] steer={STEER_IN}")
                            
                        elapsed = time.time() - self._park_reverse_start
                        rear_ok = self.tof_rear is not None and self.tof_rear < PARK_SAFE_REAR
                        timeout = elapsed >= PARK_REVERSE_TIMEOUT
                        
                        if rear_ok or timeout:
                            print(f">>> [PARKING_REVERSE] -> CORRECT ({'sensor' if rear_ok else 'timeout'})")
                            self._send_speed(0)
                            self._park_reverse_start = None
                            self._park_correct_start = None
                            self.control_mode = ControlMode.PARKING_CORRECT
                        else:
                            self.steerMotorSender.send(str(STEER_IN))
                            self._send_speed(PARK_REVERSE_SPEED)

                case ControlMode.PARKING_CORRECT:
                    STEER_CORR = (PARK_CORRECT_ANGLE if self._park_side == "right" else -PARK_CORRECT_ANGLE)
                    if self.act_enabled:
                        if self._park_correct_start is None:
                            self._park_correct_start = time.time()
                            print(f">>> [PARKING_CORRECT] steer={STEER_CORR}")
                            
                        elapsed = time.time() - self._park_correct_start
                        rear_ok = self.tof_rear is not None and self.tof_rear < 100
                        timeout = elapsed >= PARK_CORRECT_TIMEOUT
                        
                        if rear_ok or timeout:
                            print(f">>> [PARKING_CORRECT] -> PARKING_FORWARD ({'sensor' if rear_ok else 'timeout'})")
                            self._send_speed(0)
                            self.steerMotorSender.send("0")
                            self._park_correct_start = None
                            self._park_fwd_start = None
                            self.control_mode = ControlMode.PARKING_FORWARD
                        else:
                            self.steerMotorSender.send(str(STEER_CORR))
                            self._send_speed(PARK_REVERSE_SPEED)

                case ControlMode.PARKING_FORWARD:
                    if self.act_enabled:
                        if self._park_fwd_start is None:
                            self._park_fwd_start = time.time()
                            print(f">>> [PARKING_FORWARD] speed={PARK_FWD_SPEED}")
                            
                        car_close = any(
                            obj is not None and obj.get("label") == "Car" and obj.get("proximity") == "CLOSE"
                            for obj in self.detections
                        )
                        line_close = any(
                            obj is not None and obj.get("label") == "STOP_LINE" and obj.get("proximity") == "CLOSE"
                            for obj in self.detections
                        )
                        elapsed = time.time() - self._park_fwd_start
                        timeout = elapsed >= PARK_FWD_TIMEOUT
                        
                        if timeout or car_close or line_close:
                            reason = ("CAR" if car_close else ("LINE" if line_close else "TIMEOUT"))
                            print(f">>> [PARKING_FORWARD] -> PARKED ({reason})")
                            self._send_speed(0)
                            self.steerMotorSender.send("0")
                            self._park_fwd_start = None
                            self.control_mode = ControlMode.PARKED
                        else:
                            self.steerMotorSender.send("0")
                            self._apply_lane_keeping(visionRecv, axesRecv, speed_override=PARK_SIGN_SPEED)

                case ControlMode.PARKED:
                    if self._park_stop_time is None:
                        self._park_stop_time = time.time()
                        print(f">>> [PARKED] side={self._park_side} rear={self.tof_rear}mm | wait {PARK_STOP_WAIT}s")
                        
                    self._send_speed(0)
                    self.steerMotorSender.send("0")
                    if time.time() - self._park_stop_time >= PARK_STOP_WAIT:
                        print(">>> [PARKED] -> UNPARKING")
                        self._park_stop_time = None
                        self._unpark_start_time = time.time()
                        self.control_mode = ControlMode.UNPARKING

                case ControlMode.UNPARKING:
                    if self.act_enabled:
                        t = (time.time() - self._unpark_start_time if self._unpark_start_time else 999)
                        if visionRecv is not None:
                            sv = map_range(
                                float(visionRecv["steering"]), 0.0, 500.0, float(STEERING_MIN), float(STEERING_MAX)
                            )
                            self.steerMotorSender.send(str(int(constrain_num(sv, STEERING_MIN, STEERING_MAX))))
                        else:
                            exit_steer = -250 if self._park_side == "right" else 250
                            self.steerMotorSender.send(str(exit_steer if t < 2.0 else -exit_steer))
                            
                        self._send_speed(UNPARK_SPEED)
                        
                        if t >= UNPARK_DURATION:
                            print("=" * 60)
                            print(">>> [UNPARKING] -> AUTO | cooldown active")
                            print("=" * 60)
                            self._parking_completed = True
                            self._parking_done_time = time.time()
                            self._reset_parking_timers()
                            self._unpark_start_time = None
                            self.control_mode = ControlMode.EXIT_PARKING

                case ControlMode.EXIT_PARKING:
                    self._apply_lane_keeping(visionRecv, axesRecv, UNPARK_SPEED)
                    print("EXIT THE PARKING ")
                    if objectDetectRecv is not None:
                        if not any(
                            obj is not None and obj.get("label") == "Parking" for obj in self.detections
                        ):
                            self.control_mode = ControlMode.AUTO
                    else:
                        self.control_mode = ControlMode.EXIT_PARKING

                case ControlMode.ROUNDABOUT_APPROACH:
                    self._log_once(f"ROUNDABOUT APPROACH (Target Exit: {self.target_roundabout_exit})")
                    self._apply_lane_keeping(visionRecv, axesRecv, speed_override=200)
                    
                    yield_line_seen = any(
                        obj is not None and obj.get("label") == "STOP_LINE" and obj.get("proximity") in ("CLOSE")
                        for obj in self.detections
                    )
                    
                    if yield_line_seen:
                        print(">>> [ROUNDABOUT] Yield Line reached. Stopping to check traffic.")
                        self._send_speed(0)
                        self.steerMotorSender.send("0")
                        self._roundabout_yield_start = time.time()
                        self.control_mode = ControlMode.ROUNDABOUT_YIELD

                case ControlMode.ROUNDABOUT_YIELD:
                    self._send_speed(0)
                    
                    left_tof_clear = self.tof_left is not None and self.tof_left > 450
                    car_in_vision = any(
                        obj is not None and obj.get("label") == "Car" and obj.get("proximity") in ("CLOSE", "MEDIUM")
                        for obj in self.detections
                    )
                    
                    if left_tof_clear and not car_in_vision:
                        if time.time() - self._roundabout_yield_start > 1.0:
                            print(">>> [ROUNDABOUT] Clear! Entering roundabout.")
                            self.accumulated_yaw = 0.0
                            self.last_yaw = self.current_yaw
                            
                            self._roundabout_cruise_start = time.time()
                            self.control_mode = ControlMode.ROUNDABOUT_CRUISE
                    else:
                        self._roundabout_yield_start = time.time()

                case ControlMode.ROUNDABOUT_CRUISE:
                    self._log_once("ROUNDABOUT CRUISING (IMU Accumulated Angle)")

                    if not hasattr(self, '_roundabout_cruise_start') or self._roundabout_cruise_start is None:
                        self._roundabout_cruise_start = time.time()
                        self.accumulated_yaw = 0.0
                        self.last_yaw = self.current_yaw

                    elapsed = time.time() - self._roundabout_cruise_start

                    diff = (self.current_yaw - self.last_yaw + 180) % 360 - 180
                    self.accumulated_yaw += abs(diff)
                    self.last_yaw = self.current_yaw

                    instruction = str(self.next_instruction).lower().strip()
                    
                    if instruction == "right":
                        target_yaw = 120   
                    elif instruction in ["front", "straight"]:
                        target_yaw = 120  
                    elif instruction == "left":
                        target_yaw = 120  
                    elif instruction == "u_turn":
                        target_yaw = 120  
                    else:
                        target_yaw = 120  

                    if elapsed < 1.5:  
                        self._send_speed(NORMAL_SPEED)
                        self.steerMotorSender.send("190") 
                    else:
                        self._apply_lane_keeping(visionRecv, axesRecv, speed_override=NORMAL_SPEED, side="right")

                    print(f">>> [ROUNDABOUT] Target: {target_yaw} | Accumulated: {self.accumulated_yaw:.1f}")

                    if elapsed > 1.5 and self.accumulated_yaw >= target_yaw:
                        print(f">>> [ROUNDABOUT] Reached {self.accumulated_yaw:.1f} deg. Exiting {instruction.upper()}!")
                        self._roundabout_exit_start = time.time()
                        self._roundabout_cruise_start = None 
                        self.control_mode = ControlMode.ROUNDABOUT_EXIT

                case ControlMode.ROUNDABOUT_EXIT:
                    self._log_once("ROUNDABOUT EXITING")
                    self._apply_lane_keeping(visionRecv, axesRecv, NORMAL_SPEED, side="right")

                    if time.time() - self._roundabout_exit_start > 1.2:
                        print(">>> [ROUNDABOUT] Maneuver Complete. Returning to AUTO.")
                        self.control_mode = ControlMode.AUTO

                case ControlMode.START:
                    self._log_once("WAITING AT START LINE FOR GREEN LIGHT")
                    self._apply_lane_keeping(visionRecv, axesRecv, speed_override=0)

                    sees_green = any(obj is not None and obj.get("label") == "GreenLight" for obj in self.detections)

                    if sees_green:
                        print(">>> [START] Green light detected! GO GO GO!")
                        self._has_started = True 
                        self.control_mode = ControlMode.EXIT_START
                    else:
                        self.control_mode = ControlMode.START

                case ControlMode.EXIT_START:
                    self._log_once("EXITING START")
                    self._apply_lane_keeping(visionRecv, axesRecv, speed_override=NORMAL_SPEED)

                    if self.act_enabled:
                        self._apply_lane_keeping(visionRecv, axesRecv, NORMAL_SPEED)
                        if not any(
                            obj is not None and "traffic" in obj.get("label", "").lower() and obj.get("proximity") in ("CLOSE", "MEDIUM", "FAR")
                            for obj in self.detections
                        ) and not any(
                            obj is not None and obj.get("label") == "GreenLight" for obj in self.detections
                        ):
                                self.stateMachine.request_mode("dashboard_auto_button")
                                self.control_mode = ControlMode.AUTO
                        else:
                            self.control_mode = ControlMode.EXIT_START

                case ControlMode.AVOID_CAR:
                    self._log_once("CAR AVOIDANCE: Imminent collision detected.")
                    if any(
                            obj is not None and obj.get("label") == "Car" and obj.get("distance_m") is not None
                            and obj.get("distance_m") < 0.7 and obj.get("bearing_deg") is not None
                            and abs(obj.get("bearing_deg")) < 20 for obj in self.detections
                        ):
                        print(">>> [AVOID_CAR] Obstacle directly ahead, holding position.")
                        self._apply_lane_keeping(visionRecv, axesRecv, 0)
                    else:
                        self.control_mode = ControlMode.AUTO

            # =================================================================
            # TELEMETRY
            # =================================================================
            self._send_telemetry()

        except Exception as exp:
            print(f"\033[1;91m[ Controller Thread ] ERROR: {exp}\033[0m")
            import traceback
            traceback.print_exc()
            self.steerMotorSender.send("0")
            self._send_speed(0)
            self.stateMachine.request_mode("dashboard_stop_button")
            self.control_mode = ControlMode.IDLE
