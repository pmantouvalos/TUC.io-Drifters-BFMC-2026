import sys
import os
import time
import math
import base64
import threading
import re
import pathlib
from pathlib import Path
from collections import Counter
from datetime import datetime, timedelta
from typing import List
from enum import Enum
from dataclasses import dataclass

import cv2
import numpy as np
import tkinter as tk

# DepthAI setup
os.environ["DEPTHAI_USB2_FORCE"] = "1"
import depthai as dai
from depthai_nodes.node import ParsingNeuralNetwork

# Local project imports
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.templates.workerprocess import WorkerProcess
from src.statemachine.stateMachine import StateMachine

from src.utils.messages.allMessages import (
    mainCamera, serialCamera, ProcessingCameraPrimary, ProcessingCameraSecondary,
    VisionManeuver, ThresholdMin, ThresholdMax, StanleyGain, StateChange,
    UpSlice, DownSlice, ObjectDetect, IntersectionMask, CurrentSpeed,
    LeftLineClassifier, RightLineClassifier
)

# --- Constants & Globals ---
FONT = cv2.FONT_HERSHEY_SIMPLEX
COLOR = (0, 255, 0)
FPS = 9

WINDOW = 6
CONFIRM = 1
MIN_CONF = 0.75

start_line_detected = False
left_line_type = "none"
right_line_type = "none"


def log_console(module_name: str, level: str, message: str, debug_enabled: bool = True):
    """Unified console logger. Silences DEBUG messages if debug_enabled is False."""
    if not debug_enabled and level.upper() == "DEBUG":
        return

    colors = {
        "INFO": "\033[1;92m",    # Green
        "WARN": "\033[1;93m",    # Yellow
        "ERROR": "\033[1;91m",   # Red
        "DEBUG": "\033[1;94m"    # Blue
    }
    reset = "\033[0m"
    white = "\033[1;97m"
    color = colors.get(level.upper(), white)
    
    print(f"{white}[ {module_name} ] :{reset} {color}{level.upper()}{reset} - {message}")


def distanceCalculate(p1, p2):
    """Calculates Euclidean distance between p1 (x1, y1) and p2 (x2, y2)."""
    return ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5


def constrain_num(val, min_val, max_val):
    """Clamps a value within a specified range."""
    return min(max_val, max(min_val, val))


def map_range(x, in_min, in_max, out_min, out_max):
    """Maps a value from one numerical range to another."""
    tmp_x = min(in_max, max(in_min, x))
    return (((tmp_x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min


def stanley_controller(heading_error, cross_track_error, stanley_gain, vehicle_speed):
    """
    Calculates steering angle using the Stanley method.
    Includes a softening constant to prevent singularity/erratic steering at near-zero speeds.
    """
    if vehicle_speed is None or vehicle_speed < 0.01:
        return 0

    vehicle_speed /= 1000  # Convert mm/s to m/s
    softening_constant = 0.01
    
    steering_angle = heading_error + math.degrees(
        math.atan2(stanley_gain * cross_track_error, vehicle_speed + softening_constant)
    )
    
    # Cap steering to prevent physical actuator saturation
    return max(-25, min(25, steering_angle))


def detect_lines_in_roi(
    frame, rx1, ry1, rx2, ry2, angle_min, angle_max, slope_sign=None,
    threshold=10, min_length_ratio=0.05, max_gap=50, white_thresh=180,
    kernel_shape="vertical", return_edges=False, length_axis="height"
):
    """
    Applies Hough Transform on a specific bounding box (ROI) to find lane line candidates.
    Filters out noise by matching lines against minimum lengths, gaps, and slope limits.
    """
    roi = frame[ry1:ry2, rx1:rx2]
    if roi.size == 0:
        return ([], np.zeros((10, 10), np.uint8)) if return_edges else []

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if len(roi.shape) == 3 else roi.copy()
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blurred, white_thresh, 255, cv2.THRESH_BINARY)
    
    edges = cv2.Canny(binary, 50, 150)
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8))

    # Scale minimum pixel length dynamically based on the ROI size
    roi_h, roi_w = ry2 - ry1, rx2 - rx1
    min_pixels = int((roi_w if length_axis == "width" else roi_h) * min_length_ratio)

    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180, threshold=threshold, minLineLength=min_pixels, maxLineGap=max_gap
    )

    candidates = []
    if lines is not None:
        for seg in lines:
            x1, y1, x2, y2 = seg[0]
            dx, dy = x2 - x1, y2 - y1
            angle = abs(np.degrees(np.arctan2(dy, dx)))
           
            if not (angle_min <= angle <= angle_max): 
                continue
           
            # Optional slope direction check to differentiate left/right diagonal lines
            if dx != 0 and slope_sign:
                slope = dy / dx
                if slope_sign == "positive" and slope <= 0: continue
                if slope_sign == "negative" and slope >= 0: continue
            
            candidates.append((x1, y1, x2, y2))

    return (candidates, edges) if return_edges else candidates


def span_fraction(segments, axis="x"):
    """
    Merges overlapping line segments and calculates the percentage (0.0 to 1.0)
    of the ROI actually covered by line pixels on the specified axis.
    """
    if not segments:
        return 0

    intervals = [sorted([s[0], s[2]]) if axis == "x" else sorted([s[1], s[3]]) for s in segments]
    intervals.sort()
    
    merged = []
    cur_start, cur_end = intervals[0]
    for start, end in intervals[1:]:
        if start <= cur_end:
            cur_end = max(cur_end, end)
        else:
            merged.append((cur_start, cur_end))
            cur_start, cur_end = start, end
    merged.append((cur_start, cur_end))

    total = sum(end - start for start, end in merged)
    span_min = min(s for s, _ in merged)
    span_max = max(e for _, e in merged)
    roi_span = span_max - span_min if span_max > span_min else 1
    return total / roi_span


@dataclass
class TrapezoidROI:
    """Pre-computes coordinate mappings for perspective unwarping (birds-eye view)."""
    top_left_x: float
    top_right_x: float
    top_y: float
    bottom_left_x: float
    bottom_right_x: float
    bottom_y: float
    pts: np.ndarray

    @classmethod
    def from_crops(cls, crop_top: float, crop_down: float, crop_top_left_right: float, width, height):
        y_top = crop_top * 3.9
        y_bottom = 1
        top_left_x = crop_top_left_right + 0.11
        top_right_x = 1 - crop_top_left_right - 0.12
        bottom_shrink = crop_top_left_right * 2
        bottom_left_x = 0.51 + bottom_shrink
        bottom_right_x = 0.49 - bottom_shrink

        return cls(
            top_left_x=top_left_x, top_right_x=top_right_x, top_y=y_top,
            bottom_left_x=bottom_left_x, bottom_right_x=bottom_right_x, bottom_y=y_bottom,
            pts=np.array([[
                (int(top_left_x * width), int(y_top * height)),
                (int(top_right_x * width), int(y_top * height)),
                (int(bottom_right_x * width), int(y_bottom * height)),
                (int(bottom_left_x * width), int(y_bottom * height)),
            ]], dtype=np.int32),
        )


class LaneDetection:
    def __init__(self, width, height, slicethickness=70, road_width_pixels=200):
        self.width = width
        self.height = height
        self.slicethickness = slicethickness
        self.road_width_pixels = road_width_pixels
        
        # Debounce counters for track features (smooths out 1-frame glitches)
        self.stop_line_counter = 0
        self.crosswalk_counter = 0
        self.mid_stop_line_counter = 0
        self.chessboard_counter = 0
        self.LINE_THRESHOLD = 1
        
        self.crosswalk_detection = False
        self.line_detected = "STOP_LINE"
        self.early_stop_line_counter = 0
        self.early_stop_line_detected = False
        self.start_sequence_complete = False

        self.prev_scores = {"left": 0.0, "right": 0.0}
        self.prev_angles = {"left": 90.0, "right": 90.0}

        # EWMA (Exponential Weighted Moving Average) variables for smooth steering
        self.smoothed_deviation = 0.0
        self.smoothed_cte = 0.0
        self.smoothed_cte_left = 0.0
        self.smoothed_deviation_left = 0.0
        self.smoothed_cte_right = 0.0
        self.smoothed_deviation_right = 0.0

    def classify_lane(self, lane_img):
        """Classifies line style by counting large vertical pixel gaps (>20px = dashed)."""
        pixels = cv2.countNonZero(lane_img)
        visibility = min(pixels / 1500.0, 1.0)

        if pixels < 50: return "none", 0.0

        row_densities = np.count_nonzero(lane_img, axis=1)
        active_rows = np.where(row_densities > 0)[0]
        if len(active_rows) == 0: return "none", 0.0

        gaps = sum(1 for i in range(1, len(active_rows)) if active_rows[i] - active_rows[i-1] > 20)
        return ("dashed", visibility) if gaps >= 1 else ("solid", visibility)

    def calculate_trust(self, side, visibility, current_angle, lane_type):
        """Calculates a heuristic trust score. Favors solid lines and temporally consistent angles."""
        prev_angle = self.prev_angles[side]
        prev_score = self.prev_scores[side]
        consistency = max(1.0 - (abs(current_angle - prev_angle) / 20.0), 0.0) if current_angle else 0.0

        type_bonus = 0.2 if lane_type == "solid" else 0.0 if lane_type == "dashed" else -1.0
        return max((0.4 * visibility) + (0.3 * consistency) + type_bonus + (0.3 * prev_score), 0.0)

    @staticmethod
    def lane_filter(frame, pts, threshold_min, threshold_max):
        """Masks and applies dynamic thresholding (CLAHE) to handle uneven lighting conditions."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        gray_enhanced = clahe.apply(gray)
       
        dyn_thresh_min = max(0, threshold_min - 35)
        _, binary = cv2.threshold(gray_enhanced, dyn_thresh_min, threshold_max, cv2.THRESH_BINARY)  

        # Area filter to drop noise speckles smaller than ~10x10 pixels
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        clean_binary = np.zeros_like(binary)
        for cnt in contours:
            if cv2.contourArea(cnt) > 115:
                cv2.drawContours(clean_binary, [cnt], -1, 255, thickness=cv2.FILLED)
               
        mask = np.zeros_like(binary)
        cv2.fillPoly(mask, pts, 255)
        return {"frame": cv2.bitwise_and(clean_binary, mask), "trust_lk": True}

    @staticmethod
    def unwarp_camera(frame):
        """Maps the camera's FOV trapezoid into a top-down bird's-eye view using perspective transform."""
        crop_top, crop_tlr, crop_blr = 0.3, 0.2, 0.4
        height, width, _ = frame.shape

        pts = np.array([[
            (int(crop_tlr * width), int(crop_top * height)),
            (int((1 - crop_tlr) * width), int(crop_top * height)),
            (int(width * (1 + crop_blr)), int(height)),
            (int(-width * crop_blr), int(height)),
        ]], dtype=np.int32)

        mask_frame = cv2.fillPoly(np.zeros((height, width, 1), dtype=np.uint8), pts, 255)
        frame = cv2.bitwise_and(frame, frame, mask=mask_frame)

        out_w, out_h = 400, 700
        dst_pts = np.array([[(0, 0), (out_w, 0), (out_w, out_h), (0, out_h)]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(pts.astype(np.float32), dst_pts)
        
        return cv2.warpPerspective(frame, M, (out_w, out_h), cv2.INTER_LINEAR)

    def lane_detector(self, frame, roi, up_slice_val, down_slice_val, threshold_min, threshold_max, debug=False):
        raw_h, raw_w, _ = frame.shape
        
        # --- CALIBRATION: Search for chessboard floor pattern to initiate track logic ---
        if not self.start_sequence_complete:
            board_roi = frame[int(raw_h * 0.5):, :]
            gray_enhanced = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(cv2.cvtColor(board_roi, cv2.COLOR_BGR2GRAY))
            ret, _ = cv2.findChessboardCorners(gray_enhanced, (4, 3), flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
            
            self.chessboard_counter = min(self.chessboard_counter + 3, 10) if ret else max(self.chessboard_counter - 1, 0)
            chessBoard = self.chessboard_counter >= 3
            if chessBoard:
                self.start_sequence_complete = True
        else:
            chessBoard = False

        frame = LaneDetection.unwarp_camera(frame)
        stop_line = " "
        
        if debug and chessBoard:
            cv2.putText(frame, "START DETECTED", (10, 30), FONT, 1.0, (0, 255, 255), 3)

        if debug:
            cv2.polylines(frame, roi.pts, isClosed=True, color=(255, 0, 0), thickness=2)
            
        lane = LaneDetection.lane_filter(frame, roi.pts, threshold_min, threshold_max)
        binary_img = lane["frame"]
        h, w = binary_img.shape

        # --- STOP LINE LOGIC ---
        search_top = int(h * 0.15)  
        corridor_left, corridor_right = int(w * 0.2), int(w * 0.8)
        corridor_width = corridor_right - corridor_left
        density_threshold = corridor_width * 0.4  
        
        if debug:
            cv2.rectangle(frame, (corridor_left, search_top), (corridor_right, h), (0, 255, 0), 1)
        
        stop_line_roi_y1, stop_line_roi_y2 = int(h * 0.85), int(h * 0.95)
        
        if debug:
            cv2.rectangle(frame, (corridor_left, stop_line_roi_y1), (corridor_right, stop_line_roi_y2), (0, 0, 255), 1)
            
        search_region = binary_img[stop_line_roi_y1:stop_line_roi_y2, corridor_left:corridor_right]  
        valid_rows = np.where(np.count_nonzero(search_region, axis=1) > density_threshold)[0]

        if len(valid_rows) > 3 and (valid_rows[-1] - valid_rows[0]) < 40:  
            self.stop_line_counter = min(self.stop_line_counter + 1, 15)
        else:
            self.stop_line_counter = max(self.stop_line_counter - 1, 0)

        # Early Stop Line (looks further ahead)
        early_roi_y1, early_roi_y2 = int(h * 0.5), int(h * 0.83)
        
        if debug:
            cv2.rectangle(frame, (corridor_left, early_roi_y1), (corridor_right, early_roi_y2), (0, 165, 255), 1)
            
        early_search_region = binary_img[early_roi_y1:early_roi_y2, corridor_left:corridor_right]
        early_valid_rows = np.where(np.count_nonzero(early_search_region, axis=1) > density_threshold)[0]
        
        if len(early_valid_rows) > 3 and (early_valid_rows[-1] - early_valid_rows[0]) < 40:
            self.early_stop_line_counter = min(self.early_stop_line_counter + 1, 15)
        else:
            self.early_stop_line_counter = max(self.early_stop_line_counter - 1, 0)
            
        if self.early_stop_line_counter >= self.LINE_THRESHOLD and len(early_valid_rows) > 0:
            self.early_stop_line_detected = True
            if debug:
                cv2.line(frame, (corridor_left, int(np.mean(early_valid_rows)) + early_roi_y1), (corridor_right, int(np.mean(early_valid_rows)) + early_roi_y1), (0, 165, 255), 6)
                cv2.putText(frame, "EARLY STOP LINE", (corridor_left, int(np.mean(early_valid_rows)) + early_roi_y1 - 20), FONT, 0.8, (0, 165, 255), 2)
        
        if self.early_stop_line_counter == 0 and self.stop_line_counter == 0:
            self.early_stop_line_detected = False

        # Intersection Stop Line (Mid range)
        mid_search = binary_img[int(h * 0.25):int(h * 0.5), corridor_left:corridor_right]
        mid_valid_rows = np.where(np.count_nonzero(mid_search, axis=1) > corridor_width * 0.15)[0]
        self.mid_stop_line_counter = min(self.mid_stop_line_counter + 1, 15) if (len(mid_valid_rows) > 1 and (mid_valid_rows[-1] - mid_valid_rows[0]) < 15) else max(self.mid_stop_line_counter - 1, 0)

        mid_stop_line = "MID_STOP_LINE" if self.mid_stop_line_counter >= self.LINE_THRESHOLD else " "

        if self.stop_line_counter >= self.LINE_THRESHOLD and len(valid_rows) > 0:
            stop_line = "STOP_LINE"
            stop_row_global = int(np.mean(valid_rows)) + search_top
            if debug:
                cv2.line(frame, (corridor_left, stop_row_global), (corridor_right, stop_row_global), (0, 0, 255), 10)
                cv2.putText(frame, "STOP LINE", (corridor_left, stop_row_global - 20), FONT, 0.8, (0, 0, 255), 2)
            
        # --- CROSSWALK DETECTION ---
        self.crosswalk_detection = False
        search_region_cw = binary_img[search_top:h, corridor_left:corridor_right]
        row_transitions = np.count_nonzero(np.diff(search_region_cw.astype(np.int16), axis=1), axis=1)
        row_white_pixels = np.count_nonzero(search_region_cw, axis=1)

        # Detect alternating high-frequency pixel transitions (zebra stripes)
        zebra_rows = np.where((row_transitions >= 6) & (row_transitions <= 12) & (row_white_pixels > int(corridor_width * 0.20)))[0]
        self.crosswalk_counter = min(self.crosswalk_counter + 1, 15) if len(zebra_rows) > 40 else max(self.crosswalk_counter - 1, 0)

        if self.crosswalk_counter >= 3 and len(zebra_rows) > 0:
            self.crosswalk_detection = True
            cw_start = zebra_rows[0] + search_top
            cw_end = zebra_rows[-1] + search_top
            if debug:
                cv2.rectangle(frame, (corridor_left, cw_start), (corridor_right, cw_end), (0, 255, 0), 2)
                cv2.putText(frame, "CROSSWALK", (corridor_left, cw_start - 10), FONT, 0.8, (0, 255, 0), 2)

        # Clear remaining horizontal noise using a wide rectangular morphology kernel
        horizontal_mask = cv2.dilate(cv2.morphologyEx(lane["frame"], cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (25, 1))), np.ones((3, 3), np.uint8))
        lane["frame"] = cv2.bitwise_and(lane["frame"], cv2.bitwise_not(horizontal_mask))
            
        # --- LANE SLICING & POINTS ---
        up_slice = lane["frame"][int(h * up_slice_val):int(h * up_slice_val) + self.slicethickness, :]
        low_slice = lane["frame"][int(h * down_slice_val):int(h * down_slice_val) + self.slicethickness, :]

        urightest, uleftest = cv2.findNonZero(up_slice[:, int(w * 0.5):]), cv2.findNonZero(up_slice[:, :int(w * 0.5)])
        rightest, leftest = cv2.findNonZero(low_slice[:, int(w * 0.55):]), cv2.findNonZero(low_slice[:, :int(w * 0.55)])

        # Hazard Mode: Track the outer edge of thick lines when crossing hazardous areas
        use_outer_lanes = self.crosswalk_detection or chessBoard
        
        if debug and use_outer_lanes: 
            cv2.putText(frame, "HAZARD MODE: TRACKING OUTER EDGES", (10, 85), FONT, 0.6, (0, 0, 255), 2)

        if urightest is not None: 
            urightest = urightest[np.argmax(urightest[:, 0, 0]) if use_outer_lanes else np.argmin(urightest[:, 0, 0])] + [int(w * 0.5), int(h * up_slice_val)]
            if debug: cv2.circle(frame, tuple(urightest[0]), 5, (0, 255, 0), -1)
            
        if uleftest is not None: 
            uleftest = uleftest[np.argmin(uleftest[:, 0, 0]) if use_outer_lanes else np.argmax(uleftest[:, 0, 0])] + [0, int(h * up_slice_val)]
            if debug: cv2.circle(frame, tuple(uleftest[0]), 5, (0, 255, 0), -1)
            
        if rightest is not None: 
            rightest = rightest[np.argmax(rightest[:, 0, 0]) if use_outer_lanes else np.argmin(rightest[:, 0, 0])] + [int(w * 0.55), int(h * down_slice_val)]
            if debug: cv2.circle(frame, tuple(rightest[0]), 5, (0, 255, 0), -1)
            
        if leftest is not None: 
            leftest = leftest[np.argmin(leftest[:, 0, 0]) if use_outer_lanes else np.argmax(leftest[:, 0, 0])] + [0, int(h * down_slice_val)]
            if debug: cv2.circle(frame, tuple(leftest[0]), 5, (0, 255, 0), -1)

        if debug:
            if urightest is not None and rightest is not None:
                cv2.arrowedLine(frame, tuple(rightest[0]), tuple(urightest[0]), (255, 255, 0), 2)
            if uleftest is not None and leftest is not None:
                cv2.arrowedLine(frame, tuple(leftest[0]), tuple(uleftest[0]), (255, 255, 0), 2)

        # --- TRUST & SELECTION LOGIC ---
        left_angle, right_angle = None, None
        if uleftest is not None and leftest is not None:
            angle = math.degrees(math.atan2(leftest[0][1] - uleftest[0][1], uleftest[0][0] - leftest[0][0]))
            if 5.0 <= angle <= 175.0: left_angle = angle
            
        if urightest is not None and rightest is not None:
            angle = math.degrees(math.atan2(rightest[0][1] - urightest[0][1], urightest[0][0] - rightest[0][0]))
            if 5.0 <= angle <= 175.0: right_angle = angle

        left_type, left_vis = self.classify_lane(lane["frame"][:, : int(w * 0.5)])
        right_type, right_vis = self.classify_lane(lane["frame"][:, int(w * 0.5) :])
        
        global left_line_type, right_line_type
        left_line_type, right_line_type = left_type, right_type

        left_score = self.calculate_trust("left", left_vis, left_angle, left_type)
        right_score = self.calculate_trust("right", right_vis, right_angle, right_type)
        self.prev_scores["left"], self.prev_scores["right"] = left_score, right_score
        if left_angle is not None: self.prev_angles["left"] = left_angle
        if right_angle is not None: self.prev_angles["right"] = right_angle

        left_valid = left_type != "none" and left_angle is not None
        right_valid = right_type != "none" and right_angle is not None
        selected_mode = "none"

        # Force tracking the left lane when approaching early stops to avoid right-turn drifting
        if self.early_stop_line_detected and left_valid:
            selected_mode = "left"
            if debug: cv2.putText(frame, "EARLY STOP: FORCING LEFT LANE", (10, 60), FONT, 0.6, (0, 165, 255), 2)
        else:
            if left_valid and right_valid:
                if left_type == "solid" and right_type == "dashed" and left_score > 0.3: selected_mode = "left"
                elif right_type == "solid" and left_type == "dashed" and right_score > 0.3: selected_mode = "right"
                else: selected_mode = "center"
            elif left_valid and not right_valid: selected_mode = "left"
            elif right_valid and not left_valid: selected_mode = "right"

        # --- CTE & ANGLE EXECUTION ---
        meter_per_pixel = 0.35 / self.road_width_pixels
        line_thickness_px = 15 if use_outer_lanes else 0
        raw_cte, target_angle = 0.0, 90.0

        def get_left_cte(): return ((((uleftest[0][0] + leftest[0][0]) // 2) + line_thickness_px) - w // 2 + (self.road_width_pixels // 2)) * meter_per_pixel
        def get_right_cte(): return ((((urightest[0][0] + rightest[0][0]) // 2) - line_thickness_px) - w // 2 - (self.road_width_pixels // 2)) * meter_per_pixel

        raw_cte_l = get_left_cte() if (uleftest is not None and leftest is not None) else 0.0
        raw_dev_l = (90.0 - left_angle) if left_angle is not None else 0.0
        raw_cte_r = get_right_cte() if (urightest is not None and rightest is not None) else 0.0
        raw_dev_r = (90.0 - right_angle) if right_angle is not None else 0.0

        if selected_mode == "left":
            raw_cte, target_angle = get_left_cte(), left_angle
            if debug: cv2.putText(frame, "Mode: Follow Left", (10, h - 30), FONT, 0.7, (0, 255, 0), 2)
        elif selected_mode == "right":
            raw_cte, target_angle = get_right_cte(), right_angle
            if debug: cv2.putText(frame, "Mode: Follow Right", (10, h - 30), FONT, 0.7, (0, 255, 0), 2)
        elif selected_mode == "center":
            raw_cte = (get_left_cte() + get_right_cte()) / 2.0
            target_angle = (left_angle + right_angle) / 2.0
            if debug: cv2.putText(frame, "Mode: Average Center", (10, h - 30), FONT, 0.7, (0, 255, 0), 2)

        if debug and selected_mode != "none":
            pixel_offset = int(raw_cte / meter_per_pixel)
            cv2.arrowedLine(frame, (w // 2, h - 150), (w // 2 + pixel_offset, h - 150), (0, 255, 255), 5)
            cv2.putText(frame, f"CTE: {raw_cte:.2f}m", (10, h - 60), FONT, 0.7, (255, 255, 0), 2)

            length = 50
            center_point = (w // 2, h // 2)
            end_x = int(center_point[0] + length * math.cos(math.radians(target_angle)))
            end_y = int(center_point[1] - length * math.sin(math.radians(target_angle)))
            cv2.arrowedLine(frame, center_point, (end_x, end_y), (0, 255, 255), 5)

        if selected_mode != "none":
            self.smoothed_deviation = (0.3 * self.smoothed_deviation) + (0.7 * (90 - target_angle))
            self.smoothed_cte = (0.15 * self.smoothed_cte) + (0.85 * raw_cte)

        if uleftest is not None and leftest is not None and left_angle is not None:
            self.smoothed_cte_left = 0.15 * self.smoothed_cte_left + 0.85 * raw_cte_l
            self.smoothed_deviation_left = 0.3 * self.smoothed_deviation_left + 0.7 * raw_dev_l

        if urightest is not None and rightest is not None and right_angle is not None:
            self.smoothed_cte_right = 0.15 * self.smoothed_cte_right + 0.85 * raw_cte_r
            self.smoothed_deviation_right = 0.3 * self.smoothed_deviation_right + 0.7 * raw_dev_r
            
        if debug:
            cv2.putText(frame, f"Deviation: {self.smoothed_deviation:.2f} deg", (10, h - 90), FONT, 0.7, (255, 255, 0), 2)

        return (
            frame, lane, self.smoothed_deviation, self.smoothed_cte,
            self.smoothed_deviation_left, self.smoothed_cte_left,
            self.smoothed_deviation_right, self.smoothed_cte_right,
            stop_line, mid_stop_line, chessBoard,
        )


class ProcessNN(dai.node.HostNode):
    _MAX_AGE = 5
    _PATCH_SIZE = 5  
    _DEPTH_MIN_MM = 100  

    def __init__(self) -> None:
        super().__init__()
        self.detection_counter = {}
        self.CONFIRMATION_THRESHOLD = 3
        self.VISIBLE_THRESHOLD = 10
        self.list_detections_list = []

        self.current_depth_frame = None
        self.depth_cache: dict = {}
        self.intersection_history = []
        self.frames_since_intersection = 0
        self.last_valid_mask = 0

    def build(self, queuesList, logger, debugger, sync_out: dai.Node.Output, labels: List[str], quality=60) -> "ProcessNN":
        self.queuesList = queuesList
        self.logging = logger
        self.debugging = debugger
       
        self.link_args(sync_out)
        self.sendProcessingToPipeline(False)

        self.labels = labels
        self.quality = quality
       
        self.serialCameraSender = messageHandlerSender(self.queuesList, serialCamera)
        self.ObjectDetectSender = messageHandlerSender(self.queuesList, ObjectDetect)
        self.IntersectionMaskSender = messageHandlerSender(self.queuesList, IntersectionMask)
       
        self.nn_frame_counter = 0
        return self

    def _get_depth_for_detection(self, detection: dai.ImgDetection) -> str:
        """Extracts spatial distance from the aligned depth map around the detection center."""
        label_id = int(detection.label)
        if self.current_depth_frame is None:
            cached = self.depth_cache.get(label_id)
            if cached and cached[1] <= self._MAX_AGE:
                return f"{cached[0] / 1000:.2f}m"
            return "N/A"

        d_h, d_w = self.current_depth_frame.shape[:2]
        cx = int(((detection.xmin + detection.xmax) / 2) * d_w)
        cy = int(((detection.ymin + detection.ymax) / 2) * d_h)

        # Sample a localized patch to avoid background bleeding
        patch = self.current_depth_frame[
            max(0, cy - self._PATCH_SIZE): min(d_h, cy + self._PATCH_SIZE + 1),
            max(0, cx - self._PATCH_SIZE): min(d_w, cx + self._PATCH_SIZE + 1),
        ]
        valid = patch[patch > self._DEPTH_MIN_MM]

        if valid.size > 0:
            dist_mm = float(np.percentile(valid, 5))
            self.depth_cache[label_id] = [dist_mm, 0]
            return f"{dist_mm / 1000:.2f}m"

        return "N/A"

    def _draw_detection(self, img: np.ndarray, detection: dai.ImgDetection):
        height, width, _ = img.shape
        x1 = int(detection.xmin * width)
        x2 = int(detection.xmax * width)
        y1 = int(detection.ymin * height)
        y2 = int(detection.ymax * height)

        label = self.labels[detection.label] if detection.label < len(self.labels) else str(detection.label)
        dist_str = self._get_depth_for_detection(detection)
        bearing_rad, bearing_deg, dist_m = self.get_landmark_bearing_and_distance(detection, dist_str)

        if int(detection.label) in self.depth_cache:
            self.depth_cache[int(detection.label)][1] += 1

        proximity = self._dist_str_to_proximity(dist_str)
        box_h = y2 - y1

        # Differentiate scale assessment based on object expected physical size
        if label == "Car":
            proximity = "CLOSE" if (box_h > 200 and proximity == "FAR") else proximity
        else:
            proximity = "CLOSE" if (box_h > 250 and proximity == "FAR") else proximity
       
        cv2.rectangle(img, (x1, y1), (x2, y2), COLOR, 2)
        cv2.putText(img, str(label), (x1, y2 + 21), FONT, 0.8, COLOR, 2)
        cv2.putText(img, str(round(detection.confidence * 100, 2)), (x1, y2 + 45), FONT, 0.8, COLOR, 2)
        
        display_text = f"{dist_str} | {bearing_deg:.1f} deg"
        cv2.putText(img, display_text, (x1, y2 + 70), FONT, 0.55, (0, 255, 255), 2)
        
        return img, {
            "label": str(label), "proximity": str(proximity), "distance_str": dist_str,
            "distance_m": dist_m, "bearing_deg": bearing_deg, "bearing_rad": bearing_rad       
        }
   
    def get_landmark_bearing_and_distance(self, detection: dai.ImgDetection, depth_str: str):
        fx, cx = 922.569, 644.724
        u = ((detection.xmin + detection.xmax) / 2.0) * 1280.0
        bearing_rad = math.atan2(u - cx, fx)
        
        dist_m = float(depth_str.replace("m", "")) if depth_str != "N/A" else None
        return bearing_rad, math.degrees(bearing_rad), dist_m

    @staticmethod
    def _dist_str_to_proximity(dist_str: str) -> str:
        try:
            dist_m = float(dist_str.replace("m", ""))
            return "FAR" if dist_m >= 1.5 else "MEDIUM" if dist_m >= 0.6 else "CLOSE"
        except (ValueError, AttributeError):
            return "UNKNOWN"

    def det_key(self, d): return (d["label"], d["proximity"])

    def intersection_paths(self, frame: np.ndarray):
        """Polls Left, Center, Right ROIs to determine available turn paths at intersections."""
        h, w = frame.shape[:2]
        side_bw, side_bh = int(w * 0.22), int(h * 0.25)
        center_bw, center_bh = int(w * 0.32), int(h * 0.15)

        lx1, ly1 = 0, int(h * 0.12)
        cx1, cy1 = (w - center_bw) // 2, int(h * 0.11)
        rx1, ry1 = (w - int(w * 0.04)) - side_bw, int(h * 0.5)

        center_segs = detect_lines_in_roi(frame, cx1, cy1, cx1 + center_bw, cy1 + center_bh, 0, 15, length_axis="width")
        left_diag = detect_lines_in_roi(frame, lx1, ly1, lx1 + side_bw - int(w * 0.05), ly1 + side_bh, 20, 50, slope_sign="negative", kernel_shape="diag_neg", length_axis="height")
        right_diag = detect_lines_in_roi(frame, rx1, ry1, rx1 + side_bw, ry1 + side_bh, 50, 90, slope_sign="positive", kernel_shape="diag_pos", length_axis="height")

        if self.debugging:
            box_color = (0, 255, 0)
            cv2.rectangle(frame, (lx1, ly1), (lx1 + side_bw - int(w * 0.05), ly1 + side_bh), box_color, 2)
            cv2.rectangle(frame, (cx1, cy1), (cx1 + center_bw, cy1 + center_bh), box_color, 2)
            cv2.rectangle(frame, (rx1, ry1), (rx1 + side_bw, ry1 + side_bh), box_color, 2)
            cv2.putText(frame, "LEFT",   (lx1 + 10, ly1 + side_bh - 10),     FONT, 0.7, box_color, 2)
            cv2.putText(frame, "CENTER", (cx1 + 10, cy1 + 25), FONT, 0.7, box_color, 2)
            cv2.putText(frame, "RIGHT",  (rx1 + 10, ry1 + side_bh - 10),   FONT, 0.7, box_color, 2)

        def draw_clipped(img, x1, y1, x2, y2, color, thickness, rx1, ry1, rx2, ry2):
            overlay = np.zeros_like(img)
            cv2.line(overlay, (x1, y1), (x2, y2), color, thickness)
            overlay[:ry1, :] = 0
            overlay[ry2:, :] = 0
            overlay[:, :rx1] = 0
            overlay[:, rx2:] = 0
            mask = overlay.any(axis=2)
            img[mask] = overlay[mask]

        left_ok = False
        straight_ok = False
        right_ok = False

        if span_fraction(center_segs, "x") >= 0.25:
            straight_ok = True
            if self.debugging:
                best = max(center_segs, key=lambda l: abs(l[2] - l[0]))
                gx1, gy1, gx2, gy2 = best[0] + cx1, best[1] + cy1, best[2] + cx1, best[3] + cy1
                draw_clipped(frame, gx1, gy1, gx2, gy2, (0, 0, 255), 6, cx1, cy1, cx1 + center_bw, cy1 + center_bh)
                cv2.putText(frame, "BLOCK STRAIGHT", (int(np.clip(gx1, cx1, cx1 + center_bw)), int(np.clip(min(gy1, gy2) + 20, cy1 + 20, cy1 + center_bh - 5))), FONT, 0.8, (0, 0, 255), 2)

        if span_fraction(left_diag, "y") >= 0.7:
            left_ok = True
            if self.debugging:
                best = max(left_diag, key=lambda l: abs(l[3] - l[1]))
                gx1, gy1, gx2, gy2 = best[0] + lx1, best[1] + ly1, best[2] + lx1, best[3] + ly1
                draw_clipped(frame, gx1, gy1, gx2, gy2, (0, 255, 255), 4, lx1, ly1, lx1 + side_bw - int(w * 0.05), ly1 + side_bh)
                cv2.putText(frame, "LINE", (int(np.clip(gx1 + 5, lx1, lx1 + side_bw - int(w * 0.05) - 60)), int(np.clip(gy1 + 25, ly1 + 25, ly1 + side_bh - 5))), FONT, 0.7, (0, 255, 255), 2)

        if span_fraction(right_diag, "y") >= 0.5:
            right_ok = True
            if self.debugging:
                best = max(right_diag, key=lambda l: abs(l[3] - l[1]))
                gx1, gy1, gx2, gy2 = best[0] + rx1, best[1] + ry1, best[2] + rx1, best[3] + ry1
                draw_clipped(frame, gx1, gy1, gx2, gy2, (0, 255, 255), 4, rx1, ry1, rx1 + side_bw, ry1 + side_bh)
                cv2.putText(frame, "LINE", (int(np.clip(gx1 + 5, rx1, rx1 + side_bw - 60)), int(np.clip(gy1 + 25, ry1 + 25, ry1 + side_bh - 5))), FONT, 0.7, (0, 255, 255), 2)

        self.intersection_history.append((left_ok, straight_ok, right_ok))
        
        if len(self.intersection_history) > WINDOW: self.intersection_history.pop(0)

        threshold = len(self.intersection_history) / 2.0
        smooth_left = sum(1 for states in self.intersection_history if states[0]) >= threshold
        smooth_straight = sum(1 for states in self.intersection_history if states[1]) >= threshold
        smooth_right = sum(1 for states in self.intersection_history if states[2]) >= threshold

        mask = ((smooth_left << 2) | (smooth_straight << 1) | (smooth_right << 0))
        return frame, smooth_left, smooth_straight, smooth_right, mask
   
    def check_green_in_roi(self, frame: np.ndarray, x1: int, y1: int, x2: int, y2: int) -> bool:
        """Isolates circular active green LED signals using strict HSV boundaries."""
        tl_crop = frame[y1:y2, x1:x2]
        if tl_crop.size == 0: return False
            
        h, w = tl_crop.shape[:2]
        y_offset = h // 2
        bottom_half = tl_crop[y_offset:, :]

        if self.debugging:
            cv2.rectangle(frame, (x1, y1 + y_offset), (x2, y2), (255, 0, 0), 1)

        # Value > 180 ensures we ONLY detect actively glowing light sources
        green_mask = cv2.inRange(cv2.cvtColor(bottom_half, cv2.COLOR_BGR2HSV), np.array([50, 120, 180]), np.array([90, 255, 255]))
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        green_pixels = np.count_nonzero(green_mask)

        if self.debugging:
            cv2.putText(frame, f"G_px: {green_pixels}", (x1, y2 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        if green_pixels > 15: 
            contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) > 20: 
                    gx, gy, gw, gh = cv2.boundingRect(cnt)
                    if 0.4 < float(gw) / float(gh) < 2.5 and gw < (w * 0.8):
                        if self.debugging:
                            global_x = x1 + gx
                            global_y = y1 + y_offset + gy
                            cv2.rectangle(frame, (global_x, global_y), (global_x + gw, global_y + gh), (255, 255, 255), 2)
                        return True
        return False

    def process(self, group: dai.MessageGroup) -> None:
        preview_msg, nn_msg, depth_msg = group["preview"], group["nn"], group["depth"]
        if depth_msg is not None: self.current_depth_frame = depth_msg.getFrame()
        if preview_msg is None: return

        frame = preview_msg.getCvFrame().copy()

        try:
            if len(frame.shape) == 2 or (len(frame.shape) == 3 and frame.shape[2] == 1): frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif len(frame.shape) == 3 and frame.shape[2] == 2: frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_NV12)
        except Exception as rec_err:
            log_console("ProcessNN", "ERROR", f"Format conversion failed: {rec_err}", self.debugging)

        try:
            detections_list = []
            green_light_detected = False
           
            for detection in nn_msg.detections:
                if detection.confidence < MIN_CONF: continue
                frame, info = self._draw_detection(frame, detection)
                detections_list.append(info)

                if "traffic" in info["label"].lower():
                    h, w = frame.shape[:2]
                    if self.check_green_in_roi(frame, max(0, int(detection.xmin * w)), max(0, int(detection.ymin * h)), min(w, int(detection.xmax * w)), min(h, int(detection.ymax * h))):
                        cv2.putText(frame, "GREEN LIGHT ON", (max(0, int(detection.xmin * w)), max(20, max(0, int(detection.ymin * h)) - 25)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        green_light_detected = True

            # Re-append shared flags populated by ProcessMono
            if getattr(self.parent, "shared_stop_line", None): detections_list.append(self.parent.shared_stop_line)
            if getattr(self.parent, "shared_crosswalk", None): detections_list.append(self.parent.shared_crosswalk)
            if getattr(self.parent, "shared_start_line", None): detections_list.append(self.parent.shared_start_line)

            self.list_detections_list.append(detections_list)
            if len(self.list_detections_list) > WINDOW: self.list_detections_list.pop(0)

            counts, latest_det_data = Counter(), {}
            for frame_dets in self.list_detections_list:
                seen = set()
                for d in frame_dets:
                    k = self.det_key(d)
                    if k not in seen:
                        seen.add(k)
                        counts[k] += 1
                    latest_det_data[k] = d

            filtered_list = [latest_det_data[k] for k, count in counts.items() if count >= CONFIRM]
            if getattr(self.parent, "shared_intersection_approach", False): filtered_list.append({"label": "EnterIntersection", "proximity": "CLOSE"})
            if green_light_detected: filtered_list.append({"label": "GreenLight", "proximity": "CLOSE"})
            
            self.parent.shared_green_light = green_light_detected
            self.ObjectDetectSender.send(filtered_list)

            # Throttle complex tasks to run every 2nd frame
            self.nn_frame_counter += 1
            if self.nn_frame_counter % 2 == 0:
                is_approaching = getattr(self.parent, "shared_intersection_approach", False)
                is_at_stop = getattr(self.parent, "shared_stop_line", None) is not None
                is_early_stop = getattr(self.parent, "shared_early_stop_line", False)
               
                if is_approaching or is_early_stop or is_at_stop:
                    self.frames_since_intersection = 0
                    frame, _, _, _, intersection_mask = self.intersection_paths(frame)
                    
                    if self.last_valid_mask == -1: self.last_valid_mask = 0
                    self.last_valid_mask |= intersection_mask
                    self.IntersectionMaskSender.send(int(self.last_valid_mask))
                else:
                    self.frames_since_intersection += 1
                    if self.frames_since_intersection > 10:
                        self.intersection_history.clear()
                        self.last_valid_mask = -1 
                    self.IntersectionMaskSender.send(-1)
                
                # --- RECORD FULLY ANNOTATED DEBUG FRAME TO DISK ---
                if getattr(self.parent, "enable_rgb_recording", False) and hasattr(self.parent, "_init_rgb_writer"):
                    try:
                        self.parent._init_rgb_writer(frame.shape)
                        with self.parent.rgb_writer_lock:
                            if getattr(self.parent, "rgb_video_writer", None) is not None:
                                self.parent.rgb_video_writer.write(frame)
                    except Exception as rec_err:
                        log_console("ProcessNN", "ERROR", f"RGB Recording failed: {rec_err}", self.debugging)

                # Send fully annotated frame to your front-end UI
                _, serialEncodedImg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                self.serialCameraSender.send(base64.b64encode(serialEncodedImg).decode("utf-8"))

                # --- DEBUG: DISPLAY THE NN/OBJECT DETECTION WINDOW ---
                if self.debugging:
                    # cv2.imshow("DEBUG: Neural Net & Traffic", frame)
                    # cv2.waitKey(1)
                    pass # Headless bypass

        except Exception as e:
            log_console("ProcessNN", "ERROR", str(e), self.debugging)


class ProcessMono(dai.node.HostNode):
    def __init__(self) -> None:
        super().__init__()

    def build(self, queuesList, logger, debugger, camera_resolution, output_resolution, feed: dai.Node.Output, quality=60) -> "ProcessMono":
        self.queuesList = queuesList
        self.logging, self.debugging = logger, debugger
        self.link_args(feed)
        self.sendProcessingToPipeline(False)

        self.camera_resolution, self.output_resolution, self.quality = camera_resolution, output_resolution, quality
        self.threshold_min, self.threshold_max = 235, 250
        self.up_slice, self.down_slice = 0.75, 0.85
        self.width, self.height = int(camera_resolution[0]), int(camera_resolution[1])
        self.new_width, self.new_height = int(output_resolution[0]), int(output_resolution[1])
        self.stanley_gain = 0.55

        self.roi = TrapezoidROI.from_crops(0.14, 0.1, -0.145, self.new_width, self.new_height)
        self.lane_detection_instance = LaneDetection(self.new_width, self.new_height, slicethickness=65, road_width_pixels=190)

        self.mono_frame_counter, self.lastValidSpeed, self.vehicle_speed = 0, None, None

        self.processingCameraPrimarySender = messageHandlerSender(self.queuesList, ProcessingCameraPrimary)
        self.processingCameraSecondarySender = messageHandlerSender(self.queuesList, ProcessingCameraSecondary)
        self.visionManeuverSender = messageHandlerSender(self.queuesList, VisionManeuver)
        self.LeftLineClassifierSender =  messageHandlerSender(self.queuesList, LeftLineClassifier)
        self.RightLineClassifierSender = messageHandlerSender(self.queuesList, RightLineClassifier)

        self.thresholdMinSubscriber = messageHandlerSubscriber(self.queuesList, ThresholdMin, "lastOnly", True)
        self.thresholdMaxSubscriber = messageHandlerSubscriber(self.queuesList, ThresholdMax, "lastOnly", True)
        self.stanleyGainSubscriber = messageHandlerSubscriber(self.queuesList, StanleyGain, "lastOnly", True)
        self.upSliceSubscriber = messageHandlerSubscriber(self.queuesList, UpSlice, "lastOnly", True)
        self.downSliceSubscriber = messageHandlerSubscriber(self.queuesList, DownSlice, "lastOnly", True)
        self.currentSpeedSubscriber = messageHandlerSubscriber(self.queuesList, CurrentSpeed, "lastOnly", True)

        self.configs()
        return self

    def process(self, feed: dai.Buffer) -> None:
        frame = feed.getCvFrame().copy()
        if len(frame.shape) == 2 or (len(frame.shape) == 3 and frame.shape[2] == 1): frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif len(frame.shape) == 3 and frame.shape[2] == 2: frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_NV12)

        try:
            (output_frame, prebinary, heading_error, cross_track_error,
             heading_error_left, cross_track_error_left, heading_error_right, cross_track_error_right,
             stop_line, mid_stop_line, findStart) = self.lane_detection_instance.lane_detector(
                    frame, self.roi, self.up_slice, self.down_slice, self.threshold_min, self.threshold_max, debug=self.debugging)

            # Hold start line detection true for 3 seconds to accommodate slow vehicle pull-offs
            if not hasattr(self, 'start_timer'): self.start_timer = 0
            if findStart: self.start_timer = time.time()
               
            self.parent.shared_start_line = {"label": "start", "proximity": "CLOSE", "distance_m": 0.1, "bearing_deg": 0.0, "bearing_rad": 0.0} if time.time() - self.start_timer < 3.0 else None
            self.parent.shared_stop_line = {"label": "STOP_LINE", "proximity": "CLOSE"} if stop_line == "STOP_LINE" else None
            self.parent.shared_early_stop_line = self.lane_detection_instance.early_stop_line_detected
            self.parent.shared_intersection_approach = (mid_stop_line == "MID_STOP_LINE")

            speed_msg = self.currentSpeedSubscriber.receive()
            if speed_msg is not None: self.lastValidSpeed = speed_msg
            self.vehicle_speed = self.lastValidSpeed

            # Execute parallel Stanley controllers for lane isolation scenarios
            steering_angle = stanley_controller(heading_error, cross_track_error, self.stanley_gain, self.vehicle_speed)
            steering_angle_left = stanley_controller(heading_error_left, cross_track_error_left, self.stanley_gain, self.vehicle_speed)
            steering_angle_right = stanley_controller(heading_error_right, cross_track_error_right, self.stanley_gain, self.vehicle_speed)

            self.LeftLineClassifierSender.send(left_line_type)
            self.RightLineClassifierSender.send(right_line_type)

            steering_angle_for_display = -(steering_angle - 90)
            new_height, new_width, _ = output_frame.shape
            cv2.putText(output_frame, f"Steer: {steering_angle:.2f} deg", (10, new_height - 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            
            center_point = (new_width // 2, new_height // 2 - 120)
            arrow_length = 100
            end_x = int(center_point[0] + arrow_length * math.cos(math.radians(steering_angle_for_display)))
            end_y = int(center_point[1] - arrow_length * math.sin(math.radians(steering_angle_for_display)))
            cv2.arrowedLine(output_frame, center_point, (end_x, end_y), (255, 0, 255), 10)

            # Encode and send telemetry telemetry every second frame
            self.mono_frame_counter += 1
            if self.mono_frame_counter % 2 == 0:
                self.processingCameraPrimarySender.send(base64.b64encode(cv2.imencode(".jpg", output_frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])[1]).decode("utf-8"))
                self.processingCameraSecondarySender.send(base64.b64encode(cv2.imencode(".jpg", prebinary["frame"], [cv2.IMWRITE_JPEG_QUALITY, self.quality])[1]).decode("utf-8"))

            def _pack(angle): return str(int(map_range(constrain_num(angle, -23.0, 23.0), -23.0, 23.0, 0.0, 500.0)))

            self.visionManeuverSender.send({
                "throttle": "0",
                "steering": _pack(steering_angle),
                "steering_center": _pack(steering_angle),
                "steering_left": _pack(steering_angle_left),
                "steering_right": _pack(steering_angle_right),
            })

        except Exception as e:
            log_console("ProcessMono", "ERROR", str(e), self.debugging)

    def configs(self):
        """Asynchronously updates tuning parameters via message queues without blocking the pipeline."""
        if self.thresholdMinSubscriber.is_data_in_pipe(): self.threshold_min = constrain_num(int(float(self.thresholdMinSubscriber.receive())), 0, 255)
        if self.thresholdMaxSubscriber.is_data_in_pipe(): self.threshold_max = constrain_num(int(float(self.thresholdMaxSubscriber.receive())), 0, 255)
        if self.stanleyGainSubscriber.is_data_in_pipe(): self.stanley_gain = constrain_num(float(self.stanleyGainSubscriber.receive()), 0.0, 10.0)
        if self.upSliceSubscriber.is_data_in_pipe(): self.up_slice = constrain_num(float(self.upSliceSubscriber.receive()), 0.0, 1.0)
        if self.downSliceSubscriber.is_data_in_pipe(): self.down_slice = constrain_num(float(self.downSliceSubscriber.receive()), 0.0, 1.0)
        threading.Timer(1, self.configs).start()


class processOakD(WorkerProcess):
    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        super(processOakD, self).__init__(queueList, ready_event)

        FPS, CAM_RESOLUTION, OUT_RESOLUTION = 8, (256, 192), (400, 700)
        
        # Automatically record to an AVI file if debugging is turned on
        self.enable_rgb_recording = debugging
        
        try:
            self.queuesList, self.logging, self.debugging = queueList, logging, debugging
            self.device = dai.Device(maxUsbSpeed=dai.UsbSpeed.SUPER)
            self.pipeline = dai.Pipeline(self.device)
            self.nn_archive = dai.NNArchive("/home/pi/Documents/Brain/nn/best.rvc2.tar.xz")

            # Primary Color Camera & NN Build
            self.input_node = self.pipeline.create(dai.node.Camera).build()
            self.nn_with_parser = self.pipeline.create(ParsingNeuralNetwork).build(self.input_node, self.nn_archive)
            self.nn_with_parser.input.setBlocking(False)
            self.nn_with_parser.input.setMaxSize(1)
            self.nn_with_parser.setNumInferenceThreads(2)
            self.nn_with_parser.setNumNCEPerInferenceThread(1)
           
            self.rgb_full_out = self.input_node.requestOutput((640, 640), fps=FPS, type=dai.ImgFrame.Type.BGR888p)

            # Mono Stereo Cameras for Depth and Track Finding
            self.monoLeftStereo = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            self.monoRightStereo = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            self.monoLeftStereoOut = self.monoLeftStereo.requestOutput((640, 480), fps=FPS, type=dai.ImgFrame.Type.GRAY8)
            self.monoRightStereoOut = self.monoRightStereo.requestOutput((640, 480), fps=FPS, type=dai.ImgFrame.Type.GRAY8)

            self.stereo = self.pipeline.create(dai.node.StereoDepth)
            self.stereo.setLeftRightCheck(True)
            self.stereo.setSubpixel(False)
            self.stereo.setOutputSize(640, 480)
            self.stereo.initialConfig.setConfidenceThreshold(200)
            self.stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5)

            self.monoLeftStereoOut.link(self.stereo.left)
            self.monoRightStereoOut.link(self.stereo.right)
           
            # Message Synchronization (Wait max 50ms to group frames)
            self.sync = self.pipeline.create(dai.node.Sync)
            self.sync.setSyncThreshold(timedelta(milliseconds=50))
            self.rgb_full_out.link(self.sync.inputs["preview"])
            self.nn_with_parser.out.link(self.sync.inputs["nn"])
            self.stereo.depth.link(self.sync.inputs["depth"])

            self.nn_process = self.pipeline.create(ProcessNN).build(
                queueList, logging, debugging, sync_out=self.sync.out,
                labels=self.nn_archive.getConfigV1().model.heads[0].metadata.classes, quality=60,
            )

            # Use left mono output strictly for lane processing
            self.monoLeftOut = self.monoLeftStereo.requestOutput((256, 192), fps=FPS, type=dai.ImgFrame.Type.GRAY8)
            self.process_mono = self.pipeline.create(ProcessMono).build(
                queueList, logging, debugging, camera_resolution=CAM_RESOLUTION,
                output_resolution=OUT_RESOLUTION, feed=self.monoLeftOut, quality=60,
            )

            # Shared flags passed across HostNodes
            self.shared_stop_line = None
            self.shared_crosswalk = None
            self.shared_start_line = None
            self.shared_intersection_approach = False
            self.shared_green_light = False

            if self.enable_rgb_recording:
                os.makedirs("/home/pi/recordings", exist_ok=True)
                self.rec_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.rgb_video_writer = None
                self.rgb_video_path = f"/home/pi/recordings/rgb_{self.rec_timestamp}.avi"
                self.rgb_fps, self.rgb_writer_lock = FPS, threading.Lock()
                log_console("Oak-D Process", "INFO", f"RGB recording active -> {self.rgb_video_path}", self.debugging)
            
            self.nn_process.parent = self
            self.process_mono.parent = self

            self.pipeline.start()
            log_console("Oak-D Process", "INFO", "Oak-D Process initialized successfully. ", self.debugging)

        except Exception as e:
            log_console("Oak-D Process", "ERROR", f"Initialization failed: {e}", self.debugging)

    def _init_rgb_writer(self, frame_shape):
        if getattr(self, "rgb_video_writer", None) is not None: return

        h, w = frame_shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        self.rgb_video_writer = cv2.VideoWriter(self.rgb_video_path, fourcc, self.rgb_fps, (w, h))

        if not self.rgb_video_writer.isOpened():
            raise RuntimeError("Failed to open RGB video writer.")

    def stop(self):
        try:
            if getattr(self, "rgb_video_writer", None) is not None:
                self.rgb_video_writer.release()
                log_console("Oak-D Process", "INFO", f"Saved recording to: {self.rgb_video_path}", self.debugging)
        except Exception as e:
            log_console("Oak-D Process", "ERROR", f"Failed to release writer: {e}", self.debugging)

        try: self.device.close()
        except: pass

    def state_change_handler(self): pass
    def process_work(self): pass
    def _init_threads(self): pass

if __name__ == "__main__":
    sys.path.insert(0, "../../..")