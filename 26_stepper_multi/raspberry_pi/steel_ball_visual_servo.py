#!/usr/bin/env python3
"""Steel-ball detection and eye-in-hand visual servo for Raspberry Pi 5.

The web preview is always safe to open.  Motion starts only after the operator
presses START (or supplies --auto-start).  Exactly one BALL observation is sent
at a time; a new observation is not sent until the MCU answers the previous
one, so frames captured while the arm is moving cannot queue up as motion.
"""

from __future__ import annotations

import argparse
from collections import deque
import importlib
import json
import math
import os
import signal
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import serial


PREVIOUS_DETECTOR_DIR = Path("/home/wxm/shijue/08_usb_preview")
DEFAULT_CAMERA_DEVICE = (
    "/dev/v4l/by-id/"
    "usb-DHZJ-240229-XH_Integrated_Webcam_HD-video-index0"
)


def load_previous_detector():
    """Load the already-trained steel-ball ONNX detector from shijue."""
    detector_file = PREVIOUS_DETECTOR_DIR / "steel_ball_detect.py"
    if not detector_file.is_file():
        raise FileNotFoundError(f"找不到前面的钢球识别工程：{detector_file}")
    sys.path.insert(0, str(PREVIOUS_DETECTOR_DIR))
    return importlib.import_module("steel_ball_detect")


class McuLink:
    def __init__(self, port: str, baud: int, on_line) -> None:
        self.serial = serial.Serial(
            port=port, baudrate=baud, timeout=0.1, write_timeout=1.0
        )
        self.on_line = on_line
        self.write_lock = threading.Lock()
        self.running = True
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()

    def _read_loop(self) -> None:
        while self.running:
            try:
                raw = self.serial.readline()
            except serial.SerialException as exc:
                self.on_line(f"SERIAL_ERROR {exc}")
                return
            if raw:
                self.on_line(raw.decode("ascii", errors="replace").strip())

    def send(self, command: str) -> None:
        with self.write_lock:
            self.serial.write((command + "\n").encode("ascii"))
            self.serial.flush()

    def emergency_stop(self) -> None:
        with self.write_lock:
            self.serial.write(b"!\n")
            self.serial.flush()

    def close(self) -> None:
        self.running = False
        self.reader.join(timeout=0.5)
        self.serial.close()


class CameraSource:
    """Use the exact V4L2/MJPEG capture path from the previous detector."""

    def __init__(self, width: int, height: int, device: str, fps: int) -> None:
        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, fps)
        if not self.capture.isOpened():
            raise RuntimeError(
                f"无法打开前面识别工程使用的 USB 摄像头 {device}；"
                "请先停止占用摄像头的旧预览程序"
            )

    def read(self) -> np.ndarray:
        ok, frame = self.capture.read()
        if not ok:
            raise RuntimeError("摄像头取帧失败")
        return frame

    def close(self) -> None:
        self.capture.release()


class VisualServoApp:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.lock = threading.Lock()
        self.running = True
        self.tracking = False
        self.emergency_sent = False
        self.awaiting_reply = False
        self.reply_deadline = 0.0
        self.sequence = 0
        self.last_send = 0.0
        self.last_mcu = "尚未收到 MCU 消息"
        self.last_ball: tuple[int, int, int] | None = None
        self.last_confidence = 0.0
        self.detection_count = 0
        self.jpeg: bytes | None = None
        self.raw_ball: tuple[int, int, int] | None = None
        self.stable_history: deque[tuple[int, int, int]] = deque(
            maxlen=args.stable_frames
        )
        self.reset_observation_filter = False
        self.servo_state = "IDLE"
        self.servo_error = ""
        self.lost_frames = 0
        self.jog_sequence = 0
        self.jog_pending = False
        self.after_jog_state = ""
        self.calibration_axis = 0
        self.calibration_baseline: np.ndarray | None = None
        self.jacobian = np.zeros((3, 3), dtype=np.float64)
        self.last_command_deg: np.ndarray | None = None
        self.last_command_feature: np.ndarray | None = None
        self.last_command_mode: str | None = None
        self.recovery_message = ""
        self.recovery_resume_state = ""
        self.last_motion_result = "尚未执行受保护的视觉微动"
        self.bad_motion_count = 0
        self.trust_scale = 1.0
        self.center_hold = 0
        self.ready_hold = 0
        self.grasp_radius = args.grasp_radius
        self.detector_mode = args.detector
        self.detector = None
        self.detector_session = None
        self.detector_input_name = None
        self.detector_tracker = None
        if self.detector_mode == "onnx":
            self.detector = load_previous_detector()
            if not self.detector.MODEL.is_file():
                raise FileNotFoundError(f"钢球模型不存在：{self.detector.MODEL}")
            session_options = self.detector.ort.SessionOptions()
            session_options.intra_op_num_threads = 3
            session_options.graph_optimization_level = (
                self.detector.ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            )
            # The extracted ONNX Runtime used by the previous project writes
            # benign duplicate-schema lines to native stderr during creation.
            saved_stderr = os.dup(2)
            null_stderr = os.open(os.devnull, os.O_WRONLY)
            try:
                os.dup2(null_stderr, 2)
                self.detector_session = self.detector.ort.InferenceSession(
                    str(self.detector.MODEL),
                    sess_options=session_options,
                    providers=["CPUExecutionProvider"],
                )
            finally:
                os.dup2(saved_stderr, 2)
                os.close(null_stderr)
                os.close(saved_stderr)
            self.detector_input_name = self.detector_session.get_inputs()[0].name
            self.detector_tracker = self.detector.StableTracker()
        self.link = McuLink(args.port, args.baud, self._on_mcu_line)
        self.camera = CameraSource(
            args.width, args.height, args.camera_device, args.camera_fps
        )
        self.worker = threading.Thread(target=self._vision_loop, daemon=True)
        self.worker.start()

    def _on_mcu_line(self, line: str) -> None:
        print(f"MCU > {line}", flush=True)
        send_stop = False
        with self.lock:
            self.last_mcu = line
            if line.startswith("RELATIVE READY"):
                self.jog_pending = False
                self.servo_state = "CAL_WAIT_BASELINE"
                self.reset_observation_filter = True
            elif line.startswith("JOG DONE"):
                self.jog_pending = False
                self.servo_state = self.after_jog_state
                self.reset_observation_filter = True
            elif line.startswith("RELATIVE OFF"):
                self.jog_pending = False
            if (
                line.startswith("BALL MOVED")
                or line.startswith("BALL READY")
                or line.startswith("ERR ")
                or line.startswith("TRACK ")
                or line.startswith("STOPPED ")
            ):
                self.awaiting_reply = False
            if line.startswith("ERR ") or line.startswith("STOPPED "):
                self.tracking = False
                if self.servo_state != "IDLE":
                    self.servo_state = "ERROR"
                    self.servo_error = line
                    send_stop = line.startswith("ERR ")
            if line.startswith("BALL READY"):
                self.tracking = False
        if send_stop:
            self.stop_tracking(emergency=True)

    def _find_ball(self, frame: np.ndarray) -> tuple[int, int, int] | None:
        if self.detector_mode == "hough":
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (9, 9), 1.8)
            circles = cv2.HoughCircles(
                gray,
                cv2.HOUGH_GRADIENT,
                dp=1.2,
                minDist=self.args.min_distance,
                param1=self.args.canny,
                param2=self.args.hough_threshold,
                minRadius=self.args.min_radius,
                maxRadius=self.args.max_radius,
            )
            if circles is None:
                with self.lock:
                    self.detection_count = 0
                    self.last_confidence = 0.0
                return None

            height, width = frame.shape[:2]
            candidates = [
                tuple(map(int, np.rint(item))) for item in circles[0]
            ]
            candidates = [
                item
                for item in candidates
                if item[2] > 0
                and item[2] <= item[0] < width - item[2]
                and item[2] <= item[1] < height - item[2]
            ]
            with self.lock:
                self.detection_count = len(candidates)
                self.last_confidence = 0.0
                previous = self.raw_ball
            if not candidates:
                return None
            if previous is None:
                image_center = (width / 2.0, height / 2.0)
                return min(
                    candidates,
                    key=lambda item: math.hypot(
                        item[0] - image_center[0], item[1] - image_center[1]
                    ),
                )
            return min(
                candidates,
                key=lambda item: math.hypot(
                    item[0] - previous[0], item[1] - previous[1]
                ) - 0.15 * item[2],
            )

        raw_detections = self.detector.detect(
            self.detector_session, self.detector_input_name, frame
        )
        detections = self.detector_tracker.update(raw_detections)
        candidates = []
        for left, top, right, bottom, confidence in detections:
            center_x = (left + right) // 2
            center_y = (top + bottom) // 2
            radius = max(1, int(round(((right - left) + (bottom - top)) / 4.0)))
            candidates.append((center_x, center_y, radius, float(confidence)))

        with self.lock:
            self.detection_count = len(candidates)
        if not candidates:
            with self.lock:
                self.last_confidence = 0.0
            return None

        with self.lock:
            previous = self.raw_ball
        if previous is None:
            selected = max(candidates, key=lambda item: item[3])
        else:
            # Preserve the previous detector's stable tracks and avoid switching
            # balls when more than one object is visible.
            selected = min(
                candidates,
                key=lambda item: math.hypot(
                    item[0] - previous[0], item[1] - previous[1]
                ) - 20.0 * item[3],
            )

        with self.lock:
            self.last_confidence = selected[3]
        return selected[0], selected[1], selected[2]

    def _stable_observation(
        self, raw: tuple[int, int, int] | None
    ) -> tuple[int, int, int] | None:
        with self.lock:
            if self.reset_observation_filter:
                self.stable_history.clear()
                self.reset_observation_filter = False
            if raw is None:
                self.stable_history.clear()
                return None
            self.stable_history.append(raw)
            if len(self.stable_history) < self.args.stable_frames:
                return None
            samples = np.asarray(self.stable_history, dtype=np.float64)

        median = np.median(samples, axis=0)
        center_error = np.hypot(samples[:, 0] - median[0], samples[:, 1] - median[1])
        radius_error = np.abs(samples[:, 2] - median[2])
        if (
            float(np.max(center_error)) > self.args.stable_center_px
            or float(np.max(radius_error)) > self.args.stable_radius_px
        ):
            return None
        return tuple(int(round(value)) for value in median)

    @staticmethod
    def _feature(ball: tuple[int, int, int]) -> np.ndarray:
        return np.asarray(ball, dtype=np.float64)

    def _fail_servo(self, message: str) -> None:
        with self.lock:
            self.servo_state = "ERROR"
            self.servo_error = message
            self.tracking = False
            self.jog_pending = False
        self.stop_tracking(emergency=True)

    def _send_jog(
        self,
        command_deg: np.ndarray,
        wait_state: str,
        after_state: str,
        feature_before: np.ndarray | None = None,
        feature_mode: str | None = None,
    ) -> None:
        command = np.asarray(command_deg, dtype=np.float64).copy()
        maximum = float(np.max(np.abs(command)))
        if maximum > self.args.max_jog_deg:
            command *= self.args.max_jog_deg / maximum
        mdeg = np.rint(command * 1000.0).astype(np.int32)
        if int(np.max(np.abs(mdeg))) < 10:
            self._fail_servo("计算出的关节微动过小或矩阵无效")
            return

        with self.lock:
            if self.jog_pending:
                return
            self.jog_sequence = (self.jog_sequence + 1) & 0xFFFFFFFF
            sequence = self.jog_sequence
            self.jog_pending = True
            self.servo_state = wait_state
            self.after_jog_state = after_state
            if feature_before is not None:
                self.last_command_deg = mdeg.astype(np.float64) / 1000.0
                self.last_command_feature = feature_before.copy()
                self.last_command_mode = feature_mode
        self.link.send(
            f"JOG {sequence} {int(mdeg[0])} {int(mdeg[1])} {int(mdeg[2])}"
        )

    def _assess_last_motion(self, feature: np.ndarray) -> bool:
        """Learn from each move; undo and retry instead of accumulating an error."""
        with self.lock:
            command = self.last_command_deg
            before = self.last_command_feature
            mode = self.last_command_mode
            self.last_command_deg = None
            self.last_command_feature = None
            self.last_command_mode = None
        if command is None or before is None:
            return True
        denominator = float(command @ command)
        if denominator < 1e-6:
            return True
        observed = feature - before
        predicted = self.jacobian @ command

        target = np.asarray(
            [self.args.width / 2.0, self.args.height / 2.0],
            dtype=np.float64,
        )
        center_before = float(np.linalg.norm(target - before[:2]))
        center_after = float(np.linalg.norm(target - feature[:2]))
        reason = ""
        center_improvement = center_before - center_after
        radius_improvement = float(feature[2] - before[2])
        if mode == "center":
            if center_improvement < self.args.min_center_progress_px:
                reason = (
                    f"圆心距离未减小 {center_before:.1f}px -> {center_after:.1f}px"
                )
        elif mode == "approach":
            if center_after > self.args.recenter_distance_px:
                reason = (
                    f"逼近后圆心偏离 {center_before:.1f}px -> {center_after:.1f}px"
                )
            elif radius_improvement < self.args.min_radius_progress_px:
                reason = (
                    f"钢球没有变近 r={before[2]:.1f}px -> {feature[2]:.1f}px"
                )

        # A verified wrong move carries stronger information than an ordinary
        # noisy move, so correct that joint-space direction more aggressively.
        update_gain = (
            self.args.bad_model_update_gain
            if reason
            else self.args.model_update_gain
        )
        self.jacobian += (
            update_gain
            * np.outer(observed - predicted, command)
            / denominator
        )

        if not reason:
            with self.lock:
                self.bad_motion_count = 0
                self.trust_scale = min(1.0, self.trust_scale * 1.20)
                self.last_motion_result = (
                    f"运动有效：圆心 {center_before:.1f}px -> {center_after:.1f}px，"
                    f"半径 {before[2]:.1f}px -> {feature[2]:.1f}px"
                )
            return True

        with self.lock:
            self.bad_motion_count += 1
            self.trust_scale = max(0.25, self.trust_scale * 0.50)
            self.recovery_message = reason
            self.recovery_resume_state = (
                "DESCEND_WAIT_OBS" if mode == "approach" else "CENTER_WAIT_OBS"
            )
            self.last_motion_result = (
                f"自适应退回：{reason}；缩小步长后重新计算"
            )
        self._send_jog(
            -command,
            "RECOVERY_WAIT_JOG",
            "RECOVERY_WAIT_OBS",
        )
        return False

    def _center_pseudoinverse(self) -> tuple[np.ndarray, np.ndarray]:
        """Return a damped image-center inverse and its joint-space nullspace."""
        center_jacobian = self.jacobian[:2, :]
        damping2 = self.args.dls_damping * self.args.dls_damping
        normal = center_jacobian @ center_jacobian.T
        inverse = center_jacobian.T @ np.linalg.inv(
            normal + damping2 * np.eye(2, dtype=np.float64)
        )
        nullspace = np.eye(3, dtype=np.float64) - inverse @ center_jacobian
        return inverse, nullspace

    def _servo_command(self, feature: np.ndarray, descending: bool) -> None:
        if not self._assess_last_motion(feature):
            return
        target_x = self.args.width / 2.0
        target_y = self.args.height / 2.0
        error_x = target_x - feature[0]
        error_y = target_y - feature[1]
        centered = (
            abs(error_x) <= self.args.center_deadband_px
            and abs(error_y) <= self.args.center_deadband_px
        )

        if not descending:
            if centered:
                with self.lock:
                    self.center_hold += 1
                    hold = self.center_hold
                if hold >= self.args.center_confirm_frames:
                    with self.lock:
                        self.tracking = False
                        self.servo_state = "CENTERED"
                    return
                return
            with self.lock:
                self.center_hold = 0
            center_error = np.asarray([error_x, error_y], dtype=np.float64)
            wait_state = "CENTER_WAIT_JOG"
            after_state = "CENTER_WAIT_OBS"
        else:
            radius_error = self.grasp_radius - feature[2]
            if not centered:
                radius_error = 0.0
            if centered and radius_error <= self.args.radius_deadband_px:
                with self.lock:
                    self.ready_hold += 1
                    hold = self.ready_hold
                if hold >= self.args.center_confirm_frames:
                    self.link.send("RELATIVE STOP")
                    with self.lock:
                        self.tracking = False
                        self.servo_state = "READY_TO_GRAB"
                    return
                return
            with self.lock:
                self.ready_hold = 0
            center_error = np.asarray([error_x, error_y], dtype=np.float64)
            wait_state = "DESCEND_WAIT_JOG"
            after_state = "DESCEND_WAIT_OBS"

        try:
            center_inverse, nullspace = self._center_pseudoinverse()
        except np.linalg.LinAlgError as exc:
            self._fail_servo(f"视觉矩阵求解失败：{exc}")
            return

        # Task 1 always has priority: first keep the ball centre in the image.
        command = self.args.center_gain * (center_inverse @ center_error)
        if descending and centered:
            # Task 2 uses only the one joint-space direction that does not move
            # the image centre (to first order).  This prevents approach motion
            # from fighting the centring controller.
            radius_jacobian = self.jacobian[2, :]
            radius_direction = nullspace @ radius_jacobian
            radius_sensitivity = float(radius_jacobian @ radius_direction)
            damping2 = self.args.dls_damping * self.args.dls_damping
            if radius_sensitivity <= self.args.min_radius_sensitivity:
                self._fail_servo(
                    "当前姿态没有可保持圆心的逼近方向，请稍微改变机械臂初始姿态"
                )
                return
            command += (
                self.args.descend_gain
                * radius_direction
                * radius_error
                / (radius_sensitivity + damping2)
            )
        with self.lock:
            command *= self.trust_scale
        self._send_jog(
            command,
            wait_state,
            after_state,
            feature_before=feature,
            feature_mode="approach" if descending and centered else "center",
        )

    def _process_adaptive_observation(
        self,
        stable: tuple[int, int, int] | None,
        raw_present: bool,
    ) -> None:
        with self.lock:
            state = self.servo_state
            active = state not in {
                "IDLE", "STOPPED", "CENTERED", "READY_TO_GRAB", "ERROR"
            }
            if active and not raw_present:
                self.lost_frames += 1
            elif raw_present:
                self.lost_frames = 0
            lost_frames = self.lost_frames

        if active and lost_frames >= self.args.lost_frame_limit:
            self._fail_servo("连续丢失钢球，已急停")
            return
        if stable is None:
            return
        feature = self._feature(stable)

        if state == "CAL_WAIT_BASELINE":
            with self.lock:
                self.calibration_baseline = feature.copy()
                axis = self.calibration_axis
            command = np.zeros(3, dtype=np.float64)
            command[axis] = self.args.probe_deg
            self._send_jog(
                command, "CAL_WAIT_PROBE_JOG", "CAL_WAIT_PROBE_OBS"
            )
        elif state == "CAL_WAIT_PROBE_OBS":
            with self.lock:
                baseline = self.calibration_baseline.copy()
                axis = self.calibration_axis
            column = (feature - baseline) / self.args.probe_deg
            self.jacobian[:, axis] = column
            command = np.zeros(3, dtype=np.float64)
            command[axis] = -self.args.probe_deg
            self._send_jog(
                command, "CAL_WAIT_RETURN_JOG", "CAL_WAIT_RETURN_OBS"
            )
        elif state == "CAL_WAIT_RETURN_OBS":
            with self.lock:
                baseline = self.calibration_baseline.copy()
                axis = self.calibration_axis
            returned_error = float(np.linalg.norm(feature - baseline))
            if returned_error > self.args.return_tolerance_px:
                self._fail_servo(
                    f"电机{axis + 1}回程后画面偏差{returned_error:.1f}px，校准中止"
                )
                return
            axis += 1
            if axis < 3:
                with self.lock:
                    self.calibration_axis = axis
                    self.calibration_baseline = feature.copy()
                command = np.zeros(3, dtype=np.float64)
                command[axis] = self.args.probe_deg
                self._send_jog(
                    command, "CAL_WAIT_PROBE_JOG", "CAL_WAIT_PROBE_OBS"
                )
                return

            rank = int(np.linalg.matrix_rank(self.jacobian, tol=0.5))
            condition = float(np.linalg.cond(self.jacobian))
            if rank < 3 or not math.isfinite(condition) or condition > 1500.0:
                self._fail_servo(
                    f"视觉标定矩阵不可用：rank={rank}, condition={condition:.1f}"
                )
                return
            with self.lock:
                self.servo_state = "CENTER_WAIT_OBS"
                self.center_hold = 0
                self.reset_observation_filter = True
        elif state == "CENTER_WAIT_OBS":
            self._servo_command(feature, descending=False)
        elif state == "DESCEND_WAIT_OBS":
            self._servo_command(feature, descending=True)
        elif state == "RECOVERY_WAIT_OBS":
            with self.lock:
                message = self.recovery_message
                bad_count = self.bad_motion_count
                resume_state = self.recovery_resume_state
                if bad_count < self.args.max_bad_motions:
                    self.servo_state = resume_state
                    self.last_motion_result = (
                        f"已退回：{message}；第 {bad_count} 次重新计算"
                    )
            if bad_count >= self.args.max_bad_motions:
                self._fail_servo(
                    f"{message}；连续 {bad_count} 次无改善，已退回并停止"
                )

    def start_adaptive_calibration(self) -> None:
        with self.lock:
            if self.last_ball is None:
                raise RuntimeError("尚未连续稳定识别到钢球")
            if self.servo_state not in {
                "IDLE", "STOPPED", "CENTERED", "READY_TO_GRAB", "ERROR"
            }:
                raise RuntimeError("视觉自标定或运动正在进行")
            self.tracking = True
            self.emergency_sent = False
            self.servo_state = "CAL_WAIT_READY"
            self.servo_error = ""
            self.calibration_axis = 0
            self.jacobian.fill(0.0)
            self.last_command_deg = None
            self.last_command_feature = None
            self.last_command_mode = None
            self.recovery_message = ""
            self.recovery_resume_state = ""
            self.last_motion_result = "正在进行三轴微动标定"
            self.bad_motion_count = 0
            self.trust_scale = 1.0
            self.center_hold = 0
            self.ready_hold = 0
            self.lost_frames = 0
            self.reset_observation_filter = True
        self.link.send("RELATIVE START")

    def start_adaptive_descent(self, target_radius: int) -> None:
        with self.lock:
            if self.servo_state != "CENTERED":
                raise RuntimeError("尚未完成自标定和连续对中")
            if self.last_ball is None:
                raise RuntimeError("当前没有稳定钢球坐标")
            if target_radius <= self.last_ball[2] + self.args.radius_deadband_px:
                raise RuntimeError("目标半径必须明显大于当前钢球半径")
            self.grasp_radius = target_radius
            self.tracking = True
            self.emergency_sent = False
            self.servo_state = "DESCEND_WAIT_OBS"
            self.ready_hold = 0
            self.lost_frames = 0
            self.reset_observation_filter = True

    def _send_observation(
        self, ball: tuple[int, int, int] | None, width: int, height: int
    ) -> None:
        now = time.monotonic()
        with self.lock:
            if not self.tracking or self.awaiting_reply:
                return
            if now - self.last_send < 1.0 / self.args.send_rate:
                return
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            sequence = self.sequence
            self.last_send = now
            self.awaiting_reply = ball is not None
            self.reply_deadline = now + self.args.reply_timeout

        if ball is None:
            self.link.send(f"LOST {sequence}")
        else:
            x, y, radius = ball
            # The MCU is calibrated against a 640x480 coordinate frame.  Keep
            # that protocol stable even when another capture size is selected.
            protocol_width = 640
            protocol_height = 480
            protocol_x = int(round(x * protocol_width / width))
            protocol_y = int(round(y * protocol_height / height))
            protocol_radius = max(1, int(round(radius * protocol_width / width)))
            self.link.send(
                f"BALL {sequence} {protocol_x} {protocol_y} {protocol_radius} "
                f"{protocol_width} {protocol_height}"
            )

    def _vision_loop(self) -> None:
        while self.running:
            try:
                frame = self.camera.read()
                height, width = frame.shape[:2]
                raw_ball = self._find_ball(frame)
                ball = self._stable_observation(raw_ball)
                with self.lock:
                    self.raw_ball = raw_ball
                    self.last_ball = ball
                self._process_adaptive_observation(
                    ball, raw_present=(raw_ball is not None)
                )
                center = (width // 2, height // 2)
                cv2.drawMarker(
                    frame, center, (0, 255, 255), cv2.MARKER_CROSS, 28, 2
                )
                if raw_ball is not None:
                    cv2.circle(
                        frame, (raw_ball[0], raw_ball[1]), raw_ball[2],
                        (0, 180, 255), 1,
                    )
                if ball is not None:
                    cv2.circle(frame, (ball[0], ball[1]), ball[2], (0, 255, 0), 2)
                    cv2.circle(frame, (ball[0], ball[1]), 3, (0, 0, 255), -1)
                    with self.lock:
                        confidence = self.last_confidence
                    distance = math.hypot(ball[0] - center[0], ball[1] - center[1])
                    label = (
                        f"x={ball[0]} y={ball[1]} r={ball[2]} "
                        f"center_d={distance:.1f}px"
                    )
                    if self.detector_mode == "onnx":
                        label += f" conf={confidence:.2f}"
                else:
                    label = "WAITING FOR STABLE BALL"
                with self.lock:
                    state = self.servo_state
                    mode = state
                cv2.putText(
                    frame, f"{self.detector_mode.upper()} {state}",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0) if self.tracking else (0, 180, 255), 2,
                )
                cv2.putText(
                    frame, label, (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (255, 255, 255), 2,
                )
                ok, encoded = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80]
                )
                if ok:
                    with self.lock:
                        self.jpeg = encoded.tobytes()
            except Exception as exc:  # keep a useful status visible on the web UI
                with self.lock:
                    self.last_mcu = f"VISION_ERROR {exc}"
                self.stop_tracking(emergency=True)
                time.sleep(0.2)

    def start_tracking(self) -> None:
        self.start_adaptive_calibration()

    def stop_tracking(self, emergency: bool = True) -> None:
        with self.lock:
            self.tracking = False
            self.awaiting_reply = False
            if self.servo_state != "ERROR":
                self.servo_state = "STOPPED"
            send_emergency = emergency and not self.emergency_sent
            if send_emergency:
                self.emergency_sent = True
        if send_emergency:
            self.link.emergency_stop()
        elif not emergency:
            self.link.send("TRACK STOP")

    def assume_start_pose(self) -> None:
        with self.lock:
            if self.tracking:
                raise RuntimeError("请先停止跟踪")
        self.link.send("POSE START")

    def status(self) -> dict[str, Any]:
        with self.lock:
            ball = self.last_ball
            raw_ball = self.raw_ball
            return {
                "tracking": self.tracking,
                "servo_state": self.servo_state,
                "servo_error": self.servo_error,
                "awaiting_reply": self.awaiting_reply,
                "raw_ball": None if raw_ball is None else {
                    "x": raw_ball[0], "y": raw_ball[1], "radius": raw_ball[2]
                },
                "ball": None if ball is None else {
                    "x": ball[0], "y": ball[1], "radius": ball[2]
                },
                "mcu": self.last_mcu,
                "detector": (
                    str(self.detector.MODEL)
                    if self.detector_mode == "onnx"
                    else "OpenCV HoughCircles"
                ),
                "detection_count": self.detection_count,
                "confidence": round(self.last_confidence, 3),
                "grasp_radius": self.grasp_radius,
                "jacobian": np.round(self.jacobian, 3).tolist(),
                "motion_guard": self.last_motion_result,
                "controller": "distance_first_v3",
                "trust_scale": round(self.trust_scale, 3),
                "bad_motion_count": self.bad_motion_count,
                "image_center": {
                    "x": self.args.width // 2,
                    "y": self.args.height // 2,
                },
                "center_distance_px": (
                    None
                    if ball is None
                    else round(
                        math.hypot(
                            ball[0] - self.args.width / 2.0,
                            ball[1] - self.args.height / 2.0,
                        ),
                        1,
                    )
                ),
            }

    def close(self) -> None:
        self.stop_tracking(emergency=True)
        self.running = False
        self.worker.join(timeout=1.0)
        self.camera.close()
        self.link.close()


HTML = """<!doctype html><html lang=zh-CN><meta charset=utf-8>
<meta name=viewport content='width=device-width,initial-scale=1'>
<title>钢球自标定视觉伺服</title><style>
body{font-family:system-ui;background:#111;color:#eee;max-width:900px;margin:auto;padding:18px}
img{width:100%;border:1px solid #444;border-radius:8px}button{font-size:18px;padding:10px 18px;margin:8px}
.start{background:#16833c;color:white}.down{background:#1769aa;color:white}.stop{background:#b32222;color:white}
input{font-size:18px;width:85px;padding:8px}pre{background:#222;padding:12px;white-space:pre-wrap}
</style><h2>钢球识别 / 任意低位自标定</h2><img src=/stream.mjpg>
<p><button class=start onclick=post('/api/start')>① 自标定并对中</button>
<label>测试目标半径 <input id=radius type=number value=30 min=15 max=180></label>
<button class=down onclick=descend()>② 确认后垂直下降</button>
<button class=stop onclick=post('/api/stop')>急停</button></p>
<p>黄色圆为单帧候选，绿色圆表示连续多帧稳定。只有状态 CENTERED 时才可下降。</p>
<pre id=s>loading...</pre><script>
async function post(u){let r=await fetch(u,{method:'POST'});document.querySelector('#s').textContent=await r.text()}
function descend(){post('/api/descend?radius='+document.querySelector('#radius').value)}
setInterval(async()=>{document.querySelector('#s').textContent=JSON.stringify(await(await fetch('/api/status')).json(),null,2)},300)
</script></html>""".encode("utf-8")


def make_handler(app: VisualServoApp):
    class Handler(BaseHTTPRequestHandler):
        def _reply(self, body: bytes, kind: str = "text/plain; charset=utf-8") -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            if self.path == "/":
                self._reply(HTML, "text/html; charset=utf-8")
            elif self.path == "/api/status":
                self._reply(
                    json.dumps(app.status(), ensure_ascii=False).encode("utf-8"),
                    "application/json; charset=utf-8",
                )
            elif self.path == "/stream.mjpg":
                self.send_response(HTTPStatus.OK)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.end_headers()
                try:
                    while app.running:
                        with app.lock:
                            jpeg = app.jpeg
                        if jpeg is not None:
                            self.wfile.write(
                                b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                                + jpeg + b"\r\n"
                            )
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:
            try:
                parsed = urlparse(self.path)
                if parsed.path == "/api/start":
                    app.start_adaptive_calibration()
                    self._reply(
                        "已开始三轴微动自标定；每轴最大试动 0.5°。".encode("utf-8")
                    )
                elif parsed.path == "/api/descend":
                    query = parse_qs(parsed.query)
                    radius = int(query.get("radius", [str(app.grasp_radius)])[0])
                    app.start_adaptive_descent(radius)
                    self._reply(
                        f"已开始保持圆心的垂直下降，目标半径 {radius}px。".encode("utf-8")
                    )
                elif parsed.path == "/api/stop":
                    app.stop_tracking(emergency=True)
                    self._reply("已发送急停并停止相对视觉模式。".encode("utf-8"))
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)
            except Exception as exc:
                self.send_error(HTTPStatus.CONFLICT, str(exc))

        def log_message(self, format: str, *args) -> None:
            return

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="钢球检测和机械臂末端视觉伺服")
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--web-port", type=int, default=8000)
    parser.add_argument(
        "--detector", choices=("hough", "onnx"), default="hough",
        help="钢球检测器；默认使用本程序的霍夫圆检测",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--camera-device", default=DEFAULT_CAMERA_DEVICE)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--send-rate", type=float, default=3.0)
    parser.add_argument("--reply-timeout", type=float, default=5.0)
    parser.add_argument("--min-radius", type=int, default=8)
    parser.add_argument("--max-radius", type=int, default=120)
    parser.add_argument("--min-distance", type=int, default=40)
    parser.add_argument("--canny", type=float, default=100.0)
    parser.add_argument("--hough-threshold", type=float, default=25.0)
    parser.add_argument("--stable-frames", type=int, default=4)
    parser.add_argument("--stable-center-px", type=float, default=8.0)
    parser.add_argument("--stable-radius-px", type=float, default=5.0)
    parser.add_argument("--lost-frame-limit", type=int, default=15)
    parser.add_argument("--probe-deg", type=float, default=1.0)
    parser.add_argument("--max-jog-deg", type=float, default=2.0)
    parser.add_argument("--return-tolerance-px", type=float, default=14.0)
    parser.add_argument("--center-deadband-px", type=float, default=10.0)
    parser.add_argument("--center-confirm-frames", type=int, default=3)
    parser.add_argument("--center-gain", type=float, default=0.80)
    parser.add_argument("--descend-gain", type=float, default=0.60)
    parser.add_argument("--min-center-progress-px", type=float, default=1.0)
    parser.add_argument("--recenter-distance-px", type=float, default=12.0)
    parser.add_argument("--min-radius-progress-px", type=float, default=0.8)
    parser.add_argument("--model-update-gain", type=float, default=0.25)
    parser.add_argument("--bad-model-update-gain", type=float, default=0.80)
    parser.add_argument("--dls-damping", type=float, default=1.5)
    parser.add_argument("--min-radius-sensitivity", type=float, default=0.20)
    parser.add_argument("--max-bad-motions", type=int, default=5)
    parser.add_argument("--radius-deadband-px", type=float, default=3.0)
    parser.add_argument("--grasp-radius", type=int, default=30)
    parser.add_argument("--auto-start", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    app = VisualServoApp(args)
    server = ThreadingHTTPServer((args.host, args.web_port), make_handler(app))

    def shutdown(_signum=None, _frame=None) -> None:
        app.stop_tracking(emergency=True)
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    if args.auto_start:
        print("--auto-start 已禁用：请从网页人工开始自标定", flush=True)
    print(f"Web UI: http://<raspberry-pi-ip>:{args.web_port}", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
