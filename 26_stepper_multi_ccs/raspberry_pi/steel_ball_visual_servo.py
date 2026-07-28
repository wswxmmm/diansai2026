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

# The MCU uses the magnet contact face as the tool point.  With the confirmed
# startup pose (base=0 deg, lower link=90 deg, upper link=0 deg), forward
# kinematics gives (173.0, 0.0, 220.2) mm in the robot-base frame.
STANDARD_TOOL_X_MM = 173.0
STANDARD_TOOL_Y_MM = 0.0
STANDARD_TOOL_Z_MM = 220.2
STANDARD_MIN_RADIUS_MM = 80.0
STANDARD_MAX_RADIUS_MM = 293.2
STANDARD_MIN_TOOL_Z_MM = 10.0
STANDARD_MAX_TOOL_Z_MM = 221.0

# Measured with a 10 mm steel ball placed directly below the magnet.  At that
# instant the stable observation was (304, 375, r=10) in a 640x480 frame.
# 5 mm / 10 px = 0.5 mm/px, giving camera->magnet offsets of +67.5 mm radial
# and -8.0 mm tangential in the standard-pose local frame.
CAMERA_TO_MAGNET_RADIAL_MM = 67.5
CAMERA_TO_MAGNET_TANGENTIAL_MM = -8.0

# First low-height trial: the real magnet stopped about 45--55 mm behind the
# ball even though the model reported the requested XY.  Keep this separate
# from the rigid camera-to-magnet calibration: it compensates low-pose arm
# geometry and can be tuned safely from the web page.
DEFAULT_LOW_Z_RADIAL_FORWARD_MM = 50.0

# Empirical height calibration: a requested 10 mm final clearance already
# places the real magnet almost against the 10 mm ball.  Treat 10 mm as the
# safe software floor until the tool-height model is measured again.
EMPIRICAL_FINAL_CLEARANCE_MM = 10.0


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
        self.mcu_history: deque[str] = deque(maxlen=20)
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
        self.standard_snapshot: dict[str, float | int] | None = None
        self.standard_target_mm: tuple[float, float, float] | None = None
        self.standard_sequence = 0
        self.goto_phase = ""
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
        followup_command: str | None = None
        with self.lock:
            self.last_mcu = line
            self.mcu_history.append(line)
            if line.startswith("RELATIVE REBASED"):
                self.jog_pending = False
                if self.servo_state == "DESCEND_WAIT_REBASE":
                    self.tracking = True
                    self.servo_state = "DESCEND_WAIT_OBS"
                    self.reset_observation_filter = True
                    self.last_motion_result = (
                        "已把对中位置设为下降阶段相对零点；"
                        "保留单步 2°、本阶段累计 25° 保护"
                    )
            elif line.startswith("RELATIVE READY"):
                self.jog_pending = False
                self.servo_state = "CAL_WAIT_BASELINE"
                self.reset_observation_filter = True
            elif line.startswith("JOG DONE"):
                self.jog_pending = False
                self.servo_state = self.after_jog_state
                self.reset_observation_filter = True
            elif line.startswith("RELATIVE OFF"):
                self.jog_pending = False
            elif (
                line.startswith("POSE ASSUMED_START")
                and self.servo_state == "STANDARD_WAIT_POSE"
            ):
                if self.standard_target_mm is None:
                    self.tracking = False
                    self.servo_state = "ERROR"
                    self.servo_error = "标准姿态目标坐标丢失"
                else:
                    self.sequence = (self.sequence + 1) & 0xFFFFFFFF
                    self.standard_sequence = self.sequence
                    x_mm, y_mm, z_mm = self.standard_target_mm
                    followup_command = (
                        f"GOTO {self.standard_sequence} "
                        f"{round(x_mm * 10.0)} {round(y_mm * 10.0)} "
                        f"{round(z_mm * 10.0)}"
                    )
                    self.servo_state = "STANDARD_MOVING"
                    self.last_motion_result = (
                        f"已确认标准零位，正在逆解并自动分段移动："
                        f"X={x_mm:.1f}, Y={y_mm:.1f}, Z={z_mm:.1f} mm"
                    )
            elif line.startswith("GOTO SOLVED"):
                self.servo_state = (
                    "FINAL_MOVING" if self.goto_phase == "final"
                    else "STANDARD_MOVING"
                )
                self.last_motion_result = (
                    f"逆解完成，准备自动分段到位（{self.goto_phase}）：{line}"
                )
            elif line.startswith("GOTO ENABLED"):
                self.servo_state = "STANDARD_MOVING"
                self.last_motion_result = f"三台驱动器已重新使能：{line}"
            elif line.startswith("GOTO ACTIVE"):
                self.servo_state = "STANDARD_MOVING"
                self.last_motion_result = f"同步触发已发送，三轴脉冲：{line}"
            elif line.startswith("GOTO STEP"):
                self.servo_state = (
                    "FINAL_MOVING" if self.goto_phase == "final"
                    else "STANDARD_MOVING"
                )
                self.last_motion_result = (
                    f"自动分段移动中（{self.goto_phase}）：{line}"
                )
            elif line.startswith("GOTO DONE"):
                parts = line.split()
                reply_sequence = int(parts[2]) if len(parts) >= 3 else -1
                if reply_sequence == self.standard_sequence:
                    self.tracking = False
                    if self.goto_phase == "final":
                        self.servo_state = "FINAL_APPROACH_DONE"
                        self.last_motion_result = (
                            "已到达最终贴近高度；请确认磁铁是否吸住钢球"
                        )
                    else:
                        self.servo_state = "STANDARD_APPROACH_DONE"
                        self.last_motion_result = (
                            "已到达钢球上方安全高度；请核对横向位置，"
                            "确认后再点击最终贴近"
                        )
                    self.reset_observation_filter = True
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
                    # _fail_servo records the useful vision-side reason before
                    # the MCU answers the emergency stop.  Do not replace it
                    # with the generic STOPPED line.
                    if not self.servo_error:
                        self.servo_error = line
                    send_stop = line.startswith("ERR ")
            if line.startswith("BALL READY"):
                self.tracking = False
        if send_stop:
            self.stop_tracking(emergency=True)
        elif followup_command is not None:
            self.link.send(followup_command)

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
        elif mode in {"approach", "approach_fallback"}:
            # The fallback intentionally allows temporary centre drift when
            # this pose has no strict centre-preserving approach direction.
            # Accept it when the ball grows; the next iteration recentres.
            if (
                mode == "approach"
                and center_after > self.args.recenter_distance_px
            ):
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
                        self.last_motion_result = (
                            "钢球已连续稳定对中，可以开始阶段重置后下降"
                        )
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
        approach_mode = "approach"
        if descending and centered:
            # Task 2 uses only the one joint-space direction that does not move
            # the image centre (to first order).  This prevents approach motion
            # from fighting the centring controller.
            radius_jacobian = self.jacobian[2, :]
            radius_direction = nullspace @ radius_jacobian
            radius_sensitivity = float(radius_jacobian @ radius_direction)
            damping2 = self.args.dls_damping * self.args.dls_damping
            if radius_sensitivity <= self.args.min_radius_sensitivity:
                # A strict null-space move is unavailable at some poses.  Do
                # not stop: move along the raw radius gradient, permit a small
                # temporary centre error, then let the high-priority centring
                # task correct it on the next observation.
                radius_direction = radius_jacobian.copy()
                radius_sensitivity = float(
                    radius_jacobian @ radius_direction
                )
                approach_mode = "approach_fallback"
                with self.lock:
                    self.last_motion_result = (
                        "无严格保圆心下降方向；改为先靠近，再自动重新对中"
                    )
                if radius_sensitivity < 1e-4:
                    # No measured radius response at all: use the joint with
                    # the strongest recorded response as a small probe rather
                    # than entering ERROR.
                    strongest = int(np.argmax(np.abs(radius_jacobian)))
                    radius_direction = np.zeros(3, dtype=np.float64)
                    radius_direction[strongest] = 1.0
                    radius_sensitivity = 1.0
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
            feature_mode=approach_mode if descending and centered else "center",
        )

    def _process_adaptive_observation(
        self,
        stable: tuple[int, int, int] | None,
        raw_present: bool,
    ) -> None:
        with self.lock:
            state = self.servo_state
            active = state.startswith((
                "CAL_", "CENTER_WAIT", "DESCEND_WAIT", "RECOVERY_WAIT"
            ))
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
            self.tracking = False
            self.emergency_sent = False
            self.servo_state = "DESCEND_WAIT_REBASE"
            self.ready_hold = 0
            self.lost_frames = 0
            self.reset_observation_filter = True
            self.last_motion_result = (
                "正在把当前对中位置设为下降阶段的新相对零点"
            )
        self.link.send("RELATIVE REBASE")

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

    def start_standard_approach(
        self, ball_diameter_mm: float, clearance_mm: float,
        radial_forward_mm: float,
    ) -> tuple[float, float, float]:
        """Snapshot the stable ball at the standard pose and start one IK move."""
        with self.lock:
            if self.last_ball is None:
                raise RuntimeError("尚未连续稳定识别到钢球，绿色圆出现后再确认")
            if self.servo_state not in {
                "IDLE", "STOPPED", "CENTERED", "READY_TO_GRAB", "ERROR",
                "STANDARD_APPROACH_DONE", "FINAL_APPROACH_DONE",
            }:
                raise RuntimeError("机械臂校准或运动正在进行")
            if not 5.0 <= ball_diameter_mm <= 50.0:
                raise RuntimeError("钢球直径必须在 5～50 mm 之间")
            if not 20.0 <= clearance_mm <= 100.0:
                raise RuntimeError("安全高度必须在 20～100 mm 之间")
            if not 0.0 <= radial_forward_mm <= 80.0:
                raise RuntimeError("向前补偿必须在 0～80 mm 之间")

            x_px, y_px, radius_px = self.last_ball
            if radius_px < 3:
                raise RuntimeError("钢球图像半径过小，无法可靠换算距离")

            # A sphere's physical radius divided by its image radius gives the
            # local millimetres-per-pixel scale at the ball plane.  The camera
            # points vertically down in the confirmed standard pose.
            mm_per_px = (ball_diameter_mm * 0.5) / float(radius_px)
            image_dx = x_px - self.args.width * 0.5
            image_dy = y_px - self.args.height * 0.5
            tangential_mm = (
                self.args.standard_image_x_sign * image_dx * mm_per_px
            )
            radial_mm = (
                self.args.standard_image_y_sign * image_dy * mm_per_px
            )

            # The camera and magnet are not coaxial.  Translate the camera
            # observation to the magnet axis using the measured rigid offset.
            target_x = (
                STANDARD_TOOL_X_MM + radial_mm
                - CAMERA_TO_MAGNET_RADIAL_MM
                + radial_forward_mm
            )
            target_y = (
                STANDARD_TOOL_Y_MM + tangential_mm
                - CAMERA_TO_MAGNET_TANGENTIAL_MM
            )
            # Table is Z=0.  Hover the magnet face `clearance_mm` above the
            # top of the ball; final contact is deliberately not automatic.
            target_z = ball_diameter_mm + clearance_mm
            target_radius = math.hypot(target_x, target_y)
            if not STANDARD_MIN_RADIUS_MM <= target_radius <= STANDARD_MAX_RADIUS_MM:
                raise RuntimeError(
                    f"换算目标半径 {target_radius:.1f} mm 超出安全工作区"
                )
            if not STANDARD_MIN_TOOL_Z_MM <= target_z <= STANDARD_MAX_TOOL_Z_MM:
                raise RuntimeError(f"换算目标高度 {target_z:.1f} mm 超出安全工作区")

            self.standard_snapshot = {
                "x_px": x_px,
                "y_px": y_px,
                "radius_px": radius_px,
                "mm_per_px": round(mm_per_px, 4),
                "ball_diameter_mm": ball_diameter_mm,
                "clearance_mm": clearance_mm,
                "radial_forward_mm": radial_forward_mm,
                "camera_to_magnet_radial_mm": CAMERA_TO_MAGNET_RADIAL_MM,
                "camera_to_magnet_tangential_mm": (
                    CAMERA_TO_MAGNET_TANGENTIAL_MM
                ),
            }
            self.standard_target_mm = (target_x, target_y, target_z)
            self.tracking = True
            self.emergency_sent = False
            self.awaiting_reply = False
            self.servo_state = "STANDARD_WAIT_POSE"
            self.goto_phase = "safe"
            self.servo_error = ""
            self.last_motion_result = (
                "已锁定当前绿色钢球坐标，等待 MCU 确认标准零位"
            )

        # POSE START makes the manually placed pose the numerical zero.  The
        # GOTO command is sent only after POSE ASSUMED_START is received.
        self.link.send("POSE START")
        return target_x, target_y, target_z

    def start_final_approach(
        self, final_clearance_mm: float
    ) -> tuple[float, float, float]:
        """Keep the calibrated XY target and lower the magnet near the ball."""
        with self.lock:
            if self.servo_state != "STANDARD_APPROACH_DONE":
                raise RuntimeError("必须先完成安全悬停并人工确认横向已经对准")
            if self.standard_target_mm is None or self.standard_snapshot is None:
                raise RuntimeError("没有可继续使用的安全悬停目标")
            if not EMPIRICAL_FINAL_CLEARANCE_MM <= final_clearance_mm <= 20.0:
                raise RuntimeError("最终贴近距离必须在 10～20 mm 之间")

            ball_diameter_mm = float(
                self.standard_snapshot["ball_diameter_mm"]
            )
            target_z = ball_diameter_mm + final_clearance_mm
            if target_z < STANDARD_MIN_TOOL_Z_MM:
                raise RuntimeError(
                    f"最终目标 Z={target_z:.1f} mm 低于固件安全下限"
                )
            target_x, target_y, _ = self.standard_target_mm
            self.standard_target_mm = (target_x, target_y, target_z)
            self.standard_snapshot["final_clearance_mm"] = final_clearance_mm
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            self.standard_sequence = self.sequence
            self.goto_phase = "final"
            self.tracking = True
            self.emergency_sent = False
            self.servo_state = "FINAL_MOVING"
            self.servo_error = ""
            self.last_motion_result = (
                f"正在保持 X={target_x:.1f}, Y={target_y:.1f} mm，"
                f"分段下降到 Z={target_z:.1f} mm"
            )
            command = (
                f"GOTO {self.standard_sequence} "
                f"{round(target_x * 10.0)} {round(target_y * 10.0)} "
                f"{round(target_z * 10.0)}"
            )

        self.link.send(command)
        return target_x, target_y, target_z

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
                "mcu_history": list(self.mcu_history),
                "detector": (
                    str(self.detector.MODEL)
                    if self.detector_mode == "onnx"
                    else "OpenCV HoughCircles"
                ),
                "detection_count": self.detection_count,
                "confidence": round(self.last_confidence, 3),
                "grasp_radius": self.grasp_radius,
                "standard_start_tool_mm": {
                    "x": STANDARD_TOOL_X_MM,
                    "y": STANDARD_TOOL_Y_MM,
                    "z": STANDARD_TOOL_Z_MM,
                },
                "standard_snapshot": self.standard_snapshot,
                "camera_to_magnet_mm": {
                    "radial": CAMERA_TO_MAGNET_RADIAL_MM,
                    "tangential": CAMERA_TO_MAGNET_TANGENTIAL_MM,
                },
                "low_z_radial_forward_default_mm": (
                    DEFAULT_LOW_Z_RADIAL_FORWARD_MM
                ),
                "empirical_final_clearance_mm": EMPIRICAL_FINAL_CLEARANCE_MM,
                "goto_phase": self.goto_phase,
                "standard_target_mm": (
                    None
                    if self.standard_target_mm is None
                    else {
                        "x": round(self.standard_target_mm[0], 1),
                        "y": round(self.standard_target_mm[1], 1),
                        "z": round(self.standard_target_mm[2], 1),
                    }
                ),
                "gear_reduction": "90/16 = 5.625:1",
                "jacobian": np.round(self.jacobian, 3).tolist(),
                "motion_guard": self.last_motion_result,
                "controller": (
                    "standard_snapshot_ik_staged_v2 + "
                    "distance_first_v3 + phase_rebase_v1 + "
                    "approach_fallback_v1"
                ),
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
.standard{background:#8a4fd1;color:white}.start{background:#16833c;color:white}.down{background:#1769aa;color:white}.stop{background:#b32222;color:white}
input{font-size:18px;width:85px;padding:8px}pre{background:#222;padding:12px;white-space:pre-wrap}
</style><h2>钢球识别 / 标准姿态逆解</h2><img src=/stream.mjpg>
<h3>已标定磁铁中心：径向 +67.5 mm，切向 -8.0 mm</h3>
<p>先手动摆好：下臂竖直、上臂水平、底座朝正前方；等钢球出现绿色稳定圆后再点安全悬停。</p>
<p><label>钢球直径(mm) <input id=diameter type=number value=10 min=5 max=50 step=0.1></label>
<label>球顶安全高度(mm) <input id=clearance type=number value=30 min=20 max=100 step=1></label>
<label>向前补偿(mm) <input id=radial-forward type=number value=50 min=0 max=80 step=5></label>
<button class=standard onclick=standardApproach()>① 校正磁铁偏移并安全悬停</button>
<label>最终贴近(mm) <input id=final-clearance type=number value=10 min=10 max=20 step=0.5></label>
<button class=down onclick=finalApproach()>② 确认横向对准后最终贴近</button>
<button class=stop onclick=post('/api/stop')>急停</button></p>
<p>向前补偿用于修正磁铁在钢球后方的低位误差。实测最终贴近 10 mm 已接近钢球，因此页面禁止低于 10 mm；未确认横向对准时禁止点击第二步。</p>
<hr><h3>任意低位自标定（保留的备用方案）</h3>
<p><button class=start onclick=post('/api/start')>① 自标定并对中</button>
<label>测试目标半径 <input id=radius type=number value=30 min=15 max=180></label>
<button class=down onclick=descend()>② 确认后垂直下降</button>
</p>
<p>黄色圆为单帧候选，绿色圆表示连续多帧稳定。只有状态 CENTERED 时才可下降；下降前自动清零本阶段累计角度，但仍保留单步和阶段总量保护。</p>
<pre id=s>loading...</pre><script>
async function post(u){let r=await fetch(u,{method:'POST'});document.querySelector('#s').textContent=await r.text()}
function standardApproach(){post('/api/standard-approach?diameter='+document.querySelector('#diameter').value+'&clearance='+document.querySelector('#clearance').value+'&forward='+document.querySelector('#radial-forward').value)}
function finalApproach(){post('/api/final-approach?clearance='+document.querySelector('#final-clearance').value)}
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
                if parsed.path == "/api/standard-approach":
                    query = parse_qs(parsed.query)
                    diameter = float(query.get("diameter", ["20"])[0])
                    clearance = float(query.get("clearance", ["40"])[0])
                    radial_forward = float(
                        query.get(
                            "forward", [str(DEFAULT_LOW_Z_RADIAL_FORWARD_MM)]
                        )[0]
                    )
                    target = app.start_standard_approach(
                        diameter, clearance, radial_forward
                    )
                    self._reply(
                        (
                            "已锁定当前稳定钢球，并确认标准起始角。"
                            f"目标 X={target[0]:.1f}, Y={target[1]:.1f}, "
                            f"Z={target[2]:.1f} mm；正在执行安全悬停。"
                        ).encode("utf-8")
                    )
                elif parsed.path == "/api/final-approach":
                    query = parse_qs(parsed.query)
                    clearance = float(
                        query.get(
                            "clearance", [str(EMPIRICAL_FINAL_CLEARANCE_MM)]
                        )[0]
                    )
                    target = app.start_final_approach(clearance)
                    self._reply(
                        (
                            f"保持 X={target[0]:.1f}, Y={target[1]:.1f} mm，"
                            f"正在分段下降到 Z={target[2]:.1f} mm。"
                        ).encode("utf-8")
                    )
                elif parsed.path == "/api/start":
                    app.start_adaptive_calibration()
                    self._reply(
                        (
                            "已开始三轴微动自标定；"
                            f"每轴探测 {app.args.probe_deg:.1f}°。"
                        ).encode("utf-8")
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
                body = str(exc).encode("utf-8")
                self.send_response(HTTPStatus.CONFLICT)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

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
    # A synchronized 2-degree jog plus settling can blur the image for close
    # to one second.  Allow 1.5 seconds before declaring the ball lost.
    parser.add_argument("--lost-frame-limit", type=int, default=45)
    # One degree was often quantized to zero by the integer Hough radius,
    # producing a rank-deficient 3x3 visual Jacobian.  Two degrees remains
    # within the MCU per-command guard and gives a measurable radius change.
    parser.add_argument("--probe-deg", type=float, default=2.0)
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
    parser.add_argument(
        "--standard-image-x-sign", type=float, choices=(-1.0, 1.0), default=1.0,
        help="标准姿态下图像向右对应底座切向的符号",
    )
    parser.add_argument(
        "--standard-image-y-sign", type=float, choices=(-1.0, 1.0), default=1.0,
        help="标准姿态下图像向下对应底座径向的符号",
    )
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
