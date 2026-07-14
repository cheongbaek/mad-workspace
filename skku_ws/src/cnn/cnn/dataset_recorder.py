#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""모방학습 데이터 수집 — 카메라 프레임 + 조종 명령 + 조향각 피드백을 짝지어 저장.

teleop_keyboard(사람 조종)로 트랙을 달리는 동안 이 노드를 함께 띄우면
/image_raw 프레임이 도착할 때마다 그 순간의 명령/상태를 한 행으로 기록한다.

저장 구조:
  <log_dir>/session_YYYYmmdd_HHMMSS/
    images/000000.jpg ...
    labels.csv:
      frame_idx, t_sec,
      cur_angle,             # 그 순간 실제 조향각 (benz 시리얼 피드백) — state 입력
      prev_angle, prev_pwm,  # 직전 행에 기록된 명령 — state 입력
      angle, pwm,            # 그 순간 조종 명령 — 학습 라벨 (모방 대상)
      sonar1..sonar6         # 초음파 (지금은 미사용, 나중을 위해 저장)

녹화 제어: auto_start=True면 PWM이 0이 아니게 되는 순간 자동 시작.
  수동: ros2 topic pub --once /dataset_recorder/start std_msgs/msg/Bool "data: true"
        ros2 topic pub --once /dataset_recorder/stop  std_msgs/msg/Bool "data: true"
"""

import csv
import time
from pathlib import Path
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray, Bool

from cnn.ros_image_codec import imgmsg_to_bgr8


class DatasetRecorder(Node):
    def __init__(self):
        super().__init__("dataset_recorder")

        self.declare_parameter("log_dir", "~/imitation_data")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("jpeg_quality", 95)

        self.log_dir      = str(self.get_parameter("log_dir").value)
        self.auto_start   = bool(self.get_parameter("auto_start").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        # 최신값 캐시 — 이미지 콜백 시점의 값을 그대로 짝짓는다
        self.cur_angle = 0.0
        self.cmd_angle = 0.0
        self.cmd_pwm   = 0.0
        self.sonar = [100.0] * 6

        self._prev_angle = 0.0
        self._prev_pwm   = 0.0

        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        self.images_dir = None
        self.frame_idx = 0
        self.start_time = 0.0

        qos_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(Image, "image_raw", self._image_cb, qos_img)
        self.create_subscription(Twist, "drive_cmd", self._cmd_cb, qos)
        self.create_subscription(Float32, "steer_angle", self._angle_cb, qos)
        self.create_subscription(Float32MultiArray, "sonar", self._sonar_cb, qos)
        self.create_subscription(Bool, "dataset_recorder/start", self._start_cb, qos)
        self.create_subscription(Bool, "dataset_recorder/stop", self._stop_cb, qos)

        self.get_logger().info(
            f"dataset_recorder 준비 | log_dir={self.log_dir}, auto_start={self.auto_start}")

    # ── 콜백 ──────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        self.cmd_pwm   = float(msg.linear.x)
        self.cmd_angle = float(msg.angular.z)
        if self.auto_start and not self.is_recording and abs(self.cmd_pwm) > 0.5:
            self._start_recording()

    def _angle_cb(self, msg: Float32):
        self.cur_angle = float(msg.data)

    def _sonar_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 6:
            self.sonar = list(msg.data[:6])

    def _start_cb(self, msg: Bool):
        if msg.data and not self.is_recording:
            self._start_recording()

    def _stop_cb(self, msg: Bool):
        if msg.data and self.is_recording:
            self._stop_recording()

    def _image_cb(self, msg: Image):
        if not self.is_recording:
            return
        try:
            frame = imgmsg_to_bgr8(msg)
        except Exception as e:
            self.get_logger().error(f"이미지 디코딩 오류: {e}")
            return

        img_path = self.images_dir / f"{self.frame_idx:06d}.jpg"
        cv2.imwrite(str(img_path), frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])

        elapsed = time.monotonic() - self.start_time
        self.csv_writer.writerow([
            self.frame_idx, f"{elapsed:.3f}",
            f"{self.cur_angle:.1f}",
            f"{self._prev_angle:.1f}", f"{self._prev_pwm:.1f}",
            f"{self.cmd_angle:.1f}", f"{self.cmd_pwm:.1f}",
            *[f"{s:.0f}" for s in self.sonar],
        ])

        self._prev_angle = self.cmd_angle
        self._prev_pwm   = self.cmd_pwm
        self.frame_idx += 1
        if self.frame_idx % 100 == 0:
            self.csv_file.flush()
            self.get_logger().info(f"{self.frame_idx}프레임 기록됨")

    # ── 녹화 제어 ──────────────────────────────────────────

    def _start_recording(self):
        session_dir = Path(self.log_dir).expanduser() / \
            f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.images_dir = session_dir / "images"
        self.images_dir.mkdir(parents=True, exist_ok=True)

        self.csv_file = open(session_dir / "labels.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "frame_idx", "t_sec",
            "cur_angle", "prev_angle", "prev_pwm",
            "angle", "pwm",
            "sonar1", "sonar2", "sonar3", "sonar4", "sonar5", "sonar6",
        ])

        self.frame_idx = 0
        self._prev_angle = 0.0
        self._prev_pwm   = 0.0
        self.start_time = time.monotonic()
        self.is_recording = True
        self.get_logger().info(f"녹화 시작 → {session_dir}")

    def _stop_recording(self):
        if not self.is_recording:
            return
        self.is_recording = False
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
        self.get_logger().info(f"녹화 종료 | {self.frame_idx}프레임")


def main(args=None):
    rclpy.init(args=args)
    node = DatasetRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_recording()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
