#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""로지텍 HD 웹캠 드라이버 — cv2.VideoCapture 기반, Windows/Ubuntu 겸용.

/image_raw (sensor_msgs/Image, bgr8) 발행. cv_bridge를 쓰지 않는다.
기본 640x480@30fps — 차선 추출(허프)과 CNN 학습 모두 이 해상도면 충분하고,
로지텍 HD 웹캠이 이 설정에서 20~30fps를 안정적으로 유지한다(실측).
"""

import platform

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

from cnn.ros_image_codec import bgr8_to_imgmsg


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # device_index/device_path 분리 이유: ROS2 CLI(-p device:=0)가 "0"을 INTEGER로
        # 자동 해석해 문자열 파라미터와 충돌(InvalidParameterTypeException, 실측 확인).
        self.declare_parameter("device_index", 0)     # Windows/Ubuntu 공통: 장치 인덱스
        self.declare_parameter("device_path", "")     # Ubuntu 전용 명시 경로(/dev/video2), 지정 시 우선
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)

        device_index = int(self.get_parameter("device_index").value)
        device_path  = str(self.get_parameter("device_path").value)
        width  = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fps    = float(self.get_parameter("fps").value)

        device = device_path if device_path else device_index

        if platform.system() == "Windows":
            self.cap = cv2.VideoCapture(device, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(device)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            self.get_logger().error(f"카메라를 열 수 없음: {device}")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_image = self.create_publisher(Image, "image_raw", qos)
        self.create_timer(1.0 / fps, self._capture_cb)

        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(
            f"camera_node 시작 | device={device}, 요청={width}x{height}@{fps:.0f}fps, "
            f"실제={actual_w:.0f}x{actual_h:.0f} ({platform.system()})")

    def _capture_cb(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("프레임 읽기 실패", throttle_duration_sec=2.0)
            return
        self.pub_image.publish(bgr8_to_imgmsg(frame, frame_id="camera"))

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
