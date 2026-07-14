#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Variant A 주행: 카메라 전체 이미지 + 상태 → CNN → /drive_cmd.

입력: /image_raw (이미지), /steer_angle (현재 조향각 피드백)
출력: /drive_cmd (Twist: linear.x=주행PWM, angular.z=조향각 deg)
안전장치: 이미지가 image_timeout 이상 끊기면 정지(0,0) 발행.
"""

import time

import cv2
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from cnn.models import RawPilotNet, make_state, decode_outputs
from cnn.ros_image_codec import imgmsg_to_bgr8


class RawInferNode(Node):
    def __init__(self):
        super().__init__("raw_infer_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("input_width", 160)
        self.declare_parameter("input_height", 120)
        self.declare_parameter("control_rate", 30.0)
        self.declare_parameter("image_timeout", 0.5)
        self.declare_parameter("max_pwm", 150.0)        # 자율주행 시 PWM 상한 (안전)
        self.declare_parameter("allow_reverse", False)

        self.device        = str(self.get_parameter("device").value)
        self.input_width   = int(self.get_parameter("input_width").value)
        self.input_height  = int(self.get_parameter("input_height").value)
        self.image_timeout = float(self.get_parameter("image_timeout").value)
        self.max_pwm       = float(self.get_parameter("max_pwm").value)
        self.allow_reverse = bool(self.get_parameter("allow_reverse").value)
        rate = float(self.get_parameter("control_rate").value)

        model_path = str(self.get_parameter("model_path").value)
        self.model = RawPilotNet(in_h=self.input_height, in_w=self.input_width)
        if model_path:
            self.model.load_state_dict(
                torch.load(model_path, map_location=self.device, weights_only=True))
            self.get_logger().info(f"모델 로드: {model_path}")
        else:
            self.get_logger().warn("model_path 미지정 — 랜덤 가중치(테스트 전용)")
        self.model.to(self.device)
        self.model.eval()

        self._latest_frame = None
        self._latest_frame_time = 0.0
        self._cur_angle = 0.0
        self._prev_cmd_angle = 0.0
        self._prev_cmd_pwm = 0.0

        qos_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(Image, "image_raw", self._image_cb, qos_img)
        self.create_subscription(Float32, "steer_angle", self._angle_cb, qos)
        self.pub_cmd = self.create_publisher(Twist, "drive_cmd", qos)
        self.create_timer(1.0 / rate, self._infer_cb)

        self.get_logger().info(
            f"raw_infer_node 시작 | input={self.input_width}x{self.input_height}, "
            f"max_pwm={self.max_pwm}, rate={rate:.0f}Hz")

    def _image_cb(self, msg: Image):
        try:
            self._latest_frame = imgmsg_to_bgr8(msg)
            self._latest_frame_time = time.monotonic()
        except Exception as e:
            self.get_logger().error(f"이미지 디코딩 오류: {e}")

    def _angle_cb(self, msg: Float32):
        self._cur_angle = float(msg.data)

    def _infer_cb(self):
        cmd = Twist()
        stale = (self._latest_frame is None) or \
            ((time.monotonic() - self._latest_frame_time) > self.image_timeout)
        if stale:
            self.pub_cmd.publish(cmd)   # 0,0 정지
            self._prev_cmd_angle = 0.0
            self._prev_cmd_pwm = 0.0
            return

        frame = cv2.resize(self._latest_frame, (self.input_width, self.input_height))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        img_t = torch.from_numpy(frame.transpose(2, 0, 1)).unsqueeze(0).to(self.device)
        state_t = torch.tensor(
            [make_state(self._cur_angle, self._prev_cmd_angle, self._prev_cmd_pwm)],
            dtype=torch.float32).to(self.device)

        with torch.no_grad():
            out = self.model(img_t, state_t)[0].cpu().numpy()

        angle, pwm = decode_outputs(
            out[0], out[1], allow_reverse=self.allow_reverse, max_pwm=self.max_pwm)
        cmd.angular.z = angle
        cmd.linear.x  = pwm
        self.pub_cmd.publish(cmd)

        self._prev_cmd_angle = angle
        self._prev_cmd_pwm = pwm


def main(args=None):
    rclpy.init(args=args)
    node = RawInferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_cmd.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
