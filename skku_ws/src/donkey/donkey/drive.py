#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""실행 단계 — 학습 결과물(trained/train_XXX)을 불러와 즉시 자율주행.

시리얼은 mega 노드가 담당하고, 이 노드는 /in 토픽으로 "<주행PWM> <조향각>"
(각도 PD 모드 2필드)을 20Hz로 발행한다. 실행 모드에서 /in의 발행 주체는 이 노드.

meta.json의 type("lane"/"all")을 보고 모델·전처리를 자동 선택:
  lane: 프레임 → 허프 차선특징(수집 때와 동일 모듈) → LaneNet
  all : 프레임 → 리사이즈/정규화 → AllNet

실행하면 그 즉시 출발한다 (출발 위치는 학습한 위치와 동일하게 둘 것). 안전장치:
  - 카메라 프레임이 image_timeout(0.5s) 이상 끊기면 "0 0" 발행
  - PWM은 max_pwm 상한, 후진 출력은 0으로 clamp
  - Ctrl+C 종료 시 "0 0" 발행 (mega도 종료 시 자체적으로 "0 0" 전송)
"""

import json
import time

import cv2
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from donkey.common import package_root, latest_numbered_dir, denorm, open_camera
from donkey.models import build_model, IMG_W, IMG_H
from donkey.lane_detect import extract_features


class Drive(Node):
    def __init__(self):
        super().__init__("drive")

        self.declare_parameter("train", "latest")    # train_001 또는 latest
        # 기본: C920E 웹캠을 장치 "이름"으로 찾음 (내장캠/휴대폰 가상카메라 오인 방지).
        self.declare_parameter("camera_name", "c920,vu0060")
        self.declare_parameter("camera_index", -1)   # 비상용 (camera_name="" 일 때만)
        self.declare_parameter("rate", 20.0)         # /in 발행 주기
        self.declare_parameter("max_pwm", 150.0)
        self.declare_parameter("image_timeout", 0.5)
        self.declare_parameter("device", "cpu")

        train_param = str(self.get_parameter("train").value)
        cam_name    = str(self.get_parameter("camera_name").value)
        cam_index   = int(self.get_parameter("camera_index").value)
        rate        = float(self.get_parameter("rate").value)
        self.max_pwm       = float(self.get_parameter("max_pwm").value)
        self.image_timeout = float(self.get_parameter("image_timeout").value)
        self.device        = str(self.get_parameter("device").value)

        # ── 학습 결과물 로드 ──
        trained_base = package_root() / "trained"
        if train_param in ("", "latest"):
            train_dir = latest_numbered_dir(trained_base, "train")
            if train_dir is None:
                raise RuntimeError(f"학습 결과물이 없음: {trained_base} — 먼저 train.py를 실행하세요")
        else:
            train_dir = trained_base / train_param
        with open(train_dir / "meta.json", encoding="utf-8") as f:
            self.meta = json.load(f)
        self.data_type = self.meta["type"]

        self.model = build_model(self.data_type)
        self.model.load_state_dict(
            torch.load(train_dir / "model.pt", map_location=self.device, weights_only=True))
        self.model.to(self.device)
        self.model.eval()

        # ── 카메라 (이름 매칭 실패 시 예외 — 다른 카메라로 폴백하지 않음) ──
        device = cam_index if (not cam_name and cam_index >= 0) else None
        self.cap = open_camera(device, name_hints=cam_name)
        if not self.cap.isOpened():
            raise RuntimeError("카메라를 열 수 없음 (장치는 찾았으나 open 실패)")

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_in = self.create_publisher(String, "in", qos)

        self._last_frame_time = time.monotonic()
        self.create_timer(1.0 / rate, self._drive_cb)

        self.get_logger().info(
            f"주행 시작! | {train_dir.name} (type={self.data_type}, "
            f"val_loss={self.meta.get('best_val_loss')}) max_pwm={self.max_pwm}")

    def _drive_cb(self):
        ok, frame = self.cap.read()
        now = time.monotonic()
        if ok:
            self._last_frame_time = now
        elif (now - self._last_frame_time) > self.image_timeout:
            self._send(0, 0)   # 카메라 끊김 → 정지
            self.get_logger().warn("카메라 프레임 끊김 → 정지", throttle_duration_sec=2.0)
            return
        else:
            return

        if self.data_type == "lane":
            feat = extract_features(frame)
            x = torch.from_numpy(feat).unsqueeze(0).to(self.device)
        else:
            img = cv2.resize(frame, (IMG_W, IMG_H))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            x = torch.from_numpy(img.transpose(2, 0, 1)).unsqueeze(0).to(self.device)

        with torch.no_grad():
            out = self.model(x)[0].cpu().numpy()

        angle, pwm = denorm(out[0], out[1], max_pwm=self.max_pwm, allow_reverse=False)
        self._send(int(round(pwm)), int(round(angle)))

    def _send(self, pwm: int, angle: int):
        msg = String()
        msg.data = f"{pwm} {angle}"     # 각도 PD 모드 2필드
        self.pub_in.publish(msg)

    def destroy_node(self):
        try:
            self._send(0, 0)
        except Exception:
            pass
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Drive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
