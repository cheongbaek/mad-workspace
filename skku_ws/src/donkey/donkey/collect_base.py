#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""수집 단계 공통 골격 — 노드/토픽 구조.

시리얼·조종은 이 노드가 직접 하지 않는다:
  /in  구독 : joystick이 발행하는 조종 라인에서 주행PWM(첫 필드)만 캐시 — 학습 라벨용
  /out 구독 : mega가 발행하는 텔레메트리에서 현재 조향각(7번째 필드) 캐시 — 학습 라벨용
  카메라   : 이 노드가 직접 캡처 (매 프레임 = CSV 한 행)

학습 라벨 규칙(요구사항):
  주행 = /in의 주행모터 PWM(지시값), 조향 = 아두이노 실측 조향각(가변저항 환산, /out)
  → joystick의 조향 필드(시간모드 조향PWM)는 라벨로 쓰지 않는다.

기록은 처음 주행PWM이 0이 아니게 되는 순간(출발)부터 시작한다.

서브클래스는 다음만 구현:
  SESSION_PREFIX             : "lane" / "all"  (폴더명 lane_001 / all_001)
  extra_columns()            : CSV 추가 열 이름
  process_frame(frame, idx)  : 프레임 처리 후 추가 열 값 리스트 (기록 안 하려면 None)
"""

import csv
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from donkey.common import open_camera, package_root, next_numbered_dir

BASE_COLUMNS = ["idx", "t_sec", "cur_angle", "cmd_pwm"]
TELEMETRY_FIELDS = 7


class CollectBase(Node):
    SESSION_PREFIX = None   # 서브클래스에서 지정

    def __init__(self, node_name: str):
        super().__init__(node_name)

        # 기본: C920E 웹캠을 장치 "이름"으로 찾음 (내장캠/휴대폰 가상카메라 오인 방지).
        # camera_name=""으로 비우고 camera_index를 주면 인덱스로 강제(비상용).
        self.declare_parameter("camera_name", "c920,vu0060")
        self.declare_parameter("camera_index", -1)
        self.declare_parameter("fps", 30.0)

        cam_name  = str(self.get_parameter("camera_name").value)
        cam_index = int(self.get_parameter("camera_index").value)
        fps       = float(self.get_parameter("fps").value)

        device = cam_index if (not cam_name and cam_index >= 0) else None
        self.cap = open_camera(device, fps=fps, name_hints=cam_name)
        if not self.cap.isOpened():
            raise RuntimeError("카메라를 열 수 없음 (장치는 찾았으나 open 실패)")

        # ── 세션 폴더 ──
        self.session_dir = next_numbered_dir(package_root() / "data", self.SESSION_PREFIX)
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.csv_file = open(self.session_dir / "log.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(BASE_COLUMNS + self.extra_columns())
        self.csv_file.flush()

        # ── 상태 (토픽에서 캐시) ──
        self.cmd_pwm = 0        # /in 첫 필드 (주행모터 PWM 지시값)
        self.cur_angle = 0.0    # /out 7번째 필드 (실측 조향각 deg)
        self._started = False
        self._idx = 0
        self._t0 = time.monotonic()

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(String, "in", self._in_cb, qos)
        self.create_subscription(String, "out", self._out_cb, qos)
        self.create_timer(1.0 / fps, self._frame_cb)

        self.get_logger().info(
            f"수집 준비 | 세션={self.session_dir.name}\n"
            f"  joystick으로 출발(주행PWM≠0)하면 기록이 시작됩니다.")

    # ── 서브클래스 구현 대상 ──────────────────────────────
    def extra_columns(self):
        raise NotImplementedError

    def process_frame(self, frame, idx):
        raise NotImplementedError

    # ── 토픽 콜백 ─────────────────────────────────────────
    def _in_cb(self, msg: String):
        # "a"(캘리브레이션) 같은 비숫자 라인은 무시하고, 첫 필드(주행PWM)만 캐시
        parts = msg.data.split()
        if not parts:
            return
        try:
            self.cmd_pwm = int(parts[0])
        except ValueError:
            pass

    def _out_cb(self, msg: String):
        parts = msg.data.split()
        if len(parts) != TELEMETRY_FIELDS:
            return
        try:
            self.cur_angle = float(parts[6])
        except ValueError:
            pass

    # ── 프레임 처리/기록 ──────────────────────────────────
    def _frame_cb(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("프레임 읽기 실패", throttle_duration_sec=2.0)
            return

        if not self._started:
            if self.cmd_pwm == 0:
                return
            self._started = True
            self.get_logger().info("기록 시작!")

        extra = self.process_frame(frame, self._idx)
        if extra is None:
            return

        t = time.monotonic() - self._t0
        self.csv_writer.writerow(
            [self._idx, f"{t:.3f}", f"{self.cur_angle:.1f}", self.cmd_pwm] + list(extra))
        self._idx += 1
        if self._idx % 30 == 0:
            self.csv_file.flush()
        if self._idx % 300 == 0:
            self.get_logger().info(f"{self._idx}행 기록됨")

    # ── 종료 ──────────────────────────────────────────────
    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        if self.csv_file:
            self.csv_file.close()
        self.get_logger().info(f"수집 종료 | {self._idx}행 → {self.session_dir}")
        super().destroy_node()


def run_collector(node_cls):
    rclpy.init()
    node = node_cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
