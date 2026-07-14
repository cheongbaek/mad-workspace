#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""데이터 수집용 키보드 조종 — pynput 기반, Windows/Ubuntu 겸용.

/drive_cmd (Twist: linear.x=주행PWM, angular.z=조향각 deg)를 20Hz로 발행한다.
조작은 증분(incremental) 방식 — 키를 누를 때마다 값이 계단식으로 변한다.
(기존 joystick.py의 누름/뗌 방식은 조향 라벨이 0/±max 두 값뿐이라 모방학습
데이터로 부적합 — 증분 방식이 다양한 중간 조향각 라벨을 만들어준다)

  W / S     : PWM +step / -step
  A / D     : 조향각 -step / +step (좌/우)
  C         : 조향 중앙(0)
  Space     : 전체 정지 (PWM=0, 각도=0)
  Q 또는 ESC : 종료 (정지 명령 후)
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

from pynput import keyboard


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")

        self.declare_parameter("pwm_step", 10)
        self.declare_parameter("angle_step", 5)
        self.declare_parameter("max_pwm", 150)     # 수집 주행은 안전하게 저속 상한
        self.declare_parameter("max_angle", 30)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("allow_reverse", True)

        self.pwm_step   = int(self.get_parameter("pwm_step").value)
        self.angle_step = int(self.get_parameter("angle_step").value)
        self.max_pwm    = int(self.get_parameter("max_pwm").value)
        self.max_angle  = int(self.get_parameter("max_angle").value)
        self.allow_reverse = bool(self.get_parameter("allow_reverse").value)
        rate = float(self.get_parameter("publish_rate").value)

        self._lock = threading.Lock()
        self.pwm = 0
        self.angle = 0
        self._quit = False

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_cmd = self.create_publisher(Twist, "drive_cmd", qos)
        self.create_timer(1.0 / rate, self._publish_cb)

        self._listener = keyboard.Listener(on_press=self._on_press)
        self._listener.start()

        self.get_logger().info(
            f"teleop_keyboard 시작 | W/S=PWM±{self.pwm_step} A/D=조향∓{self.angle_step} "
            f"C=중앙 Space=정지 Q=종료 | max_pwm={self.max_pwm}, max_angle={self.max_angle}")

    def _on_press(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            k = key

        with self._lock:
            if k == 'w':
                self.pwm = min(self.max_pwm, self.pwm + self.pwm_step)
            elif k == 's':
                lo = -self.max_pwm if self.allow_reverse else 0
                self.pwm = max(lo, self.pwm - self.pwm_step)
            elif k == 'a':
                self.angle = max(-self.max_angle, self.angle - self.angle_step)
            elif k == 'd':
                self.angle = min(self.max_angle, self.angle + self.angle_step)
            elif k == 'c':
                self.angle = 0
            elif k == keyboard.Key.space:
                self.pwm = 0
                self.angle = 0
            elif k == 'q' or k == keyboard.Key.esc:
                self.pwm = 0
                self.angle = 0
                self._quit = True
                return False   # 리스너 종료

        print(f"\rPWM={self.pwm:4d}  각도={self.angle:4d}   ", end="", flush=True)

    def _publish_cb(self):
        with self._lock:
            pwm, angle, quit_now = self.pwm, self.angle, self._quit
        cmd = Twist()
        cmd.linear.x  = float(pwm)
        cmd.angular.z = float(angle)
        self.pub_cmd.publish(cmd)
        if quit_now:
            raise KeyboardInterrupt   # spin 탈출 → finally에서 정지 발행


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 직전 정지 명령 (benz_driver watchdog도 있지만 이중 안전)
        stop = Twist()
        try:
            node.pub_cmd.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        print("\nteleop 종료 — 정지 명령 전송됨")


if __name__ == "__main__":
    main()
