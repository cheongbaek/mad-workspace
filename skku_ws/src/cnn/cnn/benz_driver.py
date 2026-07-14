#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""benz.ino(1/10카, Mega2560) 시리얼 브릿지 — Windows/Ubuntu 겸용.

benz.ino 프로토콜 (115200 baud):
  PC → 차 : "<주행PWM> <조향각>\n"   PWM -255~255, 각도 -30~30 (각도 PD 모드)
  차 → PC : 20ms마다 "s1 s2 s3 s4 s5 s6 각도"  (초음파 6개[cm, 100=범위밖] + 현재 조향각[deg, 소수1자리])

구독:  /drive_cmd (geometry_msgs/Twist)  linear.x=주행PWM, angular.z=조향각[deg]
발행:  /steer_angle (std_msgs/Float32)   현재 조향각 — 멀티모달 모델의 state 입력
       /sonar (std_msgs/Float32MultiArray) 초음파 6채널[cm]

안전장치: /drive_cmd가 cmd_timeout 이상 끊기면 "0 0"(정지) 전송. 종료 시에도 "0 0".
"""

import threading
import time

import serial
import serial.tools.list_ports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray

# Mega2560 계열 VID/PID (kasa_example_control.py와 동일한 자동감지 규칙)
KNOWN_MEGA_VIDPID = {(0x2341, 0x0042), (0x2341, 0x0010), (0x2341, 0x003F), (0x2A03, 0x0042)}
CANDIDATE_VIDS = {0x1A86, 0x0403, 0x10C4}   # CH340/FTDI/CP210x 호환보드


def find_serial_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid is not None and (p.vid, p.pid) in KNOWN_MEGA_VIDPID:
            return p.device
    for p in ports:
        desc = (p.description or "").lower()
        if "mega" in desc or "arduino" in desc:
            return p.device
    candidates = [p for p in ports if p.vid in CANDIDATE_VIDS]
    if len(candidates) == 1:
        return candidates[0].device
    return None


class BenzDriver(Node):
    def __init__(self):
        super().__init__("benz_driver")

        self.declare_parameter("serial_port", "auto")   # "auto" 또는 COM13 / /dev/ttyACM0
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("tx_rate", 30.0)          # 명령 재전송 주기(Hz)
        self.declare_parameter("cmd_timeout", 0.5)       # 이 시간 이상 명령 없으면 정지
        self.declare_parameter("max_pwm", 255)
        self.declare_parameter("max_angle", 30)

        port_param   = str(self.get_parameter("serial_port").value)
        baudrate     = int(self.get_parameter("baudrate").value)
        tx_rate      = float(self.get_parameter("tx_rate").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.max_pwm   = int(self.get_parameter("max_pwm").value)
        self.max_angle = int(self.get_parameter("max_angle").value)

        port = find_serial_port() if port_param == "auto" else port_param
        if port is None:
            raise RuntimeError("아두이노 포트 자동감지 실패 — serial_port 파라미터로 직접 지정하세요")

        self.ser = serial.Serial(port, baudrate, timeout=0.05)
        time.sleep(2.0)   # 아두이노 리셋 대기
        self.get_logger().info(f"benz_driver 시작 | port={port} @ {baudrate}")

        self._lock = threading.Lock()
        self._cmd_pwm = 0
        self._cmd_angle = 0
        self._last_cmd_time = 0.0

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Twist, "drive_cmd", self._cmd_cb, qos)
        self.pub_angle = self.create_publisher(Float32, "steer_angle", qos)
        self.pub_sonar = self.create_publisher(Float32MultiArray, "sonar", qos)

        self.create_timer(1.0 / tx_rate, self._tx_cb)
        self.create_timer(0.01, self._rx_cb)

        self._rx_buf = b""
        self._timeout_logged = False

    def _cmd_cb(self, msg: Twist):
        with self._lock:
            self._cmd_pwm   = int(max(-self.max_pwm, min(self.max_pwm, msg.linear.x)))
            self._cmd_angle = int(max(-self.max_angle, min(self.max_angle, msg.angular.z)))
            self._last_cmd_time = time.monotonic()

    def _tx_cb(self):
        now = time.monotonic()
        with self._lock:
            timed_out = (self._last_cmd_time <= 0.0) or \
                        ((now - self._last_cmd_time) > self.cmd_timeout)
            pwm, angle = (0, 0) if timed_out else (self._cmd_pwm, self._cmd_angle)

        if timed_out and self._last_cmd_time > 0.0 and not self._timeout_logged:
            self.get_logger().warn("drive_cmd 끊김 → 정지(0 0) 전송")
            self._timeout_logged = True
        elif not timed_out:
            self._timeout_logged = False

        try:
            self.ser.write(f"{pwm} {angle}\n".encode("ascii"))
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"시리얼 쓰기 실패: {e}")

    def _rx_cb(self):
        try:
            data = self.ser.read(256)
        except (serial.SerialException, OSError):
            return
        if not data:
            return
        self._rx_buf += data
        while b"\n" in self._rx_buf:
            line, self._rx_buf = self._rx_buf.split(b"\n", 1)
            self._parse_line(line.decode(errors="replace").strip())

    def _parse_line(self, text: str):
        # 정상 텔레메트리: 정수 6개(초음파 cm) + float 1개(현재 조향각 deg)
        # 그 외(CAL:START 등 캘리브레이션 메시지)는 무시
        parts = text.split()
        if len(parts) != 7:
            return
        try:
            sonar = [float(int(p)) for p in parts[:6]]
            angle = float(parts[6])
        except ValueError:
            return

        msg_a = Float32()
        msg_a.data = angle
        self.pub_angle.publish(msg_a)

        msg_s = Float32MultiArray()
        msg_s.data = sonar
        self.pub_sonar.publish(msg_s)

    def send_stop(self):
        try:
            self.ser.write(b"0 0\n")
        except (serial.SerialException, OSError):
            pass

    def destroy_node(self):
        self.send_stop()
        time.sleep(0.05)
        try:
            self.ser.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BenzDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
