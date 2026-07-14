#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mega — donkey 패키지에서 아두이노(benz.ino)와 ROS2를 잇는 유일한 시리얼 노드.

  /in  (std_msgs/String) 구독 : 받은 라인을 그대로 아두이노로 전송
                                (발행 주체: 수집 단계=joystick, 실행 단계=drive)
  /out (std_msgs/String) 발행 : 아두이노 텔레메트리 최신 라인을 20Hz로 발행
                                ("s1 s2 s3 s4 s5 s6 현재조향각" 7필드 — /out은 mega만 발행)

benz.ino 프로토콜(115200):
  입력  "a"                                → 조향 캘리브레이션
        "<주행PWM> <조향각>"               → 각도 PD 모드
        "<주행PWM> <조향각> <조향PWM> <ms>" → 시간(오픈루프) 모드
  출력  20ms마다 "초음파6개 조향각" / 캘리브레이션 중 "CAL:..." 메시지

종료 시(Ctrl+C 포함) "0 0"을 보내 차량을 정지시킨다.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import serial

from donkey.common import SERIAL_BAUD, find_serial_port

TELEMETRY_FIELDS = 7   # 초음파 6 + 현재 조향각 1


class Mega(Node):
    def __init__(self):
        super().__init__("mega")

        self.declare_parameter("serial_port", "auto")
        self.declare_parameter("baudrate", SERIAL_BAUD)
        self.declare_parameter("out_rate", 20.0)   # /out 발행 주기(Hz)

        port_param = str(self.get_parameter("serial_port").value)
        baudrate   = int(self.get_parameter("baudrate").value)
        out_rate   = float(self.get_parameter("out_rate").value)

        port = find_serial_port() if port_param == "auto" else port_param
        if port is None:
            raise RuntimeError("아두이노 포트 자동감지 실패 — serial_port:=COM13 처럼 지정하세요")
        self.ser = serial.Serial(port, baudrate, timeout=0.05)
        time.sleep(2.0)   # 아두이노 리셋 대기

        self._rx_buf = b""
        self._latest_telemetry = None

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(String, "in", self._in_cb, qos)
        self.pub_out = self.create_publisher(String, "out", qos)

        self.create_timer(0.005, self._rx_cb)            # 시리얼 수신 폴링
        self.create_timer(1.0 / out_rate, self._out_cb)  # /out 20Hz 발행

        self.get_logger().info(f"mega 시작 | port={port} @ {baudrate}, /out {out_rate:.0f}Hz")

    def _in_cb(self, msg: String):
        line = msg.data.strip()
        if not line:
            return
        try:
            self.ser.write((line + "\n").encode("ascii"))
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
            raw, self._rx_buf = self._rx_buf.split(b"\n", 1)
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            if len(line.split()) == TELEMETRY_FIELDS:
                self._latest_telemetry = line
            else:
                # CAL:START / CAL:DONE 등 비정형 메시지는 로그로만
                self.get_logger().info(f"[아두이노] {line}")

    def _out_cb(self):
        if self._latest_telemetry is None:
            return
        msg = String()
        msg.data = self._latest_telemetry
        self.pub_out.publish(msg)

    def destroy_node(self):
        try:
            self.ser.write(b"0 0\n")
            time.sleep(0.05)
            self.ser.close()
        except (serial.SerialException, OSError):
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Mega()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
