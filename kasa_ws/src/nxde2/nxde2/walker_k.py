# walker_k : /in 구독 -> 아두이노(A/B 2보드) 시리얼 입력 분배 / 아두이노 시리얼 출력 -> /out 발행
#
# - 아두이노 A보드(주행펄스+PID)/B보드(조향+브레이크) 2대를 시리얼 포트 자동 스캔으로 찾고,
#   각 보드가 보내는 첫 텔레메트리 줄의 접두어("S," = A보드, "P," = B보드)로 식별한다.
#   (kasa_0713_protocol.md 참고. COM/tty 포트 번호를 더 이상 수동 지정하지 않음)
# - /in (std_msgs/String) 메시지 "<주행펄스> <조향각도> <브레이크> [모드]" (공백 구분,
#   기존 talker_k 발행 형식 그대로 유지)가 들어오면:
#     A보드로는 "<주행펄스>" 만, B보드로는 "<조향각도>,<브레이크>" 를 각 보드의 새
#     프로토콜 형식으로 변환해 즉시 전송한다. (모드 필드는 아두이노에서 폐기되어 무시)
# - /in이 1초 이상 끊기면 마지막 값을 1초 주기로 두 보드 모두에 재전송
#   (B보드 리니어 브레이크 2초 열린루프 재트리거 유지 목적. 아두이노 e-stop은 이제
#   13번 핀 외부개입만 조건이라 무입력 자체는 더 이상 e-stop을 유발하지 않음)
# - A보드("S,<21번펄스>,<20번펄스>")/B보드("P,<조향각>") 텔레메트리와 마지막으로
#   전송한 브레이크 명령값(브레이크는 B보드에 센서 피드백이 없어 명령값을 그대로 기록)을
#   합쳐 "<21번펄스>,<20번펄스>,<조향각>,<브레이크>" 형태로 /out (std_msgs/String) 에
#   0.1초 주기로 발행한다 (더 이상 아두이노 원문 줄을 그대로 전달하지 않음).
#   STOP 수신 중에는 해당 보드 값이 갱신되지 않고 마지막 정상값이 유지된다.
# - A·B 보드 중 한쪽이라도 "STOP"을 보내면 E-stop 발동으로 간주해 로그만 표출(OR 처리)
# - 실행 파라미터는 baud만 필요 (포트는 자동 감지):
#     ros2 run nxde2 walker_k --ros-args -p baud:=115200
# - 정상 동작 중에는 로그 출력 없이 조용히 중계만 함 (오류/보드감지/estop 전환 시만 로그)

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
from serial.tools import list_ports

BAUD_RATE = 115200

SERIAL_POLL_S  = 0.1   # 시리얼 수신 폴링 및 /out 발행 주기
RESEND_S       = 1.0   # /in 미수신 시 마지막 값 재전송 주기
DETECT_READ_S  = 5.0   # 포트 하나를 A/B로 식별하기 위해 읽어보는 시간
DETECT_RETRY_S = 2.0   # 두 보드를 아직 못 찾았을 때 재스캔 간격


def candidate_ports():
    """OS별로 아두이노로 추정되는 시리얼 포트 후보 목록을 반환"""
    devices = [p.device for p in list_ports.comports()]
    if sys.platform == 'win32':
        return [d for d in devices if d.upper().startswith('COM')]
    return [d for d in devices if ('ACM' in d) or ('USB' in d)]


def identify_port(port, baud, logger):
    """포트를 열어 DETECT_READ_S 동안 읽으며 첫 'S,'/'P,' 줄로 보드를 식별.
       반환: ('A'|'B'|None, serial.Serial 또는 None(실패 시))"""
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except serial.SerialException as e:
        logger.warn(f"{port} 열기 실패: {e}")
        return None, None

    buf = b''
    deadline = time.monotonic() + DETECT_READ_S
    while time.monotonic() < deadline:
        try:
            data = ser.read(256)
        except serial.SerialException:
            ser.close()
            return None, None
        if data:
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                text = line.decode('ascii', errors='ignore').strip()
                if text.startswith('S,'):
                    return 'A', ser
                if text.startswith('P,'):
                    return 'B', ser

    ser.close()
    return None, None


class WalkerK(Node):
    def __init__(self):
        super().__init__('walker_k')
        self.baud = int(self.declare_parameter('baud', BAUD_RATE).value)

        self.ser_a = None
        self.ser_b = None
        self.rx_buf_a = b''
        self.rx_buf_b = b''
        self.last_line_a = None
        self.last_line_b = None

        # /out으로 합쳐 발행할 최신 상태값 (STOP/형식오류 시 마지막 값 유지)
        self.pulse21 = 0
        self.pulse20 = 0
        self.angle = 0
        self.brake = 0   # 센서 피드백 없음: 마지막으로 보낸 명령값을 그대로 기록

        self.last_in = None      # 마지막으로 받은 /in 원문 (재전송용)
        self.last_sent = 0.0
        self.estop_active = False   # A/B 중 한쪽이라도 STOP인 상태

        self.sub = self.create_subscription(String, '/in', self.on_in, 10)
        self.pub = self.create_publisher(String, '/out', 10)

        self.detect_boards()

        self.timer = self.create_timer(SERIAL_POLL_S, self.on_timer)

    # ---------- 보드 자동 감지 ----------
    def detect_boards(self):
        self.get_logger().info("아두이노 A/B 보드 자동 감지 시작...")
        while rclpy.ok() and (self.ser_a is None or self.ser_b is None):
            found_a = self.ser_a.port if self.ser_a else None
            found_b = self.ser_b.port if self.ser_b else None

            for port in candidate_ports():
                if self.ser_a is not None and self.ser_b is not None:
                    break
                if port in (found_a, found_b):
                    continue

                role, ser = identify_port(port, self.baud, self.get_logger())
                if role == 'A' and self.ser_a is None:
                    ser.timeout = 0   # 이후 폴링은 논블로킹
                    self.ser_a = ser
                    found_a = port
                    self.get_logger().info(f"[A보드 감지] {port}")
                elif role == 'B' and self.ser_b is None:
                    ser.timeout = 0
                    self.ser_b = ser
                    found_b = port
                    self.get_logger().info(f"[B보드 감지] {port}")
                elif ser is not None:
                    ser.close()

            if self.ser_a is None or self.ser_b is None:
                missing = [n for n, s in (('A', self.ser_a), ('B', self.ser_b)) if s is None]
                self.get_logger().warn(f"{'/'.join(missing)}보드 미발견, {DETECT_RETRY_S}s 후 재스캔")
                time.sleep(DETECT_RETRY_S)

        self.get_logger().info("A/B 보드 모두 감지 완료, 정상 동작 시작")

    # ---------- /in -> A/B 보드 분배 ----------
    def send_line(self, ser, text):
        if ser is None:
            return
        try:
            ser.write((text + '\n').encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f"시리얼 전송 실패({ser.port}): {e}")

    def route_in(self, text):
        """/in "<주행펄스> <조향각도> <브레이크> [모드]" 를 A/B 보드 프로토콜로 변환해 전송"""
        parts = text.split()
        if len(parts) < 3:
            self.get_logger().error(f"/in 형식 오류(최소 3필드 필요): \"{text}\"")
            return
        pulse, angle_cmd, brake_cmd = parts[0], parts[1], parts[2]
        self.send_line(self.ser_a, pulse)
        self.send_line(self.ser_b, f"{angle_cmd},{brake_cmd}")
        try:
            self.brake = int(brake_cmd)
        except ValueError:
            pass
        self.last_sent = time.monotonic()

    def on_in(self, msg):
        self.last_in = msg.data.strip()
        self.route_in(self.last_in)

    def on_timer(self):
        self.poll_port('a')
        self.poll_port('b')
        self.update_estop_log()
        self.publish_state()
        if self.last_in is not None and time.monotonic() - self.last_sent >= RESEND_S:
            self.route_in(self.last_in)

    # ---------- A/B 보드 텔레메트리 파싱 ----------
    def poll_port(self, which):
        ser = self.ser_a if which == 'a' else self.ser_b
        if ser is None:
            return
        buf_attr = 'rx_buf_a' if which == 'a' else 'rx_buf_b'
        buf = getattr(self, buf_attr)

        try:
            data = ser.read(4096)
        except serial.SerialException as e:
            self.get_logger().error(f"시리얼 수신 실패({ser.port}): {e}")
            return
        if data:
            buf += data
        if b'\n' not in buf:
            setattr(self, buf_attr, buf)
            return

        lines = buf.split(b'\n')
        setattr(self, buf_attr, lines[-1])          # 미완성 줄은 버퍼에 보존
        texts = [t for t in
                 (line.decode('ascii', errors='ignore').strip() for line in lines[:-1])
                 if t]
        if not texts:
            return

        latest = texts[-1]                          # 가장 최신 완성 줄 하나만 사용
        if which == 'a':
            self.last_line_a = latest
            self.parse_a(latest)
        else:
            self.last_line_b = latest
            self.parse_b(latest)

    def parse_a(self, text):
        """"S,<21번펄스>,<20번펄스>" 파싱. STOP/형식오류 시 마지막 값 유지."""
        if not text.startswith('S,'):
            return
        fields = text.split(',')
        if len(fields) != 3:
            return
        try:
            self.pulse21 = int(fields[1])
            self.pulse20 = int(fields[2])
        except ValueError:
            pass

    def parse_b(self, text):
        """"P,<조향각>" 파싱. STOP/형식오류 시 마지막 값 유지."""
        if not text.startswith('P,'):
            return
        fields = text.split(',')
        if len(fields) != 2:
            return
        try:
            self.angle = int(fields[1])
        except ValueError:
            pass

    # ---------- 합산 상태 -> /out ----------
    def publish_state(self):
        msg = String()
        msg.data = f"{self.pulse21},{self.pulse20},{self.angle},{self.brake}"
        self.pub.publish(msg)

    def update_estop_log(self):
        """A/B 중 한쪽이라도 최신 줄이 STOP이면 e-stop으로 간주, 전환 시점만 로그"""
        active = (self.last_line_a == 'STOP') or (self.last_line_b == 'STOP')
        if active and not self.estop_active:
            self.estop_active = True
            self.get_logger().warn("[E-STOP 발동] A/B 보드 중 하나 이상이 STOP 신호를 보냄 (모든 모터 정지)")
        elif not active and self.estop_active:
            self.estop_active = False
            self.get_logger().info("[E-STOP 해제] 정상 텔레메트리 재개")

    def destroy_node(self):
        for ser in (self.ser_a, self.ser_b):
            try:
                if ser is not None and ser.is_open:
                    ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = WalkerK()
    except KeyboardInterrupt:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
