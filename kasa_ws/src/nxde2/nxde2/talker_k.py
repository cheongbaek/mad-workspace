# talker_k (nxde2) : 키보드 조작 -> /in 발행 + turtlesim 연동
#
# 조향 키 (이 콘솔 창에 포커스 필요, 영문 입력 모드에서 사용):
#    A    S    D    F   G   H   J    K    L    ;
#  -36  -27  -18   -9   0   0   9   18   27   36   (도)
#   음수 = 시계반대방향, 양수 = 시계방향. 거북이도 같은 방향으로 회전.
#
# 속도(주행펄스) 키 : ↑ / ↓ 방향키
#   - 한 번 누르면 ±2
#   - Windows: 0.8초 이상 누르고 있으면 ±2, 이후 0.2초마다 ±2 (자체 홀드 반복)
#   - Linux  : 키를 누르고 있으면 OS 자동반복 이벤트마다 ±2
#   - 범위 0 ~ 30 (음수 없음: 0에서 ↓를 눌러도 0 유지)
#
# 브레이크 키 : ← / → 방향키
#   - → 한 번 누르면 브레이크 PWM +50, ← 한 번 누르면 -50
#   - 홀드 반복 없음 (누르고 있어도 한 번만 반영, OS 자동반복 무시)
#     * Windows: GetAsyncKeyState로 키 뗌 감지
#     * Linux  : 직전 같은 키 이벤트와 0.25초 이내면 자동반복으로 간주해 무시
#   - 범위 -255 ~ 255 (아두이노 해석: 부호=리니어 방향, 절댓값=세기)
#
# /in 발행 (std_msgs/String): "<주행펄스> <조향각도> <브레이크>"
#   - 아두이노 입력 형식: 정수 3개 (kasa_0709_none.ino는 4번째 모드값 생략 가능)
#   - 값이 바뀌는 순간 즉시 발행, 이후 입력이 없으면 0.5초마다 동일 메시지 재발행
#     (아두이노 5초 무입력 E-stop 방지 + 브레이크 열린루프 2초 구동 재trigger keepalive)
#
# turtlesim 연동:
#   - 시작 시 거북이를 화면 위쪽을 바라보도록 정렬 (조향 0 = 위쪽 기준)
#   - 조향각 변경 시 /turtle1/teleport_relative 로 거북이를 각도 차이만큼 회전
#   - 주행펄스에 비례한 전진 속도를 /turtle1/cmd_vel 로 계속 발행
#   - 벽에 충돌하면 반대편 벽의 같은 지점으로 이동해 계속 주행 (wrap-around,
#     예: 오른쪽 벽 1/3 지점 충돌 -> 왼쪽 벽 1/3 지점에서 재등장)
#   - turtlesim이 꺼져 있어도 /in 발행은 정상 동작

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, TeleportRelative

IS_WINDOWS = (sys.platform == 'win32')

if IS_WINDOWS:
    import ctypes
    import msvcrt
    _user32 = ctypes.windll.user32
else:
    import select
    import termios
    import tty

# ===== 조작 파라미터 (여기서 조절) =====
PULSE_STEP    = 2      # 방향키 1회당 펄스 증감량
PULSE_MAX     = 30     # 펄스 상한
PULSE_MIN     = 0      # 펄스 하한 (음수 금지)
HOLD_DELAY_S  = 0.8    # 이 시간 이상 누르고 있으면 반복 증감 시작
HOLD_REPEAT_S = 0.2    # 반복 증감 주기

BRAKE_STEP    = 50     # ←/→ 1회당 브레이크 PWM 증감량
BRAKE_MAX     = 250    # 브레이크 PWM 상한 (아두이노 BRAKE_MAX와 동일)
BRAKE_MIN     = -250   # 브레이크 PWM 하한 (음수 = 리니어 반대방향)

BRAKE_REPEAT_GATE_S = 0.25   # (Linux) 같은 브레이크 키 이벤트가 이 간격 안에 오면 OS 자동반복으로 간주

KEEPALIVE_S   = 0.5    # 키입력 없을 때 /in 재발행 주기
POLL_S        = 0.02   # 키보드 폴링 주기

PULSE_TO_LINEAR = 0.1  # 거북이 전진 속도 = 펄스 * 이 값
CMD_VEL_S       = 0.1  # /turtle1/cmd_vel 발행 주기

# ===== 키 매핑 =====
# 조향 키: msvcrt 콘솔 이벤트 바이트 -> 조향각 (G, H는 둘 다 0)
STEER_TABLE = {
    b'a': -36, b's': -27, b'd': -18, b'f': -9, b'g': 0,
    b'h': 0, b'j': 9, b'k': 18, b'l': 27, b';': 36,
}

# 방향키 식별자 (플랫폼 공통) 및 Windows 가상 키코드 (GetAsyncKeyState 용)
KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT = 'up', 'down', 'left', 'right'
VK_CODE = {KEY_LEFT: 0x25, KEY_UP: 0x26, KEY_RIGHT: 0x27, KEY_DOWN: 0x28}

TURTLE_BASE_THETA = math.pi / 2.0   # 조향 0도일 때 거북이 방향 (위쪽)

# turtlesim 월드 좌표 범위 및 벽 wrap-around 판정
WORLD_MIN       = 0.0
WORLD_MAX       = 11.088889   # turtlesim 창 크기 (기본 500px / 45px per unit)
WALL_EPS        = 0.05        # 이 거리 안이면 벽에 닿은 것으로 판정
WRAP_COOLDOWN_S = 0.3         # 순간이동 직후 재판정 금지 시간 (비동기 teleport 중복 방지)

def _key_down(key):
    """해당 방향키가 지금 물리적으로 눌려 있는지 (Windows 전용: 키 릴리즈 감지용)"""
    return bool(_user32.GetAsyncKeyState(VK_CODE[key]) & 0x8000)


class TalkerK(Node):
    def __init__(self):
        super().__init__('talker_k')
        self.pub_in = self.create_publisher(String, '/in', 10)
        self.pub_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.teleport = self.create_client(TeleportRelative, '/turtle1/teleport_relative')
        self.teleport_abs = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        if not self.teleport.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                'turtlesim teleport 서비스 없음: 거북이 회전은 생략, /in 발행은 계속합니다.')

        # pose 상시 구독: 첫 수신 시 위쪽 정렬, 이후 벽 충돌 wrap-around 감시
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.aligned = False
        self.last_wrap = 0.0

        self.pulse = 0
        self.steer = 0
        self.brake = 0
        # (Windows) 방향키 홀드 세션: None 또는 {'start': 누른 시각, 'next': 다음 증감 시각}
        self.hold = {KEY_UP: None, KEY_DOWN: None}
        # (Windows) 브레이크 키(←/→) 눌림 상태: 단발 처리용 (OS 자동반복 무시)
        self.brake_held = {KEY_LEFT: False, KEY_RIGHT: False}
        # (Linux) 브레이크 키별 마지막 이벤트 시각: OS 자동반복 필터용
        self.brake_last_evt = {KEY_LEFT: 0.0, KEY_RIGHT: 0.0}
        self.last_in_pub = 0.0

        self.get_logger().info(
            '조향: A S D F G H J K L ; -> -36 -27 -18 -9 0 0 9 18 27 36 (도) / '
            f'속도: ↑↓ ±{PULSE_STEP} (0~{PULSE_MAX}) / '
            f'브레이크: ←→ ±{BRAKE_STEP} ({BRAKE_MIN}~{BRAKE_MAX}, 단발) / '
            '이 창에 포커스를 두고 조작 (영문 모드)')

        self.publish_in()   # 시작 즉시 "0 0 0" 전송 (아두이노 keepalive 시작)
        self.create_timer(POLL_S, self.poll_keys)
        self.create_timer(CMD_VEL_S, self.publish_cmd_vel)

    # ---------- /in 발행 ----------
    def publish_in(self):
        msg = String()
        msg.data = f"{self.pulse} {self.steer} {self.brake}"
        self.pub_in.publish(msg)
        self.last_in_pub = time.monotonic()

    # ---------- turtlesim ----------
    def on_pose(self, pose):
        if not self.aligned:
            # 첫 pose 수신: 거북이를 조향 0 = 위쪽 기준으로 정렬 (한 번만)
            self.aligned = True
            if self.teleport_abs.service_is_ready():
                req = TeleportAbsolute.Request()
                req.x = pose.x
                req.y = pose.y
                req.theta = TURTLE_BASE_THETA - math.radians(self.steer)
                self.teleport_abs.call_async(req)
            return
        self.wrap_if_hit_wall(pose)

    def wrap_if_hit_wall(self, pose):
        """벽에 닿은 채 벽 방향으로 진행 중이면 반대편 벽 같은 지점으로 이동 (방향 유지)"""
        if self.pulse <= 0:
            return   # 정지 상태로 벽에 붙어 있을 때 반복 이동 방지
        now = time.monotonic()
        if now - self.last_wrap < WRAP_COOLDOWN_S:
            return

        hx = math.cos(pose.theta)   # 진행 방향 성분
        hy = math.sin(pose.theta)
        nx, ny = pose.x, pose.y
        if pose.x >= WORLD_MAX - WALL_EPS and hx > 0:     # 오른쪽 벽 -> 왼쪽 벽
            nx = WORLD_MIN + WALL_EPS
        elif pose.x <= WORLD_MIN + WALL_EPS and hx < 0:   # 왼쪽 벽 -> 오른쪽 벽
            nx = WORLD_MAX - WALL_EPS
        if pose.y >= WORLD_MAX - WALL_EPS and hy > 0:     # 위쪽 벽 -> 아래쪽 벽
            ny = WORLD_MIN + WALL_EPS
        elif pose.y <= WORLD_MIN + WALL_EPS and hy < 0:   # 아래쪽 벽 -> 위쪽 벽
            ny = WORLD_MAX - WALL_EPS
        if nx == pose.x and ny == pose.y:
            return
        if not self.teleport_abs.service_is_ready():
            return

        req = TeleportAbsolute.Request()
        req.x = nx
        req.y = ny
        req.theta = pose.theta   # 진행 방향 그대로 유지
        self.teleport_abs.call_async(req)
        self.last_wrap = now
        self.get_logger().info(
            f"벽 충돌: ({pose.x:.2f}, {pose.y:.2f}) -> ({nx:.2f}, {ny:.2f}) 이동")

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = float(self.pulse) * PULSE_TO_LINEAR
        self.pub_vel.publish(msg)

    def rotate_turtle(self, delta_deg):
        """거북이를 조향각 변화량만큼 회전 (turtlesim theta 양수=반시계이므로 부호 반전)"""
        if not self.teleport.service_is_ready():
            return
        req = TeleportRelative.Request()
        req.linear = 0.0
        req.angular = -math.radians(delta_deg)
        self.teleport.call_async(req)

    # ---------- 키 입력 ----------
    def bump_pulse(self, delta):
        new = max(PULSE_MIN, min(PULSE_MAX, self.pulse + delta))
        if new == self.pulse:
            return False
        self.pulse = new
        return True

    def bump_brake(self, delta):
        new = max(BRAKE_MIN, min(BRAKE_MAX, self.brake + delta))
        if new == self.brake:
            return False
        self.brake = new
        return True

    def on_brake_event(self, key, delta, now):
        """←/→ 콘솔 이벤트 처리. 홀드 반복 없이 새로 누른 경우에만 1회 증감."""
        if IS_WINDOWS:
            if self.brake_held[key] and _key_down(key):
                return False   # 누르고 있는 동안 오는 OS 자동반복 -> 무시
            self.brake_held[key] = True
            return self.bump_brake(delta)
        # Linux: 키 뗌 감지가 불가능하므로 이벤트 간격으로 자동반복 판별
        last = self.brake_last_evt[key]
        self.brake_last_evt[key] = now
        if now - last < BRAKE_REPEAT_GATE_S:
            return False   # 자동반복으로 간주 -> 무시
        return self.bump_brake(delta)

    def on_arrow_event(self, key, delta, now):
        """↑/↓ 콘솔 이벤트 처리."""
        if IS_WINDOWS:
            # OS 자동반복 이벤트는 무시하고 홀드 반복은 poll_keys에서
            # GetAsyncKeyState 기준으로 처리
            if self.hold[key] is not None and _key_down(key):
                return False
            self.hold[key] = {'start': now, 'next': now + HOLD_DELAY_S}
            return self.bump_pulse(delta)
        # Linux: OS 자동반복 이벤트를 그대로 홀드 반복으로 사용
        return self.bump_pulse(delta)

    def _drain_key_events(self):
        """플랫폼별 콘솔 키 이벤트를 통일된 형태의 리스트로 반환.
           항목: KEY_UP/KEY_DOWN/KEY_LEFT/KEY_RIGHT/'ctrl_c' 또는 일반 문자(bytes)"""
        if IS_WINDOWS:
            return self._drain_key_events_win()
        return self._drain_key_events_posix()

    @staticmethod
    def _drain_key_events_win():
        events = []
        arrow = {b'H': KEY_UP, b'P': KEY_DOWN, b'M': KEY_RIGHT, b'K': KEY_LEFT}
        while msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch in (b'\x00', b'\xe0'):        # 확장키 (방향키 등): 2바이트째가 코드
                if not msvcrt.kbhit():
                    break
                code = msvcrt.getch()
                if code in arrow:
                    events.append(arrow[code])
            elif ch == b'\x03':                 # Ctrl+C
                events.append('ctrl_c')
            else:
                events.append(ch)
        return events

    @staticmethod
    def _drain_key_events_posix():
        events = []
        fd = sys.stdin.fileno()
        arrow = {b'A': KEY_UP, b'B': KEY_DOWN, b'C': KEY_RIGHT, b'D': KEY_LEFT}
        while select.select([sys.stdin], [], [], 0)[0]:
            ch = os.read(fd, 1)
            if ch == b'\x1b':                   # ESC: 방향키 시퀀스(ESC [ A~D) 시도
                seq = b''
                while len(seq) < 2 and select.select([sys.stdin], [], [], 0.01)[0]:
                    seq += os.read(fd, 1)
                if seq[:1] == b'[' and seq[1:2] in arrow:
                    events.append(arrow[seq[1:2]])
                # 그 외 ESC 시퀀스는 무시
            elif ch == b'\x03':                 # Ctrl+C (cbreak 실패로 raw가 된 경우 대비)
                events.append('ctrl_c')
            else:
                events.append(ch)
        return events

    def poll_keys(self):
        now = time.monotonic()
        changed = False

        # 1) 콘솔 키 이벤트 드레인 (창 포커스가 있어야 들어옴)
        for ev in self._drain_key_events():
            if ev == KEY_UP:
                changed |= self.on_arrow_event(KEY_UP, +PULSE_STEP, now)
            elif ev == KEY_DOWN:
                changed |= self.on_arrow_event(KEY_DOWN, -PULSE_STEP, now)
            elif ev == KEY_RIGHT:
                changed |= self.on_brake_event(KEY_RIGHT, +BRAKE_STEP, now)
            elif ev == KEY_LEFT:
                changed |= self.on_brake_event(KEY_LEFT, -BRAKE_STEP, now)
            elif ev == 'ctrl_c':
                raise KeyboardInterrupt
            else:
                angle = STEER_TABLE.get(ev.lower())
                if angle is not None and angle != self.steer:
                    self.rotate_turtle(angle - self.steer)
                    self.steer = angle
                    changed = True

        if IS_WINDOWS:
            # 2) 방향키 홀드 반복 (0.8초 이상 유지 시 0.2초마다 증감)
            for key, delta in ((KEY_UP, +PULSE_STEP), (KEY_DOWN, -PULSE_STEP)):
                sess = self.hold[key]
                if sess is None:
                    continue
                if not _key_down(key):          # 키를 뗐음 -> 세션 종료
                    self.hold[key] = None
                    continue
                if now >= sess['next']:
                    changed |= self.bump_pulse(delta)
                    sess['next'] += HOLD_REPEAT_S

            # 3) 브레이크 키(←/→) 뗌 감지: 다음 누름을 새 입력으로 인정
            for key in (KEY_LEFT, KEY_RIGHT):
                if self.brake_held[key] and not _key_down(key):
                    self.brake_held[key] = False

        # 4) 발행: 변화 즉시 / 변화 없으면 KEEPALIVE_S(0.5초)마다 keepalive
        if changed:
            self.publish_in()
            self.get_logger().info(
                f"펄스={self.pulse} 조향={self.steer} 브레이크={self.brake}")
        elif now - self.last_in_pub >= KEEPALIVE_S:
            self.publish_in()


def main(args=None):
    # Linux: 터미널을 cbreak 모드로 전환 (한 글자씩 즉시 입력, echo 끔, Ctrl+C는 유지)
    # stdin이 터미널이 아니면(launch 경유 등) 키 입력 불가 — 반드시 ros2 run으로 직접 실행
    old_termios = None
    if not IS_WINDOWS and sys.stdin.isatty():
        old_termios = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

    rclpy.init(args=args)
    node = TalkerK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if old_termios is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_termios)


if __name__ == '__main__':
    main()
