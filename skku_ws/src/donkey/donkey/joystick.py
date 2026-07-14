#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""joystick — 수집(학습) 단계의 마우스 조종 노드.

조종/조종 UI는 기존 joystick_sonar.py(성균관대 코드/Python/조종)를 따르되,
초음파 그래프는 표시하지 않고, 시리얼 대신 /in 토픽으로 발행한다(시리얼은 mega 담당).

  전송 형식(시간 모드 4필드, 조향각 0 고정): "<주행PWM> 0 <조향PWM> <조향시간>"
  좌클릭 = 좌조향 / 우클릭 = 우조향 (누르는 동안, PWM ±150, 3000ms 유지)
  휠 위/아래 = 속도 증가/감소 (최소 30, 최대 255, 스텝 5)
  휠 버튼 = 전/후진 전환 (속도 0일 때만)
  종료: 아무 키 / 좌·우 조향 5초 이상 / 창 닫기  → "0 0 0 0" 전송 후 종료

OS 구분은 a.json 대신 platform.system()으로 자동 판별 (Windows/Ubuntu 겸용).
"""

import platform
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

STEER_HOLD_MS = 3000

MOUSE_STEER      = 150    # 조향모터 PWM 세기 (좌=-, 우=+)
MOUSE_DRIVE_MIN  = 30
MOUSE_DRIVE_MAX  = 255
MOUSE_DRIVE_STEP = 5
STEER_TIMEOUT_S  = 5.0

IS_WINDOWS = platform.system() == "Windows"


class Joystick(Node):
    def __init__(self):
        super().__init__("joystick")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_in = self.create_publisher(String, "in", qos)

        self.cur_drive = 0
        self.cur_steer = 0     # 현재 조향 PWM (좌=-, 우=+, 0=정지)
        self.speed = 0
        self.direction = 1     # 1:전진 / -1:후진

    # ── /in 발행 (기존 send_command와 동일 형식) ──────────
    def send_command(self, target_pwm, steer_pwm, steer_ms):
        msg = String()
        msg.data = f"{int(target_pwm)} 0 {int(steer_pwm)} {int(steer_ms)}"
        self.pub_in.publish(msg)

    def send_state(self):
        steer_ms = STEER_HOLD_MS if self.cur_steer != 0 else 0
        self.send_command(self.cur_drive, self.cur_steer, steer_ms)

    def set_steer(self, value):
        if value != self.cur_steer:
            self.cur_steer = value
            if value == 0:
                self.send_command(self.cur_drive, 0, 0)
            else:
                self.send_command(self.cur_drive, value, STEER_HOLD_MS)

    def apply_drive(self):
        value = self.direction * self.speed
        if value != self.cur_drive:
            self.cur_drive = value
            self.send_state()


def run_mouse(node: Joystick):
    root = tk.Tk()
    root.title("마우스 조종 모드")
    root.attributes('-fullscreen', True)
    root.configure(bg='black')
    # ros2 launch가 띄우면 창이 터미널 뒤에 열려 마우스 입력이 "안 먹는 것처럼"
    # 보일 수 있음 → 최상위로 끌어올리고 포커스 강제
    root.attributes('-topmost', True)
    root.lift()
    root.focus_force()
    root.after(500, lambda: root.attributes('-topmost', False))

    tk.Label(
        root, bg='black', fg='white', justify='center',
        font=('Arial', 18),
        text=("[마우스 조종 모드 — /in 발행]\n\n"
              "좌클릭 = 좌조향 / 우클릭 = 우조향 (누르는 동안)\n"
              "휠 위 = 속도 증가 / 휠 아래 = 속도 감소\n"
              "휠 버튼 = 전/후진 전환 (속도 0일 때만)\n\n"
              "종료: 아무 키 / 좌·우 조향 5초 이상 / 창 닫기")
    ).pack(expand=True)

    timeout_job = [None]

    def close_all():
        node.speed = 0
        node.apply_drive()
        node.set_steer(0)
        node.send_command(0, 0, 0)   # 안전 정지
        root.destroy()

    def start_steer_timer():
        cancel_steer_timer()
        timeout_job[0] = root.after(int(STEER_TIMEOUT_S * 1000), on_steer_timeout)

    def cancel_steer_timer():
        if timeout_job[0] is not None:
            root.after_cancel(timeout_job[0])
            timeout_job[0] = None

    def on_steer_timeout():
        print("조향 5초 이상 유지 → 종료합니다.", flush=True)
        close_all()

    def on_left_press(e):
        node.set_steer(-MOUSE_STEER); start_steer_timer()

    def on_left_release(e):
        node.set_steer(0); cancel_steer_timer()

    def on_right_press(e):
        node.set_steer(MOUSE_STEER); start_steer_timer()

    def on_right_release(e):
        node.set_steer(0); cancel_steer_timer()

    def wheel_step(up):
        accelerate = up if node.direction == 1 else (not up)
        if accelerate:
            node.speed = MOUSE_DRIVE_MIN if node.speed == 0 else \
                min(node.speed + MOUSE_DRIVE_STEP, MOUSE_DRIVE_MAX)
        else:
            node.speed = 0 if node.speed <= MOUSE_DRIVE_MIN else \
                max(node.speed - MOUSE_DRIVE_STEP, MOUSE_DRIVE_MIN)
        node.apply_drive()

    def on_middle(e):
        if node.speed == 0:
            node.direction = -node.direction
            print("전진" if node.direction == 1 else "후진", flush=True)

    def on_key(e):
        print("키 입력 감지 → 종료합니다.", flush=True)
        close_all()

    root.bind('<ButtonPress-1>', on_left_press)
    root.bind('<ButtonRelease-1>', on_left_release)
    root.bind('<ButtonPress-3>', on_right_press)
    root.bind('<ButtonRelease-3>', on_right_release)
    root.bind('<ButtonPress-2>', on_middle)
    if IS_WINDOWS:
        root.bind('<MouseWheel>', lambda e: wheel_step(e.delta > 0))
    else:   # Linux(X11): 휠이 Button-4/5 별도 이벤트
        root.bind('<Button-4>', lambda e: wheel_step(True))
        root.bind('<Button-5>', lambda e: wheel_step(False))
    root.bind('<Key>', on_key)
    root.protocol("WM_DELETE_WINDOW", close_all)

    root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = Joystick()   # 발행 전용 — spin 불필요, tkinter mainloop가 주 루프
    try:
        run_mouse(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_command(0, 0, 0)
        node.destroy_node()
        rclpy.shutdown()
        print("joystick 종료 — 정지 명령 발행됨", flush=True)


if __name__ == "__main__":
    main()
