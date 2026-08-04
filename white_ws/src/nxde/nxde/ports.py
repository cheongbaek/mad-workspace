#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ports.py — ★USB 시리얼 장치 식별의 단일 소유자★ (Ubuntu 22.04 전용)

g.launch.py / arduino.py / iahrs.py 가 모두 이 모듈만 본다. 예전에는 VID/PID 표가
white/launch/one_launch.py 안에만 있어서, 노드가 스스로 재연결할 때는 그 표를 쓸 수 없었다
(런치가 한 번 잡아준 고정 경로로만 재시도했다 → 나중에 꽂으면 영원히 못 잡음).

═══════════════════════════════════════════════════════════════════════════════
 ★ 장치 경로 해석 순서 (resolve_device) ★
   1) udev 심볼릭링크가 있으면 그것 (/dev/gps, /dev/imu …)  ← ★권장★
   2) 없으면 VID/PID 로 현재 연결된 포트를 스캔
   3) 그것도 없으면 심볼릭링크 경로를 '그대로' 반환한다 (존재하지 않아도)

 3) 이 중요하다. 장치가 런치 시점에 안 꽂혀 있으면 VID/PID 스캔은 실패하는데, 이때
 '없는 경로'라도 **안정적인 이름**을 돌려주면 나중에 꽂는 순간 그 경로가 생기므로
 respawn(외부 노드) 또는 자체 재연결 루프(우리 노드)가 자동으로 붙는다.
 반대로 /dev/ttyUSB0 같은 열거 순서 의존 경로를 넘기면 다른 장치를 열어버릴 수 있다.

 ★ udev 설정은 사실상 필수다 ★ 설정법은 nxde/README.md 6절 참고.
   GPS·IMU·아두이노가 전부 /dev/ttyACM*·/dev/ttyUSB* 대역을 공유하기 때문이다.
═══════════════════════════════════════════════════════════════════════════════
"""

import os

try:
    from serial.tools import list_ports
except Exception:      # pyserial 이 없는 환경(런치 파서 검사 등)에서도 import 는 되게
    list_ports = None


# ── udev 심볼릭링크 권장 이름 (README 6절의 규칙과 일치해야 한다) ──
SYMLINK_GPS = '/dev/gps'
SYMLINK_IMU = '/dev/imu'

# ── VID/PID 후보 (white/launch/one_launch.py 에서 옮겨온 실측 목록) ──
GPS_VIDPID = [
    (0x1546, 0x01A9),   # u-blox 9 계열
    (0x1546, 0x01A8),   # u-blox 8 계열 / 일부 수신기
]
IMU_VIDPID = [
    (0x10C4, 0xEA60),   # iAHRS / CP210x 계열
]
# 아두이노 계열 USB-serial VID: Arduino 정품(2341) / CH340 클론(1A86) / Arduino LLC(2A03)
ARDUINO_VIDS = {0x2341, 0x1A86, 0x2A03}


def _comports():
    if list_ports is None:
        return []
    try:
        return sorted(list_ports.comports(), key=lambda p: p.device)
    except Exception:
        return []


def find_by_vidpid(candidates, exclude=None):
    """VID/PID 후보 목록으로 현재 연결된 포트를 찾는다. 없으면 None.

    exclude : 이미 다른 장치로 확정된 경로 집합 (같은 VID/PID 장치가 여러 개일 때)"""
    exclude = exclude or set()
    for port in _comports():
        if port.device in exclude:
            continue
        for vid, pid in candidates:
            if port.vid == vid and port.pid == pid:
                return port.device
    return None


def resolve_device(symlink, candidates, exclude=None, log=None):
    """장치 경로를 정한다. 위 파일 헤더의 3단 순서를 따른다.

    반환은 항상 문자열이다(존재하지 않는 경로일 수 있다 — 헤더 3) 참고)."""
    if symlink and os.path.exists(symlink):
        if log:
            log(f"✅ udev 심볼릭링크 사용: {symlink}")
        return symlink

    found = find_by_vidpid(candidates, exclude)
    if found:
        if log:
            log(f"✅ VID/PID 스캔으로 발견: {found}"
                + (f" (권장: udev 로 {symlink} 고정)" if symlink else ""))
        return found

    if log:
        log(f"⚠️ 장치를 찾지 못했습니다 → '{symlink}' 로 계속 시도합니다. "
            f"지금 안 꽂혀 있어도 나중에 꽂으면 자동으로 붙습니다 "
            f"(udev 규칙이 없으면 그 경로가 생기지 않으니 README 6절을 먼저 볼 것).")
    return symlink


def looks_like_arduino(port):
    """VID(정품/CH340/Arduino LLC) 또는 설명으로 아두이노 계열 여부 판정."""
    if port.vid in ARDUINO_VIDS:
        return True
    desc = (port.description or '').lower()
    return ('arduino' in desc) or ('ch340' in desc)


def arduino_candidate_ports(exclude=None):
    """아두이노 A/B 탐색 대상 포트 목록 (/dev/ttyACM* · /dev/ttyUSB*).

    아두이노로 추정되는 포트(VID/설명 일치)를 앞에, 그 외 USB-serial 포트를 뒤에 둔다.
    ★ A/B 두 대가 같은 VID/PID 라서 VID/PID 로는 역할을 구분할 수 없다 ★ 실제 식별은
    포트를 열어 첫 텔레메트리 접두어('S,'=A / 'P,'=B)로 한다 — arduino.py identify_port.

    exclude : GPS/IMU 로 이미 확정된 경로. 넘기면 그 포트는 열어보지 않는다.
      → 이게 없으면 GPS(NMEA)·IMU 포트를 5초씩 열어보며 탐색이 느려지고, 그 동안
        해당 장치의 드라이버가 포트를 못 잡는다(배타 open 충돌).
    """
    exclude = set(exclude or ())
    # 심볼릭링크로 제외 목록이 들어오면 실제 경로까지 함께 막는다
    resolved_exclude = set(exclude)
    for path in exclude:
        try:
            resolved_exclude.add(os.path.realpath(path))
        except OSError:
            pass

    likely, others = [], []
    for p in _comports():
        dev = p.device
        if not (('ACM' in dev) or ('USB' in dev)):
            continue
        if dev in resolved_exclude or os.path.realpath(dev) in resolved_exclude:
            continue
        (likely if looks_like_arduino(p) else others).append(dev)
    return likely + others
