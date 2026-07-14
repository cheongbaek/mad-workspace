#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""donkey 패키지 공통 유틸 — 상수, 정규화, 세션 폴더 넘버링, 시리얼 포트 탐지, 카메라 열기.

benz.ino(Mega2560, 115200) 프로토콜:
  PC → 차 : "<주행PWM> <조향각>\n"   (PWM -255~255, 각도 -30~30, 각도 PD 모드)
  차 → PC : 20ms마다 "s1 s2 s3 s4 s5 s6 현재조향각"  (초음파 6개 + 각도[deg])
"""

import platform
import re
from pathlib import Path

import cv2
import serial.tools.list_ports

MAX_ANGLE = 30.0    # benz.ino ANG_MIN/MAX
MAX_PWM   = 255.0   # benz.ino PWM constrain
SERIAL_BAUD = 115200

# Mega2560 계열 VID/PID
_KNOWN_MEGA_VIDPID = {(0x2341, 0x0042), (0x2341, 0x0010), (0x2341, 0x003F), (0x2A03, 0x0042)}
_CANDIDATE_VIDS = {0x1A86, 0x0403, 0x10C4}   # CH340/FTDI/CP210x 호환보드


def norm_angle(deg):
    return float(deg) / MAX_ANGLE


def norm_pwm(pwm):
    return float(pwm) / MAX_PWM


def denorm(angle_n, pwm_n, max_pwm=MAX_PWM, allow_reverse=False):
    """모델 출력(-1~1) → (조향각 deg, 주행PWM), 안전 clamp 포함."""
    angle = max(-MAX_ANGLE, min(MAX_ANGLE, float(angle_n) * MAX_ANGLE))
    pwm = float(pwm_n) * MAX_PWM
    lo = -max_pwm if allow_reverse else 0.0
    pwm = max(lo, min(max_pwm, pwm))
    return angle, pwm


def package_root() -> Path:
    """src/donkey 디렉터리 (data/, trained/, launch/의 부모).

    symlink-install(develop 모드) 시 이 파일은 src에서 직접 로드되므로
    parents[1] == .../src/donkey 가 된다. 일반 install이면 install 트리 하위가
    되지만 data/trained가 그 안에 만들어질 뿐 동작은 동일하다.
    """
    return Path(__file__).resolve().parents[1]


def next_numbered_dir(base: Path, prefix: str) -> Path:
    """base 안에서 prefix_001, prefix_002 ... 중 다음 번호 폴더 경로를 반환(생성은 안 함)."""
    base.mkdir(parents=True, exist_ok=True)
    pat = re.compile(rf"^{re.escape(prefix)}_(\d+)$")
    nums = [int(m.group(1)) for d in base.iterdir()
            if d.is_dir() and (m := pat.match(d.name))]
    return base / f"{prefix}_{(max(nums) + 1 if nums else 1):03d}"


def latest_numbered_dir(base: Path, prefix: str):
    """base 안에서 가장 번호가 큰 prefix_### 폴더. 없으면 None."""
    if not base.is_dir():
        return None
    pat = re.compile(rf"^{re.escape(prefix)}_(\d+)$")
    found = [(int(m.group(1)), d) for d in base.iterdir()
             if d.is_dir() and (m := pat.match(d.name))]
    return max(found)[1] if found else None


def find_serial_port():
    """Mega2560 자동 탐지. 실패 시 None."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid is not None and (p.vid, p.pid) in _KNOWN_MEGA_VIDPID:
            return p.device
    for p in ports:
        desc = (p.description or "").lower()
        if "mega" in desc or "arduino" in desc:
            return p.device
    candidates = [p for p in ports if p.vid in _CANDIDATE_VIDS]
    if len(candidates) == 1:
        return candidates[0].device
    return None


# 사용할 카메라: 로지텍 HD 웹캠 C920E(VU0060) 전용.
# 내장캠·'휴대폰과 연결' 가상 카메라(예: "S25 Ultra (Windows 가상 카메라)")가
# 인덱스 0을 차지하는 경우가 실제로 있으므로, 인덱스가 아니라 장치 "이름"으로 찾는다.
CAMERA_NAME_HINTS = ("c920", "vu0060")


def _list_cameras_windows():
    """DirectShow 장치 이름 목록 (리스트 순서 = CAP_DSHOW 인덱스 순서)."""
    try:
        from pygrabber.dshow_graph import FilterGraph   # Windows 전용
    except ImportError:
        raise RuntimeError(
            "Windows에서 카메라 이름 열거에 pygrabber가 필요합니다: "
            "pip install pygrabber==0.1  (0.2는 Python 3.9+ 전용)")
    return FilterGraph().get_input_devices()


def _list_cameras_linux():
    """/sys/class/video4linux/video*/name → [(장치경로, 이름)]."""
    import glob
    found = []
    for name_file in sorted(glob.glob("/sys/class/video4linux/video*/name")):
        dev = "/dev/" + name_file.split("/")[-2]
        try:
            with open(name_file) as f:
                found.append((dev, f.read().strip()))
        except OSError:
            pass
    return found


def resolve_camera_candidates(name_hints=CAMERA_NAME_HINTS):
    """이름 힌트에 맞는 카메라 후보 목록 [(장치, 이름)].
    Windows=int 인덱스, Linux=/dev/videoN 경로.

    Linux에서는 UVC 웹캠 하나가 /dev/video 노드를 2개 이상(영상+메타데이터) 만들고
    이름이 같으므로 후보가 여러 개일 수 있다 — 호출부(open_camera)가 실제로
    프레임이 읽히는 노드를 골라낸다.

    하나도 없으면 발견된 장치 목록과 함께 RuntimeError — 다른 카메라로 조용히
    폴백하지 않는다(폰/내장캠 오인 방지).
    """
    hints = [h.strip().lower() for h in name_hints if h.strip()]
    if platform.system() == "Windows":
        names = _list_cameras_windows()
        found = [(idx, name) for idx, name in enumerate(names)
                 if any(h in name.lower() for h in hints)]
        all_devices = names
    else:
        devices = _list_cameras_linux()
        found = [(dev, name) for dev, name in devices
                 if any(h in name.lower() for h in hints)]
        all_devices = devices
    if not found:
        raise RuntimeError(
            f"카메라(이름에 {hints} 포함)를 찾지 못함. 발견된 장치: {all_devices} "
            f"— C920E 웹캠 연결을 확인하세요")
    return found


def resolve_camera(name_hints=CAMERA_NAME_HINTS):
    """이름 힌트로 카메라 1개를 찾는다 (첫 후보). 없으면 RuntimeError."""
    return resolve_camera_candidates(name_hints)[0]


def _open_device(device, width, height, fps):
    if platform.system() == "Windows":
        cap = cv2.VideoCapture(device, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def open_camera(device=None, width=640, height=480, fps=30.0, name_hints=None):
    """C920E 웹캠 열기 (Windows/Ubuntu 겸용).

    device=None(기본)이면 이름으로 자동 탐색 후, 후보마다 실제로 프레임이
    읽히는지 확인해서 첫 정상 노드를 쓴다(Linux 메타데이터 노드 배제).
    C920E가 없으면 예외 — 다른 카메라로 폴백하지 않는다.
    device에 int 인덱스/경로를 직접 주면 검사 없이 그 장치를 연다(비상용).
    """
    if device is not None and device != "":
        return _open_device(device, width, height, fps)

    if isinstance(name_hints, str):
        name_hints = [h for h in name_hints.split(",") if h.strip()]
    candidates = resolve_camera_candidates(name_hints or CAMERA_NAME_HINTS)

    for dev, name in candidates:
        cap = _open_device(dev, width, height, fps)
        if cap.isOpened() and cap.read()[0]:
            print(f"[카메라] 이름 매칭: {name} → {dev}", flush=True)
            return cap
        cap.release()

    raise RuntimeError(
        f"카메라 후보 {[(d, n) for d, n in candidates]} 를 찾았지만 "
        f"프레임을 읽을 수 없음 — 다른 프로그램이 카메라를 사용 중인지 확인하세요")
