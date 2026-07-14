#!/usr/bin/env python3
"""
gps_imu.py  ― GPS-IMU 융합 노드  [v5.8 - 융합비 튜닝 파라미터(비교분석용)]

────────────────────────────────────────────────
v5.7.1 → v5.8 변경 (GPS:IMU 융합 비율 비교분석)
────────────────────────────────────────────────
· 클래스 최상단에 융합 노브 3개 신설 (FUSION_MODE / POS_GPS_ALPHA / HEADING_GPS_TRUST)
· POS_GPS_ALPHA < 1.0 이면 위치 융합이 '이진 스위치(GPS↔DR)'에서
  '연속 상보필터'로 전환: DR 적분(엔코더+heading)을 상시 가동하고
  매 fix마다 fused += α·(GPS − fused) 로 당김.
  - α=1.0 : 기존 v5.7.1과 완전 동일(레거시 경로, DR 전환/3초 블렌딩 그대로)
  - α=0.0 : 초기 헤딩 고정까지만 GPS 개입, 이후 순수 DR (dr_only와 동일)
  - 0<α<1 : GPS:IMU 위치 융합비 실험 구간. GPS 단절→복구도 매 fix
    α-수렴으로 자동 처리되므로 이 모드에선 DR 전환/블렌딩 머신 미사용.
· HEADING_GPS_TRUST: drift 보정·연속미세보정의 최종 correction에 곱하는
  스케일러. 0=heading에 GPS 불개입(고정 후 offset 동결), 1=현행, >1=강화.
  (게이트/시간정렬/선회보상 로직은 그대로 두고 '반영 강도'만 스케일)
· FUSION_MODE 프리셋: "fused"(노브값 그대로 사용) / "gps_raw"(α=1·trust=0·
  뒷차축 투영 OFF·DR 폴백 OFF = GPS 원시 바이패스) / "dr_only"(α=0·trust=0)
· 초기 헤딩 고정 전에는 모드와 무관하게 기존과 동일하게 동작(원점·초기헤딩 확립).

────────────────────────────────────────────────
v5.7 → v5.7.1 변경 (0703 로스백)
────────────────────────────────────────────────
· [증상] 선회보상 drift 보정이 코너에서 ±7~10°를 회전방향 따라 번갈아 측정,
  8회 발동해 헤딩을 좌우로 흔듦(슬라럼 밀림 기여).
· [실측] drift 측정치 = +0.23s × 요레이트 로 오염 — GPS 변위(0.2s 스텝)의
  시간중심 + RTK 솔루션 지연만큼 '과거'의 코스를 '현재' 헤딩과 비교한 위상차.
· [수정] 자세 이력(20Hz, 2.4s)을 축적, 변위 시간중심−솔루션지연(0.13s) 시점의
  헤딩·자이로로 기대코스를 계산해 비교. 연속미세보정도 창 중심으로 정렬.
· [효과(실측 재구성)] drift 측정 std 4.36°→2.26°, 오발동(>6°) 14%→2%.

────────────────────────────────────────────────
v5.6 → v5.7 변경 (0702 로스백 정량 분석 반영)
────────────────────────────────────────────────
① 안테나→뒷차축 투영 (ANTENNA_AHEAD_REAR_M=0.865)
   - 안테나는 앞차축+0.135m = 뒷차축 기준 0.865m 전방.
   - /ego_state 위치를 뒷차축으로 투영 발행 → Pure Pursuit 기하 정확,
     DR 킨매틱 정확(뒷차축은 횡속도 0), mapping도 뒷차축 경로 기록.
   - 기존 안테나 기록 맵: 곡선에서 ≤0.06~0.12m 바깥 오프셋(안전 방향), 재매핑 권장.
② 선회보상 drift 보정 (DRIFT_SWING_COMP_ENABLED)
   - 기존엔 gyro>12°/s면 skip → 곡선 위주 코스에서 보정 0회(로스백 103회 skip).
   - 기대 진행방향 = atan2(v·sinψ+ω·d·cosψ, v·cosψ−ω·d·sinψ)로 코너에서도
     GPS 진행방향과 비교 가능 → gyro 게이트 45°/s로 완화.
   - 실증: 이 모델의 잔차가 실측 로스백 전 구간(곡선 포함)에서 0.1°±2.8°.
③ GPS 속도추정을 투영(뒷차축) 좌표로 → 선회 스윙에 의한 과대/부호 오판 제거

────────────────────────────────────────────────
v4.9 → v5.0 변경 (로스백 분석 결과 반영)
────────────────────────────────────────────────
[문제 진단 - 로스백 rosbag2_2026_05_28-22_00_51]
  - v4.9에서 drift 보정을 너무 공격적으로 풀어놨더니, 헤딩이 1초마다 5-15°씩
    점프하면서 driving.py가 그걸 따라 미친 듯이 조향 → S자 사행 발생
  - 예: t=10.46s offset=-24.85° → t=12.06s offset=-17.10° (1.6초간 +7.75° 점프)
  - 예: t=19.65s offset=-28.10° → t=22.05s offset=-9.91° (2.4초간 +18.19° 점프)
  - 원인: DRIFT_CORRECT_GYRO_MAX=80°/s로 풀어놨더니 코너 회전 자체를 drift로
    오인해서 강제 보정. gyro 40-56°/s 한복판에서 -16.49° 보정 적용된 사례 다수
  - 직선 |CTE| 평균 0.41m (코너 0.26m보다 더 큼) — 역설적 상황

[수정 사항 - 보수적 보정 (안정성 우선)]
(1) DRIFT_CORRECT_GYRO_MAX:    80°/s → 12°/s  (코너에서 절대 보정 금지)
(2) DRIFT_CORRECT_INTERVAL:    0.3s → 1.2s   (헤딩 안정 시간 확보)
(3) DRIFT_CORRECT_THRESH_DEG:  3.0° → 5.0°   (작은 drift는 무시)
(4) DRIFT_CORRECT_GAIN_BASE:   0.18 → 0.05   (천천히 점진 보정)
(5) DRIFT_CORRECT_GAIN_MAX:    0.50 → 0.15
(6) DRIFT_FORCE_RESET_DEG:     25° → 60°    (정말 큰 발산일 때만 리셋)
(7) 연속 미세 보정: max_gyro 30→8°/s, gain 0.10→0.025 (직선에서만)

[효과 예측]
  - 헤딩 점프 없어짐 → driving.py가 안정된 헤딩으로 조향 계산
  - S자 사행 80% 감소 예상
  - 단점: drift가 천천히 쌓일 수 있으나, 매핑경로 자체가 GPS 기반이므로
    GPS 진행방향과 IMU heading 차이는 어차피 시간 평균으로 수렴함
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, String
from sensor_msgs.msg import NavSatFix, Imu
import math, time, threading
from collections import deque   # [v5.7.1] 자세 이력(시간정렬용)


def euler_from_quaternion(x, y, z, w):
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def pitch_from_quaternion(x, y, z, w):
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return math.degrees(math.asin(sinp))

def normalize_angle(deg):
    while deg >  180: deg -= 360
    while deg < -180: deg += 360
    return deg

GPS_STATUS_NO_FIX   = -1
GPS_STATUS_FIX      =  0
GPS_STATUS_SBAS_FIX =  1
GPS_STATUS_GBAS_FIX =  2
GPS_STATUS_LABEL = {
    GPS_STATUS_NO_FIX:   "NO FIX   (위성 신호 없음)",
    GPS_STATUS_FIX:      "GPS FIX  (일반 GPS, 오차 ~2-5m)",
    GPS_STATUS_SBAS_FIX: "SBAS FIX (보정 GPS, 오차 ~1m)",
    GPS_STATUS_GBAS_FIX: "RTK FIX  (고정밀, 오차 ~2cm)",
}
GPS_STATUS_EMOJI = {
    GPS_STATUS_NO_FIX: "❌", GPS_STATUS_FIX: "🟡",
    GPS_STATUS_SBAS_FIX: "🟠", GPS_STATUS_GBAS_FIX: "🟢",
}


class GpsImuNode(Node):

    # ══════════════════════════════════════════════════════════════════
    # [v5.8] GPS:IMU 융합 비율 튜닝 노브 (비교분석용) — 여기만 바꾸면 됨
    # ══════════════════════════════════════════════════════════════════
    # FUSION_MODE : 프리셋 선택
    #   "fused"   : 아래 두 노브 값을 그대로 사용 (기본)
    #   "gps_raw" : GPS 원시 바이패스 (α=1, trust=0, 뒷차축 투영 OFF, DR 폴백 OFF)
    #   "dr_only" : 초기 헤딩 고정까지만 GPS, 이후 순수 DR (α=0, trust=0)
    FUSION_MODE = "fused"

    # POS_GPS_ALPHA : 위치 융합비 (0.0 ~ 1.0)
    #   1.0  = 기존 동작(GPS 직접 반영 + 단절 시 DR + 복구 3초 블렌딩)과 완전 동일
    #   <1.0 = 연속 상보필터: DR 적분 상시 가동, 매 fix마다 fused += α·(GPS−fused)
    #          (5Hz fix 기준 수렴 시정수 ≈ 0.2s/α : α=0.5→0.4s, α=0.2→1s, α=0.05→4s)
    #   0.0  = 순수 DR (GPS는 초기 원점/헤딩 확립에만 사용)
    POS_GPS_ALPHA = 1.0

    # HEADING_GPS_TRUST : 헤딩 보정(GPS→IMU offset) 강도 스케일 (0.0 ~ 2.0 권장)
    #   1.0 = 현행 (drift 보정 gain 0.04~0.10 + 연속미세보정 gain 0.04)
    #   0.0 = 헤딩에 GPS 불개입 (고정 순간의 yaw_offset 동결 → 순수 IMU 헤딩)
    #   실효 시정수 ≈ INTERVAL/(gain·trust) : trust=1→약 10~25s, trust=2→5~12s
    HEADING_GPS_TRUST = 1.0

    # PROJECT_TO_REAR_AXLE : 안테나→뒷차축 투영(d=0.865m) 사용 여부
    #   False = 안테나 원시좌표 그대로 발행 (v5.7 이전 동작 재현 실험용)
    #   ※ gps_raw 프리셋은 강제 False
    PROJECT_TO_REAR_AXLE = True

    # DR_FALLBACK_ENABLED : α=1.0(레거시)일 때 GPS 단절 → DR 전환 + 복구 3초 블렌딩
    #   False = 단절 시 위치 갱신 정지 (α<1 연속 상보필터에선 어차피 미사용)
    #   ※ gps_raw 프리셋은 강제 False
    DR_FALLBACK_ENABLED = True

    # GPS_SOLUTION_LATENCY_S : GPS 코스 시간정렬 지연 [s] (drift 측정 품질)
    #   실측 0.13 (v5.7.1: 변위 시간중심 0.1s와 합쳐 총 0.23s 위상 보정).
    #   0.0 = 시간정렬 해제 (정렬 전(v5.7) 동작 재현 실험용)
    GPS_SOLUTION_LATENCY_S = 0.13
    # ══════════════════════════════════════════════════════════════════

    WHEEL_CIRCUMFERENCE = 0.27 * math.pi
    TICKS_PER_REV       = 300.0
    ENC_DT              = 0.01

    # ─── [v5.4] IMU drift 보정 파라미터: 코너 오보정 방지 ───────────────
    # 로스백 분석: IMU vs GPS 실제 방향 오차 = 3~6° (핵심 CTE 원인)
    # CTE = sin(IMU오차) × LFD = sin(5°) × 5m = 0.44m (관측치와 일치)
    # 목표: IMU 오차 2° 이하 → CTE = sin(2°) × 5m = 0.17m
    # 보정 강화: gain_base 0.10→0.20 (2배), interval 0.6→0.4s
    # skip 10회 발생 → gyro_max 25→28°/s로 완화
    DRIFT_CORRECT_THRESH_DEG = 6.0          # v5.4: 작은 차이는 GPS 노이즈로 보고 무시
    DRIFT_CORRECT_INTERVAL   = 1.0          # v5.4: 보정 간격 확대, offset 계단 변화 억제
    DRIFT_CORRECT_GYRO_MAX   = math.radians(12.0)  # v5.4: 코너 회전 중 drift 보정 금지
    DRIFT_CORRECT_GAIN_BASE  = 0.04         # v5.4: 한 번에 크게 틀지 않음
    DRIFT_CORRECT_GAIN_MAX   = 0.10
    DRIFT_CORRECT_GAIN_SCALE = 250.0

    DRIFT_FORCE_RESET_DEG    = 60.0         # v5.4: 정말 큰 발산일 때만 강제 리셋
    DRIFT_FORCE_RESET_DIST   = 1.5

    # ─── [v5.7] GPS 안테나 → 뒷차축 투영 ────────────────────────────────
    # 안테나가 앞차축보다 0.135m 앞 = 뒷차축 기준 0.865m 전방(0.73+0.135).
    # Pure Pursuit 기하·DR 킨매틱은 '뒷차축' 기준이 정확 → 위치를 뒷차축으로
    # 투영해 /ego_state로 발행. 효과:
    #   ① PP 조향식 δ=atan(2L·sinα/LFD)의 기준점이 정확해짐
    #   ② DR: 뒷차축은 킨매틱상 횡속도 0 → v·[cosψ,sinψ] 적분이 '정확'
    #      (안테나점은 선회 시 ω·d 횡이동이 있어 기존 DR이 코너에서 오차 누적)
    #   ③ mapping도 자동으로 뒷차축 경로 기록 → 이후 맵은 완전 정합
    # 주의: 기존(안테나 기록) 맵은 곡선에서 최대 d²/2R ≈ 0.06~0.12m 바깥 오프셋
    #       (안전한 방향) — 재매핑 권장.
    # 로스백 실증: 헤딩-GPS진행방향 잔차가 스윙항 atan(ω·d/v) 반영 시 0.1°±2.8°.
    ANTENNA_AHEAD_REAR_M = 0.865

    # ─── [v5.7] 선회 보상 drift 측정 ────────────────────────────────────
    # 기존: gyro>12°/s면 보정 skip → 곡선 위주 코스에서 보정 0회(이번 로스백 103회 skip).
    # 개선: 안테나 속도벡터 = v·û(ψ) + ω·d·n̂(ψ) 를 기대 진행방향으로 사용하면
    #       코너에서도 GPS 진행방향과 비교 가능 → gyro 게이트를 45°/s까지 완화.
    DRIFT_SWING_COMP_ENABLED     = True
    DRIFT_CORRECT_GYRO_MAX_SWING = math.radians(45.0)

    # ─── [v5.7.1] GPS 코스 시간정렬 ─────────────────────────────────────
    # 0703 로스백 실측: drift 측정치가 요레이트에 +0.23s 기울기로 비례(=GPS 변위가
    # 그만큼 과거) → 코너에서 ±7~10° phantom drift가 회전방향 따라 번갈아 발동,
    # 헤딩을 좌우로 흔들었음(슬라럼 밀림 기여). 변위 구간의 '시간 중심'(0.5×fix간격)
    # + RTK 솔루션 지연을 합쳐 그 시점의 헤딩·자이로로 기대코스를 계산해 정렬.
    # 실측 총지연 0.23s = 0.5×0.2s(5Hz) + 0.13s.
    # 정렬 효과(실측): drift 측정 std 4.36°→2.26°, 오발동(>6°) 14%→2%.
    # → [v5.8] 상수(GPS_SOLUTION_LATENCY_S)는 상단 융합 노브 블록으로 이동.

    HEADING_MAX_RATE_DPS = 200.0

    # 지형 히스테리시스
    PITCH_ENTRY_UP   =  3.0
    PITCH_EXIT_UP    =  1.5
    PITCH_ENTRY_DOWN = -3.0
    PITCH_EXIT_DOWN  = -1.5

    # ── 헤딩 lock (v4.8 유지) ─────────────────────────────────────────
    HEADING_LOCK_DIST   = 1.0   # 헤딩 고정 복구: 기존 방식처럼 1.0m 이동 기준
    HEADING_LOCK_ENCODER_DIST = 0.60  # 헤딩 고정 복구: 기존 ENC 0.60m 기준
    HEADING_LOCK_MIN_SPEED    = 0.20  # 헤딩 고정 복구: 기존 0.20m/s 기준

    GPS_ONLY_HEADING_LOCK_DIST   = 2.0
    GPS_ONLY_HEADING_SPREAD_DEG  = 5.0
    HEADING_LOCK_SPREAD_DEG      = 15.0  # v5.5: 초기 헤딩을 너무 흔들린 GPS 1m 이동값으로 고정하지 않음

    HEADING_LOCK_BUF_SIZE = 7    # v5.5: 5→7, 1초 내외 방향 안정성 확인

    # ── [v5.3 강화] 연속 헤딩 미세 보정 ─────────────────────────────────
    CONT_CORRECT_ENABLED         = True
    CONT_CORRECT_MIN_SPEED       = 0.50
    CONT_CORRECT_MAX_GYRO        = math.radians(10.0)   # v5.4: 직선에 가까울 때만
    CONT_CORRECT_GAIN            = 0.040        # v5.4: 연속 미세보정도 매우 약하게
    CONT_CORRECT_MIN_DRIFT_DEG   = 0.6
    CONT_CORRECT_MAX_DRIFT_DEG   = 8.0
    CONT_GPS_WINDOW              = 7
    CONT_MIN_GPS_DIST            = 0.80         # 0.50 → 0.40
    CONT_LINEARITY_MAX_SPREAD    = 4.0
    CONT_MIN_SEG_DIST            = 0.05

    # ── gyro_z bias 추정 (v4.8 유지) ──────────────────────────────────
    GYRO_BIAS_LEARN_SPEED_MAX  = 0.10
    GYRO_BIAS_LEARN_GYRO_MAX   = math.radians(2.0)
    GYRO_BIAS_LPF_ALPHA        = 0.002
    GYRO_BIAS_MAX_ABS          = math.radians(5.0)

    ENCODER_FRESH_TIMEOUT = 0.50
    GPS_SPEED_LPF_TAU     = 0.35
    GPS_SPEED_MAX_VALID   = 5.50
    GPS_SPEED_DEADBAND    = 0.03
    GPS_BLEND_DURATION  = 3.0
    PITCH_CALIB_SECS    = 2.0

    def __init__(self):
        super().__init__("gps_imu_node")

        self._lock = threading.Lock()

        # ── [v5.8] 융합 노브 해석 (프리셋 → 실효 파라미터) ─────────────────
        _mode = str(self.FUSION_MODE).lower()
        if _mode == "gps_raw":
            self._pos_gps_alpha     = 1.0
            self._heading_gps_trust = 0.0
            self._project_enabled   = False   # 안테나 원시좌표 그대로 발행
        elif _mode == "dr_only":
            self._pos_gps_alpha     = 0.0
            self._heading_gps_trust = 0.0
            self._project_enabled   = bool(self.PROJECT_TO_REAR_AXLE)  # (위치에 GPS 미반영이라 실질 무관)
        else:
            _mode = "fused"
            self._pos_gps_alpha     = max(0.0, min(1.0, float(self.POS_GPS_ALPHA)))
            self._heading_gps_trust = max(0.0, float(self.HEADING_GPS_TRUST))
            self._project_enabled   = bool(self.PROJECT_TO_REAR_AXLE)
        self._fusion_mode = _mode
        # α<1 이면 연속 상보필터: cb_encoder가 DR 예측을 상시 적분, fix마다 α-당김.
        # 이 모드에선 레거시 DR 전환/3초 블렌딩 머신을 쓰지 않는다(α-수렴이 대체).
        self._pos_fusion_active   = self._pos_gps_alpha < 1.0
        self._dr_fallback_enabled = ((not self._pos_fusion_active) and _mode != "gps_raw"
                                     and bool(self.DR_FALLBACK_ENABLED))

        # 위치
        self.origin_lat, self.origin_lon = None, None
        self.fused_x,    self.fused_y   = 0.0, 0.0
        self.fused_lat,  self.fused_lon = 0.0, 0.0
        self.start_x,    self.start_y   = 0.0, 0.0
        self.prev_fused_x = 0.0
        self.prev_fused_y = 0.0
        # [v5.7] drift 측정용 안테나 원시좌표 이전값(투영좌표는 heading 의존→자기참조 방지)
        self._ant_prev_x = None
        self._ant_prev_y = None
        self._ant_prev_t = None
        # [v5.7.1] (t, heading[°], gz_bias보정[rad/s]) 이력 — GPS 코스 시간정렬용(2.4s)
        self._att_hist = deque(maxlen=48)

        # 속도
        self.current_speed_ms  = 0.0
        self.current_speed_dir = 1
        self.encoder_last_rx_time = 0.0
        self.gps_speed_ms = 0.0
        self.gps_speed_raw_ms = 0.0
        self._gps_speed_last_x = None
        self._gps_speed_last_y = None
        self._gps_speed_last_t = None
        self.encoder_buf       = []
        self.encoder_active    = False
        self.heading_lock_odom_dist = 0.0
        self.heading_lock_last_time = None
        self.heading_lock_buf = []

        # 헤딩
        self.raw_imu_yaw         = 0.0
        self.raw_imu_yaw_unwrap  = 0.0
        self.prev_raw_imu_yaw    = None
        self.raw_imu_gyro_z      = 0.0
        self.heading             = 0.0
        self.yaw_offset          = 0.0
        self.is_heading_locked   = False
        self.locked_heading      = None
        self.locked_heading_time = None

        # gyro_z LPF
        self.gyro_z_abs_lpf = 0.0

        # gyro_z bias 추정
        self.gyro_z_bias = 0.0

        # GPS 상태
        self.is_rtk_fixed    = False
        self.gps_status      = GPS_STATUS_NO_FIX
        self.prev_gps_status = GPS_STATUS_NO_FIX
        self.gps_fix_count   = 0
        self.gps_nofix_count = 0
        self.gps_dropout_start = None

        # DR
        self.dr_active           = False
        self.last_encoder_time   = time.time()
        self.dr_accumulated_dist = 0.0

        # GPS 복구 블렌딩
        self.gps_blend_active = False
        self.gps_recovery_blend_active = False
        self.gps_blend_start  = 0.0
        self.gps_blend_from_x = 0.0
        self.gps_blend_from_y = 0.0
        self.gps_blend_to_x   = 0.0
        self.gps_blend_to_y   = 0.0
        self._gps_blend_duration = self.GPS_BLEND_DURATION

        # 시간
        self.last_gps_time           = time.time()
        self.last_status_print_time  = 0.0
        self.last_drift_correct_time = 0.0
        self.prev_main_time          = time.time()

        # 연속 헤딩 보정용 버퍼
        self.cont_gps_buf = []
        self.cont_correct_count = 0
        self.cont_correct_total = 0.0
        self.last_cont_log_time = 0.0

        # [v5.0 추가] drift 보정 통계 - 디버깅용
        self.drift_correct_count = 0
        self.drift_correct_total_abs = 0.0
        self.drift_skipped_gyro = 0   # gyro 때문에 skip된 횟수

        # Pitch
        self.imu_pitch_deg  = 0.0
        self.pitch_offset   = 0.0
        self.pitch_buf      = []
        self.pitch_calib_buf = []
        self.pitch_calibrated = False
        self.pitch_calib_start = time.time()

        # 지형
        self.terrain_code = 0.0

        # 퍼블리셔/서브스크라이버
        self.create_subscription(NavSatFix, "/fix",      self.cb_fix,     10)
        self.create_subscription(Imu,       "/imu/data", self.cb_imu,     10)
        self.create_subscription(Int32,     "/encoder",  self.cb_encoder, 10)

        self.ego_pub     = self.create_publisher(Float64MultiArray, "/ego_state",     10)
        self.gps_st_pub  = self.create_publisher(String,            "/gps_status",    10)
        self.heading_pub = self.create_publisher(Float64MultiArray, "/heading",       10)
        self.terrain_pub = self.create_publisher(Float64MultiArray, "/terrain_state", 10)

        self._publish_heading_now()
        self.create_timer(0.05, self.main_loop)

        self.get_logger().info(
            f"[GPS-IMU v5.8 | 융합설정] mode={self._fusion_mode} "
            f"POS_GPS_ALPHA={self._pos_gps_alpha:.2f} "
            f"HEADING_GPS_TRUST={self._heading_gps_trust:.2f} "
            f"(위치융합={'연속 α-상보필터' if self._pos_fusion_active else '레거시(GPS직접+DR폴백)'}, "
            f"투영={'ON' if self._project_enabled else 'OFF'}, "
            f"DR폴백={'ON' if self._dr_fallback_enabled else 'OFF'})")
        self.get_logger().info(
            f"[GPS-IMU v5.8 | 안테나→뒷차축 투영 d={self.ANTENNA_AHEAD_REAR_M}m "
            f"+ 선회보상 drift(gyro≤{math.degrees(self.DRIFT_CORRECT_GYRO_MAX_SWING):.0f}°/s)] "
            f"Drift보정: thresh={self.DRIFT_CORRECT_THRESH_DEG}° "
            f"interval={self.DRIFT_CORRECT_INTERVAL}s "
            f"gyro_max={math.degrees(self.DRIFT_CORRECT_GYRO_MAX):.0f}°/s "
            f"gain={self.DRIFT_CORRECT_GAIN_BASE}~{self.DRIFT_CORRECT_GAIN_MAX} "
            f"force_reset={self.DRIFT_FORCE_RESET_DEG}° | "
            f"연속보정: gain={self.CONT_CORRECT_GAIN} "
            f"max_gyro={math.degrees(self.CONT_CORRECT_MAX_GYRO):.0f}°/s "
            f"min_drift={self.CONT_CORRECT_MIN_DRIFT_DEG}°")

    def _publish_heading_now(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.heading_pub.publish(msg)

    # =========================================================================
    # 엔코더 콜백
    # =========================================================================
    def cb_encoder(self, msg: Int32):
        raw = float(msg.data)

        now_enc = time.time()
        self.encoder_last_rx_time = now_enc
        if self.heading_lock_last_time is None:
            enc_dt = self.ENC_DT
        else:
            enc_dt = max(0.001, min(now_enc - self.heading_lock_last_time, 0.1))
        self.heading_lock_last_time = now_enc

        if abs(raw) >= 1.0 and not self.encoder_active:
            self.encoder_active = True
            self.get_logger().info("✅ 엔코더 활성 — 실제 이동 감지, 헤딩 고정 허용")
        if raw > 0:
            self.current_speed_dir = 1.0
        elif raw < 0:
            self.current_speed_dir = -1.0
        speed_signed = (abs(raw) / self.TICKS_PER_REV) * self.WHEEL_CIRCUMFERENCE / self.ENC_DT
        speed_signed *= self.current_speed_dir

        if not self.is_heading_locked and abs(raw) >= 1.0:
            self.heading_lock_odom_dist += abs(speed_signed) * enc_dt

        with self._lock:
            self.current_speed_ms = speed_signed
            self.encoder_buf.append(speed_signed)
            if len(self.encoder_buf) > 5:
                self.encoder_buf.pop(0)

            # [v5.8] 레거시 DR(dr_active) 또는 연속 α-상보필터 모드면 위치 적분
            if not ((self.dr_active or self._pos_fusion_active)
                    and self.is_heading_locked and self.origin_lat is not None):
                return

            now = time.time()
            dt  = max(0.001, min(now - self.last_encoder_time, 0.1))
            self.last_encoder_time = now

            smooth_speed = sum(self.encoder_buf) / len(self.encoder_buf)

            if self.dr_active:
                # 레거시 DR: GPS 완전 두절 → 헤딩도 자이로 적분으로 유지
                corrected_gyro_z = self.raw_imu_gyro_z - self.gyro_z_bias
                self.heading += math.degrees(corrected_gyro_z) * dt
                self.heading = normalize_angle(self.heading)
            # [v5.8] α-상보필터 모드의 헤딩은 main_loop(IMU orientation+offset)가 담당
            head_rad = math.radians(self.heading)

            self.prev_fused_x = self.fused_x
            self.prev_fused_y = self.fused_y

            dx = smooth_speed * math.cos(head_rad) * dt
            dy = smooth_speed * math.sin(head_rad) * dt
            self.fused_x += dx
            self.fused_y += dy
            if self.dr_active:
                self.dr_accumulated_dist += math.hypot(dx, dy)

            self.fused_lat, self.fused_lon = self.xy_to_latlon(
                self.fused_x, self.fused_y, self.origin_lat, self.origin_lon)

    # =========================================================================
    # IMU 콜백
    # =========================================================================
    def cb_imu(self, msg: Imu):
        ox = msg.orientation.x
        oy = msg.orientation.y
        oz = msg.orientation.z
        ow = msg.orientation.w

        raw_yaw_wrapped = math.degrees(euler_from_quaternion(ox, oy, oz, ow))

        if self.prev_raw_imu_yaw is None:
            self.raw_imu_yaw_unwrap = raw_yaw_wrapped
        else:
            delta = normalize_angle(raw_yaw_wrapped - self.prev_raw_imu_yaw)
            self.raw_imu_yaw_unwrap += delta

        self.prev_raw_imu_yaw = raw_yaw_wrapped
        self.raw_imu_yaw = normalize_angle(self.raw_imu_yaw_unwrap)

        self.raw_imu_gyro_z = msg.angular_velocity.z

        # gyro_z bias 자동 추정
        if (abs(self.current_speed_ms) < self.GYRO_BIAS_LEARN_SPEED_MAX and
            abs(self.raw_imu_gyro_z) < self.GYRO_BIAS_LEARN_GYRO_MAX):
            self.gyro_z_bias += self.GYRO_BIAS_LPF_ALPHA * (
                self.raw_imu_gyro_z - self.gyro_z_bias)
            self.gyro_z_bias = max(-self.GYRO_BIAS_MAX_ABS,
                                   min(self.GYRO_BIAS_MAX_ABS, self.gyro_z_bias))

        # gyro_z LPF (bias 보정된 값)
        corrected_gz = self.raw_imu_gyro_z - self.gyro_z_bias
        alpha_g = 0.2
        self.gyro_z_abs_lpf += alpha_g * (abs(corrected_gz) - self.gyro_z_abs_lpf)

        raw_pitch = pitch_from_quaternion(ox, oy, oz, ow)

        self.pitch_buf.append(raw_pitch)
        if len(self.pitch_buf) > 20:
            self.pitch_buf.pop(0)
        avg_pitch = sum(self.pitch_buf) / len(self.pitch_buf)

        if not self.pitch_calibrated:
            elapsed = time.time() - self.pitch_calib_start
            self.pitch_calib_buf.append(raw_pitch)
            if elapsed >= self.PITCH_CALIB_SECS and len(self.pitch_calib_buf) > 10:
                self.pitch_offset = sum(self.pitch_calib_buf) / len(self.pitch_calib_buf)
                self.pitch_calibrated = True
                self.get_logger().info(
                    f"[Pitch 캘리브] 오프셋={self.pitch_offset:.2f}° "
                    f"({len(self.pitch_calib_buf)}샘플)")

        self.imu_pitch_deg = avg_pitch - self.pitch_offset

    def _update_gps_speed_estimate(self, curr_x: float, curr_y: float, now: float):
        if self._gps_speed_last_x is None:
            self._gps_speed_last_x = curr_x
            self._gps_speed_last_y = curr_y
            self._gps_speed_last_t = now
            return

        dt = now - self._gps_speed_last_t
        if dt <= 0.02:
            return

        dx = curr_x - self._gps_speed_last_x
        dy = curr_y - self._gps_speed_last_y
        dist = math.hypot(dx, dy)
        raw_abs = dist / max(dt, 1e-3)

        self._gps_speed_last_x = curr_x
        self._gps_speed_last_y = curr_y
        self._gps_speed_last_t = now

        if raw_abs > self.GPS_SPEED_MAX_VALID:
            return
        if raw_abs < self.GPS_SPEED_DEADBAND:
            raw_abs = 0.0

        if raw_abs > 0.0:
            gps_dir = math.degrees(math.atan2(dy, dx))
            heading_ref = self.heading if self.is_heading_locked else gps_dir
            sign = 1.0 if abs(normalize_angle(gps_dir - heading_ref)) <= 90.0 else -1.0
        else:
            sign = 1.0 if self.current_speed_dir >= 0 else -1.0

        raw_signed = raw_abs * sign
        alpha = max(0.05, min(0.65, dt / (self.GPS_SPEED_LPF_TAU + dt)))
        self.gps_speed_raw_ms = raw_signed
        self.gps_speed_ms += alpha * (raw_signed - self.gps_speed_ms)
        if abs(self.gps_speed_ms) < self.GPS_SPEED_DEADBAND:
            self.gps_speed_ms = 0.0

    # =========================================================================
    # GPS 콜백
    # =========================================================================
    def cb_fix(self, msg: NavSatFix):
        self.last_gps_time = time.time()
        new_status = msg.status.status

        if new_status != self.prev_gps_status:
            emoji_new  = GPS_STATUS_EMOJI.get(new_status, "?")
            emoji_prev = GPS_STATUS_EMOJI.get(self.prev_gps_status, "?")
            label_new  = GPS_STATUS_LABEL.get(new_status,          "UNKNOWN")
            label_prev = GPS_STATUS_LABEL.get(self.prev_gps_status, "UNKNOWN")

            sep = "=" * 54
            self.get_logger().warning(sep)
            self.get_logger().warning(f"  📡 GPS 상태 전환!")
            self.get_logger().warning(f"  이전: {emoji_prev} {label_prev}")
            self.get_logger().warning(f"  현재: {emoji_new} {label_new}")

            if (self.prev_gps_status >= GPS_STATUS_FIX
                    and new_status == GPS_STATUS_NO_FIX):
                if self._dr_fallback_enabled:
                    with self._lock:
                        self.dr_active           = True
                        self.last_encoder_time   = time.time()
                        self.dr_accumulated_dist = 0.0
                        self.gps_dropout_start   = time.time()
                        self.gps_fix_count       = 0
                    self.get_logger().error(
                        "  🚨 RTK 손실 → 엔코더+IMU gyro 추측항법 즉시 시작!")
                else:
                    # [v5.8] α-상보필터 모드: DR 적분이 이미 상시 가동 중이라 전환 불필요.
                    #        gps_raw 모드: 의도적으로 폴백 없음(위치는 마지막 fix에 정지).
                    self.gps_dropout_start = time.time()
                    self.gps_fix_count     = 0
                    self.get_logger().error(
                        f"  🚨 RTK 손실 (mode={self._fusion_mode}: DR 전환 없음, "
                        f"{'α-융합이 자동 흡수' if self._pos_fusion_active else '위치 갱신 정지'})")

            if (new_status >= GPS_STATUS_FIX
                    and self.prev_gps_status == GPS_STATUS_NO_FIX):
                lost = 0.0
                if self.gps_dropout_start is not None:
                    lost = time.time() - self.gps_dropout_start
                    self.gps_dropout_start = None
                with self._lock:
                    dr_dist = self.dr_accumulated_dist
                self.get_logger().info(
                    f"  ✅ RTK 복구! (단절={lost:.1f}초, DR누적={dr_dist:.2f}m)")

            self.get_logger().warning(sep)
            self.prev_gps_status = new_status

        if new_status >= GPS_STATUS_FIX:
            self.gps_fix_count   += 1
            self.gps_nofix_count  = 0
            self.is_rtk_fixed     = True
        else:
            self.gps_nofix_count += 1
            self.gps_fix_count    = 0
            if self.gps_nofix_count >= 5:
                self.is_rtk_fixed = False
        self.gps_status = new_status

        # NO_FIX 또는 비정상 좌표가 먼저 들어오면 기준점(origin)을 잡으면 안 된다.
        # 한 번 원점이 잘못 잡히면 이후 모든 local x/y, heading lock, mapping 기준이 틀어진다.
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        pos_finite = math.isfinite(lat) and math.isfinite(lon)

        if new_status == GPS_STATUS_NO_FIX or not pos_finite:
            if not pos_finite:
                self.get_logger().warning(
                    f"[GPS 좌표 무시] 비정상 좌표 lat={msg.latitude}, lon={msg.longitude}",
                    throttle_duration_sec=1.0)
            return

        if self.origin_lat is None:
            self.origin_lat = lat
            self.origin_lon = lon
            self.start_x = 0.0
            self.start_y = 0.0
            self.get_logger().info(
                f"[기준점] {self.origin_lat:.8f}, {self.origin_lon:.8f}")

        curr_x, curr_y = self.latlon_to_xy(
            lat, lon, self.origin_lat, self.origin_lon)

        # ── [v5.7] 안테나 → 뒷차축 투영 ──────────────────────────────────
        # rear = antenna − d·[cosψ, sinψ]  (헤딩 고정 후에만; 고정 전엔 원시 좌표)
        # /ego_state는 이 뒷차축 좌표를 발행 → PP·DR·mapping 기하 전부 정합.
        # [v5.8] gps_raw 모드는 투영 없이 안테나 원시좌표 발행(_project_enabled=False)
        if self.is_heading_locked and self._project_enabled:
            _hr = math.radians(self.heading)
            rear_x = curr_x - self.ANTENNA_AHEAD_REAR_M * math.cos(_hr)
            rear_y = curr_y - self.ANTENNA_AHEAD_REAR_M * math.sin(_hr)
        else:
            rear_x, rear_y = curr_x, curr_y

        now = time.time()
        # 속도추정도 투영좌표 사용: 뒷차축 진행방향≈heading → 선회 스윙에 의한
        # 속도 과대/부호 오판 제거
        self._update_gps_speed_estimate(rear_x, rear_y, now)

        # ── 초기 헤딩 고정 ────────────────────────────────────────────────
        if not self.is_heading_locked:
            dist = math.hypot(curr_x - self.start_x, curr_y - self.start_y)
            gps_heading = math.degrees(
                math.atan2(curr_y - self.start_y, curr_x - self.start_x)) if dist > 1e-6 else self.raw_imu_yaw
            if self.current_speed_ms < 0:
                gps_heading = (gps_heading + 180.0) % 360.0

            if dist >= 0.25:
                self.heading_lock_buf.append(gps_heading)
                if len(self.heading_lock_buf) > self.HEADING_LOCK_BUF_SIZE:
                    self.heading_lock_buf.pop(0)

            if len(self.heading_lock_buf) >= 3:
                base = self.heading_lock_buf[-1]
                heading_spread = max(abs(normalize_angle(h - base)) for h in self.heading_lock_buf)
            else:
                heading_spread = 999.0

            can_lock_by_encoder = (
                self.encoder_active and
                self.heading_lock_odom_dist >= self.HEADING_LOCK_ENCODER_DIST and
                abs(self.current_speed_ms) >= self.HEADING_LOCK_MIN_SPEED and
                dist >= self.HEADING_LOCK_DIST and
                heading_spread <= self.HEADING_LOCK_SPREAD_DEG
            )

            can_lock_by_gps_only = (
                dist >= self.GPS_ONLY_HEADING_LOCK_DIST and
                heading_spread <= self.GPS_ONLY_HEADING_SPREAD_DEG
            )

            if can_lock_by_encoder or can_lock_by_gps_only:
                lock_mode = "ENC+GPS" if can_lock_by_encoder else "GPS-only"
                self.yaw_offset        = normalize_angle(gps_heading - self.raw_imu_yaw_unwrap)
                self.heading           = gps_heading
                self.locked_heading    = gps_heading
                self.locked_heading_time = time.time()
                self.is_heading_locked = True
                self.get_logger().info(
                    f"[헤딩 고정:{lock_mode}] {self.locked_heading:.2f}°  "
                    f"(GPS이동={dist:.2f}m, ENC누적={self.heading_lock_odom_dist:.2f}m, "
                    f"속도={self.current_speed_ms:.2f}m/s, spread={heading_spread:.1f}°, "
                    f"IMU_unwrap={self.raw_imu_yaw_unwrap:.2f}°, offset={self.yaw_offset:.2f}°)")
            else:
                self.get_logger().info(
                    f"[헤딩 대기] GPS={dist:.2f}/{self.HEADING_LOCK_DIST:.2f}m "
                    f"ENC={self.heading_lock_odom_dist:.2f}/{self.HEADING_LOCK_ENCODER_DIST:.2f}m "
                    f"속도={abs(self.current_speed_ms):.2f}/{self.HEADING_LOCK_MIN_SPEED:.2f}m/s "
                    f"spread={heading_spread:.1f}/{self.HEADING_LOCK_SPREAD_DEG:.1f}°(ENC) / {self.GPS_ONLY_HEADING_SPREAD_DEG:.1f}°(GPS-only)",
                    throttle_duration_sec=1.0)

        with self._lock:
            if not self.is_heading_locked:
                # [v5.8] 헤딩 고정 전엔 모드 무관 GPS 직접 반영(초기 위치 확립, 기존과 동일)
                self.prev_fused_x = self.fused_x
                self.prev_fused_y = self.fused_y
                self.fused_x   = rear_x
                self.fused_y   = rear_y
                self.fused_lat, self.fused_lon = self.xy_to_latlon(
                    rear_x, rear_y, self.origin_lat, self.origin_lon)
            elif self._pos_fusion_active:
                # [v5.8] 연속 상보필터: cb_encoder의 DR 예측을 GPS로 α만큼 당김.
                #   α=0(dr_only)이면 위치에 GPS 불개입. 단절→복구도 이 식이 자동 흡수
                #   (복구 첫 fix부터 매 fix 오차의 α씩 기하수렴 → 별도 블렌딩 불필요).
                if self._pos_gps_alpha > 0.0:
                    self.prev_fused_x = self.fused_x
                    self.prev_fused_y = self.fused_y
                    self.fused_x += self._pos_gps_alpha * (rear_x - self.fused_x)
                    self.fused_y += self._pos_gps_alpha * (rear_y - self.fused_y)
                    self.fused_lat, self.fused_lon = self.xy_to_latlon(
                        self.fused_x, self.fused_y, self.origin_lat, self.origin_lon)
            elif self.dr_active:
                err = math.hypot(rear_x - self.fused_x, rear_y - self.fused_y)   # [v5.7] 뒷차축끼리 비교
                self.gps_blend_active = True
                self.gps_recovery_blend_active = True
                self.gps_blend_start  = now
                self.gps_blend_from_x = self.fused_x
                self.gps_blend_from_y = self.fused_y
                self.gps_blend_to_x   = rear_x
                self.gps_blend_to_y   = rear_y
                self.dr_active        = False
                self.get_logger().info(
                    f"[GPS 블렌딩 시작] 오차={err:.2f}m → {self.GPS_BLEND_DURATION}s 보정")
            elif not self.gps_blend_active:
                self.prev_fused_x = self.fused_x
                self.prev_fused_y = self.fused_y
                self.fused_x   = rear_x          # [v5.7] 뒷차축 좌표 발행
                self.fused_y   = rear_y
                self.fused_lat, self.fused_lon = self.xy_to_latlon(
                    rear_x, rear_y, self.origin_lat, self.origin_lon)

        # ── [v5.0 보수화] IMU drift 실시간 보정 ──────────────────────────
        # gyro_lpf < 12°/s 일 때만 작동 (실질적으로 직선 주행 구간)
        # [v5.7] 선회보상 모드에선 gyro 게이트 45°/s로 완화(코너에서도 보정 가능).
        #   근거: 안테나 속도벡터 v·û+ω·d·n̂ 모델의 잔차가 실측 0.1°±2.8° (전 구간 곡선 포함)
        _gyro_gate = (self.DRIFT_CORRECT_GYRO_MAX_SWING
                      if self.DRIFT_SWING_COMP_ENABLED else self.DRIFT_CORRECT_GYRO_MAX)
        gyro_ok = self.gyro_z_abs_lpf < _gyro_gate

        # [v5.8] trust=0이면 헤딩에 GPS 불개입(블록 전체 skip → offset 동결)
        if (self.is_heading_locked
                and self._heading_gps_trust > 0.0
                and now - self.last_drift_correct_time > self.DRIFT_CORRECT_INTERVAL
                and not self.gps_blend_active and not self.dr_active):

            if not gyro_ok:
                # 급회전 중 - drift 보정 skip
                self.drift_skipped_gyro += 1
                # 다음 interval은 정상적으로 다가오게 (skip 시 last_drift_correct_time 안 갱신)
            else:
                with self._lock:
                    # [v5.7] 이동벡터는 '안테나 원시 좌표'로 측정.
                    #   (투영좌표는 heading으로 만든 값 → 그걸로 heading을 보정하면 자기참조)
                    if self._ant_prev_x is None:
                        dx = dy = 0.0
                        t_prev = now - 0.2
                    else:
                        dx = curr_x - self._ant_prev_x
                        dy = curr_y - self._ant_prev_y
                        t_prev = self._ant_prev_t if self._ant_prev_t is not None else now - 0.2
                    spd = self.current_speed_ms
                move_dist = math.hypot(dx, dy)

                # 충분히 이동했고 속도도 있을 때만 (정지 중 GPS 노이즈로 인한 오보정 방지)
                if move_dist > 0.35 and abs(spd) > 0.50:
                    gps_dir = math.degrees(math.atan2(dy, dx))
                    # [v5.7.1] GPS 변위의 시간중심 − 솔루션지연 시점의 자세와 비교(위상 정렬)
                    #   실측: 정렬 전 drift가 요레이트×0.23s로 오염(코너 phantom ±7~10°)
                    t_ref = 0.5*(t_prev + now) - self.GPS_SOLUTION_LATENCY_S
                    hd_ref, gz_ref = self._att_at(t_ref)
                    if self.DRIFT_SWING_COMP_ENABLED:
                        # [v5.7] 기대 진행방향(안테나): vel = v·û(ψ) + ω·d·n̂(ψ)
                        #   전/후진 모두 부호가 식 안에서 자동 처리됨.
                        _hr = math.radians(hd_ref)
                        _d  = self.ANTENNA_AHEAD_REAR_M
                        exp_dir = math.degrees(math.atan2(
                            spd * math.sin(_hr) + gz_ref * _d * math.cos(_hr),
                            spd * math.cos(_hr) - gz_ref * _d * math.sin(_hr)))
                        drift = normalize_angle(exp_dir - gps_dir)
                    else:
                        # 후진 시에는 GPS 진행방향이 차량 뒷방향 → +180° 보정
                        if spd < 0:
                            gps_dir = normalize_angle(gps_dir + 180.0)
                        drift = normalize_angle(hd_ref - gps_dir)

                    if abs(drift) > self.DRIFT_CORRECT_THRESH_DEG:
                        force_reset = (
                            abs(drift) > self.DRIFT_FORCE_RESET_DEG
                            and move_dist > self.DRIFT_FORCE_RESET_DIST
                        )
                        if force_reset:
                            # [v5.7] 목표헤딩 = 현재헤딩 − drift (선회보상/비보상 공통 성립)
                            _new_head = normalize_angle(self.heading - drift)
                            self.yaw_offset = normalize_angle(_new_head - self.raw_imu_yaw_unwrap)
                            self.heading = _new_head
                            self.get_logger().warn(
                                f"[헤딩 강제 리셋] drift={drift:+.1f}° → {_new_head:.1f}°로 초기화 "
                                f"offset={self.yaw_offset:.2f}° move={move_dist:.2f}m")
                        else:
                            # [v5.4] gain 매우 약하게. 코너 중 GPS 진행방향을 실제 drift로 오인하지 않도록 보수화
                            # [v5.8] HEADING_GPS_TRUST로 반영 강도 스케일(1.0=현행)
                            gain = min(self.DRIFT_CORRECT_GAIN_MAX,
                                       self.DRIFT_CORRECT_GAIN_BASE
                                       + abs(drift) / self.DRIFT_CORRECT_GAIN_SCALE)
                            gain *= self._heading_gps_trust
                            correction = drift * gain
                            self.yaw_offset = normalize_angle(self.yaw_offset - correction)
                            self.drift_correct_count += 1
                            self.drift_correct_total_abs += abs(correction)
                            _mode = "선회보상" if self.DRIFT_SWING_COMP_ENABLED else "직선전용"
                            self.get_logger().info(
                                f"[Drift 보정·{_mode}] {drift:+.2f}° gain={gain:.3f} "
                                f"→ offset={self.yaw_offset:.2f}° "
                                f"(gyro_lpf={math.degrees(self.gyro_z_abs_lpf):.1f}°/s, "
                                f"누적={self.drift_correct_count}회 {self.drift_correct_total_abs:.2f}°, "
                                f"skip={self.drift_skipped_gyro}회)",
                                throttle_duration_sec=1.0)
                self.last_drift_correct_time = now

        # [v5.7] drift 측정용 안테나 원시좌표 갱신(매 유효 fix, 5Hz 스텝 변위 확보)
        self._ant_prev_x = curr_x
        self._ant_prev_y = curr_y
        self._ant_prev_t = now   # [v5.7.1] 변위 시간중심 계산용

        # 연속 헤딩 미세 보정 (직선만) — 투영좌표가 아닌 안테나 원시좌표로
        # (직선에선 안테나 진행방향 = 뒷차축 진행방향 = heading이므로 기존 로직 그대로 유효)
        if self.CONT_CORRECT_ENABLED:
            self._continuous_heading_correction(curr_x, curr_y, now)

    # =========================================================================
    # [v5.0 보수화] 연속 헤딩 미세 보정 - 정말 직선일 때만
    # =========================================================================
    def _att_at(self, t_ref):
        """[v5.7.1] t_ref 시점의 (heading°, gz rad/s) — 이력 선형보간(랩 안전).
        이력 부족 시 현재값 폴백."""
        h = self._att_hist
        if len(h) < 2:
            return self.heading, (self.raw_imu_gyro_z - self.gyro_z_bias)
        if t_ref <= h[0][0]:
            return h[0][1], h[0][2]
        if t_ref >= h[-1][0]:
            return h[-1][1], h[-1][2]
        for i in range(len(h)-1, 0, -1):
            t0, hd0, g0 = h[i-1]
            t1, hd1, g1 = h[i]
            if t0 <= t_ref <= t1:
                a = (t_ref - t0) / max(1e-6, (t1 - t0))
                dh = normalize_angle(hd1 - hd0)
                return normalize_angle(hd0 + a*dh), g0 + a*(g1 - g0)
        return self.heading, (self.raw_imu_gyro_z - self.gyro_z_bias)

    def _continuous_heading_correction(self, curr_x, curr_y, now):
        if not self.is_heading_locked:
            return
        if self._heading_gps_trust <= 0.0:   # [v5.8] trust=0 → 헤딩에 GPS 불개입
            return
        if self.gps_blend_active or self.dr_active:
            return

        self.cont_gps_buf.append((curr_x, curr_y, now))
        while len(self.cont_gps_buf) > self.CONT_GPS_WINDOW:
            self.cont_gps_buf.pop(0)

        if len(self.cont_gps_buf) < self.CONT_GPS_WINDOW:
            return

        if abs(self.current_speed_ms) < self.CONT_CORRECT_MIN_SPEED:
            return
        if self.current_speed_ms < 0:
            return
        # [v5.4] gyro 8°/s 이상이면 회전 중 → 절대 보정 안 함
        if self.gyro_z_abs_lpf > self.CONT_CORRECT_MAX_GYRO:
            return

        p_first = self.cont_gps_buf[0]
        p_last  = self.cont_gps_buf[-1]
        dx_total = p_last[0] - p_first[0]
        dy_total = p_last[1] - p_first[1]
        move_dist_total = math.hypot(dx_total, dy_total)
        if move_dist_total < self.CONT_MIN_GPS_DIST:
            return

        gps_motion_heading = math.degrees(math.atan2(dy_total, dx_total))

        segment_headings = []
        for i in range(1, len(self.cont_gps_buf)):
            sx = self.cont_gps_buf[i][0] - self.cont_gps_buf[i-1][0]
            sy = self.cont_gps_buf[i][1] - self.cont_gps_buf[i-1][1]
            if math.hypot(sx, sy) < self.CONT_MIN_SEG_DIST:
                continue
            segment_headings.append(math.degrees(math.atan2(sy, sx)))
        if len(segment_headings) < 3:
            return
        base = gps_motion_heading
        spread = max(abs(normalize_angle(h - base)) for h in segment_headings)
        # [v5.4] 정말 직선일 때만 (spread 3° 이내)
        if spread > self.CONT_LINEARITY_MAX_SPREAD:
            return

        # [v5.7.1] 창의 시간중심 − 솔루션지연 시점의 헤딩과 비교(위상 정렬)
        t_ref = 0.5*(p_first[2] + p_last[2]) - self.GPS_SOLUTION_LATENCY_S
        hd_ref, _ = self._att_at(t_ref)
        drift = normalize_angle(hd_ref - gps_motion_heading)
        abs_drift = abs(drift)

        if abs_drift < self.CONT_CORRECT_MIN_DRIFT_DEG:
            return
        if abs_drift >= self.CONT_CORRECT_MAX_DRIFT_DEG:
            return

        # [v5.8] HEADING_GPS_TRUST로 반영 강도 스케일(1.0=현행)
        correction = drift * self.CONT_CORRECT_GAIN * self._heading_gps_trust
        self.yaw_offset = normalize_angle(self.yaw_offset - correction)

        self.cont_correct_count += 1
        self.cont_correct_total += abs(correction)

        if now - self.last_cont_log_time > 3.0:
            self.last_cont_log_time = now
            self.get_logger().info(
                f"[연속미세보정] gps_dir={gps_motion_heading:+.2f}° "
                f"head={self.heading:+.2f}° drift={drift:+.2f}° "
                f"→ −{correction:+.3f}° "
                f"(win={move_dist_total:.2f}m spread={spread:.1f}° "
                f"누적={self.cont_correct_count}회 {self.cont_correct_total:.2f}°)")

    # =========================================================================
    # 메인 루프 (20Hz)
    # =========================================================================
    def main_loop(self):
        now = time.time()
        loop_dt = max(0.02, min(0.15, now - self.prev_main_time))
        self.prev_main_time = now

        gps_age = now - self.last_gps_time
        if gps_age > 1.5 and self.is_rtk_fixed:
            if self._dr_fallback_enabled:
                self.get_logger().error(f"[GPS 단절] {gps_age:.1f}s 미수신 → DR 전환")
                with self._lock:
                    self.is_rtk_fixed        = False
                    self.dr_active           = True
                    self.gps_dropout_start   = now
                    self.last_encoder_time   = now
                    self.dr_accumulated_dist = 0.0
            else:
                # [v5.8] α-상보필터 모드: DR 적분 상시 가동 중 → 전환 불필요.
                #        gps_raw 모드: 폴백 없음(위치 갱신 정지).
                self.get_logger().error(
                    f"[GPS 단절] {gps_age:.1f}s 미수신 (mode={self._fusion_mode}: DR 전환 없음)")
                with self._lock:
                    self.is_rtk_fixed      = False
                    self.gps_dropout_start = now

        # GPS 복구 블렌딩
        with self._lock:
            if self.gps_blend_active:
                alpha  = min(1.0, (now - self.gps_blend_start) / self._gps_blend_duration)
                smooth = 0.5 * (1.0 - math.cos(math.pi * alpha))
                self.fused_x = self.gps_blend_from_x + (self.gps_blend_to_x - self.gps_blend_from_x) * smooth
                self.fused_y = self.gps_blend_from_y + (self.gps_blend_to_y - self.gps_blend_from_y) * smooth
                self.fused_lat, self.fused_lon = self.xy_to_latlon(
                    self.fused_x, self.fused_y, self.origin_lat, self.origin_lon)
                if alpha >= 1.0:
                    was_recovery = self.gps_recovery_blend_active
                    self.gps_blend_active = False
                    self.gps_recovery_blend_active = False
                    self._gps_blend_duration = self.GPS_BLEND_DURATION
                    if was_recovery:
                        self.get_logger().info(
                            "[GPS 블렌딩 완료] ✅ RTK 정밀 주행 완전 복귀!")

        # /heading 발행
        drift  = normalize_angle(self.heading - self.locked_heading) if self.locked_heading is not None else 0.0
        locked = self.locked_heading if self.locked_heading is not None else 0.0
        hdg_msg = Float64MultiArray()
        hdg_msg.data = [locked, self.heading, drift]
        self.heading_pub.publish(hdg_msg)

        # 지형 판정
        p = self.imu_pitch_deg
        prev_t = self.terrain_code
        if self.terrain_code == 0.0:
            if   p > self.PITCH_ENTRY_UP:   self.terrain_code = 1.0
            elif p < self.PITCH_ENTRY_DOWN:  self.terrain_code = -1.0
        elif self.terrain_code == 1.0:
            if p < self.PITCH_EXIT_UP:       self.terrain_code = 0.0
        elif self.terrain_code == -1.0:
            if p > self.PITCH_EXIT_DOWN:     self.terrain_code = 0.0

        if self.terrain_code != prev_t:
            lbl = {0.0:"평지 ➡", 1.0:"오르막🔺", -1.0:"내리막🔻"}
            self.get_logger().info(
                f"[지형] {lbl[prev_t]} → {lbl[self.terrain_code]}  pitch={p:.2f}°")

        terrain_msg = Float64MultiArray()
        terrain_msg.data = [self.imu_pitch_deg, self.terrain_code]
        self.terrain_pub.publish(terrain_msg)

        # gps_status는 헤딩 고정 전에도 계속 발행한다.
        # 기존 코드는 heading lock 전 return 때문에 sensor_monitor에서 GPS 상태가 안 보일 수 있었다.
        e   = GPS_STATUS_EMOJI.get(self.gps_status, "?")
        lbl = GPS_STATUS_LABEL.get(self.gps_status, "UNKNOWN")
        dr_str    = " [DR중]"   if self.dr_active       else ""
        blend_str = " [GPS복구중]" if self.gps_blend_active else ""
        self.gps_st_pub.publish(String(data=f"{e} {lbl}{dr_str}{blend_str}"))

        if not self.is_heading_locked or self.origin_lat is None:
            return

        # ── 헤딩 업데이트 (bias 보정된 raw_yaw + offset) ─────────────────
        if not self.dr_active:
            raw_heading = normalize_angle(self.raw_imu_yaw_unwrap + self.yaw_offset)
            if self.is_heading_locked:
                max_delta = self.HEADING_MAX_RATE_DPS * loop_dt
                delta = normalize_angle(raw_heading - self.heading)
                if abs(delta) > max_delta:
                    self.heading = normalize_angle(
                        self.heading + math.copysign(max_delta, delta))
                    self.get_logger().warn(
                        f"[IMU 급변 제한] Δ={delta:.1f}°/step → {max_delta:.1f}°로 클램프 "
                        f"(raw={raw_heading:.1f}° cur={self.heading:.1f}°)")
                else:
                    self.heading = raw_heading
            else:
                self.heading = raw_heading

        # [v5.7.1] 자세 이력 축적(GPS 코스 시간정렬용) — DR 중에도 heading은
        #   cb_encoder가 갱신하므로 매 틱 기록해 두면 복귀 직후 보정도 정확.
        self._att_hist.append((now, self.heading,
                               self.raw_imu_gyro_z - self.gyro_z_bias))

        # ego_state 발행
        with self._lock:
            fl, flo = self.fused_lat, self.fused_lon
            fx, fy  = self.fused_x,   self.fused_y
            enc_fresh = (self.encoder_last_rx_time > 0.0 and
                         (now - self.encoder_last_rx_time) <= self.ENCODER_FRESH_TIMEOUT)
            spd = self.current_speed_ms if enc_fresh else self.gps_speed_ms

        state = Float64MultiArray()
        state.data = [fl, flo, fx, fy, self.heading,
                      float(spd), 0.0,
                      float(self.imu_pitch_deg), float(self.terrain_code)]
        self.ego_pub.publish(state)

        if now - self.last_status_print_time >= 5.0:
            self._print_status()
            self.last_status_print_time = now

    def _print_status(self):
        gps_age = time.time() - self.last_gps_time
        e   = GPS_STATUS_EMOJI.get(self.gps_status, "?")
        lbl = GPS_STATUS_LABEL.get(self.gps_status, "UNKNOWN")

        if self.dr_active:
            status_extra = f" ⚠️ DR중 (누적={self.dr_accumulated_dist:.1f}m)"
        elif self.gps_blend_active:
            status_extra = " 🔄 GPS복구 블렌딩중"
        else:
            status_extra = ""

        drift = normalize_angle(self.heading - self.locked_heading) if self.locked_heading is not None else 0.0
        drift_warn = (" ⚠️ 큰drift!" if abs(drift)>30 else
                      " ⚠️ drift주의" if abs(drift)>10 else "")
        t_lbl = {0.0:"평지 ➡", 1.0:"오르막🔺", -1.0:"내리막🔻"}.get(self.terrain_code, "평지")
        calib = f"오프셋={self.pitch_offset:.2f}°" if self.pitch_calibrated else "캘리브중..."

        sep = "─" * 54
        self.get_logger().info(sep)
        self.get_logger().info("  📊 [센서 대시보드]  (5초 갱신)")
        self.get_logger().info(sep)
        self.get_logger().info(f"  📡 GPS : {e} {lbl}{status_extra}")
        self.get_logger().info(f"  ⏱  수신 : {gps_age:.1f}초 전")
        self.get_logger().info(sep)
        hdg_str = (f"{self.locked_heading:.2f}°  (기준={self.HEADING_LOCK_DIST}m)"
                   if self.locked_heading is not None else f"⚠️ 미고정 ({self.HEADING_LOCK_DIST}m 이동필요)")
        self.get_logger().info(f"  🔒 고정헤딩: {hdg_str}")
        self.get_logger().info(
            f"  🧭 현재헤딩: {self.heading:.2f}°  (drift={drift:+.2f}°{drift_warn})")
        self.get_logger().info(
            f"  🔢 IMU raw : yaw={self.raw_imu_yaw:.2f}° unwrap={self.raw_imu_yaw_unwrap:.2f}° "
            f"offset={self.yaw_offset:.2f}°  "
            f"gyro={math.degrees(self.raw_imu_gyro_z):+.1f}°/s "
            f"(lpf={math.degrees(self.gyro_z_abs_lpf):.1f}°/s, "
            f"bias={math.degrees(self.gyro_z_bias):.3f}°/s)")
        self.get_logger().info(
            f"  🔧 Drift보정: {self.drift_correct_count}회 / {self.drift_correct_total_abs:.2f}° "
            f"(코너skip={self.drift_skipped_gyro}회)")
        self.get_logger().info(
            f"  🔧 연속미세보정: {self.cont_correct_count}회 / {self.cont_correct_total:.2f}°")
        self.get_logger().info(sep)
        self.get_logger().info(
            f"  ⛰  지형 : {t_lbl}  pitch={self.imu_pitch_deg:.2f}°  ({calib})")
        self.get_logger().info(
            f"     히스테리시스: 진입±{self.PITCH_ENTRY_UP}°  탈출±{self.PITCH_EXIT_UP}°")
        enc_fresh = (self.encoder_last_rx_time > 0.0 and (time.time() - self.encoder_last_rx_time) <= self.ENCODER_FRESH_TIMEOUT)
        spd_print = self.current_speed_ms if enc_fresh else self.gps_speed_ms
        src_print = "ENC" if enc_fresh else "GPS-est"
        self.get_logger().info(f"  🚗 속도 : {spd_print:.3f} m/s ({src_print})")
        self.get_logger().info(f"  📍 위치 : ({self.fused_lat:.8f}, {self.fused_lon:.8f})")
        self.get_logger().info(sep)

    def latlon_to_xy(self, lat, lon, ref_lat, ref_lon):
        R = 6378137.0
        x = math.radians(lon - ref_lon) * R * math.cos(math.radians(ref_lat))
        y = math.radians(lat - ref_lat) * R
        return x, y

    def xy_to_latlon(self, x, y, ref_lat, ref_lon):
        R = 6378137.0
        lat = ref_lat + (y / R) * (180.0 / math.pi)
        lon = ref_lon + (x / (R * math.cos(math.radians(ref_lat)))) * (180.0 / math.pi)
        return lat, lon


def main(args=None):
    rclpy.init(args=args)
    node = GpsImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()