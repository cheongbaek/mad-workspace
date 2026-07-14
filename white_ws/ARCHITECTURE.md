# white 패키지 — GPS 경로추종 자율주행 아키텍처

> 대상: `white_ws/src/white` (ROS2 Humble, Ubuntu 22.04)
> 최종 갱신: 2026-07-09 (gps_imu v5.8 융합 노브 추가 시점)

## 1. 전체 구조 (노드/토픽 그래프)

```
[하드웨어]           [드라이버 노드]              [융합/제어 노드]                  [액추에이터]

u-blox RTK GPS ──▶ nmea_serial_driver ──/fix──────────┐
(5Hz, USB)         (외부 패키지)                       │
                                                      ▼
iAHRS IMU ───────▶ iahrs_node ──/imu/data──────▶ gps_imu_node ──/ego_state──┬▶ driving_node ──/cmd_vel_raw──▶ motor_node ──▶ Arduino Mega
(20Hz, USB)        (iahrs.py)                    (gps_imu.py)               │  (driving.py)                    (motor.py)     (조향서보+구동모터)
                                                  │  │  │                   └▶ mapping_node                        ▲
Arduino Mega ────▶ motor_node ──/encoder─────────┘  │  │                       (mapping.py)                        │
(엔코더 100Hz)                                       │  └─/heading, /terrain_state, /gps_status                     │
                                                    │                                                             │
사용자 터미널 ────▶ prompt_node ──/mapping_cmd──▶ mapping_node                                                      │
                   (prompt.py)  ──/drive_cmd───▶ driving_node                                                      │
                                ──/control_state──────────────────────────────────────────────────────────────────┘
모니터링: sensor_monitor_node (sensor_monitor.py) — /fix, /imu/data, /encoder, /ego_state, /gps_status 구독 대시보드
```

실행: `ros2 launch white one_launch.py` — USB VID/PID로 GPS/Arduino/IMU 포트 자동 탐색 후 전 노드 기동.
사용자 조작은 별도 터미널에서 `ros2 run white prompt` (1=매핑, 2=주행, 3=경로목록, 4=종료).

## 2. 노드별 역할

| 노드 (파일) | 역할 | 입력 | 출력 |
|---|---|---|---|
| `nmea_serial_driver` (외부) | RTK GPS NMEA → NavSatFix | 시리얼 | `/fix` (5Hz) |
| `iahrs_node` (iahrs.py) | iAHRS IMU 드라이버. 쿼터니언 자세+각속도 발행, TF 브로드캐스트 | 시리얼 | `/imu/data` (기본 20Hz, `sync_period_ms` 파라미터로 50Hz 가능) |
| `motor_node` (motor.py) | Arduino Mega 브리지. Twist→`C,틱속도,조향각` 시리얼 명령(20Hz), 엔코더 수신. RX watchdog는 경고만(강제 리셋 금지) | `/cmd_vel_raw`, `/control_state` | `/encoder` (Int32, 틱) |
| `gps_imu_node` (gps_imu.py) | **로컬라이제이션 허브.** GPS+IMU+엔코더 융합 → 자기 상태 발행 (상세 §3) | `/fix`, `/imu/data`, `/encoder` | `/ego_state` (20Hz), `/heading`, `/terrain_state`, `/gps_status` |
| `mapping_node` (mapping.py) | 매핑 모드 중 `/ego_state`를 5Hz CSV 기록 → 종료 시 원본+리모델링(등간격 0.2m/스무딩) 경로 저장 | `/ego_state`, `/mapping_cmd`, `/cmd_vel_raw` | `~/white_ws/gps_data/route_*.csv`, `*_remodeled.csv` |
| `driving_node` (driving.py) | **경로추종 제어.** CSV 웨이포인트 로드 → Pure Pursuit(주) + PID(보조) 조향, 곡률/CTE 기반 속도계획 (상세 §4) | `/ego_state`, `/drive_cmd`, `/encoder`, `/imu/data`, `/terrain_state`, `/gps_status` | `/cmd_vel_raw`, `/drive_status`, `/drive_event`, `/driving_debug`, `/control_state` |
| `prompt_node` (prompt.py) | 터미널 메뉴 UI. 매핑 시작/종료, 주행 파일 선택/시작/정지, 모터 제어권한 부여 | 키보드 | `/mapping_cmd`, `/drive_cmd`, `/control_state` |
| `sensor_monitor_node` | 센서 상태 통합 대시보드 (launch 인자 `use_monitor`로 on/off) | 주요 토픽 전부 | 콘솔 출력 |

### 핵심 토픽 계약

- **`/ego_state`** (Float64MultiArray, 20Hz) = `[lat, lon, x, y, heading°, speed_m/s, 0, pitch°, terrain_code]`
  — driving/mapping은 **lat/lon(절대 위경도)과 heading, speed만 사용**. x/y는 gps_imu 내부 원점 기준 로컬 좌표(디버그용).
- **`/cmd_vel_raw`** (Twist) = `linear.x`: 목표속도 m/s, `angular.z`: 조향각 deg.
- 경로 CSV 컬럼: `latitude, longitude, heading, speed, steer, …` — 위경도 절대좌표라 원점이 달라도 호환.

## 3. gps_imu_node — 융합 구조 (v5.8)

철학: **칼만필터가 아닌 역할 분담 + 상보 보정.** 위치는 GPS 우위, 헤딩은 IMU 우위.

### 3.1 헤딩 (IMU 주도, GPS가 천천히 교정)

```
heading = IMU_yaw_unwrap + yaw_offset
```

1. **초기 헤딩 고정(lock)**: 출발 후 GPS 이동벡터로 절대방위 확립.
   ENC+GPS 조건(이동 1.0m + 엔코더 0.6m + 속도 0.2m/s + spread ≤15°) 또는 GPS-only 조건(2.0m + spread ≤5°).
2. **gyro bias 자동 학습**: 정지 중에만 LPF(α=0.002)로 학습. DR 헤딩 품질의 기반.
3. **선회보상 drift 보정** (1초 간격): 기대 진행방향 `atan2(v·sinψ+ω·d·cosψ, v·cosψ−ω·d·sinψ)`(안테나 스윙 모델)과 GPS 코스를 비교해 yaw_offset을 게인 0.04~0.10으로 미세 조정. 자세 이력(2.4s) 선형보간으로 GPS 지연(0.13s+변위 시간중심)을 위상 정렬(v5.7.1).
4. **연속 미세보정**: 확실한 직선에서만(7-fix 창, spread ≤4°, gyro ≤10°/s) 게인 0.04의 2차 채널.
5. **강제 리셋**: drift >60° && 이동 >1.5m일 때만 GPS 코스로 스냅.

실효 GPS→헤딩 수렴 시정수 ≈ 10~25초 (IMU 우위 설정).

### 3.2 위치 (GPS 주도, 단절 시 DR)

- RTK 정상: GPS 안테나 좌표를 **뒷차축으로 투영**(d=0.865m, PP/DR 기하 정합) 후 그대로 발행.
- GPS 단절(NO_FIX 또는 1.5s 미수신): **DR**(엔코더 속도 × heading 적분, 뒷차축 킨매틱) 즉시 전환.
- 복구: DR→GPS 3초 코사인 블렌딩.

### 3.3 [v5.8] 융합 비율 튜닝 노브 — `gps_imu.py` 클래스 최상단

| 노브 | 범위 | 의미 |
|---|---|---|
| `FUSION_MODE` | `"fused"` / `"gps_raw"` / `"dr_only"` | 프리셋. gps_raw=GPS 원시 바이패스(투영·DR폴백·헤딩보정 강제 OFF), dr_only=초기 lock 후 순수 DR. fused면 아래 노브 값 그대로 사용 |
| `POS_GPS_ALPHA` | 0.0~1.0 | 위치 융합비. **1.0=기존 동작 그대로**(레거시 경로). <1.0이면 DR 적분 상시 가동 + 매 fix `fused += α(GPS−fused)` 연속 상보필터. 수렴 시정수 ≈ 0.2s/α |
| `HEADING_GPS_TRUST` | 0.0~2.0 권장 | 헤딩 보정(3.1의 ③④) 반영 강도 스케일. 1.0=현행, 0=GPS 불개입(offset 동결) |
| `PROJECT_TO_REAR_AXLE` | bool | 안테나→뒷차축 투영(0.865m) on/off. False=v5.7 이전 동작 재현 |
| `DR_FALLBACK_ENABLED` | bool | α=1.0(레거시)일 때 GPS 단절→DR 전환+복구 블렌딩 on/off |
| `GPS_SOLUTION_LATENCY_S` | 초 | GPS 코스 시간정렬 지연(실측 0.13). 0=시간정렬 해제(v5.7 동작 재현) |

α<1.0 모드에서는 DR 전환/블렌딩 머신을 쓰지 않고 α-수렴이 단절·복구를 자동 흡수한다. 초기 헤딩 lock 전에는 모드와 무관하게 기존 동작(GPS 직접 반영).

### 3.4 부가 기능

- GPS 변위 기반 속도 추정(엔코더 0.5s 두절 시 /ego_state 속도 폴백)
- 피치 캘리브레이션(시작 2초) + 지형 판정(±3° 진입/±1.5° 탈출 히스테리시스) → `/terrain_state`
- 헤딩 급변 클램프 200°/s

## 4. driving_node — 경로추종 제어 (v6.7.x)

20Hz 제어 루프. 조향 = **Pure Pursuit(기여 ~94%) + PID(보조 ~6%, 정상상태 쏠림 제거)**.

### 조향 체인
1. **상태 예측**: 조향 지연 실측 0.45s만큼 요레이트로 자세를 앞으로 예측(ctrl_lat/lon), WP탐색·CTE·PP 전 채널에 적용.
2. **LFD(전방주시거리)**: 속도 테이블(2.3~5.5m) + 곡률캡 `√(1.44·R_ahead)` 중 작은 값. 조회용 속도는 LPF(τ0.3s), LFD는 비대칭 슬루(감소 빠르게/증가 완만).
3. **Pure Pursuit**: `δ = atan(2L·sinα / LFD)`, L=0.73m(휠베이스).
4. **PID**: 속도별 게인 테이블(Kp 1.6~2.6, Ki 0.12~0.25, Kd 0.14~0.30, 선형보간). 적분 anti-windup 4단(부호반전 ×0.35 소프트리셋 / 데드밴드 내 ×0.80 / 클램프 ±6.0 / 포화근접 ×0.85 누설). PID 단독 상한 9°.
5. **후처리**: 직선 잔떨림 억제 → 스무딩+슬루레이트 → ±20° 클램프 → 저속 조향권한 램프 → 발행식 `δ_pub = (δ_ctrl − TRIM)/GAIN` (트림 −1.5°, 실효게인 0.85 실측 보정).

### 속도 체인
근/원거리 요구조향 기반 감속 → ω_n 결속(v ≤ Ω·LFD/√2) → CTE 회복 감속 → DR 감속 → 속도 PI 미세보정(가속 22%/감속 6% 비대칭) → 출발 램프 → 가감속 슬루레이트.

## 5. 운용 절차

1. `ros2 launch white one_launch.py` — 전 노드 기동, GPS RTK FIX 대기.
2. `ros2 run white prompt` → **1 매핑**: 수동 주행하며 경로 기록 → Enter로 저장(`route_*.csv` + `*_remodeled.csv` 자동 생성).
3. prompt → **2 주행**: `*_remodeled.csv` 선택 → 차량이 출발 후 수 m 이내에서 헤딩 lock → 자율주행. Enter로 정지.
4. 분석: `ros2 bag record`로 `/ego_state /driving_debug /heading /fix …` 기록 → tool/ 스크립트로 CTE/헤딩 분석.

## 6. 개발 관례

- 모든 튜닝은 **로스백 정량 분석 → 파라미터/모델 수정 → 재주행 검증** 사이클로 진행하며, 근거를 파일 헤더 주석에 버전 이력으로 남긴다 (gps_imu v5.x, driving v6.x 이력 참조).
- 좌표 규약: 로컬 x=동, y=북, heading=atan2(dy,dx) [deg, -180~180]. 위경도↔로컬 변환은 equirectangular 근사(수 km 이내 유효).
- 차량 기하: 휠베이스 0.73m, GPS 안테나=뒷차축+0.865m, 바퀴둘레 0.8482m, 엔코더 300틱/회전.
- v5.7 이후 맵은 **뒷차축 경로**로 기록됨 — 그 이전(안테나 기준) 맵과 혼용 금지, 재매핑 권장.
