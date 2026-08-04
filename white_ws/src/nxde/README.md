# nxde — 하드웨어 계층 (white 자율주행 스택용)

> Ubuntu 22.04 / ROS2 Humble 전용. Windows 분기는 모두 제거했다.
> **`g.launch.py` 하나가 모든 하드웨어의 연결·통신을 전담한다.**
> `kasa_ws/src/nxde` 에서 아두이노 통신 노드만 가져와 토픽 계약을 white 규약으로 바꾸고,
> 여기에 white 에 있던 IMU 드라이버와 GPS·카메라 런치를 모아 하드웨어 계층으로 만들었다.
> **kasa_ws 쪽은 수정하지 않았다.** 아두이노 펌웨어(`kasa_0730_A.ino` / `kasa_0804_B.ino`)도 무수정.

---

## 1. 구조

```
┌─ 터미널 1 : ros2 launch nxde g.launch.py ─ 하드웨어 계층 ────────────────────┐
│  arduino              kasa A/B 2보드 (인휠 PID / 조향·제동)                  │
│  iahrs                iAHRS IMU          → /imu/data (+TF)                  │
│  nmea_serial_driver   u-blox RTK GPS     → /fix                             │
│  usb_cam (+v4l2-ctl)  USB 카메라         → /image_raw                        │
└─────────────────────────────────────────────────────────────────────────────┘
┌─ 터미널 2 : ros2 launch white one_launch.py ─ 자율주행 계층 ─────────────────┐
│  gps_imu  mapping  driving  perception  camera_judgment  sensor_monitor     │
└─────────────────────────────────────────────────────────────────────────────┘
┌─ 터미널 3 : ros2 run white prompt ─ CLI 상호작용 ────────────────────────────┐
│  1 수집(매핑)[수동조종]  2 주행[자율주행]  3 경로목록  5~8 비교실험  4 종료    │
└─────────────────────────────────────────────────────────────────────────────┘

  ※ 검증 단계에서는 터미널 2·3 대신 `ros2 run nxde master` (GUI) 를 쓴다 — 3절 참고.
    ★둘을 동시에 띄우면 /cmd_vel_raw 발행자가 겹친다★
```

두 런치는 서로를 모르지만 같은 `ROS_DOMAIN_ID`(기본 0)면 토픽으로 자동 연결된다.
패키지 경계는 빌드 단위일 뿐이고, 통신은 DDS 가 토픽 이름·타입·QoS 로만 맺는다.
**기동 순서는 상관없다** — g.launch.py 는 장치가 없어도 뜨고, one_launch.py 는 토픽이 없어도 뜬다.

### kasa_ws 원본과의 차이

| | kasa_ws 원본 | 이 패키지 |
|---|---|---|
| 범위 | 아두이노 통신만 | **모든 하드웨어** (아두이노 + IMU + GPS + 카메라) |
| ROS 인터페이스 | `/in`·`/out`·`/info` (String) | white 규약 토픽 직접 (Twist/Bool/Int32) |
| 조종 입력 | master(GUI)·joystick·csv_read·keyboard | **없음** — 자율은 white 의 driving, 수동은 물리 페달·핸들 |
| 좌우 차동 | master.py 가 계산 | 하지 않음 (좌우 동일 펄스) |
| 직접 PWM(16~255) | master 의 PWM모드로 사용 가능 | **봉쇄** — A보드로 항상 단일값만 보낸다 |
| 최초 연결 | 생성자에서 블로킹 | **논블로킹** (백그라운드 스레드) |
| 플랫폼 | Windows COM* + POSIX | POSIX(`/dev/ttyACM*`·`/dev/ttyUSB*`)만 |

파일 구성:

```
nxde/arduino.py     A/B 2보드 브리지 (구 white/motor.py 대체)
nxde/iahrs.py       iAHRS IMU 드라이버 (구 white/white/iahrs.py 이관)
nxde/ports.py       ★USB 장치 식별의 단일 소유자★ (udev 링크 → VID/PID → 폴백)
nxde/proc_guard.py  부모 프로세스 사망 감지 (고아 방지, POSIX 전용판)
launch/g.launch.py  전 하드웨어 런치
calibration/usb_cam_calibration.yaml   usb_cam 의 camera_info
```

`clean.py`(psutil 의존)는 가져오지 않았다 — 그 파일이 대응하는 고아 프로세스 문제는
Windows 특유의 `.EXE` 래퍼 구조에서 발생하고 POSIX 에는 없다(`proc_guard.py` 헤더 참고).
잔재가 남으면 `pkill -f nxde.arduino` 또는 `fuser -k /dev/ttyACM*` 로 정리한다.

---

## 2. ★ 연결 실패 / 도중 단절 대응 ★

**장치가 하나도 안 꽂혀 있어도 이 런치는 정상 기동한다.** 각 노드가 자기 장치를 계속
다시 찾으므로, 나중에 꽂거나 도중에 뺐다 꽂아도 자동으로 붙는다.

| 노드 | 최초 실패 | 도중 단절 | 수단 |
|---|---|---|---|
| `arduino` | 백그라운드 재스캔 (3s) | 재스캔 — **한쪽만 빠져도 나머지는 계속 동작** | 자체 `_link_loop` 스레드 |
| `iahrs` | 2s 재시도 + VID/PID 재탐색 | 재시도 + 재탐색 | 자체 재연결 타이머 |
| GPS | respawn (3s) | respawn | ★udev 링크 필요★ |
| `usb_cam` | respawn (3s) | respawn | ★`video_device` 경로 고정★ |

- `arduino` 는 **생성자가 블로킹하지 않는다.** 예전에는 두 보드를 다 찾을 때까지
  `__init__` 안에서 돌아 노드가 spin 조차 못 했다(구독·`/board_status` 전부 죽어 있었다).
- 전송 실패는 곧 포트 단절로 보고 그 보드만 떨어뜨린다. 그때 **변경감지 캐시를 비우므로**
  재연결 직후 최신 명령이 즉시 다시 나간다(명령 유실 없음).
- GPS·카메라는 외부 패키지라 노드 코드를 고칠 수 없어 `respawn` 에 의존한다.
  respawn 은 **같은 파라미터로 프로세스를 다시 띄우는 것**이라 장치 경로가 안정적일 때만
  복구된다 → **6절의 udev 설정이 사실상 필수다.**

---

## 3. 실행

```bash
ros2 launch nxde g.launch.py                       # 기본
ros2 launch nxde g.launch.py use_camera:=false     # 카메라 없이 (GPS 단독 주행 시)
ros2 launch nxde g.launch.py cam_exposure:=10      # 주간 (기본 120 은 야간 기준)
ros2 launch nxde g.launch.py manual_pulse_max:=3   # ★수집 주행 권장 — 페달 상한 억제★
```

### ⚠️ 종료 순서 : **이 런치를 먼저 내린다**

A보드 펌웨어에는 무입력 타임아웃이 없다(0713에서 제거). 마지막 수신 명령을 계속 물고 있다.

- **이 런치를 Ctrl+C** → arduino 노드가 종료 직전에 정지값(`0` / `x,0`)을 시리얼로 직접
  써 넣는다(`stop_and_close`). 차가 선다. **안전**
- **one_launch.py 만 내림** → `/cmd_vel_raw` 가 끊길 뿐이고 arduino 는 마지막 명령을
  1초 주기로 계속 재전송한다. **차가 계속 간다**

급할 때는 E-stop 스위치를 쓴다.

### ★ 하드웨어 검증 GUI : `ros2 run nxde master` ★

판단 스택을 올리기 전에 **차가 실제로 움직이는지** 확인하는 도구다.

```bash
ros2 launch nxde g.launch.py use_camera:=false manual_pulse_max:=3   # 터미널 1
ros2 run nxde master                                                 # 터미널 2
```

`kasa_ws` 의 master GUI 와 레이아웃·조작이 거의 같다. 차이는 세 가지뿐이다:
**① 컨트롤러(조이스틱) 모드 제거** (무선 컨트롤러를 쓰지 않는다)
**② 통신이 white 규약 토픽** (`/in`·`/out` String 대신 `/cmd_vel_raw`+`/control_state`)
**③ 디퍼렌셜·PWM모드 체크박스 제거** (arduino 노드가 A보드로 항상 단일값만 보낸다)

- **자율주행 모드** : 마우스(또는 키보드 ↑↓←→)로 엑셀·조향 레버를 움직이고, 발행 토글을
  ON 으로 두면 차가 움직인다.
- **수동조종 모드** : 레버가 잠기고 **실측을 비추는 계기판**이 된다 — 페달을 밟으면 엑셀
  레버가 올라가고(`/drive_pulse_cmd`), 핸들을 돌리면 조향 레버가 움직인다
  (`/steer_angle_measured`). "밟히는 게 보인다".
- **E-stop** 이 걸리면 상단이 빨간 `E-Stop 발동!!!` 으로 바뀐다.

⚠️ **`one_launch.py`(driving_node) / `prompt` 와 동시에 쓰지 말 것** — `/cmd_vel_raw` 와
`/control_state` 의 발행자가 겹쳐 두 명령이 교대하며 차가 떤다. master 는 `driving_node`
가 떠 있으면 상단에 주황색 경고를 띄운다.

체크할 것:

| 확인 항목 | 기대 결과 |
|---|---|
| 조향 레버를 **왼쪽(음수 아님, `+`쪽)** 으로 | 바퀴가 **왼쪽**으로. 반대면 `steer_invert:=false` |
| 명령 조향각 ↔ 실측 조향각 | 몇 초 안에 수렴 (B보드 PD 폐루프) |
| 엑셀 1~3펄스 | 실측 주행펄스(좌+우 합)가 명령의 **약 2배**로 올라온다 |
| 수동조종에서 페달 | 엑셀 레버가 따라 올라오고, 브레이크가 풀린다 |
| E-stop 스위치 | 상단 빨간 경고 + 실측값 정지 |

### 토픽으로 직접 확인 (GUI 없이)

```bash
ros2 topic pub /control_state std_msgs/Bool "{data: true}"
ros2 topic pub -r 10 /cmd_vel_raw geometry_msgs/Twist \
  "{linear: {x: 1.0}, angular: {z: 10.0}}"         # 1펄스 전진 + 좌조향 10°
ros2 topic echo /board_status                      # A:1,B:1,ESTOP:0,MODE:1
ros2 topic echo /encoder
```

---

## 4. 토픽 계약

### ROS → 보드

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/cmd_vel_raw` | `geometry_msgs/Twist` | `linear.x` = 주행 목표펄스 **0~15 (m/s 아님)**<br>`angular.z` = 조향각 **−40~40, white 부호(+좌/−우)** |
| `/control_state` | `std_msgs/Bool` | `True` = 구동 허용 / `False` = 정지 |

### 보드 → ROS

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/encoder` | `Int32` | A보드 **좌+우 펄스의 합** (부호 없음, 20ms 창). 1카운트 = 0.442 m/s |
| `/steer_angle_measured` | `Int32` | B보드 가변저항 실측 조향각. **white 부호로 변환해서** 발행 |
| `/vehicle_mode` | `Bool` | B보드 D5 : `True` 자율주행 / `False` 수동조종 |
| `/throttle_pedal` | `Int32` | A보드 A0 쓰로틀 페달 raw 0~1023 |
| `/drive_pulse_cmd` | `Int32` | **A보드로 실제 나간 주행 목표펄스** (자율=계획값 / 수동=페달 환산값) |
| `/estop` | `Bool` | A·B 중 한쪽이라도 `STOP` 송신 중이면 `True` (OR) |
| `/board_status` | `String` | `"A:1,B:1,ESTOP:0,MODE:1"` — 진단·로스백용 |

### 센서

| 토픽 | 발행 노드 |
|---|---|
| `/imu/data` (+ TF `base_link→imu_link`) | `iahrs` |
| `/fix` | `nmea_serial_driver` |
| `/image_raw` | `usb_cam` |

`/motor_pwm`·`/steer_pwm` 은 **발행하지 않는다** — kasa 펌웨어가 PWM 을 텔레메트리로
내보내지 않는다. white 쪽 구독자도 없었으므로(로스백 진단 전용) 그냥 사라진다.

### 시리얼 (참고 — arduino 노드 내부에서만 쓰인다)

```
A보드 입력 :  <펄스>\n                  0~15 만 (단일값 = 펄스 전용 경로)
A보드 출력 :  S,<좌펄스>,<우펄스>,<쓰로틀raw>\n     (50ms) / STOP\n (e-stop 중)
B보드 입력 :  <조향각>,<브레이크단계>\n   조향각 정수 −40~40 또는 'x'(힘빼기)
                                        브레이크 단계 0/1/2 (★0~255 PWM 아님★)
B보드 출력 :  P,<조향각>,<모드>\n         (50ms) / STOP\n (e-stop 중)
```

---

## 5. 주행 상태 판단 (우선순위)

`arduino.py` 의 `compose()` 가 매 전송 주기에 아래 순서로 판정한다.

| 우선 | 조건 | A보드 | B보드 |
|---|---|---|---|
| 1 | **E-stop** (`STOP` 수신) | `0` | `x,0` |
| 2 | **수동조종** (D5 개방) | 페달 환산 펄스 | `x,<2→0>` |
| 3 | `/control_state=False` | `0` | `<마지막 조향각>,<stop_brake_level>` |
| 4 | 정상 자율주행 | `<펄스>` | `<조향각>,0` |

- **E-stop** : 리니어 2단 체결과 해제(0단 복귀)는 **B보드 펌웨어가 스스로 한다**
  (`kasa_0804_B.ino [0804-3]`). ROS 가 브레이크를 지시할 필요가 없고, e-stop 중에는
  B보드 `handleLine` 이 명령을 통째로 무시한다. `x,0` 을 보내는 이유는 **해제 직후에
  적용될 마지막 명령**을 안전하게 두기 위함이다(조향 힘빼기 = 급조향 없음).
- **수동조종** : 사람이 핸들·페달을 직접 잡는다. `/control_state` 와 **무관하게 항상** 이
  경로다 — 자율 명령을 보내면 사람과 싸운다. **즉 g.launch.py 만 떠 있어도 수동주행이 된다.**
  - 조향 `x`(힘빼기) — DC모터에 힘이 들어가면 사람이 핸들을 못 돌린다
  - 주행은 A보드가 보고한 페달 raw 를 펄스로 환산해 되돌려 보낸다
  - 브레이크는 **자율→수동 전환 엣지에서 2단(정지 확보) → 쓰로틀 페달을 밟으면 0단(해제)**.
    리니어가 브레이크 페달을 물고 있으면 사람이 출발할 수 없으므로 반드시 풀어야 한다.
    해제 임계는 `manual_release_raw`(기본 240, 실측 놓음 177 / 최대 800).
    ※ **노드 시작 직후에는 2단을 걸지 않는다** — 그때 `auto_mode` 는 페일세이프로 `False`
      인데, 그것만 보고 체결하면 B보드 텔레메트리도 받기 전에 주차된 차의 페달을 밟는다.
- **`/control_state=False`** : 조향각을 0 으로 리셋하지 않고 마지막 값을 유지한다
  (정지 순간 바퀴가 정면으로 튀는 것을 막는다). 기본값은 코스트(`stop_brake_level=0`)로
  white/motor.py 의 `S,0` 과 같은 동작이다. 더 빨리 세우려면 `1` 로 올린다.

---

## 6. ★ udev 설정 (사실상 필수) ★

GPS·IMU·아두이노 A/B 가 **전부 `/dev/ttyACM*`·`/dev/ttyUSB*` 대역을 공유한다.**
`/dev/ttyUSB0` 같은 열거 순서 의존 경로를 쓰면 재부팅·재연결 때 다른 장치를 열 수 있고,
GPS·카메라는 respawn 이 고정 경로로 재시도하므로 경로가 흔들리면 복구되지 않는다.

```bash
# 1) 각 장치의 시리얼 번호 확인 (하나씩 꽂아가며)
udevadm info -a -n /dev/ttyACM0 | grep -m1 'ATTRS{serial}'

# 2) /etc/udev/rules.d/99-white.rules 작성
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", SYMLINK+="gps"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="imu"
# 아두이노는 A/B 가 같은 VID/PID 이므로 시리얼 번호로 구분한다(역할 식별은 접두어가 하므로
# 링크 이름이 뒤바뀌어도 무해하다 — 탐색 범위를 좁히는 최적화일 뿐이다)
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{serial}=="<A보드 시리얼>", SYMLINK+="kasa_a"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{serial}=="<B보드 시리얼>", SYMLINK+="kasa_b"

# 3) 적용
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/gps /dev/imu           # 심볼릭링크 확인

# 4) dialout 그룹 (권한 오류 시)
sudo usermod -aG dialout $USER    # 재로그인 필요
```

`nxde/ports.py` 의 `resolve_device()` 가 **① udev 링크 → ② VID/PID 스캔 → ③ 링크 경로
그대로 반환** 순서로 동작한다. ③ 이 중요하다 — 장치가 아직 없어도 **안정적인 이름**을
넘겨두면 나중에 꽂는 순간 그 경로가 생기고 respawn·재연결이 자동으로 붙는다.

아두이노는 udev 링크가 없어도 동작한다. GPS/IMU 포트는 `exclude_ports` 파라미터로
탐색에서 제외되며(g.launch.py 가 자동으로 넘긴다), 역할 식별은 첫 텔레메트리 접두어
(`S,`=A / `P,`=B)로 한다 — A/B 두 대가 같은 VID/PID 라서 VID/PID 로는 구분할 수 없다.

카메라는 `/dev/video*` 라 udev 규칙이 다르다. 여러 대를 꽂는다면
`v4l2-ctl --list-devices` 로 확인해 `video_device:=/dev/video2` 처럼 지정한다.

---

## 7. 파라미터

### 하드웨어 경로 / 카메라

| 이름 | 기본 | 설명 |
|---|---|---|
| `gps_port` | udev→VID/PID→`/dev/gps` | GPS 시리얼 경로 |
| `imu_port` | udev→VID/PID→`/dev/imu` | iAHRS 시리얼 경로 |
| `imu_sync_period_ms` | `50` | IMU 출력주기. `20`(50Hz)이면 driving 지연보상 예측이 정밀해진다 |
| `use_camera` | `true` | usb_cam 기동 여부. **white one_launch.py 의 `use_camera` 와 맞출 것** |
| `video_device` | `/dev/video0` | 카메라 V4L2 경로 |
| `cam_exposure` | `120` | 수동노출. 기본은 야간 기준 — 주간엔 `10` 정도 |

### 아두이노

| 이름 | 기본 | 설명 |
|---|---|---|
| `baud` | 115200 | A/B 공통 |
| `steer_invert` | `true` | white(+좌) ↔ kasa B보드(+우) 부호 반전 |
| `stop_brake_level` | `0` | `/control_state=False` 시 브레이크 단계 (0=코스트 / 1=약) |
| `manual_brake_level` | `2` | 수동조종 진입 시 브레이크 단계 |
| `manual_release_raw` | `240` | 위 브레이크를 풀 쓰로틀 raw 임계 |
| `manual_pulse_max` | `15` | 수동조종에서 페달 최대치가 대응할 펄스. **수집·초기 시험에서는 3~5 로 낮출 것** |
| `throttle_raw_min` / `_max` | `177` / `800` | 페달 실측 (2026-07-30) |
| `exclude_ports` | g.launch.py 자동 | 아두이노 탐색에서 제외할 경로(GPS/IMU) |

---

## 8. 수집(매핑) 절차 — ★수동조종 모드★

무선 컨트롤러는 더 이상 쓰지 않는다. 사람이 차에 타서 **실제 페달과 핸들로** 몰고,
그때의 실계측을 `mapping` 노드가 CSV 로 기록한다.

```bash
# 터미널 1                          # 페달 상한을 낮춰 두면 수집 주행이 안전하다
ros2 launch nxde g.launch.py manual_pulse_max:=3
# 터미널 2
ros2 launch white one_launch.py
# 터미널 3
ros2 run white prompt      →  차량 D5 스위치를 '수동조종' 으로  →  메뉴 1 선택
```

`prompt` 가 모드를 강제한다 — 자율주행 모드에서 `1`(수집)을 고르면 "스위치를 수동조종으로
전환하세요" 안내만 띄우고 메뉴로 돌아간다. 반대로 `2`(주행)는 자율주행 모드에서만 된다.
E-stop 이 걸린 동안에는 둘 다 막힌다.

기록되는 수동조종 실계측 3종 (`route_*.csv` 뒤쪽 컬럼):

| 컬럼 | 토픽 | 의미 |
|---|---|---|
| `throttle_pulse` | `/drive_pulse_cmd` | ① 페달 raw → 환산된 주행 목표펄스 (0~15) |
| `wheel_pulse` / `wheel_speed` | `/encoder` | ② 실제로 돈 주행 펄스(좌+우 합) 와 그 m/s 환산 |
| `steer_measured` | `/steer_angle_measured` | ③ DC 조향모터 가변저항 실측 각도 [deg, +좌] |
| `throttle_raw` | `/throttle_pedal` | 부수 : A0 페달 원값 0~1023 |
| `auto_mode` / `estop` | `/vehicle_mode` `/estop` | **수집 유효구간 판별용** — 1 인 행은 사람 조작이 아니다 |

기존 `steer` 컬럼(열 위치 5)도 ③ 의 값으로 채운다 — 예전에는 `/ego_state[6]`(gps_imu 가
항상 0 을 넣는 미사용 필드)에서 읽어 **늘 0.00 이었다.** 열 위치는 `route_remodeler` 와
구버전 분석툴 호환을 위해 그대로 유지했다.

---

## 9. 알려진 한계

- **명령 분해능** : A보드 목표가 정수 펄스라 1펄스 = 0.884 m/s(3.18 km/h)다.
  `max_speed_ms=2.65` 에서 사용 가능한 단계가 1·2·3 세 개뿐이다. 상세는
  `white/kasa_units.py` 헤더 참고.
- **측정 공백** : 펄스 필드는 '직전 20ms 창'의 카운트인데 보고는 50ms 마다다 →
  50ms 중 30ms 는 계측되지 않는다. `gps_imu` 의 DR 거리적분이 그만큼 거칠다.
  근본 해결은 A보드 텔레메트리에 **누적 펄스 카운터** 필드를 추가하는 것인데,
  펌웨어 무수정 방침이라 적용하지 않았다.
- **저속 양자화** : 20ms 창에서 1~2펄스면 ±1펄스가 값의 50~100%다(`CLAUDE.md` 11절).
  좌+우 합을 쓰면 눈금이 절반(0.442 m/s)이 되지만 원리적 한계는 남는다.
- **`usb_cam_ctrl` 은 respawn 대상이 아니다** — 카메라가 respawn 되면 v4l2-ctl 설정이
  다시 적용되지 않는다. 노출이 이상해지면 `g.launch.py` 안의 그 명령을 손으로 한 번 돌린다.
