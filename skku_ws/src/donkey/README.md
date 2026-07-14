# donkey — 수집 · 학습 · 실행 3단계 모방학습 주행

Donkeycar 방식(수집→학습→실행)을 benz.ino 1/10카에 맞춘 패키지.
아두이노 시리얼은 **mega 노드 하나만** 담당하고, 나머지는 /in·/out 토픽으로 통신한다.
Windows 11 / Ubuntu 22.04 겸용 (cv_bridge·YOLO 불사용).

## 노드/토픽 구조

```
                    /in (String, 아두이노로 보낼 라인)
  joystick ──────────────▶ ┌──────┐
   (수집 단계 조종)          │ mega │──시리얼──▶ benz.ino (Mega2560, 115200)
  drive    ──────────────▶ └──────┘
   (실행 단계 자율주행)          │ /out (String, "초음파6개 현재조향각", 20Hz — mega만 발행)
                              ▼
                    collect_lane / collect_all (수집 시 라벨 기록용)
```

```
donkey/
  donkey/
    mega.py           아두이노↔ROS2 유일 시리얼 브릿지 (/in 구독 → 전송, /out 20Hz 발행)
    joystick.py       [1.수집] 마우스 조종 (기존 joystick_sonar.py 방식, 초음파 표시 없음) → /in 발행
    collect_lane.py   [1.수집-차선] 허프변환 차선 특징(각도·기울기 등)만 기록
    collect_all.py    [1.수집-전체] 카메라 프레임 전체(jpg)를 기록
    collect_base.py     └ 공통 골격 (카메라 + /in·/out 구독 + CSV)
    drive.py          [3.실행] 학습 결과물 로드 → /in으로 "<PWM> <각도>" 발행
    lane_detect.py    허프 차선특징 — 수집·주행 공유
    models.py         LaneNet(MLP) / AllNet(CNN) — 학습·주행 공유
    common.py         상수·정규화·세션 넘버링·시리얼포트/카메라 헬퍼
  train.py            [2.학습] ROS2 무관 단독 스크립트 (tkinter 폴더 선택)
  launch/             collect_lane / collect_all / run
  data/               lane_001, all_001 ...   (수집 결과, 자동 넘버링)
  trained/            train_001 ...           (model.pt + meta.json)
```

## 학습 라벨 규칙

- **조향 = 아두이노 실측 조향각** (`cur_angle`, 가변저항 환산값 — /out 텔레메트리 7번째 필드)
- **주행 = joystick의 주행모터 PWM 지시값** (`cmd_pwm` — /in 첫 필드)
- joystick의 조향 필드는 시간모드 조향PWM이라 라벨로 쓰지 않는다.

## 카메라 정책 — C920E 전용

카메라는 **로지텍 HD 웹캠 C920E(VU0060)만** 사용한다. 장치를 인덱스가 아니라 **이름**으로
찾으므로(기본 힌트 `c920,vu0060`), PC 내장캠이나 '휴대폰과 연결' 가상 카메라(예: "S25 Ultra
(Windows 가상 카메라)")가 실수로 열리는 일이 없다. **C920E가 연결돼 있지 않으면 다른 카메라로
폴백하지 않고 발견된 장치 목록을 보여주며 에러로 멈춘다.**
(비상시 인덱스 강제: 노드 파라미터 `camera_name:=''` + `camera_index:=N`)

## 0. 준비 — 클린 Python 3.8 환경 기준 설치 명령

전제: ROS2 Humble이 이미 설치되어 있고(`rclpy`/`std_msgs`는 ROS2가 제공, pip 아님),
아래 pip은 **rclpy가 사용하는 파이썬**에 설치해야 한다
(Windows는 `C:\Python38\python.exe`, Ubuntu 22.04 Humble은 시스템 python3).

### Windows 11 (Python 3.8)

```powershell
C:\Python38\python.exe -m pip install numpy==1.24.4 opencv-python pyserial torch==2.4.1 pygrabber==0.1
```

- `numpy==1.24.4` : Python 3.8을 지원하는 마지막 numpy
- `torch==2.4.1`  : Python 3.8을 지원하는 마지막 PyTorch (CPU 빌드 — 추론에 충분)
- `pygrabber==0.1`: 카메라 장치이름 열거용, **Windows 전용** (0.2는 Python 3.9+라 설치 불가.
  의존성 comtypes는 자동 설치됨)
- `opencv-python`, `pyserial`: 핀 불필요 (pip이 3.8 호환 버전을 자동 선택)
- tkinter(joystick·train.py의 GUI)는 python.org 설치본에 기본 포함 — 별도 설치 없음

### Ubuntu 22.04

```bash
sudo apt install python3-tk                 # tkinter (joystick 조종창 + train.py 폴더선택창)
sudo usermod -aG dialout $USER              # 시리얼 포트 권한 (재로그인 필요)
python3 -m pip install numpy==1.24.4 opencv-python pyserial torch==2.4.1
```

- `pygrabber`는 설치하지 않는다 — Ubuntu는 `/sys/class/video4linux`로 카메라를 열거하므로 불필요
- 참고: Ubuntu 22.04의 ROS2 Humble은 시스템 Python 3.10에 연결되므로, 그 경우 3.8용 버전 핀
  (`numpy==1.24.4`, `torch==2.4.1`)은 없어도 된다(`pip install numpy opencv-python pyserial torch`).
  핀은 "Python 3.8 환경"일 때만 필수.

### 검증된 조합 (이 저장소 개발 PC, Python 3.8.3 실측)

| 패키지 | 버전 |
|---|---|
| numpy | 1.24.4 |
| opencv-python | 4.13.0.92 |
| pyserial | 3.5 |
| torch | 2.4.1 (CPU) |
| pygrabber (Windows) | 0.1 (+comtypes 1.4.12) |

설치 확인:
```
python -c "import numpy, cv2, serial, torch, tkinter; print('OK')"
python -c "from pygrabber.dshow_graph import FilterGraph; print(FilterGraph().get_input_devices())"   # Windows만
```

## 빌드

```powershell
# Windows 빌드 (관리자 또는 개발자모드 — symlink, 빌드 때만)
$env:PYTHONUTF8='1'; . C:\humble\setup.ps1
cd C:\humble\workspace\skku_ws
colcon build --symlink-install --packages-select donkey
. .\install\setup.ps1
```
```bash
# Ubuntu 빌드
source /opt/ros/humble/setup.bash
cd ~/skku_ws && colcon build --symlink-install --packages-select donkey
source install/setup.bash
```

## 1. 수집 — joystick으로 조종하며 기록

차(USB)와 웹캠 연결 후 둘 중 하나 실행 (mega+joystick+수집노드가 함께 뜬다):

```
ros2 launch donkey collect_lane.launch.py    # → data/lane_001/log.csv (차선 특징만)
ros2 launch donkey collect_all.launch.py     # → data/all_001/images/ + log.csv (전체 프레임)
```

전체화면 조종 창: **좌/우클릭**=조향(누르는 동안) **휠**=속도± **휠버튼**=전·후진 전환 **아무 키**=조종 종료.
휠을 올려 출발(주행PWM≠0)하면 기록이 시작된다. 곡선을 충분히, 양방향으로 (3,000행+ 권장).
수집 종료: 조종 창에서 아무 키 → 창 닫힘 → 터미널에서 Ctrl+C.

## 2. 학습 — ROS2 무관 단독 실행

```
cd C:\humble\workspace\skku_ws\src\donkey     (Ubuntu: ~/skku_ws/src/donkey)
python train.py                # 폴더 선택 창에서 data/lane_XXX 또는 all_XXX 선택
python train.py data/all_001   # GUI 생략하고 직접 지정
```
epoch 수 등은 `train.py` 상단 파라미터 블록에서 수정. 결과: `trained/train_001/` (model.pt + meta.json).

## 3. 실행 — 즉시 출발하므로 학습한 위치에 차를 놓고!

```
ros2 launch donkey run.launch.py                    # 최신 train_XXX 자동 선택
ros2 launch donkey run.launch.py train:=train_001
ros2 launch donkey run.launch.py max_pwm:=100       # 첫 검증은 저속으로
```
meta.json의 타입에 따라 차선특징→LaneNet / 전체프레임→AllNet이 자동 선택된다.
정지: Ctrl+C (drive가 "0 0" 발행 + mega도 종료 시 "0 0" 전송). 카메라 0.5초 끊겨도 자동 정지.
주행 중 상태 확인: `ros2 topic echo /out` (초음파 6개 + 현재 조향각).

## 자주 겪는 것

- **포트 자동감지 실패** → `serial_port:=COM13` (Ubuntu: `/dev/ttyACM0`)
- **Ubuntu에서 시리얼 Permission denied** → `sudo usermod -aG dialout $USER` 후 재로그인
- **"카메라를 찾지 못함" 에러** → C920E USB 연결 확인 (에러 메시지에 현재 발견된 장치 목록이 나옴)
- **조향 캘리브레이션** → 아두이노 IDE 시리얼 모니터에서 수동 수행 (이 패키지는 관여하지 않음)
- **차선 특징이 전부 0(미검출)** → 카메라 각도에서 차선이 ROI(640x480 기준 300~380행) 밖.
  `donkey/lane_detect.py` 상단 `ROI_START_ROW/ROI_END_ROW` 조정 후 **재수집**.
- **lane vs all 선택** → 같은 트랙에서 각각 수집·학습·주행해보고 잘 되는 쪽 채택.
