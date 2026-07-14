# skku_ws — benz.ino 1/10카 모방학습 차선주행

카메라(로지텍 HD 웹캠) 기반 차선주행을 **모방학습(behavior cloning)** 으로 구현하는
단일 패키지(`cnn`) 워크스페이스. Windows / Ubuntu 겸용 (cv_bridge, YOLO 불사용).

학습 입력 2가지를 만들어 실차에서 비교 후 잘 되는 쪽을 택한다:
- **Variant A (raw)**: 카메라 화면 전체 → CNN(PilotNet 스타일)
- **Variant B (feature)**: 허프변환 차선특징(7차원) → MLP

두 variant 모두 멀티모달 — 이미지/특징에 **[현재 조향각(시리얼 피드백), 직전 명령 각도, 직전 명령 PWM]** 상태 3차원을 함께 입력.
출력은 둘 다 `[조향각(-30~30°), 주행PWM]` — benz.ino의 각도 PD 모드(`"<PWM> <각도>\n"`)로 바로 전송된다.

## 구성

```
src/cnn/
  cnn/
    camera_node.py        웹캠 → /image_raw (640x480@30, cv2.VideoCapture)
    benz_driver.py        시리얼 브릿지: /drive_cmd → "<PWM> <각도>", 텔레메트리 → /steer_angle, /sonar
    teleop_keyboard.py    수집용 키보드 조종 (W/S/A/D 증분, Space 정지, Q 종료)
    dataset_recorder.py   /image_raw 프레임마다 이미지+라벨(labels.csv) 저장
    lane_features.py      허프변환 차선특징 추출 (학습·추론 공유 — Variant B)
    models.py             RawPilotNet / FeatureMLP + 정규화 (학습·추론 공유)
    raw_infer_node.py     Variant A 주행
    feature_infer_node.py Variant B 주행
    ros_image_codec.py    cv_bridge 대체 (Image ↔ numpy)
  launch/  collect_data / drive_raw / drive_feature
  training/  dataset.py, train_raw.py, train_feature.py  (ROS 빌드 무관, 오프라인)
```

## 빌드

```powershell
# Windows (관리자 터미널 또는 개발자모드 — symlink 때문. 빌드 때만 필요)
$env:PYTHONUTF8='1'          # 필수! (한글경로 cp949 사고 방지)
. C:\humble\setup.ps1
cd C:\humble\workspace\skku_ws
colcon build --symlink-install
. .\install\setup.ps1
```
```bash
# Ubuntu 22.04
source /opt/ros/humble/setup.bash
cd ~/skku_ws && colcon build --symlink-install
source install/setup.bash
```
의존성(pip): `torch`, `opencv-python`, `numpy`, `pyserial`, `pynput` — rclpy가 쓰는 파이썬에 설치.

## 사용 순서

### 1. 데이터 수집 (트랙에서 사람이 조종)
```
터미널1> ros2 launch cnn collect_data.launch.py            # 카메라+시리얼+레코더
터미널2> ros2 run cnn teleop_keyboard                      # W/S=속도 A/D=조향 Space=정지 Q=종료
```
W로 출발하는 순간 `~/imitation_data/session_*/`에 자동 녹화 시작.
직진 위주가 되지 않게 곡선 구간을 충분히, 양방향으로 (권장 3,000프레임+).

### 2. 학습 (오프라인, 같은 PC)
```
cd src/cnn/training
python train_raw.py     --data ~/imitation_data/session_XXXX --out models/raw_model.pt
python train_feature.py --data ~/imitation_data/session_XXXX --out models/feature_model.pt
```
세션 여러 개면 `--data 세션1 세션2 ...`. `val_loss`가 안 내려가면 과적합 — epoch을 줄이거나 데이터 추가.

### 3. 자율주행 (두 방식 비교)
```
ros2 launch cnn drive_raw.launch.py     model_path:=C:/.../raw_model.pt
ros2 launch cnn drive_feature.launch.py model_path:=C:/.../feature_model.pt
```
`max_pwm:=100` 처럼 상한을 낮춰 저속으로 먼저 검증할 것.

## 안전장치
- benz_driver: `/drive_cmd` 0.5초 끊기면 `0 0`(정지) 자동 전송, 종료 시에도 정지
- 추론 노드: 카메라 프레임 0.5초 끊기면 정지 발행, PWM 상한(`max_pwm`)·후진 금지 기본
- teleop: Q/ESC 종료 시 정지 명령 후 종료

## 자주 쓰는 파라미터
| 어디 | 파라미터 | 기본 | 설명 |
|---|---|---|---|
| collect/drive 공통 | `serial_port` | auto | Mega 자동감지 실패 시 `COM13`/`/dev/ttyACM0` 지정 |
| 〃 | `camera_index` | 0 | 웹캠이 여러 개면 변경 |
| drive_* | `max_pwm` | 150 | 자율주행 속도 상한 |
| feature 계열 | `lane_features.DEFAULT_PARAMS` | 640x480 기준 | 트랙/카메라 각도에 맞게 ROI·Canny 조정 (학습·추론 동일하게!) |
