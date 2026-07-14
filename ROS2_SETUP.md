# ROS2 개발환경 정리 (Windows 11)

> 최종 갱신: 2026-07-09

## 1. ROS2 설치 위치

- **ROS2 Humble이 `C:\humble`에 설치되어 있음** (pixi 기반 배포판)
- Python은 시스템 공용 **Python 3.8.3** 사용 (`rclpy` 포함 정상 동작 확인)
- 환경 소싱:
  ```powershell
  . C:\humble\setup.ps1        # PowerShell
  C:\humble\setup.bat          # cmd
  ```

## 2. 매 터미널 필수 절차

```powershell
$env:PYTHONUTF8='1'                  # ★ 필수: 한글 경로 + cp949 문제 방지
. C:\humble\setup.ps1                # ROS2 Humble 소싱
cd C:\humble\workspace\kasa_ws
. .\install\setup.ps1                # 워크스페이스 오버레이 소싱 (빌드 후)
```

- `PYTHONUTF8=1` 없이 colcon build 시 **한글 경로가 cp949로 깨져 전역 Python에 잘못 설치되는 사고**가 실제로 발생했음. 반드시 설정할 것.
- PowerShell **Start-Job(백그라운드 잡) 안에서 소싱하면 한글 경로가 깨져** 패키지를 못 찾음. 노드 실행/테스트는 일반(동기) 셸에서 할 것.

## 3. `--symlink-install` — Windows에서 되는가?

**결론: 명령어는 Ubuntu와 완전히 동일하고 추가 설치도 필요 없다. 단, symlink 생성 "권한"이 있어야 한다.**

| 항목 | 내용 |
|---|---|
| 명령 | `colcon build --symlink-install` (Ubuntu와 동일) |
| 추가 설치 | 불필요 (colcon 기본 기능) |
| 전제 조건 | 아래 둘 중 하나 필요 |
| 방법 A | **개발자 모드 ON**: 설정 → 개인 정보 및 보안 → 개발자용 → "개발자 모드" 켜기 |
| 방법 B | 터미널을 **관리자 권한**으로 실행 |

- Windows는 심볼릭 링크 생성에 특수 권한(`SeCreateSymbolicLinkPrivilege`)이 필요함.
  일반 사용자에게는 없고, **개발자 모드를 켜면 일반 사용자도 생성 가능**해짐.
- **2026-07-09 실측 경과**:
  - 개발자 모드 OFF + 일반 권한 → **WinError 1314**로 실패 확인.
  - **VS Code를 관리자 권한으로 재실행 후 → `colcon build --symlink-install` 성공.**
    - `install → build → src`까지 심링크 체인 확인 (launch 파일, 노드 .py 모두)
    - `import nxde1.talker_k` 의 realpath가 `src\nxde1\nxde1\talker_k.py` 로 해석됨
    - → **src 파이썬/launch 수정이 재빌드 없이 즉시 반영되는 상태**
- 주의: 심링크 빌드는 **빌드할 때만** 관리자 권한이 필요. 실행(ros2 launch)은 일반 권한으로도 됨.
  단, 일반 권한 터미널에서 재빌드하면 다시 1314로 실패하므로, 재빌드는 관리자 터미널에서 하거나
  개발자 모드를 켜서 권한 문제를 영구 해결할 것.
- ~~참고: 워크스페이스가 OneDrive 동기화 폴더 안에 있으므로 symlink 동기화 주의~~
  → **2026-07-09 워크스페이스를 `C:\humble\workspace\kasa_ws`로 이전하여 해당 우려 해소됨.**

## 4. kasa_ws 워크스페이스 현황

- 경로: `C:\humble\workspace\kasa_ws` (2026-07-09 OneDrive의 `mad_control\code\ros2_test\kasa_ws`에서 이전)
- 패키지: `nxde1` (ament_python)
- 빌드: `$env:PYTHONUTF8='1'; . C:\humble\setup.ps1; colcon build --symlink-install`
  (**현재 symlink-install로 빌드되어 있음** — src 수정 즉시 반영, 재빌드는 관리자 터미널 필요)

### 빌드 검증 결과 (2026-07-09 재확인, 모두 통과)

| 검증 항목 | 결과 |
|---|---|
| `ros2 pkg list` | `nxde1` 인식됨 |
| `ros2 pkg executables nxde1` | `talker_k`, `walker_k`, `listener_k` 3개 등록 |
| `ros2 launch nxde1 k.launch.py --print` | 3개 노드 launch 구성 정상 해석 |
| install 산출물 | `talker_k.py`, `walker_k.py`, `listener_k.py`, `in.csv`, `k.launch.py` 설치 확인 |
| talker_k 실구동 | in.csv 6행을 순서·유지시간대로 `/in` 발행 확인 |

### 노드 구성

| 노드 | 역할 |
|---|---|
| `talker_k` | `in.csv`를 읽어 순서대로 `/in` 토픽 발행 (행 형식: 유지시간,주행펄스,조향각도,브레이크,모드) |
| `walker_k` | `/in` 구독 → 시리얼(기본 **COM13, 115200**)로 아두이노에 전달, 아두이노 출력을 `/out`으로 발행 |
| `listener_k` | `/out` 구독 → **1초 간격으로만** 수용해 `out.csv`에 덮어쓰기 저장 |

### 실행

```powershell
ros2 launch nxde1 k.launch.py     # 3개 노드 동시 실행
```

- 아두이노 포트가 COM13이 아니면 `src\nxde1\nxde1\walker_k.py`의 `SERIAL_PORT` 수정
  (symlink 빌드 상태이므로 재빌드 없이 바로 반영됨).
