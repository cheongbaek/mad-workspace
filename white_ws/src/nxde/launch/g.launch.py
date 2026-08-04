#!/usr/bin/env python3
"""g.launch.py — ★모든 하드웨어의 연결·통신을 전담하는 런치★ (Ubuntu 22.04 / ROS2 Humble)

    ros2 launch nxde g.launch.py

┌─ g.launch.py (이 파일) — 하드웨어 계층 ────────────────────────────────────────┐
│  arduino            A/B 2보드 (인휠 PID / 조향·제동)                          │
│      구독 /cmd_vel_raw /control_state /brake_level                           │
│      발행 /encoder /steer_angle_measured /vehicle_mode /throttle_pedal        │
│           /drive_pulse_cmd /estop /board_status                              │
│  iahrs              iAHRS IMU            → /imu/data (+ TF)                  │
│  nmea_serial_driver u-blox RTK GPS       → /fix                              │
│  usb_cam            USB 카메라           → /image_raw   (use_camera 게이트)    │
│  usb_cam_ctrl       v4l2-ctl 강제 적용   (드라이버가 파라미터를 무시할 때 보정)  │
└──────────────────────────────────────────────────────────────────────────────┘
┌─ white one_launch.py — 자율주행 계층 (별 터미널) ─────────────────────────────┐
│  gps_imu  mapping  driving  perception  camera_judgment  sensor_monitor      │
└──────────────────────────────────────────────────────────────────────────────┘

    터미널 1 :  ros2 launch nxde  g.launch.py       ← 하드웨어 (이 파일)
    터미널 2 :  ros2 launch white one_launch.py     ← 자율주행 노드
    터미널 3 :  ros2 run   white prompt             ← CLI 상호작용 (수집/주행/관리)

두 런치는 서로를 모르지만 같은 ROS_DOMAIN_ID(기본 0)면 토픽으로 자동 연결된다.
패키지 경계는 빌드 단위일 뿐이고, 통신은 DDS 가 토픽 이름·타입·QoS 로만 맺기 때문이다.

═══════════════════════════════════════════════════════════════════════════════
 ★★ 연결 실패 / 도중 단절에 대한 대응 ★★
═══════════════════════════════════════════════════════════════════════════════
 이 런치는 **장치가 하나도 안 꽂혀 있어도 정상 기동한다.** 각 노드가 자기 장치를 계속
 다시 찾으므로, 나중에 꽂거나 도중에 뺐다 꽂아도 자동으로 붙는다.

 | 노드      | 최초 실패          | 도중 단절            | 수단                          |
 |-----------|-------------------|---------------------|-------------------------------|
 | arduino   | 백그라운드 재스캔  | 재스캔 (한쪽만 빠져도 나머지는 계속) | 자체 _link_loop (3s)          |
 | iahrs     | 2s 재시도 + 재탐색 | 재시도 + 재탐색      | 자체 재연결 타이머 + VID/PID   |
 | GPS       | respawn           | respawn             | ★udev 심볼릭링크 필요★         |
 | usb_cam   | respawn           | respawn             | respawn_delay 3s              |

 ★ GPS·카메라는 외부 패키지라 노드 코드를 고칠 수 없어 respawn 에 의존한다 ★
   respawn 은 **같은 파라미터로 프로세스를 다시 띄우는 것**이므로, 장치 경로가 안정적일
   때만 복구된다. /dev/ttyACM0 처럼 열거 순서에 의존하는 경로를 쓰면 재부팅·재연결 때
   다른 장치를 열 수 있다.
   → **udev 심볼릭링크(/dev/gps, /dev/imu)를 만들어 두는 것이 사실상 필수다.**
     설정법은 nxde/README.md 6절. 링크가 없으면 이 런치가 VID/PID 로 한 번 찾아보고,
     그래도 없으면 '/dev/gps' 를 그대로 넘긴다 — 나중에 udev 로 링크가 생기는 순간
     respawn 이 붙는다(링크가 아예 없으면 계속 실패하며 재시도한다).

═══════════════════════════════════════════════════════════════════════════════
 ⚠️⚠️ 종료 순서 : 이 런치를 ★먼저★ 내린다 ⚠️⚠️
═══════════════════════════════════════════════════════════════════════════════
 A보드 펌웨어에는 무입력 타임아웃이 없다(0713에서 제거). 마지막 수신 명령을 계속 물고
 있으므로:
   · 이 런치를 Ctrl+C  → arduino 노드가 종료 직전에 정지값('0' / 'x,0')을 시리얼로 직접
                        써 넣는다(stop_and_close). 차가 선다. ★안전★
   · one_launch.py 만 내림 → /cmd_vel_raw 가 끊길 뿐이고 arduino 는 마지막 명령을
                        KEEPALIVE_S(1초) 주기로 계속 재전송한다. ★차가 계속 간다★
 급할 때는 E-stop 스위치를 쓴다.

환경변수:
  PYTHONUNBUFFERED=1                : launch 가 stdout 을 파이프로 받으면 Python 이 완전
                                      버퍼링으로 전환되어 print 가 갇힌다
  RCUTILS_LOGGING_BUFFERED_STREAM=0 : get_logger() 로그(rcutils C 스트림) 버퍼링 해제.
                                      이게 없으면 '보드 감지' 로그가 종료 시점에야 몰려
                                      나오거나 유실된다(PYTHONUNBUFFERED 로는 안 잡힘)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nxde import ports

NODE_ENV = {'PYTHONUNBUFFERED': '1',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '0'}

# 외부 노드(GPS/카메라) 재기동 간격 [s]. 장치가 아예 없으면 이 주기로 계속 재시도한다.
RESPAWN_DELAY = 3.0


def generate_launch_description():
    print("\n=====================================================")
    print(" 🔌 하드웨어 장치 경로를 확인합니다 (GPS / IMU)")
    print("    아두이노 A/B 는 arduino 노드가 텔레메트리 접두어로 자체 식별합니다.")

    # ★ 여기서 찾지 못해도 실패가 아니다 ★ resolve_device 는 udev 심볼릭링크 → VID/PID →
    #   (그래도 없으면) 심볼릭링크 경로를 그대로 돌려준다. 안정적인 이름을 넘겨두면
    #   나중에 장치를 꽂는 순간 respawn / 자체 재연결이 자동으로 붙는다. 파일 헤더 참고.
    used = set()
    gps_dev = ports.resolve_device(ports.SYMLINK_GPS, ports.GPS_VIDPID,
                                  exclude=used, log=lambda m: print(f"    [GPS] {m}"))
    used.add(gps_dev)
    imu_dev = ports.resolve_device(ports.SYMLINK_IMU, ports.IMU_VIDPID,
                                  exclude=used, log=lambda m: print(f"    [IMU] {m}"))
    used.add(imu_dev)
    print("=====================================================\n")

    # 아두이노 탐색에서 제외할 경로 (GPS/IMU 포트를 열면 배타 open 충돌로 그쪽 드라이버가
    # 자기 포트를 못 잡고, 탐색도 포트당 5초씩 느려진다)
    exclude_for_arduino = [gps_dev, imu_dev]

    use_camera   = LaunchConfiguration('use_camera')
    video_device = LaunchConfiguration('video_device')
    cam_exposure = LaunchConfiguration('cam_exposure')

    args = [
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='USB 카메라(usb_cam) 기동 여부. white one_launch.py 의 use_camera 와 '
                        '맞춰야 한다 — 여기서 false 면 /image_raw 가 없어 perception 이 굶는다'),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='USB 카메라 V4L2 장치 경로. `v4l2-ctl --list-devices` 로 확인'),
        DeclareLaunchArgument(
            'cam_exposure', default_value='120',
            description='See3CAM 수동노출(exposure_time_absolute). 기본 120 은 야간 기준 — '
                        '주간엔 과다노출로 신호등 블루밍·차선 대비 붕괴 확인됨(실측: '
                        'exp=120 대비 48 / exp=2 대비 90, 포화 0%→2.6%). 주간엔 cam_exposure:=10'),
        DeclareLaunchArgument(
            'gps_port', default_value=gps_dev,
            description='GPS 시리얼 경로 override (기본: udev 링크 → VID/PID 스캔 결과)'),
        DeclareLaunchArgument(
            'imu_port', default_value=imu_dev,
            description='iAHRS 시리얼 경로 override (기본: udev 링크 → VID/PID 스캔 결과)'),
        DeclareLaunchArgument(
            'imu_sync_period_ms', default_value='50',
            description='IMU 출력주기[ms]. 기본 50=20Hz. driving 의 지연보상 예측 정밀도를 '
                        '높이려면 20(=50Hz) 권장'),
        # ── arduino 노드 파라미터 ──
        DeclareLaunchArgument(
            'baud', default_value='115200',
            description='A/B 보드 공통 시리얼 보드레이트'),
        DeclareLaunchArgument(
            'steer_invert', default_value='false',
            description='조향 부호 반전. ★기본 false★ — ROS 토픽과 B보드가 같은 규약'
                        '(− 좌 / + 우)을 쓴다. 배선이나 펌웨어를 뒤집어 방향이 반대가 '
                        '되었을 때만 true 로 둔다'),
        DeclareLaunchArgument(
            'stop_brake_level', default_value='0',
            description='/control_state=False 일 때 걸 브레이크 단계. '
                        '0=코스트(white 기존 동작) / 1=약한 브레이킹으로 더 빨리 정지'),
        DeclareLaunchArgument(
            'manual_brake_level', default_value='2',
            description='수동조종(D5 개방) 진입 시 걸 브레이크 단계. '
                        '사람이 쓰로틀 페달을 밟으면 0단으로 풀린다'),
        DeclareLaunchArgument(
            'manual_release_raw', default_value='240',
            description='위 브레이크를 풀 쓰로틀 페달 raw 임계 (실측: 놓음 177 / 최대 800)'),
        DeclareLaunchArgument(
            'manual_pulse_max', default_value='15',
            description='수동조종에서 페달 최대치가 대응할 펄스. 기본 15 ≈ 47km/h. '
                        '★수집 주행·초기 시험에서는 3~5 로 낮출 것★'),
    ]

    # ═══════════════════════════════════════════════════════════════════
    #  아두이노 A/B 보드 — 자체 백그라운드 탐색·재연결 (respawn 불필요)
    # ═══════════════════════════════════════════════════════════════════
    arduino = Node(
        package='nxde',
        executable='arduino',
        name='arduino',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{
            'baud':               LaunchConfiguration('baud'),
            'steer_invert':       LaunchConfiguration('steer_invert'),
            'stop_brake_level':   LaunchConfiguration('stop_brake_level'),
            'manual_brake_level': LaunchConfiguration('manual_brake_level'),
            'manual_release_raw': LaunchConfiguration('manual_release_raw'),
            'manual_pulse_max':   LaunchConfiguration('manual_pulse_max'),
            'exclude_ports':      exclude_for_arduino,
        }],
    )

    # ═══════════════════════════════════════════════════════════════════
    #  iAHRS IMU — 자체 2초 재연결 + VID/PID 재탐색.
    #    respawn 은 '노드가 예외로 죽는' 경우의 이중 안전망으로만 걸어둔다.
    # ═══════════════════════════════════════════════════════════════════
    iahrs = Node(
        package='nxde',
        executable='iahrs',
        name='iahrs_node',
        output='screen',
        additional_env=NODE_ENV,
        respawn=True,
        respawn_delay=RESPAWN_DELAY,
        parameters=[{
            'port':            LaunchConfiguration('imu_port'),
            'baud':            115200,
            'send_tf':         True,
            'rescan':          True,
            'sync_period_ms':  LaunchConfiguration('imu_sync_period_ms'),
            'exclude_ports':   [gps_dev],
        }],
    )

    # ═══════════════════════════════════════════════════════════════════
    #  u-blox RTK GPS — 외부 패키지(nmea_navsat_driver). 코드를 고칠 수 없으므로
    #    respawn 에 의존한다 → 경로가 안정적이어야 한다(udev 필수, 헤더 참고).
    # ═══════════════════════════════════════════════════════════════════
    gps = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_serial_driver',
        output='screen',
        additional_env=NODE_ENV,
        respawn=True,
        respawn_delay=RESPAWN_DELAY,
        parameters=[{'port': LaunchConfiguration('gps_port'), 'baud': 115200}],
    )

    # ═══════════════════════════════════════════════════════════════════
    #  USB 카메라 — 외부 패키지(usb_cam). respawn 으로 재시도.
    #    카메라 내부 파라미터(camera_info)는 이 패키지 share 에서 읽는다.
    #    ※ perception 은 /camera_info 를 구독하지 않으므로 이 yaml 은 사실상 기록용이다.
    #      실제 BEV 캘리브는 white 쪽 pixel_to_meter_bev 파라미터가 담당한다.
    # ═══════════════════════════════════════════════════════════════════
    calib_path = os.path.join(
        get_package_share_directory('nxde'), 'calibration', 'usb_cam_calibration.yaml')
    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        additional_env=NODE_ENV,
        respawn=True,
        respawn_delay=RESPAWN_DELAY,
        parameters=[{
            'video_device': video_device,
            'framerate': 30.0,
            'image_width': 1920,
            'image_height': 1080,
            'pixel_format': 'uyvy',
            'camera_name': 'narrow_stereo',
            'io_method': 'mmap',
            'camera_info_url': 'file://' + calib_path,
            'brightness': 0,
            'contrast': 128,
            'saturation': 60,
            'sharpness': 64,
            'gain': 10,
            'auto_exposure': False,
            'exposure': cam_exposure,
            # [2026-07-29] image_transport 부가 플러그인 비활성화 — raw 만 남긴다.
            #   usb_cam 은 /image_raw 외에 compressed(JPEG)·compressedDepth·theora 를
            #   함께 광고한다. 구독자는 perception 하나뿐이고 원본만 쓰는데,
            #   `ros2 bag record -a` 가 그 부가 토픽까지 구독하면 인코딩이 실제로 돌아
            #   CPU 를 먹는다. 특히 compressedDepth 는 컬러(yuv422)를 깊이영상으로
            #   압축하려다 매 프레임 실패해 ERROR 를 쏟아낸다.
            #   ⚠️ Humble 의 image_transport 는 'disable_pub_plugins' 가 아니라
            #      '<base_topic>.enable_pub_plugins' (화이트리스트) 를 쓴다.
            #      이름을 틀리면 조용히 무시되고 에러가 계속 뜬다(실제로 그랬다).
            'image_raw.enable_pub_plugins': ['image_transport/raw'],
        }],
        condition=IfCondition(use_camera),
    )

    # usb_cam 기동 2초 뒤 v4l2-ctl 강제 적용 (드라이버가 파라미터를 무시하는 경우 보정).
    #   ※ 이건 respawn 대상이 아니다 — 카메라가 respawn 되면 이 설정은 다시 적용되지 않는다.
    #     노출이 이상하면 이 명령을 손으로 한 번 더 돌리면 된다(아래 cmd 그대로).
    usb_cam_ctrl = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    (
                        'v4l2-ctl -d "$DEV" --set-ctrl=auto_exposure=1 && '
                        'v4l2-ctl -d "$DEV" --set-ctrl=exposure_time_absolute="$EXPOSURE" && '
                        'v4l2-ctl -d "$DEV" --set-ctrl=gain=10 && '
                        'v4l2-ctl -d "$DEV" --set-ctrl=saturation=60 && '
                        'echo "[v4l2-ctl] camera controls applied on $DEV" && '
                        'v4l2-ctl -d "$DEV" --get-ctrl=auto_exposure && '
                        'v4l2-ctl -d "$DEV" --get-ctrl=exposure_time_absolute && '
                        'v4l2-ctl -d "$DEV" --get-ctrl=gain && '
                        'v4l2-ctl -d "$DEV" --get-ctrl=saturation'
                    )
                ],
                additional_env={'DEV': video_device, 'EXPOSURE': cam_exposure},
                output='screen',
            )
        ],
        condition=IfCondition(use_camera),
    )

    return LaunchDescription(args + [
        arduino,
        iahrs,
        gps,
        usb_cam,
        usb_cam_ctrl,
    ])
