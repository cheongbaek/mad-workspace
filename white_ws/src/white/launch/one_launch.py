#!/usr/bin/env python3
"""
one_launch.py ― ★자율주행 노드 계층★ launch 파일 (kasa A/B 2보드 대응판)

★★ 이 런치는 어떤 하드웨어에도 직접 연결하지 않는다 ★★
  아두이노·IMU·GPS·카메라의 연결·통신은 전부 nxde 의 g.launch.py 가 전담한다.

      터미널 1 :  ros2 launch nxde  g.launch.py     ← 하드웨어 (아두이노 A/B, IMU, GPS, 카메라)
      터미널 2 :  ros2 launch white one_launch.py   ← 이 파일 (자율주행 노드)
      터미널 3 :  ros2 run   white prompt           ← CLI 상호작용 (수집 / 주행 / 관리)

  두 런치는 서로를 모르지만 같은 ROS_DOMAIN_ID(기본 0)면 토픽으로 자동 연결된다.
  패키지 경계는 빌드 단위일 뿐이고, 통신은 DDS 가 토픽 이름·타입·QoS 로만 맺기 때문이다.
  ★ 순서는 상관없다 ★ g.launch.py 는 장치가 없어도 뜨고, 이 런치는 토픽이 없어도 뜬다.
  각각 상대가 나타나면 자동으로 붙는다.

이 런치가 기동하는 노드 (전부 하드웨어 비접촉):
    gps_imu           /fix + /imu/data + /encoder → 융합 → /ego_state
    driving           경로추종 → /cmd_vel_raw (또는 /cmd_vel_drive) + /control_state
    mapping           수집 — /ego_state + 수동조종 계측 3종을 CSV 기록
    perception        /image_raw → /lane/state, /tl/state …          (use_camera)
    camera_judgment   /lane_metrics 브리지 + 신호등 게이트           (use_camera)
    sensor_monitor    센서 상태 대시보드                              (use_monitor)

╔══════════════════════════════════════════════════════════════════════════════╗
║  하드웨어 토픽 계약 (nxde arduino 노드와 맞물리는 부분)                        ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  발행  /cmd_vel_raw (Twist)  linear.x = ★주행 목표펄스 0~15 (m/s 가 아니다)★   ║
║                              angular.z = 조향각 −40~40 (white 부호: +좌/−우)  ║
║        /control_state (Bool) True = 구동 허용 / False = 정지                  ║
║  구독  /encoder (Int32)              A보드 좌+우 펄스의 합 → 속도 피드백        ║
║        /steer_angle_measured (Int32) B보드 실측 조향각 (white 부호)            ║
║        /vehicle_mode (Bool)          B보드 D5 : True 자율 / False 수동조종     ║
║        /throttle_pedal (Int32)       A보드 A0 쓰로틀 페달 raw                 ║
║        /drive_pulse_cmd (Int32)      A보드로 실제 나간 주행 목표펄스           ║
║        /estop (Bool) /board_status (String)                                  ║
╚══════════════════════════════════════════════════════════════════════════════╝

★ 수동조종 모드는 이 런치와 무관하게 항상 살아 있다 ★
  B보드 D5 스위치가 개방(수동)이면 nxde 의 arduino 노드가 /control_state·/cmd_vel_raw 를
  무시하고 "쓰로틀 페달 raw → 주행펄스" 경로로 직접 넘긴다(조향은 힘빼기 'x').
  즉 g.launch.py 만 떠 있어도 사람이 페달·핸들로 차를 몬다. 이 런치는 그 계측값
  (/drive_pulse_cmd · /encoder · /steer_angle_measured)을 mapping 노드가 수집하는 역할이다.

⚠️ 종료 순서 : g.launch.py 를 ★먼저★ 내린다. A보드 펌웨어에 무입력 타임아웃이 없어서
   이 런치만 내리면 arduino 노드가 마지막 명령을 1초마다 계속 재전송한다(= 차가 계속 간다).
   자세한 내용은 g.launch.py 헤더 참고.

주의:
    - GPS가 /fix를 만들어야 gps_imu.py가 초기 헤딩을 고정할 수 있음.
    - /encoder(= nxde arduino 노드)가 있어야 ENC+GPS 헤딩 고정이 가능함.
      ★ kasa 는 1카운트 = 0.442 m/s 라 그보다 느리면 엔코더 '활성' 판정이 안 된다 ★
      driving 의 min_speed_ms(≈0.95 m/s)가 그 위에 있어 정상 주행에서는 문제없다.
    - 엔코더가 없어도 GPS-only 조건을 만족하면 헤딩 고정 가능.
    - use_camera 는 g.launch.py 의 use_camera 와 맞춰야 한다 — 여기만 true 로 두면
      /image_raw 가 없어 perception 이 굶고, 여기만 false 로 두면 카메라 게이트가 빠져
      driving 이 /cmd_vel_raw 로 직결된다(그 자체는 정상 동작이다).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'white'
    use_monitor = LaunchConfiguration('use_monitor')
    use_camera = LaunchConfiguration('use_camera')
    image_topic = LaunchConfiguration('image_topic')
    dr_speed_factor = LaunchConfiguration('dr_speed_factor')

    print("\n=====================================================")
    print(" 🧠 자율주행 노드 계층을 기동합니다 (하드웨어 비접촉)")
    print("    하드웨어는 별 터미널에서: ros2 launch nxde g.launch.py")
    print("    CLI 조작은 별 터미널에서: ros2 run white prompt")
    print("=====================================================\n")

    args = [
        DeclareLaunchArgument(
            'use_monitor',
            default_value='true',
            description='sensor_monitor 노드 실행 여부'
        ),
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='카메라 융합 체인(perception+camera_judgment) 실행 여부. '
                        'true=driving→/cmd_vel_drive→게이트→/cmd_vel_raw, '
                        'false=driving→/cmd_vel_raw 직결(GPS 단독). '
                        '★usb_cam 자체는 nxde g.launch.py 가 띄운다 — 그쪽 use_camera 와 맞출 것★'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/image_raw',
            description='perception 이 구독할 카메라 이미지 토픽 (nxde 의 usb_cam 이 발행)'
        ),
        DeclareLaunchArgument(
            'dr_speed_factor',
            default_value='0.6',
            description='GPS 두절(DR) 진입 시 속도 감속 계수(낮출수록 더 감속)'
        ),
    ]

    # ── 측위 융합 허브 ──
    gps_imu = Node(
        package=package_name,
        executable='gps_imu',
        name='gps_imu_node',
        output='screen',
    )

    # ── 수집(매핑) ──
    #   ★ 수집은 '수동조종 모드'에서 한다 ★ 사람이 페달·핸들로 차를 몰고, 그때의
    #     ①페달 환산 목표펄스(/drive_pulse_cmd) ②실 주행 펄스(/encoder)
    #     ③DC모터 가변저항 실측 조향각(/steer_angle_measured) 을 함께 기록한다.
    #     무선 컨트롤러는 더 이상 쓰지 않는다. 모드 강제는 prompt 가 담당한다.
    mapping = Node(
        package=package_name,
        executable='mapping',
        name='mapping_node',
        output='screen',
    )

    # [카메라 융합] driving 출력 토픽을 use_camera 로 분기.
    #   use_camera=true  → /cmd_vel_drive (camera_judgment 게이트 경유)
    #   use_camera=false → /cmd_vel_raw   (nxde arduino 직결, GPS 단독)
    # ★ [kasa] /cmd_vel_raw 는 linear.x 가 '펄스 0~15' 다 ★ m/s → 펄스 환산은
    #   /cmd_vel_raw 를 실제로 발행하는 노드가 한다:
    #     use_camera=true  → camera_judgment.cb_cmd 가 환산 (게이트 판정은 m/s 로 계산)
    #     use_camera=false → driving.publish_cmd 가 환산 (self._publish_pulse)
    driving_cam = Node(
        package=package_name,
        executable='driving',
        name='driving_node',
        output='screen',
        parameters=[{'cmd_vel_topic': '/cmd_vel_drive',
                     'dr_speed_factor': dr_speed_factor}],
        condition=IfCondition(use_camera),
    )
    driving_nocam = Node(
        package=package_name,
        executable='driving',
        name='driving_node',
        output='screen',
        parameters=[{'cmd_vel_topic': '/cmd_vel_raw',
                     'dr_speed_factor': dr_speed_factor}],
        condition=UnlessCondition(use_camera),
    )

    # [카메라 융합] 인지 — 차선 polyfit(/lane/state) + 신호등(/tl/state 등)
    perception = Node(
        package=package_name,
        executable='perception',
        name='perception_node',
        output='screen',
        parameters=[{
            'image_topic':        image_topic,
            'show_window':        True,          # 차량 배포는 헤드리스
            'pixel_to_meter_bev': 0.006,
        }],
        condition=IfCondition(use_camera),
    )

    # [카메라 융합] 판단 — /lane_metrics 브리지 + 신호등 게이트(/cmd_vel_drive→/cmd_vel_raw)
    camera_judgment = Node(
        package=package_name,
        executable='camera_judgment',
        name='camera_judgment',
        output='screen',
        parameters=[{'pixel_to_meter_bev': 0.006}],
        condition=IfCondition(use_camera),
    )

    monitor = Node(
        package=package_name,
        executable='sensor_monitor',
        name='sensor_monitor_node',
        output='screen',
        condition=IfCondition(use_monitor),
    )

    return LaunchDescription(args + [
        gps_imu,
        mapping,
        driving_cam,
        driving_nocam,
        perception,
        camera_judgment,
        monitor,
    ])
