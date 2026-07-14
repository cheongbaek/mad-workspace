#!/usr/bin/env python3
"""
one_launch.py ― 전체 자율주행 시스템 실행용 launch 파일 [헤딩 고정 복구판]

핵심:
1. USB VID/PID로 GPS / Arduino / IMU 포트를 자동 탐색
2. nmea_navsat_driver를 실행해서 GPS /fix 토픽 생성
3. iahrs, motor, gps_imu, mapping, driving, sensor_monitor 실행
4. setup.py console_scripts 기준에 맞게 IMU executable은 iahrs 사용

실행:
    ros2 launch white one_launch.py

주의:
    - GPS가 /fix를 만들어야 gps_imu.py가 초기 헤딩을 고정할 수 있음.
    - Arduino가 /encoder를 만들어야 ENC+GPS 헤딩 고정이 가능함.
    - 엔코더가 없어도 GPS-only 조건을 만족하면 헤딩 고정 가능.
"""

import serial.tools.list_ports

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_usb_port(candidates, device_name, used_ports=None):
    """여러 VID/PID 후보 중 현재 연결된 USB 포트를 찾는다.

    used_ports를 넘기면 이미 GPS/IMU/Arduino로 잡힌 포트를 다시 선택하지 않는다.
    같은 VID/PID 장치가 여러 개 연결된 경우에는 launch 인자로 직접 포트를 넘길 수 있다.
    """
    if used_ports is None:
        used_ports = set()

    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    for port in ports:
        if port.device in used_ports:
            continue
        for vid, pid in candidates:
            if port.vid == vid and port.pid == pid:
                used_ports.add(port.device)
                desc = port.description or ""
                print(f"✅ [{device_name}] 감지 성공: {port.device} {desc}")
                return port.device

    print(f"⚠️ [경고] {device_name} 장치를 찾을 수 없습니다! (연결 없이 디버깅 진행)")
    return "/dev/tty_NOT_FOUND"


def generate_launch_description():
    package_name = 'white'
    use_monitor = LaunchConfiguration('use_monitor')
    gps_port_arg = LaunchConfiguration('gps_port')
    arduino_port_arg = LaunchConfiguration('arduino_port')
    imu_port_arg = LaunchConfiguration('imu_port')
    used_ports = set()

    print("\n=====================================================")
    print(" 🔍 USB 포트 자동 탐색을 시작합니다...")

    # U-Blox GPS 후보
    gps_port = find_usb_port([
        (0x1546, 0x01A9),  # u-blox 9 계열
        (0x1546, 0x01A8),  # u-blox 8 계열 / 일부 수신기
    ], "U-Blox/SMC GPS", used_ports)

    # Arduino Mega 후보: 정품 Mega + CH340 호환 보드
    arduino_port = find_usb_port([
        (0x2341, 0x0042),
        (0x2341, 0x0010),
        (0x2A03, 0x0042),
        (0x1A86, 0x7523),
    ], "Arduino Mega", used_ports)

    # iAHRS / CP210x 계열 후보
    imu_port = find_usb_port([
        (0x10C4, 0xEA60),
    ], "IMU Sensor", used_ports)

    print("=====================================================\n")

    args = [
        DeclareLaunchArgument(
            'use_monitor',
            default_value='true',
            description='sensor_monitor 노드 실행 여부'
        ),
        DeclareLaunchArgument(
            'gps_port',
            default_value=gps_port,
            description='GPS serial port override, 예: /dev/ttyACM0 또는 /dev/ttyUSB0'
        ),
        DeclareLaunchArgument(
            'arduino_port',
            default_value=arduino_port,
            description='Arduino Mega serial port override'
        ),
        DeclareLaunchArgument(
            'imu_port',
            default_value=imu_port,
            description='iAHRS serial port override'
        ),
    ]

    gps = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_serial_driver',
        output='screen',
        parameters=[{'port': gps_port_arg, 'baud': 115200}],
    )

    iahrs = Node(
        package=package_name,
        executable='iahrs',
        name='iahrs_node',
        output='screen',
        parameters=[{'port': imu_port_arg, 'baud': 115200, 'send_tf': True}],
    )

    motor = Node(
        package=package_name,
        executable='motor',
        name='motor_node',
        output='screen',
        parameters=[{'port': arduino_port_arg, 'baud': 115200}],
    )

    gps_imu = Node(
        package=package_name,
        executable='gps_imu',
        name='gps_imu_node',
        output='screen',
    )

    mapping = Node(
        package=package_name,
        executable='mapping',
        name='mapping_node',
        output='screen',
    )

    driving = Node(
        package=package_name,
        executable='driving',
        name='driving_node',
        output='screen',
    )

    monitor = Node(
        package=package_name,
        executable='sensor_monitor',
        name='sensor_monitor_node',
        output='screen',
        condition=IfCondition(use_monitor),
    )

    return LaunchDescription(args + [
        gps,
        iahrs,
        motor,
        gps_imu,
        mapping,
        driving,
        monitor,
    ])