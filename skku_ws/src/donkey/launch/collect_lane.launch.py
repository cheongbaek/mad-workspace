# -*- coding: utf-8 -*-
"""수집 (차선 정보 방식) — mega + joystick + collect_lane 세 노드.

  ros2 launch donkey collect_lane.launch.py
  ros2 launch donkey collect_lane.launch.py serial_port:=COM13

토픽 흐름:  joystick ──/in──▶ mega ──시리얼──▶ benz.ino
                              mega ──/out──▶ collect_lane (실측 조향각)
            joystick ──/in──────────────────▶ collect_lane (주행PWM 라벨)

마우스 조종(전체화면 창): 좌/우클릭=조향, 휠=속도, 휠버튼=전후진, 아무 키=조종 종료.
출발(주행PWM≠0)하면 data/lane_XXX/log.csv 기록 시작.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # launch 하의 파이썬 노드는 stdout이 파이프라 print가 버퍼에 갇힘 → 무버퍼 강제.
        # PYTHONUTF8은 Windows에서 한글 로그 깨짐(cp949) 방지.
        SetEnvironmentVariable('PYTHONUNBUFFERED', '1'),
        SetEnvironmentVariable('PYTHONUTF8', '1'),

        DeclareLaunchArgument('serial_port', default_value='auto'),
        DeclareLaunchArgument('camera_name', default_value='c920,vu0060',
                              description='카메라 장치이름 힌트(쉼표구분) — C920E 전용, 내장카메라·폰카메라 배제'),

        Node(package='donkey', executable='mega', name='mega',
             output='screen', emulate_tty=True,
             parameters=[{'serial_port': LaunchConfiguration('serial_port')}]),

        Node(package='donkey', executable='joystick', name='joystick',
             output='screen', emulate_tty=True),

        Node(package='donkey', executable='collect_lane', name='collect_lane',
             output='screen', emulate_tty=True,
             parameters=[{'camera_name': LaunchConfiguration('camera_name')}]),
    ])
