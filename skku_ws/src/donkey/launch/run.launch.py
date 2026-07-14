# -*- coding: utf-8 -*-
"""실행 단계 — mega + drive 두 노드. 즉시 출발하므로 학습한 위치에 차를 놓고 실행!

  ros2 launch donkey run.launch.py                       # 최신 train_XXX 자동 선택
  ros2 launch donkey run.launch.py train:=train_001
  ros2 launch donkey run.launch.py max_pwm:=100          # 첫 검증은 저속 권장

토픽 흐름:  drive ──/in──▶ mega ──시리얼──▶ benz.ino
                           mega ──/out──▶ (모니터링용: ros2 topic echo /out)
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

        DeclareLaunchArgument('train', default_value='latest',
                              description='trained/ 안의 폴더명 (train_001) 또는 latest'),
        DeclareLaunchArgument('serial_port', default_value='auto'),
        DeclareLaunchArgument('camera_name', default_value='c920,vu0060',
                              description='카메라 장치이름 힌트(쉼표구분) — C920E 전용, 내장카메라·폰카메라 배제'),
        DeclareLaunchArgument('max_pwm', default_value='150.0'),

        Node(package='donkey', executable='mega', name='mega',
             output='screen', emulate_tty=True,
             parameters=[{'serial_port': LaunchConfiguration('serial_port')}]),

        Node(package='donkey', executable='drive', name='drive',
             output='screen', emulate_tty=True,
             parameters=[{
                 'train': LaunchConfiguration('train'),
                 'camera_name': LaunchConfiguration('camera_name'),
                 'max_pwm': LaunchConfiguration('max_pwm'),
             }]),
    ])
