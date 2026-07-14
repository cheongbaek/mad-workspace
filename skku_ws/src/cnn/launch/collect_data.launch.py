# -*- coding: utf-8 -*-
"""collect_data.launch.py — 모방학습 데이터 수집.

실행:
  ros2 launch cnn collect_data.launch.py
  ros2 launch cnn collect_data.launch.py serial_port:=COM13 camera_index:=1

구성: camera_node + benz_driver + dataset_recorder.
조종(teleop_keyboard)은 별도 터미널에서 실행한다(키 입력 상태 표시를 보기 위해):
  ros2 run cnn teleop_keyboard

W로 전진을 시작하는 순간(PWM≠0) 자동으로 녹화가 시작된다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='auto',
                              description='아두이노 포트 (auto/COM13//dev/ttyACM0)'),
        DeclareLaunchArgument('camera_index', default_value='0',
                              description='웹캠 장치 인덱스'),
        DeclareLaunchArgument('log_dir', default_value='~/imitation_data',
                              description='세션 저장 폴더'),

        Node(package='cnn', executable='camera_node', name='camera_node',
             output='screen',
             parameters=[{'device_index': LaunchConfiguration('camera_index')}]),

        Node(package='cnn', executable='benz_driver', name='benz_driver',
             output='screen',
             parameters=[{'serial_port': LaunchConfiguration('serial_port')}]),

        Node(package='cnn', executable='dataset_recorder', name='dataset_recorder',
             output='screen',
             parameters=[{'log_dir': LaunchConfiguration('log_dir')}]),
    ])
