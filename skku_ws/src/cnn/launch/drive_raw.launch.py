# -*- coding: utf-8 -*-
"""drive_raw.launch.py — Variant A(카메라 전체 이미지 CNN) 자율주행.

실행:
  ros2 launch cnn drive_raw.launch.py model_path:=C:/path/to/raw_model.pt
  ros2 launch cnn drive_raw.launch.py model_path:=~/models/raw_model.pt serial_port:=COM13
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path',
                              description='training/train_raw.py로 학습한 .pt 경로'),
        DeclareLaunchArgument('serial_port', default_value='auto'),
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('max_pwm', default_value='150.0',
                              description='자율주행 PWM 상한(안전)'),

        Node(package='cnn', executable='camera_node', name='camera_node',
             output='screen',
             parameters=[{'device_index': LaunchConfiguration('camera_index')}]),

        Node(package='cnn', executable='benz_driver', name='benz_driver',
             output='screen',
             parameters=[{'serial_port': LaunchConfiguration('serial_port')}]),

        Node(package='cnn', executable='raw_infer_node', name='raw_infer_node',
             output='screen',
             parameters=[{
                 'model_path': LaunchConfiguration('model_path'),
                 'max_pwm': LaunchConfiguration('max_pwm'),
             }]),
    ])
