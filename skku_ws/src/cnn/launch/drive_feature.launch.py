# -*- coding: utf-8 -*-
"""drive_feature.launch.py — Variant B(허프 차선특징 MLP) 자율주행.

실행:
  ros2 launch cnn drive_feature.launch.py model_path:=C:/path/to/feature_model.pt
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path',
                              description='training/train_feature.py로 학습한 .pt 경로'),
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

        Node(package='cnn', executable='feature_infer_node', name='feature_infer_node',
             output='screen',
             parameters=[{
                 'model_path': LaunchConfiguration('model_path'),
                 'max_pwm': LaunchConfiguration('max_pwm'),
             }]),
    ])
