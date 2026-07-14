# k.launch.py : walker_k / listener_k 2개 노드 실행
# 사용법: ros2 launch nxde2 k.launch.py
#
# talker_k는 여기서 실행하지 않음: msvcrt로 콘솔 키 입력을 읽는 노드인데,
# ros2 launch가 자식 프로세스의 stdin을 터미널에 그대로 연결해주지 않아
# 키 입력을 못 받음. 반드시 별도 터미널에서 아래처럼 직접 실행할 것:
#   ros2 run nxde2 talker_k

from launch import LaunchDescription
from launch_ros.actions import Node

UNBUFFERED_ENV = {'PYTHONUNBUFFERED': '1'}


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nxde2',
            executable='walker_k',
            name='walker_k',
            output='screen',
            additional_env=UNBUFFERED_ENV,
        ),
        Node(
            package='nxde2',
            executable='listener_k',
            name='listener_k',
            output='screen',
            additional_env=UNBUFFERED_ENV,
        ),
    ])
