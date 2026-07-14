# k.launch.py : talker_k / walker_k / listener_k 3개 노드 동시 실행
# 사용법: ros2 launch nxde1 k.launch.py
#
# launch가 자식 프로세스 stdout을 파이프로 캡처하면 Python이 tty가 아니라고
# 판단해 완전 버퍼링 모드로 전환되어 로그가 버퍼에 갇힘. emulate_tty=True는
# Windows에 POSIX pty가 없어서 오히려 출력 캡처 자체를 깨뜨리므로 대신
# PYTHONUNBUFFERED=1을 환경변수로 넘겨 Python 출력 버퍼링을 끈다.

from launch import LaunchDescription
from launch_ros.actions import Node

UNBUFFERED_ENV = {'PYTHONUNBUFFERED': '1'}


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nxde1',
            executable='walker_k',
            name='walker_k',
            output='screen',
            additional_env=UNBUFFERED_ENV,
        ),
        Node(
            package='nxde1',
            executable='listener_k',
            name='listener_k',
            output='screen',
            additional_env=UNBUFFERED_ENV,
        ),
        # talker_k를 마지막에 배치 (walker/listener가 먼저 떠서 첫 메시지 유실 최소화)
        Node(
            package='nxde1',
            executable='talker_k',
            name='talker_k',
            output='screen',
            additional_env=UNBUFFERED_ENV,
        ),
    ])
