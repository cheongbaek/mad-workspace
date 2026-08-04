from setuptools import setup
import os
from glob import glob

package_name = 'white'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            sorted(set(glob('launch/*.launch.py')) | set(glob('launch/*.py')))),
        # 카메라 캘리브레이션(camera_info) — ★nxde 로 이관됨★
        #   usb_cam 노드를 nxde 의 g.launch.py 가 띄우므로 캘리브도 그쪽이 소유한다:
        #     nxde/calibration/usb_cam_calibration.yaml
        #   calibration/ 폴더는 원본 보존을 위해 남겨 두었지만 설치되지 않는다.
        # (os.path.join('share', package_name, 'calibration'),
        #     glob('calibration/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='domi',
    maintainer_email='domi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ★ [하드웨어 계층 분리] iahrs 노드는 nxde 로 이관했다 ★
            #   "모든 하드웨어 연결·통신은 nxde 의 g.launch.py 가 전담"하는 방침에 따라
            #   센서 드라이버를 판단 스택에서 분리했다. 실행은:
            #       ros2 launch nxde g.launch.py   (또는 ros2 run nxde iahrs)
            #   white/white/iahrs.py 파일은 스냅샷으로 남아 있으나 실행되지 않는다.
            # 'iahrs        = white.iahrs:main',
            # ★ [kasa 이식] motor 노드는 제거했다 ★
            #   white 차량의 단일 아두이노(C/S 프레임, 300틱 엔코더) 전용이라 kasa 의
            #   A/B 2보드에 쓸 수 없다. 그 역할은 nxde 패키지의 arduino 노드가 대신한다:
            #       ros2 launch nxde g.launch.py     (별 터미널)
            #   white/motor.py 파일 자체는 프로토콜 참고용으로 남겨 두었지만 실행되지 않는다.
            # 'motor        = white.motor:main',
            'gps_imu        = white.gps_imu:main',
            'mapping        = white.mapping:main',
            'driving        = white.driving:main',
            'prompt         = white.prompt:main',
            'sensor_monitor = white.sensor_monitor:main',  # 🌟 추가
            # ── 카메라 융합 ──
            'perception       = white.perception:main',        # 인지(차선 polyfit + 신호등)
            'camera_judgment  = white.camera_judgment:main',   # /lane_metrics 브리지 + 신호등 게이트
        ],
    },
)