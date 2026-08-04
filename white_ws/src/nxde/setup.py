from setuptools import setup

package_name = 'nxde'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # g.launch.py = ★모든 하드웨어★ 연결·통신 전담 런치 (아두이노 A/B + IMU + GPS + 카메라).
        #   white 의 one_launch.py(자율주행 노드)와 별 터미널에서 함께 띄운다. README 참고.
        ('share/' + package_name + '/launch',
            ['launch/g.launch.py']),
        # usb_cam 의 camera_info 용 내부 파라미터. 카메라 노드를 이 패키지가 띄우므로
        # 캘리브 파일도 여기서 소유한다(white/calibration 의 사본이 원본이었다).
        ('share/' + package_name + '/calibration',
            ['calibration/usb_cam_calibration.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='windo',
    maintainer_email='test@test.com',
    description='Hardware layer for the white autonomous stack: kasa A/B arduino bridge, '
                'iAHRS IMU, GPS and camera launch (Ubuntu 22.04 only)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 아두이노 A/B 2보드 통신 전담 (구 white/motor.py 의 대체)
            'arduino = nxde.arduino:main',
            # iAHRS IMU 드라이버 (white/white/iahrs.py 에서 이관 — 하드웨어는 nxde 소관)
            'iahrs   = nxde.iahrs:main',
            # ★검증용 GUI★ g.launch.py 만 띄운 상태에서 차가 실제로 움직이는지 확인한다.
            #   ros2 run nxde master
            #   ⚠️ one_launch.py(driving_node) / prompt 와 동시에 쓰지 말 것 —
            #      /cmd_vel_raw·/control_state 발행자가 겹친다(창 상단에 경고가 뜬다).
            'master  = nxde.master:main',
            # ※ 조이스틱 / csv_read / keyboard 조종 노드는 가져오지 않았다 —
            #   무선 컨트롤러를 더 이상 쓰지 않고, 수동조종은 물리 스위치(B보드 D5) +
            #   실제 페달·핸들로 하며 arduino 노드가 직접 처리한다.
        ],
    },
)
