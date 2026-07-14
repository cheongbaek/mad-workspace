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
            'iahrs          = white.iahrs:main',
            'motor          = white.motor:main',
            'gps_imu        = white.gps_imu:main',
            'mapping        = white.mapping:main',
            'driving        = white.driving:main',
            'prompt         = white.prompt:main',
            'sensor_monitor = white.sensor_monitor:main',  # 🌟 추가
        ],
    },
)