from setuptools import setup
from glob import glob

package_name = 'donkey'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='windo',
    maintainer_email='test@test.com',
    description='Donkeycar-style 3-stage imitation driving: collect (lane/all) -> train -> run',
    license='TODO',
    entry_points={
        'console_scripts': [
            'mega         = donkey.mega:main',
            'joystick     = donkey.joystick:main',
            'collect_lane = donkey.collect_lane:main',
            'collect_all  = donkey.collect_all:main',
            'drive        = donkey.drive:main',
        ],
    },
)
