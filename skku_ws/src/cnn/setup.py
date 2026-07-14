from setuptools import setup
from glob import glob

package_name = 'cnn'

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
    description='benz.ino 1/10카 모방학습 차선주행: 데이터 수집 + 학습(오프라인) + 두 방식(raw 이미지 CNN / 허프 차선특징 MLP) 주행',
    license='TODO',
    entry_points={
        'console_scripts': [
            'camera_node        = cnn.camera_node:main',
            'benz_driver        = cnn.benz_driver:main',
            'teleop_keyboard    = cnn.teleop_keyboard:main',
            'dataset_recorder   = cnn.dataset_recorder:main',
            'raw_infer_node     = cnn.raw_infer_node:main',
            'feature_infer_node = cnn.feature_infer_node:main',
        ],
    },
)
