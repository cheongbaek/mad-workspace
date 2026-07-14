from setuptools import setup

package_name = 'nxde2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/k.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='windo',
    maintainer_email='test@test.com',
    description='internal test package (dummy)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker_k = nxde2.talker_k:main',
            'walker_k = nxde2.walker_k:main',
            'listener_k = nxde2.listener_k:main',
        ],
    },
)
