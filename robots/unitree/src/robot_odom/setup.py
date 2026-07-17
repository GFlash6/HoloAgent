from glob import glob
import os

from setuptools import setup

package_name = 'robot_odom'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [f'resource/{package_name}'],
        ),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='siyu.zhou@horizon.cc',
    description='ROS 2 node for publishing Unitree robot IMU and odometry messages',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_extractor = robot_odom.imu_extractor:main',
        ],
    },
)
