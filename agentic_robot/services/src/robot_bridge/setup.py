from setuptools import setup
from glob import glob

package_name = 'robot_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools', 'fastapi',
                      'uvicorn', 'pyyaml', 'requests'],
    zip_safe=True,
    maintainer='yu.zhao',
    maintainer_email='yu.zhao@horizon.auto',
    description='General-purpose HTTP ↔ ROS bridge replacing api_bridge + fastapi_robot',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'robot_bridge_node = robot_bridge.robot_bridge_node:main',
        ],
    },
)
