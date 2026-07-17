import glob as _glob
import os as _os
from setuptools import setup
import os

package_name = 'perception'

# Find config files
config_files = []
config_dir = os.path.join(os.path.dirname(__file__), 'config')
if os.path.exists(config_dir):
    for f in os.listdir(config_dir):
        if f.endswith('.yaml') or f.endswith('.yml'):
            config_files.append(os.path.join('config', f))

# Find launch files
launch_dir = _os.path.join(_os.path.dirname(__file__), 'launch')
_launch_files = []
if _os.path.exists(launch_dir):
    for f in _glob.glob(_os.path.join(launch_dir, '*.launch.py')):
        _launch_files.append(_os.path.join('launch', _os.path.basename(f)))

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        'perception',
        'perception.detectors',
        'perception.modules',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + (
        [('share/' + package_name + '/config', config_files)] if config_files else []
    ) + (
        [('share/' + package_name + '/launch', _launch_files)] if _launch_files else []
    ),
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='yu.zhao',
    maintainer_email='yu.zhao@horizon.auto',
    description='ROS 2 Python node for open-vocabulary 3D detection using YOLO-E, ZED camera and Redis bridge',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'perception_node = perception:main',
        ],
    },
)
