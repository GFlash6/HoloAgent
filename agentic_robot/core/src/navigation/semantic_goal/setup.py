import os
from setuptools import setup
from glob import glob

package_name = 'semantic_goal'

_default_conda_python_candidates = [
    '/home/unitree/miniconda3/envs/holoagent_semantic_mapping/bin/python',
    '/root/miniconda3/envs/holoagent_semantic_mapping/bin/python',
]
conda_python = os.environ.get('SEMANTIC_GOAL_CONDA_PYTHON')
if not conda_python:
    for candidate in _default_conda_python_candidates:
        if os.path.exists(candidate):
            conda_python = candidate
            break
if not conda_python:
    conda_python = _default_conda_python_candidates[0]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yu.zhao',
    maintainer_email='yu.zhao@horizon.auto',
    description='Semantic goal publisher: queries HMSG scene graph and publishes Nav2 waypoints',
    license='Apache-2.0',
    tests_require=['pytest'],
    scripts=['semantic_goal/semantic_goal_node.py'],
)
