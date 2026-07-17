from setuptools import setup

package_name = 'relative_goal'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yu.zhao',
    maintainer_email='yu.zhao@horizon.auto',
    description='Converts relative movement commands to absolute PoseStamped goals via tf2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'relative_goal_node = relative_goal.relative_goal_node:main',
        ],
    },
)
