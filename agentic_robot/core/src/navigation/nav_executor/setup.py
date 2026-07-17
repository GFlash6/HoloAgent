from setuptools import setup

package_name = 'nav_executor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mengxinrui',
    maintainer_email='390271476@qq.com',
    description='Nav2 central dispatcher: subscribes to multi-source goals and calls BasicNavigator.goToPose',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_executor_node = nav_executor.pubpose:main',
        ],
    },
)
