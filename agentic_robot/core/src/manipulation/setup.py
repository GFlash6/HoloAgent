from setuptools import setup


package_name = "manipulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HoloAgent maintainers",
    maintainer_email="yu.zhao@horizon.auto",
    description="ROS action facade for robot-specific manipulation skills.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "arm_skill_server = manipulation.arm_skill_server:main",
        ],
    },
)
