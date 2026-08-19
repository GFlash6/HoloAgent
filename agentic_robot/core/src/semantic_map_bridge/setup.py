from setuptools import setup


package_name = "semantic_map_bridge"

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
    description="ROS facade for HoloAgent semantic-map HTTP backends.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "semantic_map_bridge_node = semantic_map_bridge.node:main",
        ],
    },
)
