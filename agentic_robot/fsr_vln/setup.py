from setuptools import find_packages, setup

setup(
    name="fsr_vln",
    version="1.0.0",
    description="FSR-VLN: Fast and Slow Reasoning for Vision-Language Navigation with Hierarchical Multi-modal Scene Graph",
    author="See https://horizonrobotics.github.io/robot_lab/fsr-vln/",
    packages=find_packages(),
    py_modules=["fsr_vln", "api"],
    include_package_data=True,
)
