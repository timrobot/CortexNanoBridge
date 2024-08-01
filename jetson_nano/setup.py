from setuptools import setup, find_packages

with open("requirements.txt", "r") as f:
    install_requires = [line.strip() for line in f.readlines() if line]

setup(
    name="cortano",
    version="0.0.6",
    packages=find_packages(exclude=["test"]),
    author="Timothy Yong",
    author_email="tyong_23@hotmail.com",
    description="Bridge between VEX Cortex Microcontroller and Nvidia Jetson Nano",
    install_requires=install_requires,
    zip_safe=False
)
