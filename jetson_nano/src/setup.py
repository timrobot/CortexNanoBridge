from setuptools import setup, find_packages
from setuptools.command.install import install
import shutil
import os
import atexit

def _post_install():
    with open("pyrealsense2/install_manifest.txt", "r") as f:
        install_manifest = [line.strip() for line in f.readlines() if line]

    username = os.listdir('/home')[0]

    # format in the .whl
    # whl/
    #   device.py
    #   lan.py
    #   ...
    #   pyrealsense2/
    #     bin/
    #     lib/
    #     include/
    #     Documents/
    
    for path in install_manifest:
        split_path = path[1:].split("/")
        if split_path[0] == "usr" and split_path[1] == "local":
            if len(split_path) == 4: # file
                shutil.copy(os.path.join("pyrealsense2", split_path[2:]), path)
            else: # folder
                dst = "/" + "/".join(split_path[:4])
                if os.path.exists(dst): continue # already copied over
                shutil.copytree(os.path.join("pyrealsense2", split_path[2], split_path[3]), dst)
        elif split_path[0] == "home":
            split_path[1] = username
            dst = "/" + "/".join(split_path[:4])
            if os.path.exists(dst): continue
            shutil.copytree(os.path.join("pyrealsense2", split_path[2], split_path[3]), dst)


class PostInstallRealsenseLib(install):
    def __init__(self, *args, **kwargs):
        super(PostInstallRealsenseLib, self).__init__(*args, **kwargs)
        # NOTE: this is a hack. Better alternatives should be implemented in the future
        atexit.register(_post_install)

with open("requirements.txt", "r") as f:
    install_requires = [line.strip() for line in f.readlines() if line]

setup(
    name="nvcn",
    version="0.0.1",
    packages=find_packages(exclude=["scripts", "test"]),
    author="Timothy Yong",
    author_email="tyong_23@hotmail.com",
    description="Bridge between VEX Cortex Microcontroller and Nvidia Jetson Nano",
    install_requires=install_requires,
    zip_safe=False,
    cmdclass={
        "install": PostInstallRealsenseLib
    }
)