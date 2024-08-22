#!/bin/sh

echo "installing dependencies"
sudo python3 install.py
sudo python3 -m pip install .

# do operating system dependent actions
ubuntu_name=$(lsb_release -cs)
if [ "${ubuntu_name}" == 'jammy' ]; then
  echo 'Ubuntu 22.04 detected'
  echo 'installing kernel driver module for CH34x'
  sudo apt-get remove brltty
  git clone https://github.com/juliagoda/CH341SER
  cd CH341SER
  sudo make clean
  sudo make
  sudo make load
  sudo make install
  echo "ch34x" | sudo tee -a /etc/modules
  cd ..
elif [ "${ubuntu_name}" == 'focal' ]; then
  echo 'Ubuntu 20.04 detected; you are most likely on Jetpack 5'
  echo 'Please upgrade your kernel firmware using the Getting Started steps'
  exit 1
elif [ "${ubuntu_name}" == 'bionic' ]; then
  echo 'Ubuntu 18.04 detected'
  echo 'manually installing pyrealsense and qoi'
  sudo apt-get install python3-opencv
  curl -L "https://www.dropbox.com/scl/fi/0nhkxncc546qrksb4r5k6/pyrealsense2.zip?rlkey=26ix5qznmrmusebvvefgpcbi1&st=bwb4kug2&dl=1" -o pyrealsense2.zip
  unzip pyrealsense2.zip
  cd pyrealsense2
  sudo bash ./install.sh
  cd ..
  sudo cp -r qoi /usr/local/lib/python3.6/dist-packages
else
  echo 'Error: Unsupported or undetected Ubuntu version, quitting.'
  exit 1
fi

# /dev/ttyUSB* access from user, although it doesn't matter for su worker
if [ $(getent group dialout) ]; then
  echo "adding: $USER :to group dialout"
  sudo usermod -aG dialout $USER
  sudo usermod -aG tty $USER
else
  echo "creating group dialout and adding: $USER :"
  sudo newgrp dialout
  sudo usermod -aG dialout $USER
  sudo usermod -aG tty $USER
fi

echo "done installing, reboot your Jetson Nano"
