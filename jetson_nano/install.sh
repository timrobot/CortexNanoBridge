#!/bin/sh

echo "installing dependencies"
sudo python3 install.py
sudo python3 -m pip install .

# /dev/ttyUSB* access from user, although it doesn't matter for su worker
if [ $(getent group dialout) ]; then
  echo "adding: $USER :to group dialout"
  sudo usermod -a -G dialout $USER
  sudo usermod -a -G tty $USER
else
  echo "creating group dialout and adding: $USER :"
  sudo newgrp dialout
  sudo usermod -a -G dialout $USER
  sudo usermod -a -G tty $USER
fi

# just in case we want to "reset" everything to its original state, disable the service
sudo systemctl disable nvcortexnano.service
echo "done installing, reboot your Jetson Nano"