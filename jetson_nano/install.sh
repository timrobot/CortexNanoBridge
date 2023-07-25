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

# worker enable
echo "enabling nvcortexnano.service. to disable, run the following cmd:"
echo "sudo systemctl disable nvcortexnano.service"
echo ""
sudo systemctl disable nvcortexnano.service
sudo cp scripts/nvcortexnano.service /lib/systemd/system/
sudo systemctl enable nvcortexnano.service
sudo systemctl daemon-reload
sudo systemctl start nvcortexnano.service

echo "done installing, reboot your Jetson Nano"