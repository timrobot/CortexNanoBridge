#!/bin/sh

PYTHON3VERSION=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')

if [[ "$PYTHON3VERSION" != "3.6" ]]; then
  echo "adding qoi (0.6.0) to requirements.txt"
  echo "qoi==0.6.0" >> requirements.txt
fi

echo "installing dependencies"
sudo python3 install.py
sudo python3 -m pip install .

if [[ "$PYTHON3VERSION" == "3.6" ]]; then
  echo "installing qoi for py36arm"
  sudo cp -r qoi /usr/local/lib/python3.6/dist-packages
fi

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