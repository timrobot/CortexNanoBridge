#!/bin/sh

PYTHON3VERSION=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')

if [[ "$PYTHON3VERSION" != "3.6" ]]; then
  echo "adding qoi (0.6.0) to requirements.txt"
  echo "qoi==0.6.0" >> requirements.txt
fi

echo "installing dependencies"
sudo python3 install.py
sudo python3 -m pip install .

if [[ "$PYTHON3VERSION" != "3.6" ]]; then
  echo "qoi not added to requirements.txt"
else
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

# just in case we want to "reset" everything to its original state, disable the service
sudo systemctl disable nvcortexnano.service
echo "done installing, reboot your Jetson Nano"