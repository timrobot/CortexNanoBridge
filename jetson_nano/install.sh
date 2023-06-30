#!/bin/sh

# pyrealsense2
python3 install.py
sudo cp ./pyrealsense2/config/99-realsense-libusb.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && udevadm trigger

# /dev/ttyACM* access from user
usermod -a -G dialout $USER
usermod -a -G tty $USER

# worker enable
sudo copy scripts/nvcortexnano.service /lib/systemd/system/
sudo systemctl enable nvcortexnano.service
sudo daemon-reload
sudo systemctl start nvcortexnano.service