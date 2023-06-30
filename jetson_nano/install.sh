#!/bin/sh

python3 install.py

# /dev/ttyACM* access from user
usermod -a -G dialout $USER
usermod -a -G tty $USER

# worker enable
sudo copy scripts/nvcortexnano.service /lib/systemd/system/
sudo systemctl enable nvcortexnano.service
sudo daemon-reload
sudo systemctl start nvcortexnano.service