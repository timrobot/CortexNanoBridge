#!/bin/sh

sudo python3 install.py
sudo python3 -m pip install .

# /dev/ttyUSB* access from user, although it doesn't matter for su worker
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER

# worker enable
sudo cp scripts/nvcortexnano.service /lib/systemd/system/
sudo systemctl enable nvcortexnano.service
sudo systemctl daemon-reload
sudo systemctl start nvcortexnano.service