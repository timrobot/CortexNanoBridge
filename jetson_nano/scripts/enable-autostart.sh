#!/bin/sh

sudo copy nvcortexnano.service /lib/systemd/system/
sudo systemctl enable nvcortexnano.service
sudo daemon-reload
sudo systemctl start nvcortexnano.service