#!/bin/sh
echo "Enabling nvcortexnano.service..."
sudo cp scripts/nvcortexnano.service /lib/systemd/system/
sudo systemctl enable nvcortexnano.service
sudo systemctl daemon-reload
sudo systemctl start nvcortexnano.service
echo "Done."