#!/bin/sh
echo "Disabling nvcortexnano.service..."
sudo systemctl stop nvcortexnano.service
sudo systemctl disable nvcortexnano.service
echo "Done."