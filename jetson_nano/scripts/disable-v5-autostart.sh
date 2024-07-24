#!/bin/sh
echo "Disabling nvv5nano.service..."
sudo systemctl stop nvv5nano.service
sudo systemctl disable nvv5nano.service
echo "Done."