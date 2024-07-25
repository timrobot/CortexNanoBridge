#!/bin/sh
echo "Enabling nvv5nano.service..."
sudo systemctl disable nvv5nano.service
sudo cp nvv5nano.service /lib/systemd/system/
sudo systemctl enable nvv5nano.service
sudo systemctl daemon-reload
sudo systemctl start nvv5nano.service
echo "Done."