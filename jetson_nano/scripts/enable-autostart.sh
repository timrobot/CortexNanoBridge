#!/bin/sh
cat "python3 $PWD/../app.py" > robotdaemon.sh
sudo ln -s $PWD/robotdaemon.sh /etc/profile.d/robotdaemon.sh