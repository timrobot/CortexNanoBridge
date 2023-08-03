#!/bin/sh
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