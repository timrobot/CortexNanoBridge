#!/bin/sh
sudo kill -9 $(ps aux | grep 'main.py' | grep -v grep | awk '{print $2}')