Download the Jetson Nano Dev Kit SD Card Image (https://developer.nvidia.com/embedded/downloads)
Flash the Image onto the available SD Card using balenaEtcher. Close any popups that open up. Eject the SD card once finished.
After putting the SD card in, attach an HDMI cable, a keyboard and a mouse, then plug in the power cable to boot it up.
Use any username and password combo you wish. Remember to set "log in automatically" instead of "require my password to log in". Set power usage to MAXN (not 5W)

Once you are logged in, open a Terminal (you can right click on desktop to show Open Terminal) and type the following:
```bash
sudo apt-get update && sudo apt-get upgrade
sudo reboot
```
Keep all default (N) options as it prompts you for a choice

After reboot, open a new Terminal and type of the following:
```bash
sudo apt-get install python3-pip libssl-dev libxinerama-dev libxcursor-dev libcanberra-gtk-module libcanberra-gtk3-module
sudo pip3 install --upgrade pip
sudo pip3 install pyserial numpy scipy
sudo apt-get install python3-opencv
```

Then, after downloading and unzipping the pyrealsense library from , install it:
```bash
cd /path/to/pyrealsense
sudo ./install.sh
```

Once pyrealsense has been installed, go to the code repository for the cortex nano bridge, located at
https://github.com/timrobot/CortexNanoBridge

Push vex_cortex/main_app.c from the RobotC interface (only on windows and possibly mac) onto the VexCortex

Clone the repo to the Jetson Nano, navigate to the jetson_nano/ folder and run the installer:
```bash
sudo ./install.sh
```
Log out and log back in for the permissions on the TTY ports (connected to the VexCortex) to refresh correctly

**For now we use .sh scripts for installation of both the jetson nano material and the pyrealsense library since the
PyPI setup isn't working fully.

Once you have recorded the ip address for the robot, proceed with installing the API onto your laptop to connect:
https://github.com/timrobot/Cortano