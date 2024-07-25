### Setting up your Jetson (Orin) Nano
Follow the [Jetson Orin Nano guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit) or the [Jetson Nano Guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit). Remember to set "log in automatically" instead of "require my password to log in". Set power usage to MAXN (not 5W). Expand storage to max size as shown (ie. 59300).

### Robot Package Installation

Once you are logged in, open a Terminal, and type in the update cmds below.
Try to keep all default (N) options as it prompts you for a choice. If you pick (Y), don't worry - as long as it boots you are good to go. You don't have to restart docker daemon.
```bash
sudo apt-get update && sudo apt-get upgrade
sudo reboot
```

After reboot, open a new Terminal and type of the following:
```bash
sudo apt-get install python3-pip
sudo pip3 install --upgrade pip
sudo pip3 install pyserial numpy scipy websockets requests
sudo apt-get install python3-opencv
```

Then, after downloading and unzipping the pyrealsense library from this [package](https://1drv.ms/u/c/8c3293b14db03b6a/EZwnQdvx1BhGig5cujsEzWsB_hDSkxKt6gR09siBo1fkGw?e=0IuBHC), install it:
```bash
cd /path/to/pyrealsense
sudo ./install.sh
```

Clone [this repo](https://github.com/timrobot/CortexNanoBridge) to the Jetson, navigate to the `jetson_nano/` folder and run the installer:
```bash
git clone https://github.com/timrobot/CortexNanoBridge.git
cd CortexNanoBridge/jetson_nano
chmod +x install.sh
sudo ./install.sh
sudo reboot
```

### Controlling the Motors

Once pyrealsense has been installed, clone [this repo](https://github.com/timrobot/CortexNanoBridge) to your local machine. Push either
1. `vex_v5/src/main.cpp` from the VSCode Vex extension to the V5 Brain. Plug in the RS485 cables on ports 18 and 20, and plug in their USBs into the Jetson.
2. `vex_cortex/main_app.c` from the RobotC interface onto the Vex Cortex. Wire the UART Cable to the UART1 pins on the Vex Cortex, and plug in the USB into the Jetson.

#### Option 1. Jetson Control

Navigate to the `cortano` folder. Copy `run-robot.py` to anywhere. Then run it.
```bash
cp CortexNanoBridge/jetson_nano/cortano .
sudo python3 run-robot.py
```

#### Option 2. Remote Control from a local machine

On the Jetson, run either
```bash
# Vex V5 Brain
cd CortexNanoBridge/jetson_nano/scripts
chmod +x enable-v5-autostart.sh
./enable-v5-autostart.sh
```
```bash
# Vex Cortex
cd CortexNanoBridge/jetson_nano/scripts
chmod +x enable-cortex-autostart.sh
./enable-cortex-autostart.sh
```

Then once you have [obtained the ip address](https://learnubuntu.com/check-ip-address/) for the Jetson, proceed with installing the [laptop API](https://github.com/timrobot/Cortano) onto your laptop to connect.
