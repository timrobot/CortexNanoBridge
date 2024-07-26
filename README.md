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

On your local machine, clone [this repo](https://github.com/timrobot/CortexNanoBridge). Push either
1. `vex_v5/src/main.cpp` from the [VSCode Vex extension](https://www.vexrobotics.com/vexcode/vscode-extension) to the V5 Brain. Plug in the RS485 cables on ports 18 and 20, and plug in their USBs into the Jetson.
2. `vex_cortex/main_app.c` from the [RobotC interface](https://www.robotc.net/) onto the Vex Cortex. Wire the UART Cable to the UART1 pins on the Vex Cortex, and plug in the USB into the Jetson.

#### Option 1. Jetson Control

Copy `CortexNanoBridge/jetson_nano/run-robot.py` to anywhere you wish. Then run it.
```bash
cp CortexNanoBridge/jetson_nano/cortano .
sudo python3 run-robot.py
```

Alternatively, you can create a basic script on the Jetson nano, which does the same thing:
```python
from cortano import VexV5, RealsenseCamera

if __name__ == "__main__":
  realsense = RealsenseCamera()
  robot = VexV5()

  while robot.running():
    color, depth = realsense.read()
    sensors, battery = robot.sensors()
    robot.motor[0] = 0 # you can set this to any value from -100 to 100
    robot.motor[9] = 0
```

### Action Space (V5)

| Num | Action | Control Min | Control Max | Unit |
| --- | ------ | ----------- | ----------- | ---- |
| 0 | Angular velocity target of the left motor | -100 | 100 | percent |
| 1 | ❌ |  |  |  |
| 2 | Angular velocity target of the claw | -100 | 100 | percent |
| 3 | ❌ |  |  |  |
| 4 | ❌ |  |  |  |
| 5 | ❌ |  |  |  |
| 6 | ❌ |  |  |  |
| 7 | Angular velocity target of the arm | -100 | 100 | percent |
| 8 | ❌ |  |  |  |
| 9 | Angular velocity target of the right motor | -100 | 100 | percent |

### Observation Space

| Num | Action | Min | Max | Unit |
| --- | ------ | --- | --- | ---- |
| 0  | Left Motor ang position | -inf | inf | position (degrees) |
| 1  | Left Motor ang velocity | -inf | inf | velocity (degrees/second) |
| 2  | Left Motor torque | -inf | inf | Nm * 1e3 |
| 3  | Right Motor ang position | -inf | inf | position (degrees) |
| 4  | Right Motor ang velocity | -inf | inf | velocity (degrees/se5ond) |
| 5  | Right Motor torque | -inf | inf | Nm * 1e3 |
| 6  | Arm ang position | -inf | inf | position (degrees) |
| 7  | Arm ang velocity | -inf | inf | velocity (degrees/second) |
| 8  | Arm torque | -inf | inf | Nm * 1e3 |
| 9  | Claw ang position | -inf | inf | position (degrees) |
| 10 | Claw ang velocity | -inf | inf | velocity (degrees/second) |
| 11 | Claw torque | -inf | inf | Nm * 1e3 |

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

Then once you have [obtained the ip address](https://learnubuntu.com/check-ip-address/) for the Jetson, proceed with installing the [laptop API](https://github.com/timrobot/Cortano) onto your local machine to connect.
