## Cortex Nano Bridge

This library is meant to be a lightweight bridge between the VEX V5 or Cortex systems and the Jetson (Orin) Nano. Note that different versions of Jetson and VEX systems have different library support and behaviors, and unfortunately that means that this guide will be quite involved. However, once you are able to set up the bridge, creating your robot code should be straightforward.

### Getting Started
Follow the [Jetson Orin Nano guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit) or the [Jetson Nano guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit). Remember to set "log in automatically" instead of "require my password to log in". Set power usage to 15W (Jetson Orin Nano) or MAXN (Jetson Nano). Expand storage to max size as shown (ie. 59300).

### Jetson Package Installation

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
# only for Jetson Orin Nano, as the older Jetson Nano fails:
sudo pip3 install opencv-python pyrealsense2 qoi
```

Clone [this repo](https://github.com/timrobot/CortexNanoBridge) to the Jetson, navigate to the `jetson_nano/` folder and run the installer:
```bash
git clone https://github.com/timrobot/CortexNanoBridge.git
cd CortexNanoBridge/jetson_nano
sudo bash ./install.sh
sudo reboot
```

You have finished installing the software on the Jetson, now to install the software on the microcontroller.

### VEX Microcontroller Installation

In addition to installing the library on the Jetson Nano, we will need to push comms code to the robot controller. On your laptop or desktop, clone [this repo](https://github.com/timrobot/CortexNanoBridge) so that we can upload the comms code. You will only have to do this once.

#### V5 Brain
1. Install the [VEX Extension for VSCode](https://www.vexrobotics.com/vexcode/vscode-extension). Connect a microusb cable to the V5 Brain, and update firmware.
2. Create a new VEX Project > V5 > C/C++ > Clawbot Template (Motors). Copy `CortexNanoBridge/vex_v5/src/main.cpp` to the new project's `src/` folder. Build and download the code to the V5 Brain, and disconnect the microusb cable.
3. Plug in the RS485 cables on ports 19 and 20, and plug in their USBs into the Jetson. Run the application from the V5 Brain screen.

#### VEX Cortex
1. Install the [RobotC GUI](https://www.robotc.net/). Connect a usb cable to the Cortex, and update firmware. Remember to select (USB only) mode from the communication style menu.
2. Open `CortexNanoBridge/vex_cortex/main_app.c`. Compile and download the code to the Cortex, and disconnect the usb cable.
3. Wire the UART Cable to the UART1 pins on the Cortex, and plug in the USB into the Jetson.

Congratulations, you have finished setting everything up for the bridge! 👏

## Running Basic Code

You have two options to control your robot. If you have the Jetson Nano and a beefy desktop or laptop GPU, it is highly recommended that you do option 2.

#### Option 1. Jetson Control

Remember to plug in your Realsense camera into the robot if you wish to get the color and depth frames.

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

### Observation Space (V5)

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

#### Option 2. Remote Control from a Laptop or Desktop

On the Jetson, run either of the following to enable to automagical websocket communication stack.
```bash
# Vex V5 Brain
cd CortexNanoBridge/jetson_nano/scripts
bash ./enable-v5-autostart.sh
```
```bash
# Vex Cortex
cd CortexNanoBridge/jetson_nano/scripts
bash ./enable-cortex-autostart.sh
```

Then once you have [obtained the ip address](https://learnubuntu.com/check-ip-address/) for the Jetson, proceed with installing the [remote API](https://github.com/timrobot/Cortano) onto your laptop or desktop to connect. The service should be running every time the robot powers on.
