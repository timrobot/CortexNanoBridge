# Goal
Create a tiny framework to be used at NVIDIA for the club for controlling Vex Robots

# CortexNanoBridge
- [x] Create a comms bridge between the VEX Cortex and Jetson Nano over UART
- [x] Create simple display for robot sensors, motors, controls, video, gps
- [x] Add autostart script (on linux login)
- [x] Add connection to server for getting poses

# Example

1. Push vex_cortex .c files onto the cortex
2. Run `test-basic.py`