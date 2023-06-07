import sys
import os
USER = os.listdir("/home")[0]
sys.path.append(f"/home/{USER}/.local/lib/python3.8/site-packages")

from core.device import Robot, RealsenseCamera
from core import lan

if __name__ == "__main__":
  robot = Robot("/dev/ttyUSB0")
  lan.start("test-robot", source=True)
  cam = RealsenseCamera()

  while robot.running():
    color, depth = cam.read()
    lan.set_frame(color)
    msg = lan.recv()
    if msg and isinstance(msg, dict) and "motor" in msg:
      motor_values = msg["motor"]
      for i in range(10):
        robot.motor[i] = motor_values[i]

  lan.stop()