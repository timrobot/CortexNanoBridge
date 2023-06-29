import sys
import os
USER = os.listdir("/home")[0]
sys.path.append(f"/home/{USER}/.local/lib/python3.8/site-packages")

from ..src.device import Robot, RealsenseCamera
from ..src import lan

if __name__ == "__main__":
  robot = Robot("/dev/ttyUSB0")
  lan.start("test-robot", frame_shape=(360, 1280, 3), source=True)
  cam = RealsenseCamera()

  while robot.running():
    frame = cam.get_combined_frame()
    lan.set_frame(frame)
    msg = lan.recv()
    if msg and isinstance(msg, dict) and "motor" in msg:
      motor_values = msg["motor"]
      for i in range(10):
        robot.motor[i] = motor_values[i]
    lan.send({ "sensor": list(robot.sensors()) })

  lan.stop()