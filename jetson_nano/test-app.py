from core.robocomms import CortexController
from core import netcomms as nc
import time
import numpy as np

if __name__ == "__main__":
  robot = CortexController("COM4")
  robot.connect()
  
  start_time = time.time()
  while robot.running():
    left = np.sin(time.time() - start_time) * 30
    right = np.sin(time.time() - start_time) * 30

    robot.motor[1] = -right
    robot.motor[2] = -right
    robot.motor[3] = left
    robot.motor[4] = left

    time.sleep(0.01)
