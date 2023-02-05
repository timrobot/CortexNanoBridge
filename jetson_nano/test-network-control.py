from core.robocomms import CortexController
from core import netcomms as nc
import time

if __name__ == "__main__":
  robot = CortexController("COM4")
  robot.connect()
  nc.init(robot)

  while robot.running():
    keys = nc.keyboard()
    robot.motor[0] = (keys.KeyQ - keys.KeyA) * 63
    robot.motor[1] = (keys.KeyW - keys.KeyS) * 63
    time.sleep(0.01)