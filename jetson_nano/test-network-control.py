from core.robocomms import CortexController
from core import netcomms as nc

if __name__ == "__main__":
  robot = CortexController("COM4")
  robot.connect()
  nc.init(robot)

  while robot.running():
    keys = nc.keyboard()
    robot.motor[0] = (keys.KeyS - keys.KeyW) * 63
    robot.motor[9] = (keys.KeyQ - keys.KeyA) * 63

  nc.close()