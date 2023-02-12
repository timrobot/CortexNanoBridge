from core.robocomms import CortexController
from core import netcomms as nc

if __name__ == "__main__":
  robot = CortexController("COM4", desc="robot.json")
  robot.connect()
  nc.init(robot)

  while robot.running():
    keys = nc.keyboard()
    robot.motor[1] = (keys.KeyS - keys.KeyW) * 63
    robot.motor[2] = (keys.KeyS - keys.KeyW) * 63
    robot.motor[3] = (keys.KeyQ - keys.KeyA) * 63
    robot.motor[4] = (keys.KeyQ - keys.KeyA) * 63

    x, y, zdeg = nc.position("chassis")
    print(f"pose: {x}, {y}, {zdeg}")

  nc.close()